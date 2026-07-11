`default_nettype wire

/*
    SH7709S unified cache - simple-dual-port (1R1W) banks, parallel-way, SINGLE CLOCK.

    Geometry: 16 kB, 4-way, 16-byte lines, 256 sets (see SH7709S.PDF pp.103-104).

    Two-beat lookup pipeline (one architectural cycle per beat, both overlapped so
    hits are back-to-back, IPC 1):

      beat 0 (request cycle): the pipe presents the live AGU address; the ACCEPT
        (req_ready) is decided from registered state + the PREVIOUS access's live
        resolve (z_ok below). At the edge the RAMs capture the read index and
        bram_* captures the request descriptor (acc_*_q capture the accept).
      beat 1 (resolve cycle): tag/data q are compared against bram_*; a HIT is
        answered COMBINATIONALLY (rsp_valid off the resolve) and consumed by the
        pipe at the closing edge - the same edge the NEXT access is captured.
        A MISS dispatches into the serialising FSM at that closing edge.

    The dclk half-clock speculative registers (do_*, resq stale-pair machinery,
    the 4-rail late-select) are DELETED: with one edge there is no held response
    slot - the resolve IS the response. Loss-free holding: a live hit response
    that the pipe cannot consume this cycle RETIRES into the registered rsp_*
    flags (held until consumed), so no response is ever overwritten.

    ACCEPT vs DISPATCH: an access Z accepted at edge E dispatches (on a miss) at
    edge E+1 - the same edge a new access A wants acceptance. z_ok blocks A
    whenever Z's resolve occupies the cache (any state-leaving transition, a
    registered-response post, or a CCR flush request). This is the single
    global-stall term; it carries the tag compare into req_ready by design.

    Back-to-back stores: a write-back store hit commits its strobed bytes at the
    resolve edge; the NEXT access's read captures at that same edge, so the
    write-through bypass registers inside cache_mem supply the just-written
    bytes (write-before-read order restored; see cache_mem.sv).

    Miss / external bus FSM: unchanged from the dclk design - each state lasts a
    full cycle and its RAM read lands one state later (the read-address capture
    at a state's closing edge presents that state's address). The only addition:
    a memory-mapped dispatch (S_MMTAG/S_MMDATA) redirects the read-address
    capture at the dispatch edge so the state's first cycle reads cur_*, keeping
    the dclk cycle counts.

    Replacement: 6-bit pseudo-LRU, Table 5.2 (cache_pkg). Write policy
    (pp.105,110-111): per-region mode (P1->CCR.CB, P0/U0/P3->~CCR.WT, wb_mode);
    U (dirty) bit in the tag entry; write hit WB=cache+U / WT=cache+memory;
    write miss WB=write-allocate / WT=memory only; read/inst miss fills.
    Write-back buffer (1 line, p.111-112, Fig 5.5). Memory-mapped cache windows
    (p.112-114): tag 0xF0xx_xxxx, data 0xF1xx_xxxx.
*/

import cache_pkg::*;

module cache #(
    parameter        BIG_ENDIAN = 1'b1
) (
    /* CLOCK AND RESET */
    input   wire            i_RST_n,
    input   wire            i_CLK,
    input   wire            i_CEN,

    /* PIPELINE UNIFIED L BUS (SH7709S L bus): the time-shared AGU drives one address
       per cycle - an IF fetch OR an MA data access - selected by req_fetch. DATA keeps
       priority: req_fetch is asserted only when no data access is presented this cycle. */
    LBus.slave              PIPE_L_BUS,
    input   wire            i_I_SQUASH,     //sideband: outstanding fetch is wrong-path; abort its fill
    input   wire            i_PIPE_D_PREF,  //sideband: the data request is a PREF line allocate

    /* UNIFIED EXTERNAL MEMORY BUS - one master. The single-state FSM serialises
       every external transaction (I-fill, D-fill, store, write-back drain, and
       non-cacheable bypass), so a 32-bit DBus master suffices; see pp.110-112. */
    IBus_1.master           I_BUS,

    /* L-BUS LOCAL EXCEPTION-REGISTER READ LINE (exc_handler group; that module owns the
       registers AND the decode) - a port on the live D response mux. */
    input   wire            i_LMMIO_EXC_HIT,    //the running access targets an exc_handler register
    input   wire            i_LMMIO_EXC_HIT_LIVE, //same decode off the LIVE address (accept-edge
                                                  //dispatch classification; write forward)
    input   wire    [31:0]  i_LMMIO_EXC_RDATA,  //that register's read value (stable for the resolve cycle)
    //Fire-and-forget exc-register WRITE forward: the cache accepts the access (local-register
    //class) and pulses this; exc_handler keys its commit on the live L-bus addr/wdata it snoops.
    output  wire            o_LMMIO_EXC_WE
);

///////////////////////////////////////////////////////////
//////  Cache-Control Registers (CCR/CCR2 MMIO)
////

localparam logic [31:0] ADDR_CCR       = 32'hFFFF_FFEC;
localparam logic [31:0] ADDR_CCR2_P4   = 32'hA400_00B0;

logic           ccr_ce;     //cache enable bit; see p.106
logic           ccr_wt;     //write-through bit for P0/U0/P3 (1=WT, 0=write-back)
logic           ccr_cb;     //write-back/through switch for P1 (1=write-back)
logic   [31:0]  ccr2;       //way-lock controls; stored only (DSP-gated, unused)
logic           flush_req;  //CCR.CF requested a flush; serviced from S_IDLE

(* direct_enable *) wire cen = i_CEN; //architectural edge enable; binds to DFF CE

//L-bus (P4) local registers, served on the MAIN live D response path. CCR/CCR2 live here;
//the exc_handler group arrives pre-selected on i_LMMIO_EXC_* (a port on the response mux).
//The decodes are edge latches off the live AGU address (like i_LMMIO_EXC_HIT in cpu_core),
//captured with bram_*: the 32-bit compares spend the request cycle in parallel with the
//RAM address capture, not the resolve cycle. NO handshake: a read is a live D response
//(res_d below), a write commits at the dispatch edge (fire-and-forget).
logic           bram_is_ccr;    //edge latch: running access == ADDR_CCR
logic           bram_is_ccr2;   //edge latch; no TLB: the phys 0x0400_ variant is unused
wire            bram_is_lmmio = bram_is_ccr || bram_is_ccr2 || i_LMMIO_EXC_HIT;
wire    [31:0]  lmmio_rdata   = bram_is_ccr  ? {29'd0, ccr_cb, ccr_wt, ccr_ce} :
                                bram_is_ccr2 ? ccr2 :
                                               i_LMMIO_EXC_RDATA;


///////////////////////////////////////////////////////////
//////  Controller State
////

typedef enum logic [4:0] {
    S_FLUSH,        //clear-walk over all 256 sets (reset + CCR.CF)
    S_IDLE,         //RUNNING: accept live, resolve bram_* one cycle later
    S_STORE_WR,     //store hit (write-through): write the merged word back to the data bank
    S_ALLOC_WR,     //write-allocate: merge the store into the filled line (U=1)
    S_WBUF_RD,      //write-back buffer: present a victim-word read
    S_WBUF_CAP,     //write-back buffer: capture the victim word
    S_DRAIN_REQ,    //write-back buffer: issue an external write of a buffered word
    S_DRAIN_WAIT,
    S_IFILL_REQ,    //instruction line fill: request a 16-bit halfword
    S_IFILL_WAIT,
    S_DFILL_REQ,    //data line fill: request a 32-bit longword
    S_DFILL_WAIT,
    S_STORE_REQ,    //write-through of a store
    S_STORE_WAIT,
    S_IBYP_REQ,     //non-cacheable instruction access
    S_IBYP_WAIT,
    S_DBYP_REQ,     //non-cacheable / write-through store-miss data access
    S_DBYP_WAIT,
    S_MMTAG,        //memory-mapped tag array: read / decide
    S_MMTAG_WR,     //memory-mapped tag array: write
    S_MMDATA        //memory-mapped data array: read / write
} state_t;

state_t         state;

//Post-write-back-buffer-load destination.
localparam logic [1:0] AW_IFILL = 2'd0;
localparam logic [1:0] AW_DFILL = 2'd1;
localparam logic [1:0] AW_MMTAG = 2'd2;
logic   [1:0]   after_wb;

logic   [8:0]   flush_idx;      //clear-walk index, counts 0..256

//Access context for whichever request the MISS FSM is currently servicing.
logic           cur_is_data;    //1 = data (MA) access, 0 = instruction (IF) access; cpu_core_tb I/D probe
logic   [31:0]  cur_addr;
logic           cur_write;
logic           cur_pref;       //current access is a PREF line allocate (read-like, no data out)
logic   [1:0]   cur_size;
logic   [31:0]  cur_wdata;
logic   [3:0]   cur_wstrb;
logic           cur_lock;
logic   [1:0]   cur_way;        //resolved hit/victim/addressed way
logic           mm_assoc_wr;    //memory-mapped tag write is associative (keep tag/LRU)
logic   [1:0]   fill_word;      //line fill: 4 longwords per line, WRAPPING from the missed word
logic   [31:0]  fill_base;
logic   [18:0]  victim_tag;     //evicted way's tag, for the write-back address

//Write-back buffer (one cache line + its physical address). See Fig 5.5.
logic           wb_valid;
logic   [31:4]  wb_pa;          //PA[31:4] of the buffered line
logic   [31:0]  wb_data [0:3];
logic   [1:0]   wb_load_word;   //victim copy-in progress
logic   [1:0]   wb_drain_word;  //drain-out progress
logic           drain_for_vic;  //draining to free the buffer for a new victim
logic           drain_to_wbuf;  //post-drain route: 1 = buffer the dirty victim (S_WBUF_RD),
                                //0 = alias-only drain - go straight to the fill (after_wb)

logic           mem_pending;

//Per-side REGISTERED responses (miss/bypass/MMIO completions, and any live hit
//response the pipe could not consume in its resolve cycle - the retirement path).
//Held until the owning master retires them.
logic           rsp_valid_i;
logic   [15:0]  rsp_inst;
logic           rsp_fault_i;
logic   [15:0]  rsp_sib;        //fetch-pair sibling opcode held with rsp_inst (see LBus)
logic           rsp_pair_q;     //pair qualifier captured with it (even fetch, fault-free)
logic           rsp_valid_d;
logic   [31:0]  rsp_rdata;
logic           rsp_fault_d;

//Bypass-fetch reuse buffer: a non-cacheable fetch reads a full longword but the pipe
//consumes 16 bits; keep the word so the sibling halfword needs no second bus run
//(the real chip also fetches 32 bits per external access). Mirrors MEMORY: loaded
//only from a fault-free bus read, dropped on any external write accept.
logic           ibyp_buf_valid;
logic   [31:2]  ibyp_buf_addr;  //longword address of the buffered word
logic   [31:0]  ibyp_buf_data;

///////////////////////////////////////////////////////////
//////  Lookup pipeline registers (request-edge captures)
////

//bram_* : the access captured at the last edge, phase-aligned with the RAM q.
//This IS the access being resolved this cycle. Latched UNCONDITIONALLY off the
//live grant every edge (read-ahead; a HIT costs 0 bubbles); cur_* backs it up
//on a dispatch.
logic           bram_is_data;   //1 = data (MA), 0 = instruction (IF)
logic   [31:0]  bram_addr;
logic           bram_write;
logic   [1:0]   bram_size;
logic   [3:0]   bram_wstrb;     //COMBINATIONAL - derived from bram_size/bram_addr below
logic           bram_pref;
logic           bram_lock;
//Store data captured WITH the descriptor: the commit happens at the resolve edge,
//one full cycle after capture, and by then the live pipe wdata belongs to the NEXT
//access (the dclk lock-step argument no longer holds across a whole cycle).
logic   [31:0]  bram_wdata_q;

//resq_q: the (bram_*, RAM q) pair captured at the last edge is a FRESH S_IDLE
//lookup (the read-address mux ran in S_IDLE). On the first cycle after an FSM
//excursion the pair is stale-mismatched (bram_* = the live/parked access, q =
//the FSM's cur-index read) - resq_q masks every resolve consumer then.
logic           resq_q;

//Accept captures: the request accepted at the last edge (its dispatch happens
//at the coming edge, off this cycle's resolve). At most one is set.
logic           acc_d_q;
logic           acc_i_q;

//Store byte-strobe DERIVED from the latched size + address low bits (no wstrb
//latch needed). Size encoding matches int_pipe mem_size_t (0=byte, 1=word, else long).
always_comb begin
    case(bram_size)
        2'd0:    bram_wstrb = BIG_ENDIAN ? (4'b1000 >> bram_addr[1:0]) : (4'b0001 << bram_addr[1:0]);  //byte
        2'd1:    bram_wstrb = BIG_ENDIAN ? (bram_addr[1] ? 4'b0011 : 4'b1100)                            //word
                                         : (bram_addr[1] ? 4'b1100 : 4'b0011);
        default: bram_wstrb = 4'b1111;                                                                   //long
    endcase
end

//Running-access address classification - edge latches off the live AGU address,
//captured with bram_*. The region decodes (2 LUT over the adder MSBs) spend the
//request cycle in PARALLEL with the bram_addr capture, off the resolve cone.
logic           bram_wb_mode_p1;    //edge latch: addr in P1 (write-back mode from CCR.CB; p.105)
logic           bram_region_c;      //edge latch: cacheable region (not P2/P4; pkg cacheable())
logic           bram_is_tagarr;     //edge latch: P4 tag-array window 0xF0 (p.112)
logic           bram_is_dataarr;    //edge latch: P4 data-array window 0xF1 (p.112)

wire            bram_wb_mode = bram_wb_mode_p1 ? ccr_cb : ~ccr_wt;

//Cacheability of the running access (D needs !lock; both need CCR.CE).
wire            bram_cacheable_d = ccr_ce && bram_region_c && !bram_lock;
wire            bram_cacheable_i = ccr_ce && bram_region_c;


///////////////////////////////////////////////////////////
//////  Tag / LRU / Data RAM Instances and Controls
////

//SINGLE address source (DATA priority over INSTRUCTION, h12p0.pdf 7.4.1). PIPE_L_BUS.req_addr
//IS the time-shared AGU output - data EA on an MA cycle, fetch PC on an IF cycle - so the RAM
//read index and the bram_* capture take it DIRECTLY. grant_d is the SHALLOW role bit
//(data-present = !req_fetch); the pipeline asserts req_fetch only when no D access is
//presented, so D priority is preserved.
logic   [7:0]   grant_idx;
logic   [1:0]   grant_word;
wire            grant_d     = !PIPE_L_BUS.req_fetch;                          //data cycle (shallow)
wire            d_req_valid = PIPE_L_BUS.req_valid && !PIPE_L_BUS.req_fetch;
wire            i_req_valid = PIPE_L_BUS.req_valid &&  PIPE_L_BUS.req_fetch;
assign  grant_idx  = PIPE_L_BUS.req_addr[11:4];
assign  grant_word = PIPE_L_BUS.req_addr[3:2];

//Simple-dual-port (1R1W): read port presents the live-grant lookup, the miss-FSM
//victim/array read, or the mm-dispatch redirect; write port carries the commits.
logic   [7:0]   tag_raddr;          //read-port index
logic   [7:0]   tag_waddr;          //write-port index
logic   [3:0]   tag_we;
logic   [20:0]  tag_wdata;
logic   [20:0]  tag_rdata [0:3];    //{valid, U, tag[18:0]} (write-through bypassed)

logic   [7:0]   lru_raddr;
logic   [7:0]   lru_waddr;
logic           lru_we;
logic   [5:0]   lru_wdata;
logic   [5:0]   lru_rdata;          //6-bit pseudo-LRU for the set; see p.104

logic   [9:0]   data_raddr;         //read-port {index, word}
logic   [9:0]   data_waddr;         //write-port {index, word}
logic   [9:0]   data_caddr;         //per-miss-state {index, word}
logic   [3:0]   data_we;            //per-way write enable
logic   [3:0]   data_bwe;           //per-BYTE lane enable (sub-word store; 1111 on fills)
logic   [31:0]  data_wdata;
logic   [31:0]  data_rdata [0:3];   //one word per way, read in parallel (bypassed)

//Hit / victim resolve from the registered reads (valid this cycle off bram_*).
logic   [3:0]   hit_w;
logic           hit;
logic   [1:0]   hit_way;
logic   [1:0]   victim;
logic   [3:0]   victim_oh;          //one-hot victim (parallel form; rows disjoint, see pkg)
logic   [3:0]   dirty_w;            //per-way V && U off the tag q - one LUT each
logic           victim_dirty;       //victim way holds a dirty line: 2 PARALLEL levels off
                                    //the RAM q, replacing lru->encode->tag_rdata[victim]
logic   [18:0]  victim_tag_w;       //tag of the victim way, one-hot AND-OR (no index chain)
logic   [31:0]  hit_word;           //the addressed longword of the hit way
logic   [3:0]   hit_pri;            //priority one-hot of hit_w (way0 wins ... way3 default)
logic   [5:0]   lru_wdata_hit;      //flat lru_update(hit_way, lru_rdata)

//Memory-mapped tag-array way selection (the MISS-FSM mm access in cur_*).
logic   [1:0]   mm_way;             //addr-field way
logic   [3:0]   mm_match_w;         //associative tag match per way
logic           mm_match;
logic   [1:0]   mm_match_way;
logic   [1:0]   mm_sel_way;         //assoc -> matched way, else addr-field way

// synthesis translate_off
//Exhaustive check (64 LRU codes): the one-hot victim must be one-hot and encode the same
//way as the priority casez form for every code, including the all-zero reset default.
initial begin
    for(int i = 0; i < 64; i++) begin
        logic [3:0] oh;
        logic [1:0] enc;
        oh  = lru_victim_oh(i[5:0]);
        enc = lru_victim(i[5:0]);
        if($countones(oh) != 1 || !oh[enc])
            $fatal(1, "lru_victim_oh mismatch: lru=%06b oh=%04b enc=%0d", i[5:0], oh, enc);
    end
end
// synthesis translate_on

//EARLY store-commit qualifier: every run_d_store_wb factor EXCEPT the tag compare -
//edge-latched classify bits plus the pair-freshness gate, one LUT plane.
wire    store_wb_qual = resq_q && bram_is_data && bram_write && !bram_pref &&
                        bram_cacheable_d && bram_wb_mode;

always_comb begin
    //Hit resolve of the running access (bram_*). tag_rdata/data_rdata were read at
    //bram's capture edge at its index, so they match bram_* this cycle.
    hit_w[0] = tag_rdata[0][20] && (tag_rdata[0][18:0] == tag_of(bram_addr));
    hit_w[1] = tag_rdata[1][20] && (tag_rdata[1][18:0] == tag_of(bram_addr));
    hit_w[2] = tag_rdata[2][20] && (tag_rdata[2][18:0] == tag_of(bram_addr));
    hit_w[3] = tag_rdata[3][20] && (tag_rdata[3][18:0] == tag_of(bram_addr));
    //Priority one-hot mirroring hit_way (= w0?0:w1?1:w2?2:3, so way 3 is the default row;
    //rows only matter when lru_we is set, i.e. on a real hit).
    hit_pri[0] =  hit_w[0];
    hit_pri[1] = !hit_w[0] &&  hit_w[1];
    hit_pri[2] = !hit_w[0] && !hit_w[1] && hit_w[2];
    hit_pri[3] = !hit_w[0] && !hit_w[1] && !hit_w[2];
    //Flat lru_update(hit_way, lru_rdata): AND-OR of the four per-way candidates (see
    //cache_pkg lru_update - each row is constants + lru_rdata bit moves, zero logic).
    lru_wdata_hit = ({6{hit_pri[0]}} & lru_update(2'd0, lru_rdata)) |
                    ({6{hit_pri[1]}} & lru_update(2'd1, lru_rdata)) |
                    ({6{hit_pri[2]}} & lru_update(2'd2, lru_rdata)) |
                    ({6{hit_pri[3]}} & lru_update(2'd3, lru_rdata));
    hit      = |hit_w;
    hit_way  = hit_w[0] ? 2'd0 : hit_w[1] ? 2'd1 : hit_w[2] ? 2'd2 : 2'd3;
    //Flatten hit data selection; non-assoc tag writes can create duplicate tags, p.112.
    hit_word = ({32{hit_w[0]}} & data_rdata[0]) |
               ({32{hit_w[1]}} & data_rdata[1]) |
               ({32{hit_w[2]}} & data_rdata[2]) |
               ({32{hit_w[3]}} & data_rdata[3]);
    victim   = lru_victim(lru_rdata);
    victim_oh    = lru_victim_oh(lru_rdata);
    dirty_w[0]   = tag_rdata[0][20] && tag_rdata[0][19];
    dirty_w[1]   = tag_rdata[1][20] && tag_rdata[1][19];
    dirty_w[2]   = tag_rdata[2][20] && tag_rdata[2][19];
    dirty_w[3]   = tag_rdata[3][20] && tag_rdata[3][19];
    victim_dirty = |(dirty_w & victim_oh);
    victim_tag_w = ({19{victim_oh[0]}} & tag_rdata[0][18:0]) |
                   ({19{victim_oh[1]}} & tag_rdata[1][18:0]) |
                   ({19{victim_oh[2]}} & tag_rdata[2][18:0]) |
                   ({19{victim_oh[3]}} & tag_rdata[3][18:0]);

    //Memory-mapped: way from addr[13:12]; associative compares tag field to all ways.
    //Off cur_* - the mm access is owned by the MISS FSM (S_MMTAG) in cur_*.
    mm_way       = cur_addr[13:12];
    mm_match_w[0] = tag_rdata[0][20] && (tag_rdata[0][18:0] == cur_wdata[28:10]);
    mm_match_w[1] = tag_rdata[1][20] && (tag_rdata[1][18:0] == cur_wdata[28:10]);
    mm_match_w[2] = tag_rdata[2][20] && (tag_rdata[2][18:0] == cur_wdata[28:10]);
    mm_match_w[3] = tag_rdata[3][20] && (tag_rdata[3][18:0] == cur_wdata[28:10]);
    mm_match     = |mm_match_w;
    mm_match_way = mm_match_w[0] ? 2'd0 : mm_match_w[1] ? 2'd1 : mm_match_w[2] ? 2'd2 : 2'd3;
    mm_sel_way   = cur_addr[3] ? mm_match_way : mm_way;     //addr[3] = associative bit
end

//Running-access classification (combinational, off bram_* + resolve). Response terms
//are gated by the accept capture (only an accepted access is answered); the store
//commit stays accept-FREE like dclk (an unaccepted store-hit commit is idempotent -
//the held request re-commits the same bytes when finally accepted).
wire    run_d_hit_load  = bram_is_data && !bram_write && !bram_pref &&
                          bram_cacheable_d && hit;
wire    run_d_hit_pref  = bram_is_data && bram_pref &&
                          bram_cacheable_d && hit;
wire    run_i_hit       = !bram_is_data && bram_cacheable_i && hit;
//Every factor of the MRU-update terms above EXCEPT the compare, for the LRU write
//enable: lru_we = lru_mru_qual && hit (one LUT after the compares).
wire    lru_mru_qual =
        resq_q && ((bram_is_data && bram_cacheable_d &&
                    ((!bram_write && !bram_pref) || bram_pref ||
                     (bram_write && !bram_pref && bram_wb_mode))) ||
                   (!bram_is_data && bram_cacheable_i));

//Live response of the running access. A write-back store hit is a NOTIFY: it commits
//its bank word (data_we) but raises NO response - the pipe retires it on accept
//(ma_cpl_now). Local L-bus MMIO (CCR/CCR2/exc) READ resolves like a hit: lmmio_rdata
//is an added input to the res_d_rdata mux, selected off the latched classify. No RAM hit needed.
wire            run_d_lmmio_read = bram_is_data && !bram_write && bram_is_lmmio;
wire            res_d_valid = run_d_hit_load || run_d_hit_pref || run_d_lmmio_read;
wire    [31:0]  res_d_rdata = run_d_lmmio_read ? lmmio_rdata :
                              run_d_hit_load   ? hit_word    : 32'd0;   //pref returns no data
wire            res_i_valid = run_i_hit;
wire    [15:0]  res_i_inst  = pick_inst(hit_word, bram_addr[1], BIG_ENDIAN);
//Fetch-pair sibling: the hit longword's OTHER halfword (LBus rsp_inst_sib).
wire    [15:0]  res_i_sib   = pick_inst(hit_word, !bram_addr[1], BIG_ENDIAN);

//LIVE (accept-edge) classification of the request being accepted THIS edge. Dispatches
//whose target does not depend on the resolve - non-cacheable bypass, the memory-mapped
//array windows, CCR/CCR2 writes, the non-cacheable PREF ack - happen AT the accept edge
//(as the dclk design did), so those accesses keep their dclk cycle counts. Only the
//cacheable hit/miss decisions wait for the resolve (the acc_*_q arm). These decodes sit
//on the live AGU output; they feed only the state/cur_* capture at this edge.
wire            live_region_c   = cacheable(PIPE_L_BUS.req_addr);
wire            live_is_tagarr  = PIPE_L_BUS.req_addr[31:24] == 8'hF0;
wire            live_is_dataarr = PIPE_L_BUS.req_addr[31:24] == 8'hF1;
wire            live_is_ccr     = PIPE_L_BUS.req_addr == ADDR_CCR;
wire            live_is_ccr2    = PIPE_L_BUS.req_addr == ADDR_CCR2_P4;
wire            live_is_lmmio   = live_is_ccr || live_is_ccr2 || i_LMMIO_EXC_HIT_LIVE;
wire            live_pref       = grant_d && i_PIPE_D_PREF;
wire            live_cacheable_d = ccr_ce && live_region_c && !PIPE_L_BUS.req_lock;
wire            live_cacheable_i = ccr_ce && live_region_c;
//Bypass-buffer hit: the live fetch targets the buffered longword. Feeds only the
//S_IDLE registered-response arm (internal regs) - never the external request pins.
wire            ibyp_buf_hit    = ibyp_buf_valid &&
                                  (PIPE_L_BUS.req_addr[31:2] == ibyp_buf_addr);
//Live store byte-strobe (cur_wstrb capture at a live dispatch); twin of bram_wstrb.
logic   [3:0]   live_wstrb;
always_comb begin
    case(PIPE_L_BUS.req_size)
        2'd0:    live_wstrb = BIG_ENDIAN ? (4'b1000 >> PIPE_L_BUS.req_addr[1:0])
                                         : (4'b0001 << PIPE_L_BUS.req_addr[1:0]);
        2'd1:    live_wstrb = BIG_ENDIAN ? (PIPE_L_BUS.req_addr[1] ? 4'b0011 : 4'b1100)
                                         : (PIPE_L_BUS.req_addr[1] ? 4'b1100 : 4'b0011);
        default: live_wstrb = 4'b1111;
    endcase
end

//Tag RAM controls. Entry = {valid, U, tag}.
always_comb begin
    tag_we    = 4'b0000;
    tag_wdata = {1'b1, 1'b0, tag_of(cur_addr)};
    if(state == S_FLUSH) begin
        tag_raddr = flush_idx[7:0];
        tag_waddr = flush_idx[7:0];
        tag_wdata = 21'd0;                          //clear valid + U
        tag_we    = 4'b1111;
    end
    else begin
        //Read port: live grant while RUNNING (mm dispatch redirects to the serviced
        //index); the serviced index in miss states. Write port: bram index for the
        //running store-hit U write; cur index for fills/mm.
        tag_raddr = (state == S_IDLE) ? grant_idx : cur_addr[11:4];
        tag_waddr = (state == S_IDLE) ? bram_addr[11:4] : cur_addr[11:4];
        //1-cycle write-back store hit: set the dirty bit on the hit way, but only when it is
        //not already dirty - so a run of stores to a now-dirty line skips the tag write. V/tag
        //unchanged, so a clean-line first store writes {V=1, U=1, tag}. (WT hits never set U.)
        //DATAIN is set off state ONLY (the S_IDLE arm writes nothing else): the deep
        //resolve gates just the 1-bit WE, keeping the tag-compare cone off the 21-bit
        //M10K datain input registers.
        if(state == S_IDLE) begin
            tag_wdata = {1'b1, 1'b1, tag_of(bram_addr)};
            tag_we[0] = store_wb_qual && hit_w[0] && !tag_rdata[0][19];
            tag_we[1] = store_wb_qual && hit_w[1] && !tag_rdata[1][19];
            tag_we[2] = store_wb_qual && hit_w[2] && !tag_rdata[2][19];
            tag_we[3] = store_wb_qual && hit_w[3] && !tag_rdata[3][19];
        end
        //Fill tag update: completion validates the line. A mid-line FAULT instead
        //INVALIDATES the victim way: earlier beats already overwrote its data words,
        //and the old tag would otherwise stay valid over the half-filled line - a
        //later access to the old address would hit CORRUPT data (victim-integrity golden).
        if(state == S_IFILL_WAIT && I_BUS.rsp_valid && I_BUS.rsp_ready) begin
            if(I_BUS.rsp_fault) begin
                tag_wdata       = 21'd0;                    //kill V (and U) of the victim way
                tag_we[cur_way] = 1'b1;
            end
            else if(fill_last) begin
                tag_wdata       = {1'b1, 1'b0, tag_of(cur_addr)};
                tag_we[cur_way] = 1'b1;
            end
        end
        if(state == S_DFILL_WAIT && I_BUS.rsp_valid && I_BUS.rsp_ready) begin
            if(I_BUS.rsp_fault) begin
                tag_wdata       = 21'd0;
                tag_we[cur_way] = 1'b1;
            end
            else if(fill_last) begin
                tag_wdata       = {1'b1, cur_write, tag_of(cur_addr)};  //U=1 for write-allocate
                tag_we[cur_way] = 1'b1;
            end
        end
        if(state == S_MMTAG_WR) begin
            //Memory-mapped tag write. Associative keeps tag+LRU, sets V/U only;
            //non-associative writes the full tag field.
            tag_wdata       = mm_assoc_wr ? {cur_wdata[0], cur_wdata[1], tag_rdata[cur_way][18:0]}
                                          : {cur_wdata[0], cur_wdata[1], cur_wdata[28:10]};
            tag_we[cur_way] = 1'b1;
        end
    end
end

//LRU RAM controls.
always_comb begin
    lru_we    = 1'b0;
    lru_wdata = lru_wdata_hit;      //flat form (== lru_update(hit_way, lru_rdata))
    if(state == S_FLUSH) begin
        lru_raddr = flush_idx[7:0];
        lru_waddr = flush_idx[7:0];
        lru_wdata = 6'd0;
        lru_we    = 1'b1;
    end
    else begin
        lru_raddr = (state == S_IDLE) ? grant_idx : cur_addr[11:4];
        lru_waddr = (state == S_IDLE) ? bram_addr[11:4] : cur_addr[11:4];
        if(state == S_IDLE) begin
            //Mark the hit way MRU on any running hit (load / inst / write-back store).
            //FLAT WE: lru_mru_qual folds every run_* factor except the compare.
            if(lru_mru_qual && (hit_w[0] || hit_w[1] || hit_w[2] || hit_w[3])) begin
                lru_we    = 1'b1;
                lru_wdata = lru_wdata_hit;
            end
        end
        else if((state == S_IFILL_WAIT || state == S_DFILL_WAIT) &&
                I_BUS.rsp_valid && I_BUS.rsp_ready &&
                !I_BUS.rsp_fault && fill_last) begin
            lru_wdata = lru_update(cur_way, lru_rdata);
            lru_we    = 1'b1;
        end
        else if(state == S_MMTAG_WR && !mm_assoc_wr) begin
            lru_wdata = cur_wdata[9:4];                 //software sets LRU from the data field
            lru_we    = 1'b1;
        end
    end
end

//Data bank controls. Reads use {index, word}; writes target one way's bank. Sub-word
//stores use the M10K BYTE lanes (data_bwe): unstrobed bytes keep the array content.
//datain is the raw store/fill word - EARLY sources only (captured wdata / cur_* / MEM
//rsp) - and the deep tag-compare resolve drives only the 1-bit way enables.
always_comb begin
    data_we    = 4'b0000;
    data_bwe   = 4'b1111;
    data_wdata = 32'd0;
    data_caddr = {cur_addr[11:4], cur_addr[3:2]};
    unique case(state)
        S_IDLE: begin
            //1-cycle write-back store hit: commit the strobed bytes at this resolve edge.
            //datain and byte-lanes are UNCONDITIONAL off the running access (don't-care
            //unless a way WE fires). On (manual-discouraged, p.112) duplicate tags the
            //store commits to EVERY matching way, consistent with the OR-ed read resolve.
            data_wdata = bram_wdata_q;
            data_bwe   = bram_wstrb;
            data_we    = {4{store_wb_qual}} & hit_w;
        end
        S_STORE_WR: begin
            data_wdata = cur_wdata;
            data_bwe   = cur_wstrb;
            data_we    = 4'b0001 << cur_way;
        end
        S_ALLOC_WR: begin
            data_wdata = cur_wdata;
            data_bwe   = cur_wstrb;
            data_we    = 4'b0001 << cur_way;
        end
        S_WBUF_RD, S_WBUF_CAP:
            data_caddr = {cur_addr[11:4], wb_load_word};        //copy victim line out
        S_IFILL_WAIT, S_DFILL_WAIT: begin   //I and D fills both store one longword per beat
            data_caddr = {cur_addr[11:4], fill_word};
            data_wdata = I_BUS.rsp_rdata;
            if(I_BUS.rsp_valid && I_BUS.rsp_ready && !I_BUS.rsp_fault)
                data_we = 4'b0001 << cur_way;
        end
        S_MMDATA: begin
            data_wdata = cur_wdata;
            if(cur_write) data_we = 4'b0001 << cur_addr[13:12];
        end
        default: ;
    endcase
    //Read port: live grant while RUNNING (a live mm dispatch reads its own grant index,
    //so the mm state's first cycle sees cur_*'s q); the miss-state {index,word} otherwise.
    //The write port targets the store cell (bram) in S_IDLE, else the serviced cell (cur).
    data_raddr = (state == S_IDLE) ? {grant_idx, grant_word} : data_caddr;
    data_waddr = (state == S_IDLE) ? {bram_addr[11:4], bram_addr[3:2]} : data_caddr;
end

genvar gw;
generate
    for(gw = 0; gw < 4; gw = gw + 1) begin : g_way
        cache_tag_ram_wt u_tag (
            .i_CLK    (i_CLK          ),
            .i_EN     (cen            ),
            .i_RADDR  (tag_raddr      ),
            .i_WE     (tag_we[gw]     ),
            .i_WADDR  (tag_waddr      ),
            .i_DI     (tag_wdata      ),
            .o_DO     (tag_rdata[gw]  )
        );
        cache_data_bank_wt u_data (
            .i_CLK    (i_CLK          ),
            .i_EN     (cen            ),
            .i_RADDR  (data_raddr     ),
            .i_WE     (data_we[gw]    ),
            .i_BWE    (data_bwe       ),  //byte lanes: sub-word store keeps unstrobed bytes
            .i_WADDR  (data_waddr     ),
            .i_DI     (data_wdata     ),
            .o_DO     (data_rdata[gw] )
        );
    end
endgenerate

cache_lru_ram_wt u_lru (
    .i_CLK    (i_CLK      ),
    .i_EN     (cen        ),
    .i_RADDR  (lru_raddr  ),
    .i_WE     (lru_we     ),
    .i_WADDR  (lru_waddr  ),
    .i_DI     (lru_wdata  ),
    .o_DO     (lru_rdata  )
);


///////////////////////////////////////////////////////////
//////  Bus Handshakes (DATA prioritised over INSTRUCTION)
////

//Live hit responses: only an ACCEPTED access is answered (acc_*_q implies the pair is
//fresh - an accept requires two consecutive S_IDLE cycles); state gating keeps a
//dispatched-away FSM run from re-presenting.
wire    hit_rsp_d = (state == S_IDLE) && acc_d_q && res_d_valid;
wire    hit_rsp_i = (state == S_IDLE) && acc_i_q && res_i_valid;

//Response channel arbitration. DATA has priority over INSTRUCTION: a held/registered D
//response must reach the waiting MA stage before a fetch response (which is only running
//ahead), else the fetch would shadow the D response forever and deadlock MA. rsp_fetch
//tells the pipeline which stage consumes. Live and registered responses of ONE side
//never coexist (a registered response blocks that side's accept), so each data field
//is a plain 2:1 live/registered pick.
wire    rsp_d_act = hit_rsp_d || rsp_valid_d;
wire    rsp_i_act = hit_rsp_i || rsp_valid_i;
wire    rsp_fetch_w = !rsp_d_act && rsp_i_act;              //D priority

assign  PIPE_L_BUS.rsp_valid = rsp_d_act || rsp_i_act;
assign  PIPE_L_BUS.rsp_fetch = rsp_fetch_w;
assign  PIPE_L_BUS.rsp_rdata = hit_rsp_d ? res_d_rdata : rsp_rdata;         //D-only 2:1
assign  PIPE_L_BUS.rsp_rdata_hit  = res_d_rdata;    //dual-aligner raw feeds (see LBus)
assign  PIPE_L_BUS.rsp_rdata_miss = rsp_rdata;
assign  PIPE_L_BUS.rsp_hit_d      = hit_rsp_d;      //post-alignment select
assign  PIPE_L_BUS.rsp_inst  = hit_rsp_i ? res_i_inst : rsp_inst;           //I-only 2:1
//Fetch pair: a live hit pairs on an even fetch (a hit never faults); registered
//responses carry the qualifier captured alongside them (LBus rsp_pair contract).
assign  PIPE_L_BUS.rsp_inst_sib = hit_rsp_i ? res_i_sib : rsp_sib;
assign  PIPE_L_BUS.rsp_pair     = hit_rsp_i ? !bram_addr[1] : (rsp_valid_i && rsp_pair_q);
//A hit never faults, and a registered response excludes a same-side live hit, so each
//fault line is a pure registered-flag product - no live select.
assign  PIPE_L_BUS.rsp_dfault = rsp_valid_d && rsp_fault_d;
assign  PIPE_L_BUS.rsp_ifault = rsp_valid_i && rsp_fault_i;

//Response consumes (the pipe routes rsp_ready by rsp_fetch).
wire    cons_d = rsp_d_act && !rsp_fetch_w && PIPE_L_BUS.rsp_ready;
wire    cons_i = rsp_i_act &&  rsp_fetch_w && PIPE_L_BUS.rsp_ready;

//Z-RESOLVE STALL (the single global-stall term): the access accepted last edge is
//dispatching into the FSM (or posting a squash ack) at the coming edge, so a new
//accept this edge would double-book the cache. Only the RESOLVE-dependent dispatch
//cases appear: everything classify-only dispatched LIVE at its accept edge, so its
//state/flush/rsp effects already show in the registered accept terms. The two
//"don't care" 1'b1 arms are accesses that left S_IDLE at their accept edge - the
//(state == S_IDLE) factor of the accepts kills those cycles regardless.
wire    d_clean = bram_pref       ? (!bram_cacheable_d || hit) :
                  bram_is_lmmio   ? 1'b1 :
                  bram_cacheable_d ? (hit && !(bram_write && !bram_wb_mode)) :
                  1'b1;
wire    i_clean = !bram_cacheable_i || hit;     //a squashed HIT is clean (drop-consumed);
                                                //a squashed miss posts rsp_valid_i
wire    z_ok    = !(acc_d_q || acc_i_q) || (acc_d_q ? d_clean : i_clean);

//Accept (live, resolve-free apart from z_ok): registered state/flags + the live
//request role. The accept captures (acc_*_q) at the edge are exactly valid && ready.
wire    d_req_ready = (state == S_IDLE) && !flush_req && !rsp_valid_d && z_ok;
wire    i_req_ready = (state == S_IDLE) && !flush_req && !rsp_valid_i && z_ok;

assign  PIPE_L_BUS.req_ready = PIPE_L_BUS.req_fetch ? i_req_ready : d_req_ready;

wire    acc_d_nx = d_req_valid && d_req_ready;
wire    acc_i_nx = i_req_valid && i_req_ready;

//Exc-register write forward: the access is being accepted as a local register and it is
//an exc-group write. CCR/CCR2 writes commit inside the cache instead; exc-group writes
//are owned by exc_handler, so pulse it at the accept edge (fire-and-forget, one cycle);
//exc_handler snoops the live L-bus addr/wdata itself.
assign  o_LMMIO_EXC_WE    = acc_d_nx && PIPE_L_BUS.req_write && i_LMMIO_EXC_HIT_LIVE;

//Single external master. Address/size/data are muxed by state. Both line fills
//read 4 longwords (req_size=2); an instruction fetch (fill or bypass) is a read,
//and pick_inst extracts the addressed 16-bit opcode when the response returns.
wire    st_fill = (state == S_IFILL_REQ) || (state == S_DFILL_REQ);

assign  I_BUS.req_valid = !mem_pending &&
                            (st_fill || (state == S_IBYP_REQ) || (state == S_STORE_REQ) ||
                             (state == S_DBYP_REQ) || (state == S_DRAIN_REQ));
assign  I_BUS.req_write = (state == S_STORE_REQ) || (state == S_DRAIN_REQ) ||
                            (state == S_DBYP_REQ && cur_write);
assign  I_BUS.req_size  = (state == S_DBYP_REQ || state == S_STORE_REQ) ? cur_size : 2'd2;
assign  I_BUS.req_burst = st_fill || (state == S_DRAIN_REQ);    //line-aligned 4-beat fill/drain
assign  I_BUS.req_addr  = st_fill                ? (fill_base + {28'd0, fill_word, 2'b00}) :
                            (state == S_DRAIN_REQ) ? {wb_pa, wb_drain_word, 2'b00} : cur_addr;
assign  I_BUS.req_wdata = (state == S_DRAIN_REQ) ? wb_data[wb_drain_word] : cur_wdata;
assign  I_BUS.req_wstrb = st_fill                ? 4'b0000 :
                            (state == S_DRAIN_REQ) ? 4'b1111 : cur_wstrb;
assign  I_BUS.req_lock  = cur_lock;
//DMAC sideband: CPU accesses never carry DACK/single-address tags
assign  I_BUS.req_dack    = 1'b0;
assign  I_BUS.req_dack_ch = 1'b0;
assign  I_BUS.req_saddr   = 1'b0;
assign  I_BUS.rsp_ready = mem_pending &&
                            ((state == S_IFILL_WAIT) || (state == S_IBYP_WAIT) ||
                             (state == S_DFILL_WAIT) || (state == S_STORE_WAIT) ||
                             (state == S_DBYP_WAIT)  || (state == S_DRAIN_WAIT));

//Miss-dispatch victim-capture qualifier: EVERY factor of the three S_IDLE fill sites
//(PREF allocate / D fill / I fill) EXCEPT the tag compare - accepts + region/kind
//classify latches only. The dispatch CE (vic_ld) then ANDs the miss (no way hit).
wire            vic_ctl_d = bram_cacheable_d &&
                            (bram_pref || (!bram_is_tagarr && !bram_is_dataarr &&
                                           !bram_is_lmmio &&
                                           !(bram_write && !bram_wb_mode)));
wire            vic_ctl_i = bram_cacheable_i && !i_I_SQUASH;
wire            vic_pre = (acc_d_q && vic_ctl_d) || (acc_i_q && vic_ctl_i);
wire            vic_ld  = vic_pre && !hit;

//WB-BUFFER ALIAS GUARD: the missed line's own dirty copy still sits undrained in the
//write-back buffer (fill-first ordering, pp.111-112). Filling from memory now would read
//stale data AND the later drain would strand the stale line in the array (lost update).
//Force a full drain first, then fill. PA line compare: region bits [31:29] ignored
//(P0/P1 alias to one PA); wb_pa[31:29] is 000 by construction. Fills only - non-cacheable
//bypass vs a buffered line stays software-coherency territory, like bypass vs the array.
wire    wb_alias = wb_valid && (bram_addr[28:4] == wb_pa[28:4]);

//Miss-dispatch next state, one shared select tree for all three fill sites (drain a
//dirty victim or an aliased buffer first; else straight to the fill).
state_t         vic_state_nx;
assign  vic_state_nx = (victim_dirty || wb_alias) ? (wb_valid ? S_DRAIN_REQ : S_WBUF_RD)
                                                  : (bram_is_data ? S_DFILL_REQ : S_IFILL_REQ);

//cur_way next value, FLAT: (hit ? priority-encode(hit_w) : victim) folded to one
//5-input LUT per bit (the encode truth table absorbs the fallback select).
wire    [1:0]   cur_way_nx;
assign  cur_way_nx[1] = hit ? (!hit_w[0] && !hit_w[1])                : victim[1];
assign  cur_way_nx[0] = hit ? (!hit_w[0] && (hit_w[1] || !hit_w[2])) : victim[0];


//Wrap-order fill (p.110): beats leave from the MISSED word and wrap round the
//line, so the first beat is the requested word (forwarded to the CPU "in
//parallel with being loaded to the cache") and the last is its mod-4 neighbor.
wire            fill_first = (fill_word == cur_addr[3:2]);
wire            fill_last  = (fill_word == (cur_addr[3:2] - 2'd1));


///////////////////////////////////////////////////////////
//////  Controller Sequencer
////

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        ccr_ce               <= 1'b0;
        ccr_wt               <= 1'b0;
        ccr_cb               <= 1'b0;
        ccr2                 <= 32'd0;
        flush_req            <= 1'b0;

        //Block RAM powers up to 0 (valid=0), so no reset clear-walk is needed; per
        //p.104 a manual reset must NOT clear V/U anyway. S_FLUSH is for CCR.CF only.
        state         <= S_IDLE;
        after_wb      <= AW_DFILL;
        flush_idx     <= 9'd0;
        cur_is_data   <= 1'b0;
        cur_addr      <= 32'd0;
        cur_write     <= 1'b0;
        cur_pref      <= 1'b0;
        cur_size      <= 2'd0;
        cur_wdata     <= 32'd0;
        cur_wstrb     <= 4'd0;
        cur_lock      <= 1'b0;
        cur_way       <= 2'd0;
        mm_assoc_wr   <= 1'b0;
        fill_word     <= 2'd0;
        fill_base     <= 32'd0;
        victim_tag    <= 19'd0;
        wb_valid      <= 1'b0;
        wb_pa         <= 28'd0;
        wb_data[0]    <= 32'd0;
        wb_data[1]    <= 32'd0;
        wb_data[2]    <= 32'd0;
        wb_data[3]    <= 32'd0;
        wb_load_word  <= 2'd0;
        wb_drain_word <= 2'd0;
        drain_for_vic <= 1'b0;
        drain_to_wbuf <= 1'b0;
        mem_pending   <= 1'b0;
        rsp_valid_i   <= 1'b0;
        rsp_inst      <= 16'd0;
        rsp_fault_i   <= 1'b0;
        rsp_sib       <= 16'd0;
        rsp_pair_q    <= 1'b0;
        rsp_valid_d   <= 1'b0;
        rsp_rdata     <= 32'd0;
        rsp_fault_d   <= 1'b0;
        ibyp_buf_valid <= 1'b0;
        ibyp_buf_addr  <= 30'd0;
        ibyp_buf_data  <= 32'd0;
        bram_is_data  <= 1'b0;
        bram_addr     <= 32'd0;
        bram_write    <= 1'b0;
        bram_size     <= 2'd0;
        bram_pref     <= 1'b0;
        bram_lock     <= 1'b0;
        bram_wdata_q  <= 32'd0;
        bram_wb_mode_p1 <= 1'b0;
        bram_region_c <= 1'b0;
        bram_is_tagarr <= 1'b0;
        bram_is_dataarr <= 1'b0;
        bram_is_ccr   <= 1'b0;
        bram_is_ccr2  <= 1'b0;
        resq_q        <= 1'b0;
        acc_d_q       <= 1'b0;
        acc_i_q       <= 1'b0;
    end
    else begin if(cen) begin
        ////////  Registered-response retirement (consume clears)
        //Gated on the arbitration so each response clears only when the pipeline is
        //actually accepting THAT response type (rsp_fetch=1 -> IF, =0 -> MA).
        if(cons_i && rsp_valid_i) rsp_valid_i <= 1'b0;
        if(cons_d && rsp_valid_d) rsp_valid_d <= 1'b0;

        ////////  Live-response holding: a live hit response the pipe could not consume
        //this cycle retires into the registered flags (loss-free; delivered later
        //through the same arbitration). A pending same-side registered response is
        //impossible here (it blocks the accept), so no overwrite can occur.
        if(hit_rsp_i && !cons_i) begin
            rsp_inst    <= res_i_inst;
            rsp_sib     <= res_i_sib;
            rsp_pair_q  <= !bram_addr[1];
            rsp_fault_i <= 1'b0;
            rsp_valid_i <= 1'b1;
        end
        if(hit_rsp_d && !cons_d) begin
            rsp_rdata   <= res_d_rdata;
            rsp_fault_d <= 1'b0;
            rsp_valid_d <= 1'b1;
        end

        ////////  Bypass-buffer coherence: any external WRITE accept (WT store, bypass
        //store, WB drain) may change the buffered word - blanket drop. Cannot coincide
        //with the S_IBYP_WAIT load below (mem_pending holds req_valid low there).
        if(I_BUS.req_valid && I_BUS.req_ready && I_BUS.req_write)
            ibyp_buf_valid <= 1'b0;

        ////////  Access pipeline
        unique case(state)
            S_FLUSH: begin
                if(flush_idx == 9'd255) begin
                    flush_idx <= 9'd0;
                    state     <= S_IDLE;
                end
                else
                    flush_idx <= flush_idx + 9'd1;
            end

            //RUNNING. TWO dispatch sites share this arm, never both writing state:
            //  (1) the RESOLVE site (acc_*_q): the access accepted LAST edge resolves
            //      now - cacheable hit/miss decisions, write-back store commit, fills.
            //  (2) the LIVE site (acc_*_nx below): the access accepted THIS edge whose
            //      target is classify-only - bypass, mm array, CCR writes, PREF no-op -
            //      dispatches immediately (the dclk accept-edge timing). z_ok blocked
            //      this edge's accept whenever site (1) writes state, so they exclude
            //      each other. flush + background drain run when neither dispatches.
            S_IDLE: begin
                if(flush_req) begin
                    flush_req <= 1'b0;
                    flush_idx <= 9'd0;
                    state     <= S_FLUSH;
                end
                else if(acc_d_q) begin
                    //D resolve site. Classify-only cases already dispatched at their
                    //accept edge and cannot appear here in S_IDLE (state left).
                    if(bram_pref) begin
                        //PREF: allocate if cacheable & miss (vic_ld below); a hit is
                        //acked live; the non-cacheable no-op ack fired at the accept edge.
                        if(bram_cacheable_d && !hit) begin
                            if(victim_dirty || wb_alias) begin
                                if(wb_valid) begin  //alias implies wb_valid: drain first
                                    drain_for_vic <= 1'b1;
                                    drain_to_wbuf <= victim_dirty;
                                end
                                else wb_load_word <= 2'd0;
                            end
                        end
                    end
                    else if(bram_is_lmmio) begin
                        //Local register: READ served live this edge (res_d); writes
                        //committed at the accept edge (CCR/CCR2) or via the exc pulse.
                    end
                    else if(bram_cacheable_d) begin
                        if(hit) begin
                            //Write-THROUGH store hit: 2-step (write bank, then memory).
                            //Write-back store hits and loads are 1-cycle (data_we + live rsp).
                            if(bram_write && !bram_wb_mode) begin
                                state    <= S_STORE_WR;
                            end
                            //else: load / write-back store hit -> served, stay S_IDLE.
                            //A hit resolve writes no state: an early-restarted hit
                            //stream has no request-free edges, so the background
                            //drain launches HERE (still yields to a live accept).
                            else if(wb_valid && !acc_d_nx && !acc_i_nx) begin
                                drain_for_vic <= 1'b0;
                                state         <= S_DRAIN_REQ;
                            end
                        end
                        else if(bram_write && !bram_wb_mode)
                            state <= S_DBYP_REQ;            //write-through store miss: no allocate
                        else begin
                            //Read / write-allocate miss: fill (write back a dirty victim first).
                            if(victim_dirty || wb_alias) begin
                                if(wb_valid) begin  //alias implies wb_valid: drain first
                                    drain_for_vic <= 1'b1;
                                    drain_to_wbuf <= victim_dirty;
                                end
                                else wb_load_word <= 2'd0;
                            end
                        end
                    end
                end
                else if(acc_i_q) begin
                    //I resolve site (D not competing). bram_* == this fetch.
                    if(bram_cacheable_i) begin
                        if(hit) begin
                            //inst hit -> served live; a hit resolve writes no state,
                            //so the background drain may launch (see the D twin)
                            if(wb_valid && !acc_d_nx && !acc_i_nx) begin
                                drain_for_vic <= 1'b0;
                                state         <= S_DRAIN_REQ;
                            end
                        end
                        else if(i_I_SQUASH) begin
                            //Wrong-path instruction miss: the pipeline is discarding this fetch,
                            //so do NOT allocate; ack with a dummy response and free at once.
                            rsp_inst    <= 16'd0;
                            rsp_pair_q  <= 1'b0;    //dummy squash ack carries no pair
                            rsp_fault_i <= 1'b0;
                            rsp_valid_i <= 1'b1;
                        end
                        else begin
                            if(victim_dirty || wb_alias) begin
                                if(wb_valid) begin  //alias implies wb_valid: drain first
                                    drain_for_vic <= 1'b1;
                                    drain_to_wbuf <= victim_dirty;
                                end
                                else wb_load_word <= 2'd0;
                            end
                        end
                    end
                end
                else if(!acc_d_nx && !acc_i_nx && wb_valid) begin
                    //Background drain of the write-back buffer. Yields to requests: a
                    //LIVE accept this edge keeps the cache free for that access's resolve.
                    drain_for_vic <= 1'b0;
                    state         <= S_DRAIN_REQ;
                end

                //LIVE dispatch site: classify-only targets of the access accepted THIS
                //edge, exactly the dclk accept-edge dispatch tree with the resolve
                //branches removed. Runs after (never concurrent with) the resolve site's
                //state writes - z_ok guarantees exclusivity - and beats the drain arm
                //(which tests !acc_*_nx). Priority mirrors the dclk tree:
                //pref > tagarr > dataarr > lmmio > (cacheable: resolve site) > bypass.
                if(acc_d_nx) begin
                    if(live_pref) begin
                        if(!live_cacheable_d) begin
                            //Non-cacheable / locked PREF: no-op ack (p.111, 5.3.3).
                            rsp_rdata   <= 32'd0;
                            rsp_fault_d <= 1'b0;
                            rsp_valid_d <= 1'b1;
                        end
                    end
                    else if(live_is_tagarr)  state <= S_MMTAG;
                    else if(live_is_dataarr) state <= S_MMDATA;
                    else if(live_is_lmmio) begin
                        //CCR/CCR2 write commits here (fire-and-forget, stay S_IDLE);
                        //exc-group writes pulse o_LMMIO_EXC_WE this same edge.
                        if(PIPE_L_BUS.req_write && live_is_ccr) begin
                            ccr_ce <= PIPE_L_BUS.req_wdata[0];
                            ccr_wt <= PIPE_L_BUS.req_wdata[1];
                            ccr_cb <= PIPE_L_BUS.req_wdata[2];
                            if(PIPE_L_BUS.req_wdata[3]) flush_req <= 1'b1;    //CCR.CF
                        end
                        else if(PIPE_L_BUS.req_write && live_is_ccr2)
                            ccr2 <= PIPE_L_BUS.req_wdata & 32'h0000_0303;
                    end
                    else if(!live_cacheable_d) begin
                        state <= S_DBYP_REQ;               //non-cacheable data access
                    end
                end
                else if(acc_i_nx) begin
                    if(!live_cacheable_i) begin
                        if(ibyp_buf_hit) begin
                            //Sibling halfword of the last bypass read: serve from the
                            //buffer as a registered response, no FSM run, no bus run.
                            rsp_inst    <= pick_inst(ibyp_buf_data, PIPE_L_BUS.req_addr[1], BIG_ENDIAN);
                            rsp_sib     <= pick_inst(ibyp_buf_data, !PIPE_L_BUS.req_addr[1], BIG_ENDIAN);
                            rsp_pair_q  <= !PIPE_L_BUS.req_addr[1];
                            rsp_fault_i <= 1'b0;
                            rsp_valid_i <= 1'b1;
                        end
                        else state <= S_IBYP_REQ;   //non-cacheable fetch
                    end
                end
            end

            //Write-THROUGH store hit only. The bank was written at the dispatch edge
            //(data_we, S_IDLE), then memory via S_STORE_REQ.
            S_STORE_WR: begin
                mem_pending <= 1'b0;
                state       <= S_STORE_REQ;
            end

            S_ALLOC_WR: begin
                state <= S_IDLE;                //write-allocate store: notify, no pipe ack
            end

            ////////  Write-back buffer: copy the victim line in
            S_WBUF_RD: state <= S_WBUF_CAP;

            S_WBUF_CAP: begin
                wb_data[wb_load_word] <= data_rdata[cur_way];
                if(wb_load_word == 2'd3) begin
                    wb_valid      <= 1'b1;
                    wb_pa         <= {3'b000, victim_tag[18:2], cur_addr[11:4]};
                    wb_drain_word <= 2'd0;
                    unique case(after_wb)
                        AW_IFILL: state <= S_IFILL_REQ;
                        AW_MMTAG: state <= S_MMTAG_WR;
                        default:  state <= S_DFILL_REQ;
                    endcase
                end
                else begin
                    wb_load_word <= wb_load_word + 2'd1;
                    state        <= S_WBUF_RD;
                end
            end

            ////////  Write-back buffer: drain out to memory
            S_DRAIN_REQ: begin
                if(I_BUS.req_valid && I_BUS.req_ready) begin
                    mem_pending <= 1'b1;
                    state       <= S_DRAIN_WAIT;
                end
            end

            S_DRAIN_WAIT: begin
                if(I_BUS.rsp_valid && I_BUS.rsp_ready) begin
                    mem_pending <= 1'b0;
                    if(wb_drain_word == 2'd3) begin
                        wb_valid      <= 1'b0;
                        wb_drain_word <= 2'd0;
                        if(drain_for_vic) begin
                            drain_for_vic <= 1'b0;
                            if(drain_to_wbuf) begin
                                wb_load_word <= 2'd0;
                                state        <= S_WBUF_RD; //buffer free: load the new victim
                            end
                            else    //alias-only drain: memory is fresh, go fill the miss
                                state <= (after_wb == AW_IFILL) ? S_IFILL_REQ : S_DFILL_REQ;
                        end
                        else
                            state <= S_IDLE;
                    end
                    else begin
                        wb_drain_word <= wb_drain_word + 2'd1;
                        //A drain runs to COMPLETION once its head beat is on the bus:
                        //the BSC burst engine owns the line until beat 3 (fe_b_cont),
                        //so a foreign request interleaved mid-line deadlocks the pair
                        //(cache waits for the accept, engine waits for the beats -
                        //found by the fetch-pair bring-up). Background-ness only
                        //chooses WHEN the burst starts: a request-free S_IDLE edge.
                        state <= S_DRAIN_REQ;
                    end
                end
            end

            S_IFILL_REQ: begin
                if(I_BUS.req_valid && I_BUS.req_ready) begin
                    mem_pending <= 1'b1;
                    state       <= S_IFILL_WAIT;
                end
            end

            //Instruction fill: 4 longwords wrapping from the missed word. The FIRST
            //beat IS the addressed longword: its opcode (and pair sibling) go to the
            //pipe at that beat, in parallel with the array write (p.110) - the fill
            //then completes in the background while the pipe runs on.
            S_IFILL_WAIT: begin
                if(I_BUS.rsp_valid && I_BUS.rsp_ready) begin
                    mem_pending <= 1'b0;
                    if(I_BUS.rsp_fault) begin
                        //First beat: the fetch itself faults. A LATER beat's fault only
                        //kills the line validation (tag control block) - the pipe
                        //already consumed correct memory data at the first beat.
                        if(fill_first) begin
                            rsp_inst    <= 16'd0;
                            rsp_pair_q  <= 1'b0;    //a faulted response never pairs
                            rsp_fault_i <= 1'b1;
                            rsp_valid_i <= 1'b1;
                        end
                        state <= S_IDLE;
                    end
                    else begin
                        if(fill_first) begin
                            rsp_inst    <= pick_inst(I_BUS.rsp_rdata, cur_addr[1], BIG_ENDIAN);
                            rsp_sib     <= pick_inst(I_BUS.rsp_rdata, !cur_addr[1], BIG_ENDIAN);
                            rsp_pair_q  <= !cur_addr[1];
                            rsp_fault_i <= 1'b0;
                            rsp_valid_i <= 1'b1;
                        end
                        if(fill_last) state <= S_IDLE;
                        else begin
                            fill_word <= fill_word + 2'd1;
                            state     <= S_IFILL_REQ;
                        end
                    end
                end
            end

            S_DFILL_REQ: begin
                if(I_BUS.req_valid && I_BUS.req_ready) begin
                    mem_pending <= 1'b1;
                    state       <= S_DFILL_WAIT;
                end
            end

            //Data fill: 4 longwords wrapping from the missed word. A read miss (and
            //a PREF allocate) answers the pipe at the FIRST beat - the requested word
            //arrives first and transfers "in parallel with being loaded" (p.110);
            //remaining beats fill in the background. A write-allocate store is
            //notify-only (S_ALLOC_WR merges after the fill, no pipe ack needed).
            S_DFILL_WAIT: begin
                if(I_BUS.rsp_valid && I_BUS.rsp_ready) begin
                    mem_pending <= 1'b0;
                    if(I_BUS.rsp_fault) begin
                        //First beat: the load itself faults (a prefetch abandons
                        //silently). A LATER beat's fault only kills the line
                        //validation - the pipe already got correct memory data.
                        //The victim way is invalidated either way (tag block).
                        if(fill_first) begin
                            rsp_rdata   <= 32'd0;
                            rsp_fault_d <= cur_pref ? 1'b0 : 1'b1;
                            rsp_valid_d <= 1'b1;
                        end
                        state <= S_IDLE;
                    end
                    else begin
                        if(fill_first && !cur_write) begin
                            rsp_rdata   <= I_BUS.rsp_rdata;     //the missed word itself
                            rsp_fault_d <= 1'b0;
                            rsp_valid_d <= 1'b1;
                        end
                        if(fill_last)
                            state <= cur_write ? S_ALLOC_WR : S_IDLE;
                        else begin
                            fill_word <= fill_word + 2'd1;
                            state     <= S_DFILL_REQ;
                        end
                    end
                end
            end

            S_STORE_REQ: begin
                if(I_BUS.req_valid && I_BUS.req_ready) begin
                    mem_pending <= 1'b1;
                    state       <= S_STORE_WAIT;
                end
            end

            S_STORE_WAIT: begin
                if(I_BUS.rsp_valid && I_BUS.rsp_ready) begin
                    mem_pending <= 1'b0;
                    rsp_valid_d <= cur_lock;     //plain WT store notify-only; locked RMW write acks
                    state       <= S_IDLE;
                end
            end

            S_IBYP_REQ: begin
                if(I_BUS.req_valid && I_BUS.req_ready) begin
                    mem_pending <= 1'b1;
                    state       <= S_IBYP_WAIT;
                end
            end

            //Non-cacheable fetch: one longword read, addressed halfword picked out;
            //the whole word is kept for the sibling halfword (reuse buffer).
            S_IBYP_WAIT: begin
                if(I_BUS.rsp_valid && I_BUS.rsp_ready) begin
                    mem_pending <= 1'b0;
                    rsp_inst    <= pick_inst(I_BUS.rsp_rdata, cur_addr[1], BIG_ENDIAN);
                    rsp_sib     <= pick_inst(I_BUS.rsp_rdata, !cur_addr[1], BIG_ENDIAN);
                    rsp_pair_q  <= !I_BUS.rsp_fault && !cur_addr[1];
                    rsp_fault_i <= I_BUS.rsp_fault;
                    rsp_valid_i <= 1'b1;
                    state       <= S_IDLE;
                    ibyp_buf_valid <= !I_BUS.rsp_fault;     //a faulted word is not kept
                    ibyp_buf_addr  <= cur_addr[31:2];
                    ibyp_buf_data  <= I_BUS.rsp_rdata;
                end
            end

            S_DBYP_REQ: begin
                if(I_BUS.req_valid && I_BUS.req_ready) begin
                    mem_pending <= 1'b1;
                    state       <= S_DBYP_WAIT;
                end
            end

            S_DBYP_WAIT: begin
                if(I_BUS.rsp_valid && I_BUS.rsp_ready) begin
                    mem_pending <= 1'b0;
                    rsp_rdata   <= I_BUS.rsp_rdata;
                    rsp_fault_d <= I_BUS.rsp_fault;
                    rsp_valid_d <= !cur_write || cur_lock;//load + locked RMW write ack; plain store notify-only
                    state       <= S_IDLE;
                end
            end

            ////////  Memory-mapped tag array (0xF0xx_xxxx)
            S_MMTAG: begin
                if(!cur_write) begin
                    //Read: tag, LRU, U, V of the addressed way (no associative op).
                    rsp_rdata   <= {3'b000, tag_rdata[mm_way][18:0], lru_rdata,
                                    2'b00, tag_rdata[mm_way][19], tag_rdata[mm_way][20]};
                    rsp_fault_d <= 1'b0;
                    rsp_valid_d <= 1'b1;
                    state       <= S_IDLE;
                end
                else if(cur_addr[3] && !mm_match) begin
                    state <= S_IDLE;            //assoc write hit no way: no-op, notify (no ack)
                end
                else begin
                    //Write (assoc selected way, or non-assoc addressed way).
                    cur_way     <= mm_sel_way;
                    mm_assoc_wr <= cur_addr[3];
                    after_wb    <= AW_MMTAG;
                    //Write back first if displacing/invalidating a dirty line:
                    //non-assoc needs both U and V set; assoc needs U set on the hit way.
                    if((!cur_addr[3] && tag_rdata[mm_sel_way][20] && tag_rdata[mm_sel_way][19]) ||
                       ( cur_addr[3] && tag_rdata[mm_sel_way][19])) begin
                        victim_tag <= tag_rdata[mm_sel_way][18:0];
                        if(wb_valid) begin
                            drain_for_vic <= 1'b1;
                            drain_to_wbuf <= 1'b1;      //displaced entry is dirty: buffer it
                            state         <= S_DRAIN_REQ;
                        end
                        else begin
                            wb_load_word <= 2'd0;
                            state        <= S_WBUF_RD;
                        end
                    end
                    else
                        state <= S_MMTAG_WR;
                end
            end

            S_MMTAG_WR: begin
                //Tag (and LRU for non-assoc) written combinationally this cycle. MM tag write is a
                //notify, like any store - it raises no ack.
                state <= S_IDLE;
            end

            ////////  Memory-mapped data array (0xF1xx_xxxx)
            S_MMDATA: begin
                rsp_rdata   <= data_rdata[cur_addr[13:12]];     //read result (ignored on write)
                rsp_fault_d <= 1'b0;
                rsp_valid_d <= !cur_write;                      //read acks; write commits, notify-only
                state       <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase

        ////////  SPECULATIVE dispatch capture. These five hold miss-flow context that is
        //read ONLY outside S_IDLE, and every FSM exit passes through the dispatch edge -
        //so they load EVERY S_IDLE edge unconditionally: dead loads while running,
        //the dispatched access's exact values on the exit edge, held (CE=0) during the
        //excursion. The MM flow's own victim_tag/after_wb writes happen in non-IDLE
        //states, so never overridden. Only the state transition keeps the REAL miss
        //qualifier (vic_ld).
        if(state == S_IDLE) begin
            victim_tag  <= victim_tag_w;                    //one-hot AND-OR, no index chain
            fill_base   <= {bram_addr[31:4], 4'b0000};
            fill_word   <= bram_addr[3:2];                  //fills WRAP from the missed word (p.110)
            mem_pending <= 1'b0;
            after_wb    <= bram_is_data ? AW_DFILL : AW_IFILL;  //D flag alone selects the site
        end
        if(vic_ld) state <= vic_state_nx;                   //shared drain/wbuf/fill select

        ////////  Accepted-access backup - SPECULATIVE capture (cost-free recovery).
        //The descriptor tracks the LIVE request every idle edge EXCEPT when the
        //previous access's resolve is dispatching (z_ok=0 holds the dispatched
        //access's values for the FSM). An accept edge always has CE=1 (the accept
        //itself required idle && z_ok), a resolve-dispatch edge always has CE=0
        //(its d/i_clean is 0 by definition), and junk captured on request-free
        //idle edges is never consumed (no dispatch without an accept). This keeps
        //the deep accept tail (req_valid/issue cone) off this 100+ FF cluster's
        //enable - only the resolve (z_ok) gates it. The D-only fields capture
        //fetch-cycle leftovers harmlessly (consumed only after a D dispatch).
        if((state == S_IDLE) && z_ok) begin
            cur_is_data <= grant_d;
            cur_addr    <= PIPE_L_BUS.req_addr;
            cur_write   <= grant_d && PIPE_L_BUS.req_write;
            cur_pref    <= live_pref;
            cur_lock    <= grant_d && PIPE_L_BUS.req_lock;
            cur_size    <= PIPE_L_BUS.req_size;
            cur_wdata   <= PIPE_L_BUS.req_wdata;
            cur_wstrb   <= live_wstrb;
        end
        //cur_way_nx is the one-LUT-per-bit form of hit ? hit_way : victim.
        if((acc_d_q || acc_i_q) && state == S_IDLE) begin
            cur_way     <= cur_way_nx;
        end

        ////////  Request-edge captures (the former CEN_n block, now the same edge).
        //The cache ALWAYS captures something: D when present, else the IF fetch_pc
        //(driven even when I req_valid is low) - the unconditional read-ahead that
        //keeps hits back-to-back. The RAMs capture the same live index this edge.
        bram_is_data <= grant_d;
        bram_addr    <= PIPE_L_BUS.req_addr;                //SINGLE source (the shared AGU)
        bram_write   <= grant_d && PIPE_L_BUS.req_write;
        bram_size    <= PIPE_L_BUS.req_size;
        bram_pref    <= grant_d && i_PIPE_D_PREF;
        bram_lock    <= grant_d && PIPE_L_BUS.req_lock;
        bram_wdata_q <= PIPE_L_BUS.req_wdata;               //store data held for the commit edge

        //Running-access classification latches - region decode off the LIVE address, in
        //parallel with the bram_addr capture.
        bram_wb_mode_p1 <= PIPE_L_BUS.req_addr[31:29] == 3'b100;
        bram_region_c   <= cacheable(PIPE_L_BUS.req_addr);
        bram_is_tagarr  <= PIPE_L_BUS.req_addr[31:24] == 8'hF0;
        bram_is_dataarr <= PIPE_L_BUS.req_addr[31:24] == 8'hF1;
        bram_is_ccr     <= PIPE_L_BUS.req_addr == ADDR_CCR;
        bram_is_ccr2    <= PIPE_L_BUS.req_addr == ADDR_CCR2_P4;

        //Pair freshness + accept captures for the coming resolve edge.
        resq_q  <= (state == S_IDLE);
        acc_d_q <= acc_d_nx;
        acc_i_q <= acc_i_nx;
    end end
end

endmodule

`default_nettype none
