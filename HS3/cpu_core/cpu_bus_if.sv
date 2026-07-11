`default_nettype wire

/*
    CPU memory bus interfaces.

    LBus is the unified pipeline<->cache channel (the SH7709S "L bus" of the
    block diagram). Once the time-shared AGU drives one address per cycle - an
    IF instruction fetch OR an MA data access - the separate IBus/DBus collapse
    onto this single bus, selected by req_fetch. LBus carries three sub-buses:

        - address + control : master -> slave  (the shared AGU address)
        - write data  (M2S) : master -> slave  (store payload; inert on a fetch)
        - read data   (S2M) : slave -> master  (load word / fetched line word)

    SINGLE-CLOCK model: every wire below is a plain scalar evaluated inside one
    architectural cycle. A hit response is COMBINATIONAL off the cache resolve
    (RAM q read the previous edge) and is consumed the same cycle; miss/bypass
    completions arrive on the registered rsp_* flags. The former dual-CEN 4-rail
    late-select machinery (rl_*, late_*) is deleted - the rails were Shannon
    covers for half-cycle-late selects that no longer exist.

    IBus_1 is the SH7709S "I bus 1" (Fig 1.1, hw manual p.6): the cache is the
    single master; the splitter routes to the BRIDGE (I bus 2 registers) or the
    BSC leg (external memory). Plain ready/valid; plain stores are posted.
*/

interface LBus;
/* REQUEST + ADDRESS - master -> slave */
logic           req_valid;      //request presented this cycle
logic           req_ready;      //slave accepts the request (accept = valid && ready edge)
logic   [31:0]  req_addr;       //shared AGU address: fetch PC (IF) or data EA (MA)
logic           req_fetch;      //role select - 1: instruction fetch (IF); 0: data access (MA)
logic           req_write;      //1: store; a fetch/load is a read (MA only)
logic   [1:0]   req_size;       //access size byte/word/long (data); WORD on a fetch
logic           req_lock;       //atomic RMW lock, holds the line (MA only)

/* WRITE DATA - master -> slave (M2S) */
logic   [31:0]  req_wdata;      //store data (don't-care on a read/fetch)
logic   [3:0]   req_wstrb;      //store byte strobes (zero on a read/fetch)

/* RESPONSE - slave -> master */
logic           rsp_valid;      //response presented this cycle
logic           rsp_ready;      //master accepts the response
logic           rsp_fetch;      //echoes req_fetch so the master routes S2M data to IF vs MA
/* Faults are SPLIT per role like the data: a HIT never faults, so each line is a pure
   registered-flag product (valid && fault regs) with no live hit select in its cone. */
logic           rsp_dfault;     //data access fault: misalign / data abort (registered response only)
logic           rsp_ifault;     //fetch fault (registered response only)

/* READ DATA - slave -> master (S2M). SPLIT per role: rsp_rdata carries ONLY the
   data-side load word and rsp_inst ONLY the fetched opcode. One shared field muxed
   by rsp_fetch put the (never-selected) I word inside the D load-align cone and
   vice versa - physically real but architecturally false paths. */
logic   [31:0]  rsp_rdata;      //load word (data response only)
logic   [15:0]  rsp_inst;       //fetched opcode, addr[1]-picked half (fetch response only)
/* FETCH PAIR: every fetch reads a full longword (like the real chip's 32-bit IF); on an
   EVEN fetch the sibling opcode at PC+2 rides along so the master's pair slot can issue
   it without a second request - that port slot is freed for MA or left idle (drain).
   rsp_pair qualifies the sibling: even fetch AND fault-free. Fetch responses only. */
logic   [15:0]  rsp_inst_sib;   //sibling opcode (the fetched longword's other half)
logic           rsp_pair;       //rsp_inst_sib is usable this response
/* DUAL-ALIGNER feed: rsp_rdata above is the 2:1-muxed word (raw-word consumers keep it);
   the pipe's load aligner instead takes BOTH sources and re-applies the hit select AFTER
   alignment, so the late cache word (and the late hit select) cross the aligner-depth
   levels in parallel with the early registered miss word, not in series behind the mux. */
logic   [31:0]  rsp_rdata_hit;  //combinational cache hit word (RAM q resolve, late)
logic   [31:0]  rsp_rdata_miss; //registered miss/bypass/MMIO response word (early)
logic           rsp_hit_d;      //post-alignment select (1 = live hit word)

modport master (
    output req_valid, req_addr, req_fetch, req_write, req_size, req_lock,
           req_wdata, req_wstrb, rsp_ready,
    input  req_ready, rsp_valid, rsp_fetch, rsp_dfault, rsp_ifault, rsp_rdata, rsp_inst,
           rsp_inst_sib, rsp_pair, rsp_rdata_hit, rsp_rdata_miss, rsp_hit_d
);

modport slave (
    input  req_valid, req_addr, req_fetch, req_write, req_size, req_lock,
           req_wdata, req_wstrb, rsp_ready,
    output req_ready, rsp_valid, rsp_fetch, rsp_dfault, rsp_ifault, rsp_rdata, rsp_inst,
           rsp_inst_sib, rsp_pair, rsp_rdata_hit, rsp_rdata_miss, rsp_hit_d
);

/* Input-only snoop for register owners on the L bus (exc_handler group): the
   live request address/wdata feed their own decode; no drive rights. */
modport monitor (
    input  req_valid, req_addr, req_fetch, req_write, req_wdata
);
endinterface

/* SH7709S I bus 1 (Fig 1.1, p.6). One outstanding transaction (the cache FSM
   serializes on mem_pending); a store is posted - rsp only frees the master.
   The DMAC sideband (req_dack fields, req_saddr) rides along like req_lock: the
   DMAC master tags its accesses, only the BSC consumes them (DACK framed on
   the CSn window, p.345; single-address cycles fig 11.10); the CPU/cache
   master ties them off. */
interface IBus_1;
logic           req_valid;
logic           req_ready;
logic           req_write;
logic   [1:0]   req_size;
logic           req_burst;      //beat of a 16-byte line transfer (cache fill/drain, 4 longwords)
logic   [31:0]  req_addr;
logic   [31:0]  req_wdata;
logic   [3:0]   req_wstrb;
logic           req_lock;
/* DMAC sideband - AM (which dual-mode cycle gets DACK, p.338) is resolved by
   the DMAC, so the BSC only ever sees "frame DACK on THIS bus cycle"; the BSC
   returns an active-high window strobe and the DMAC applies the AL polarity */
logic           req_dack;       //assert DACKn over this cycle's CSn window
logic           req_dack_ch;    //DACK pin select: 0 = DACK0, 1 = DACK1
logic           req_saddr;      //single-address mode cycle ("saddr": write = external
                                //device drives D31-0 while WE runs, fig 11.10a)
logic           rsp_valid;
logic           rsp_ready;
logic   [31:0]  rsp_rdata;
logic           rsp_fault;

modport master (
    output req_valid, req_write, req_size, req_burst, req_addr, req_wdata,
            req_wstrb, req_lock, req_dack, req_dack_ch, req_saddr,
            rsp_ready,
    input  req_ready, rsp_valid, rsp_rdata, rsp_fault
);

modport slave (
    input  req_valid, req_write, req_size, req_burst, req_addr, req_wdata,
            req_wstrb, req_lock, req_dack, req_dack_ch, req_saddr,
            rsp_ready,
    output req_ready, rsp_valid, rsp_rdata, rsp_fault
);
endinterface

`default_nettype none
