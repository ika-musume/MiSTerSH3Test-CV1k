`default_nettype wire

/*
    SH7709S five-stage integer instruction pipeline.

    The module contains execution pipeline, GPR, PR, and MAC state. Control
    registers are read from ctrl_reg and updated through retired writeback.
    Instruction and data ports are logical ready/valid memory interfaces.

    Abbreviations:
    IF: instruction fetch. ID: instruction decode. EX: execute.
    MA: memory access. WB: writeback. GPR: general-purpose register.
    SR: status register. GBR: global base register. PR: procedure register.
    SSR/SPC: saved status/program counter. RTE: return from exception.
    LDTLB: load translation lookaside buffer. CEN: clock enable.
    REQ/RSP: request/response. WE: write enable. DST: destination.
    WDATA/WSTRB: write data/write-byte strobe. IMM: immediate operand.
*/

import int_pipe_pkg::*;

module int_pipe #(
    parameter [31:0] RESET_PC   = 32'hA000_0000, //reset PC; manual table 2.1, p.22
    parameter        BIG_ENDIAN = 1'b1          //byte order selection; manual section 2.2.2, p.25
) (
    /* CLOCK AND RESET */
    input   wire            i_RST_n,
    input   wire            i_CLK,      //single architectural clock
    input   wire            i_CEN,      //architectural clock enable

    /* EXTERNAL PIPELINE REDIRECT */
    input   wire            i_REDIRECT_VALID,
    input   wire    [31:0]  i_REDIRECT_PC,

    /* CONTROL REGISTER READS */
    input   wire    [31:0]  i_SR,
    input   wire    [31:0]  i_GBR,
    input   wire    [31:0]  i_SSR,
    input   wire    [31:0]  i_SPC,
    input   wire    [31:0]  i_VBR,

    /* MEMORY ACCESS - unified L bus (SH7709S L bus). The time-shared AGU drives ONE address
       per cycle; L_BUS.req_fetch (= !data-present, D-priority) selects IF fetch vs MA data. */
    LBus.master             L_BUS,
    output  wire            o_D_PREF,       //sideband: current data access is a PREF allocate
    output  wire            o_I_SQUASH,     //sideband: outstanding fetch is wrong-path (abort its fill)

    /* PRECISE EXCEPTION AND STATE EVENTS */
    output  logic           o_EXC_VALID,
    output  logic   [2:0]   o_EXC_CAUSE,
    output  logic   [31:0]  o_EXC_PC,
    output  logic           o_EXC_IN_DELAY_SLOT,
    // Bare-metal handler maps these to read/write address-error codes.
    output  logic           o_EXC_ACCESS_WRITE,
    output  logic   [31:0]  o_EXC_ACCESS_ADDR,
    output  logic           o_TRAPA_VALID,
    output  logic   [7:0]   o_TRAPA_IMM,
    output  logic           o_RTE_VALID,
    output  logic           o_SLEEP_VALID,
    output  logic           o_LDTLB_VALID,

    /* RETIRE TRACE */
    output  logic           o_RETIRE_VALID,
    output  logic   [31:0]  o_RETIRE_PC,
    output  logic   [15:0]  o_RETIRE_INST,
    //Interrupt sidebands (exc_handler): the retiree is a DELAYED BRANCH whose slot has
    //not retired (defer acceptance, 4.5.3), and the restart PC = the OLDEST instruction
    //the redirect will discard (mawb..fetch_pc priority; retire+2 on a plain stream,
    //the branch-target stream head after a taken branch).
    output  wire            o_INT_BOUNDARY,     //this edge is a LEGAL interrupt-acceptance
                                                //boundary: an instruction just retired, no
                                                //delayed pair is open, no accepted D access /
                                                //RMW-MAC sequence is in flight (4.5.3)
    output  logic   [31:0]  o_INT_NEXT_PC,
    output  logic           o_RETIRE_GPR_WE,
    output  logic   [4:0]   o_RETIRE_GPR,
    output  logic   [31:0]  o_RETIRE_GPR_DATA,

    /* CONTROL REGISTER WRITEBACK */
    output  logic           o_CTRL_WE,
    output  logic   [2:0]   o_CTRL_DST,
    output  logic   [31:0]  o_CTRL_DATA,
    output  logic           o_SR_T_WE,
    output  logic           o_SR_T,
    output  logic           o_SR_S_WE,
    output  logic           o_SR_S,
    output  logic   [1:0]   o_SR_MQ_WE,
    output  logic   [1:0]   o_SR_MQ,

    /* ARCHITECTURAL STATE OBSERVATION */
    output  logic   [31:0]  o_FETCH_PC,
    output  logic   [31:0]  o_MACH,
    output  logic   [31:0]  o_MACL,
    output  logic   [31:0]  o_PR
);

// Internal exception classifications; exc_handler maps them to vector events.
localparam logic [2:0] EXC_NONE       = 3'd0; //no exception
localparam logic [2:0] EXC_ILLEGAL    = 3'd1; //unknown instruction encoding
localparam logic [2:0] EXC_PRIVILEGE  = 3'd2; //privileged instruction in user mode
localparam logic [2:0] EXC_IFETCH     = 3'd3; //instruction-port fault
localparam logic [2:0] EXC_DATA       = 3'd4; //data-port fault
localparam logic [2:0] EXC_ADDRESS    = 3'd5; //misaligned word or longword

(* direct_enable *) wire cen = i_CEN; //architectural edge enable; binds to DFF CE

///////////////////////////////////////////////////////////
//////  Architectural State
////

/*
    R0-R7 have BANK0 and BANK1 copies; R8-R15 are shared.
    User mode always selects BANK0. Privileged mode uses SR.RB.
    See manual sections 2.1.1-2.1.2, pp.19-22.

    One 32-bit true-dual-port BRAM stores the complete register file.
    CEN_p writes up to two WB results. CEN_n reads Rn and Rm synchronously.
    Banked R0 mirrors provide the third source required by indexed stores.
*/

logic   [31:0]  gpr_r0_bank0, gpr_r0_bank1; //third-read mirrors; BRAM remains authoritative

logic   [31:0]  mach; //multiply-and-accumulate high register; driven by u_mac_dsp
logic   [31:0]  macl; //multiply-and-accumulate low register; driven by u_mac_dsp
logic   [31:0]  pr;   //procedure register stores subroutine return address

assign  o_MACH = mach;
assign  o_MACL = macl;
assign  o_PR   = pr;

///////////////////////////////////////////////////////////
//////  GPR File (single-clock 2R2W)
////

/*
    One edge carries up to two WB writes AND both ID read-address captures, so
    the file is a 2R2W LVT bank pair (gpr_2r2w in int_pipe_mem.sv). The read
    addresses are captured at the SAME edge IF/ID loads, so their D-cone is the
    NEXT-ifid function (nx_* below): the arriving fetch response when ifid loads
    this edge, the held ifid otherwise. A same-edge write is invisible to that
    read (old data); the one-cycle WB shadow lanes (wb0z/wb1z) forward it.
*/

logic           gpr_active_bank1;
logic           reads_inactive_bank;             //LDC/STC Rm_BANK selects the inactive R0-R7 half
logic   [4:0]   gpr_read_address_a, gpr_read_address_b;
logic   [31:0]  gpr_read_data_a, gpr_read_data_b;
logic           gpr_wb0_we, gpr_wb1_we;          //WB writes consumed by this GPR edge
logic   [4:0]   gpr_wb0_dst, gpr_wb1_dst;
logic   [31:0]  gpr_wb0_data, gpr_wb1_data;
logic   [4:0]   gpr_read0_address, gpr_read1_address;

//One-cycle WB shadow lanes: the lanes that wrote the file at the LAST edge. A read
//address captured at that edge returned old data (same-edge write, and mawb has
//already advanced), so the operand early legs forward from these instead.
logic           wb0z_we, wb1z_we;
logic   [4:0]   wb0z_dst, wb1z_dst;
logic   [31:0]  wb0z_data, wb1z_data;

assign  gpr_active_bank1  = i_SR[30] & i_SR[29]; //privileged mode and SR.RB select BANK1

//Registered LOCAL mirror of gpr_active_bank1 (r = running, like r_t): the ctrl_reg o_SR
//FF sat cross-module on every dec_*_id leg (fit4 o_SR[29] -> idex.src_b class, -2.36).
//Captures bank1_nx, so it equals i_SR[30]&[29] EXACTLY at every edge a LDC ...,SR commit
//can produce (the sr_wr_wb arm); RTE/exception-entry SR writes redirect + flush IF/ID,
//and r_bank1 re-converges off the live SR before any refetched packet reaches ID.
//Sim-asserted == gpr_active_bank1 on every live packet. Sequential block rides below
//the bank1_nx assign (declaration-before-use, Quartus 17).
logic           r_bank1;

//NEXT-cycle read-address cone (nx_read0/1): what IF/ID will hold during the read-data
//cycle, addressed with the NEXT-cycle bank bits. DECLARED here, ASSIGNED below the
//handshake tail (its inputs - ifid/mawb/ifid_ld - are declared later in the file, and
//Quartus requires declaration before use for struct member selects).
logic   [4:0]   nx_read0, nx_read1;

gpr_2r2w u_gpr_bram (
    .i_CLK                  (i_CLK                      ),
    .i_EN                   (cen                        ),
    .i_WE0                  (gpr_wb0_we                 ),
    .i_WADDR0               (gpr_wb0_dst                ),
    .i_WDATA0               (gpr_wb0_data               ),
    .i_WE1                  (gpr_wb1_we                 ),
    .i_WADDR1               (gpr_wb1_dst                ),
    .i_WDATA1               (gpr_wb1_data               ),
    .i_RADDR0               (nx_read0                   ),
    .i_RADDR1               (nx_read1                   ),
    .o_RDATA0               (gpr_read_data_a            ),
    .o_RDATA1               (gpr_read_data_b            )
);

//WB shadow lane capture (mirrors the write lanes exactly, one cycle behind).
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        wb0z_we   <= 1'b0;
        wb1z_we   <= 1'b0;
        wb0z_dst  <= 5'd0;
        wb1z_dst  <= 5'd0;
        //wb0z_data/wb1z_data: R7 strip - shadow words, consumed only under wb*z_we
    end
    else begin if(cen) begin
        wb0z_we   <= gpr_wb0_we;
        wb1z_we   <= gpr_wb1_we;
        wb0z_dst  <= gpr_wb0_dst;
        wb1z_dst  <= gpr_wb1_dst;
        wb0z_data <= gpr_wb0_data;
        wb1z_data <= gpr_wb1_data;
    end end
end


///////////////////////////////////////////////////////////
//////  Pipeline Registers
////

/*
    Stage packets advance only when the downstream stage can accept them.
    A valid bit represents occupancy; an invalid packet is a pipeline bubble.
    IF, ID, EX, MA, and WB follow the five-stage model in inst_pipe.md.
*/

ifid_t  ifid;
idex_t  idex, id_decode;
exma_t  exma, ex_result;
mawb_t  mawb;             //registered WB packet; commits after one full WB cycle
mawb_t  ma_result;        //combinational MA-completion packet for MA/WB and forwarding

logic           fetch_pending;    //one instruction request awaits completion
logic           fetch_drop;       //discard stale response after redirect
logic           fault_hold;       //wait for external exception redirect
logic   [31:0]  fetch_pc;          //address used by the next fetch request
logic   [31:0]  fetch_pending_pc;  //PC paired with the outstanding response

//FETCH-PAIR slot: every fetch response carries the full longword (LBus rsp_inst_sib);
//an even fetch's sibling (PC+2) is held here as ONE extra in-order fetch-queue entry
//and inserted into IF/ID without a bus request - the freed port slot goes to MA or
//idles (drain). Next-sequential ONLY, killed on every redirect: architecturally the
//same class as IF/ID itself, so it needs no store/eviction coherency (SH prefetch law).
logic           pair_ready;       //slot holds the next-in-order opcode
logic   [31:0]  pair_pc;
logic   [15:0]  pair_inst;
pd_route_t      pair_pd;          //sibling predecode, computed once at capture
logic           data_req_sent;    //MA request accepted; response still pending
logic           data_req_sent_agu;    //AGU-only preserved duplicate (u_ma_seq o_req_sent_agu)
logic           ma_second_access; //MEM_MAC second read or MEM_RMW write phase
logic           ma_second_access_agu; //AGU-only preserved duplicate (u_ma_seq o_second_agu)
logic           mac_started;      //current multiply command has entered u_mac_dsp
logic   [31:0]  ma_first_value;   //first MAC operand or original RMW byte
logic           mac_armed;        //operands captured in dsp_a/dsp_b; multiply launches next cycle
logic   [31:0]  dsp_a, dsp_b;     //pipelined DSP operands; clean register read into the multiplier
logic           mac_dsp_done;     //u_mac_dsp holds a completed result for WB
//AGU base shadow register: feeds the AGU base as a SINGLE registered net (no input mux on the
//5 ns path). Normally tracks idex.src_a_value; while a MAC/RMW second access is being DISPATCHED
//it holds that op's second-access address (EA2 / RMW write addr) so the AGU - not a ma_seq mux -
//sources it. idex.src_a_value is left intact for the post-inc writeback.
logic   [31:0]  agu_base_q;
//Pre-decoded address-UPDATE addend (ID): address_update = ex_a + au_addend_q, one bare
//adder in EX - the addr_op case decode and the AGU (PREDEC) leg leave the gpr1-forward
//cone. Values: PREDEC -step, POSTINC +step, MAC +step (+2step same-pointer), else 0.
logic   [31:0]  au_addend_q;
logic           ma_second_pending_agu; //the MAC/RMW second access is presenting its AGU address now
                                       //(rides the preserved *_agu duplicates; sole load is the AGU)
//Manual duplicate of idex.is_data, which is multi-consumer (l_is_data also drives the L bus
//req_fetch). preserve stops Quartus merging it back into the original, so the fitter places
//it beside the AGU. Written wherever idex is written (reset / redirect / idex_allow load /
//WB-fault flush); its only load is l_is_data_agu. The single-consumer idex.agu_en_mode
//needs no copy - the AGU reads the packet field directly.
(* preserve *) logic            idex_is_data_agu;

//EX-head forward state (one set per operand port a/b/st). The lane pick is latched
//at issue from REGISTERED compares (fwd_lane_pick). The WB view: a producer's word
//is readable from the registered mawb packet for exactly ONE cycle (mawb zeroes on
//non-completing cycles), so that cycle also DEPOSITS it reg->reg into the shadow
//and flips fwd_dep_*; a longer-held consumer keeps reading the shadow. No live
//cache/completion term enters any of these CEs, D-legs, or selects.
fwd_lane_t      fwd_lane_a, fwd_lane_b, fwd_lane_st;    //EX-head patch source per port
logic           fwd_wbsel_a, fwd_wbsel_b, fwd_wbsel_st; //REGISTERED "read the WB view" pick:
                                                        //set at issue for a load release, or at
                                                        //the producer's drain edge (exma_allow)
                                                        //under a held consumer; self-holding
logic           fwd_dep_a, fwd_dep_b, fwd_dep_st;       //WB word deposited in the shadow
                                                        //(= wbsel one held-cycle later)
logic   [31:0]  fwd_shadow_a, fwd_shadow_b, fwd_shadow_st; //deposited operand words
//AGU-cluster duplicates of the port-a/b forward state (the ma_second_pending_agu /
//idex_is_data_agu pattern): the AGU address mux selects read ONLY these copies so
//the fitter can place them at the adder; D-cones mirror the originals exactly.
(* preserve *) fwd_lane_t fwd_lane_a_agu, fwd_lane_b_agu;
(* preserve *) logic      fwd_wbsel_a_agu, fwd_wbsel_b_agu;
(* preserve *) logic      fwd_dep_a_agu,  fwd_dep_b_agu;

assign  o_FETCH_PC = fetch_pc;

// gpr_read_address_a/b are the BRAM read addresses for the n- and m-field operands.
// active_gpr_id() and gpr_bram_address() yield identical encodings (see pp.19-22), so
// read_address_a is just dec_n_id; reusing it drops a redundant n-field decode. The
// bank-swap form (LDC/STC Rm_BANK) and the port-0 assignment are PREDECODED at fetch
// (ifid.pd.rib / ifid.pd.need_a, bank-free - see pd_need_n): the old ID-time 3-source
// hz-vs-dec compare cone fed the cen_n read-address captures (gpr_read_ctx / RAM
// address input regs) - the decode->ctx half-cycle limiter. Now those captures see a
// registered 2-bit select over the 1-LUT bank-qualified ids. The old cone's only other
// product (gpr_read_ports_ready) was constant: two line addresses can never demand
// more than two ports, so the count<=2 guard is deleted, not moved.
assign  reads_inactive_bank = ifid.pd.rib;
assign  gpr_read_address_a = dec_n_id;
assign  gpr_read_address_b = reads_inactive_bank ? dec_bank_id : dec_m_id;
assign  gpr_read0_address  = ifid.pd.need_a ? gpr_read_address_a : gpr_read_address_b;
assign  gpr_read1_address  = gpr_read_address_b;

// synthesis translate_off
//Exhaustive one-shot check: the FLAT need_a decode (pd_route) must equal the selector-
//composed form (pd_need_n over the routed sources) for all 65536 opcodes. Together with
//the live check below (selector form == old ID-time cone) the equivalence chain closes.
initial begin
    for(int i = 0; i < 65536; i++) begin
        pd_route_t p;
        logic ref_na;
        p = pd_route(i[15:0]);
        ref_na = (p.a_used  && pd_need_n(p.a_sel,  i[15:0])) ||
                 (p.b_used  && pd_need_n(p.b_sel,  i[15:0])) ||
                 (p.st_used && pd_need_n(p.st_sel, i[15:0]));
        if(p.need_a !== ref_na)
            $fatal(1, "pd need_a flat/selector mismatch: inst=%04x flat=%b sel=%b",
                   i[15:0], p.need_a, ref_na);
    end
end

//Reference-model check: the fetch-time bank-free predecode must equal the old ID-time
//bank-qualified port classification for every live packet, in every bank state.
always_comb begin
    logic ref_need_a;
    ref_need_a = 1'b0;
    if(ifid.pd.a_used  && hz_a_id  != 5'd0 && hz_a_id  != 5'd8 && hz_a_id  == dec_n_id) ref_need_a = 1'b1;
    if(ifid.pd.b_used  && hz_b_id  != 5'd0 && hz_b_id  != 5'd8 && hz_b_id  == dec_n_id) ref_need_a = 1'b1;
    if(ifid.pd.st_used && hz_st_id != 5'd0 && hz_st_id != 5'd8 && hz_st_id == dec_n_id) ref_need_a = 1'b1;
    if(ifid.valid && ifid.pd.need_a !== ref_need_a)
        $fatal(1, "pd.need_a mismatch: inst=%04x pd=%b ref=%b", ifid.inst, ifid.pd.need_a, ref_need_a);
end

//Reference-model checks for the fetch-time HAZARD classification bits: each must equal
//the old ID-time expression for every live packet (the 64-test suite covers the classes).
always_comb begin
    logic ref_sr, ref_mac, ref_cm;
    ref_sr  = (ifid.inst[15:12] == 4'h0 && ifid.inst[7:0] == 8'h02) ||
              (ifid.inst[15:12] == 4'h4 && ifid.inst[7:0] == 8'h03);
    ref_mac = id_decode.mac_cmd != MAC_NONE ||
              (ifid.inst[15:12] == 4'h0 && ifid.inst[3:0] == 4'hA &&
               (ifid.inst[7:4] == 4'h0 || ifid.inst[7:4] == 4'h1)) ||
              (ifid.inst[15:12] == 4'h4 && ifid.inst[3:0] == 4'h2 &&
               (ifid.inst[7:4] == 4'h0 || ifid.inst[7:4] == 4'h1));
    ref_cm  = id_decode.addr_op == ADDR_GBR_DISP || id_decode.addr_op == ADDR_GBR_INDEX ||
              (ifid.inst[15:12] == 4'h0 && ifid.inst[3:0] == 4'h2)  ||
              (ifid.inst[15:12] == 4'h4 && ifid.inst[3:0] == 4'h3)  ||
              (ifid.inst[15:12] == 4'h0 && ifid.inst[7:0] == 8'h2A) ||
              (ifid.inst[15:12] == 4'h4 && ifid.inst[7:0] == 8'h22) ||
              (ifid.inst == 16'h00_0B) ||
              (ifid.inst == 16'h00_2B);   //RTE reads SPC (EX target) and SSR
    if(ifid.valid && ifid.pd.reads_sr !== ref_sr)
        $fatal(1, "pd.reads_sr mismatch: inst=%04x pd=%b ref=%b", ifid.inst, ifid.pd.reads_sr, ref_sr);
    if(ifid.valid && ifid.pd.uses_mac !== ref_mac)
        $fatal(1, "pd.uses_mac mismatch: inst=%04x pd=%b ref=%b", ifid.inst, ifid.pd.uses_mac, ref_mac);
    if(ifid.valid && ifid.pd.reads_cmisc !== ref_cm)
        $fatal(1, "pd.reads_cmisc mismatch: inst=%04x pd=%b ref=%b", ifid.inst, ifid.pd.reads_cmisc, ref_cm);
end

//Reference-model checks for the fetch-time ADDR-OP class bits: each must equal the
//old id_decode.addr_op expression for every live packet.
always_comb begin
    if(ifid.valid && ifid.pd.agbr !== (id_decode.addr_op == ADDR_GBR_DISP ||
                                       id_decode.addr_op == ADDR_GBR_INDEX))
        $fatal(1, "pd.agbr mismatch: inst=%04x pd=%b", ifid.inst, ifid.pd.agbr);
    if(ifid.valid && ifid.pd.apc  !== (id_decode.addr_op == ADDR_PC_WORD ||
                                       id_decode.addr_op == ADDR_PC_LONG))
        $fatal(1, "pd.apc mismatch: inst=%04x pd=%b", ifid.inst, ifid.pd.apc);
    if(ifid.valid && ifid.pd.pdec !== (id_decode.addr_op == ADDR_PREDEC))
        $fatal(1, "pd.pdec mismatch: inst=%04x pd=%b", ifid.inst, ifid.pd.pdec);
    if(ifid.valid && ifid.pd.gbrx !== (id_decode.addr_op == ADDR_GBR_INDEX))
        $fatal(1, "pd.gbrx mismatch: inst=%04x pd=%b", ifid.inst, ifid.pd.gbrx);
end

//The registered bank mirror must equal the live SR-derived select on every live packet
//(staleness may exist only inside redirect shadows, where IF/ID holds no instruction).
//Exception: during an RTE restore the mirror legally LEADS the committed SR by the two
//snoop cycles (rte_wr_wb lookahead + the o_RTE_VALID commit cycle) - the held target's
//read must already use the restored bank while SR still shows the handler's.
always_comb begin
    if(ifid.valid && r_bank1 !== gpr_active_bank1 &&
       !o_RTE_VALID && !(wb_valid && mawb.event_rte))
        $fatal(1, "r_bank1 stale under live packet: r=%b sr=%b", r_bank1, gpr_active_bank1);
end
// synthesis translate_on


always_comb begin
    //ctrl_reg samples the registered WB packet on its own enabled edge.
    o_CTRL_WE  = wb_valid && !i_REDIRECT_VALID && !mawb.fault &&
                 mawb.ctrl_dst != CTRL_NONE;
    o_CTRL_DST = mawb.ctrl_dst;
    o_CTRL_DATA= mawb.ctrl_data;

    o_SR_T_WE  = wb_valid && !i_REDIRECT_VALID && !mawb.fault &&
                 mawb.ctrl_dst == CTRL_NONE && mawb.t_we;
    o_SR_T     = mawb.t_data;
    o_SR_S_WE  = wb_valid && !i_REDIRECT_VALID && !mawb.fault &&
                 mawb.ctrl_dst == CTRL_NONE && mawb.s_we;
    o_SR_S     = mawb.s_data;
    o_SR_MQ_WE = (wb_valid && !i_REDIRECT_VALID && !mawb.fault &&
                  mawb.ctrl_dst == CTRL_NONE) ? mawb.mq_we : 2'b00;
    o_SR_MQ    = mawb.mq_data;
end


///////////////////////////////////////////////////////////
//////  Instruction Fetch
////

/*
    IF permits one outstanding request. Request PC advances after acceptance.
    External redirect has priority over an EX branch redirect.
    Redirected outstanding responses are accepted and discarded with fetch_drop.
*/

logic           branch_event;
logic           branch_taken;  //resolved condition from EX
logic           branch_delayed; //branch architecture preserves one delay slot
logic   [31:0]  branch_target;
logic           branch_redirect;
logic           redirect_active;
logic           early_i_req_raw_valid; //pre-IF request before D-priority suppression

// --- Unified L bus adapter (IF+MA share one bus) -----------------------------------------
// The pipeline drives ONE address per cycle (the time-shared AGU output ea_addr_sum), and
// L_BUS.req_fetch (= !l_is_data, D-priority) selects IF vs MA. These wires reconstruct the
// pipeline's former separate I-bus / D-bus VIEWS so the fetch and MA logic below stay intact.
// l_is_data and the L_BUS request drive are assigned lower down (after u_ma_seq, where
// ma_second_access is declared). All plain scalars: the dual-CEN 4-rail machinery is gone.
wire            i_rsp_valid = L_BUS.rsp_valid &&  L_BUS.rsp_fetch;  //fetch response present
wire    [15:0]  i_rsp_inst  = L_BUS.rsp_inst;                      //fetched opcode (I-only field)
wire            i_rsp_fault = L_BUS.rsp_ifault;                    //registered-flag product
wire            d_rsp_valid = L_BUS.rsp_valid && !L_BUS.rsp_fetch;  //data response present
wire    [31:0]  d_rsp_rdata = L_BUS.rsp_rdata;
wire            d_rsp_fault = L_BUS.rsp_dfault;                    //registered-flag product
logic           i_rsp_ready;           //fetch consume (was I_BUS.rsp_ready)
logic           d_rsp_ready;           //MA    consume (was D_BUS.rsp_ready)
logic           if_accept;             //fetch response consumed this cycle
logic           i_req_fire;            //fetch issued this cycle (0 on a data cycle)

assign  redirect_active = i_REDIRECT_VALID || branch_redirect;

//Predecode source routing in the fetch (response) cycle: pure carry-chain-free
//classification of the arriving opcode, latched into ifid.pd so ID skips the ~3-level
//decode on the hazard/issue cone. See int_pipe.md.
pd_route_t      pd_fetch;
assign  pd_fetch = pd_route(i_rsp_inst);

//Sibling opcode + its predecode (second pd_route instance): both are captured into the
//pair slot registers, so a pair-served IF/ID load is a pure register read - the serve
//path adds NO depth to the rsp_inst -> predecode cone.
wire    [15:0]  i_rsp_sib = L_BUS.rsp_inst_sib;
pd_route_t      pd_fetch_sib;
assign  pd_fetch_sib = pd_route(i_rsp_sib);


///////////////////////////////////////////////////////////
//////  Instruction Decode
////

/*
    Decode follows the 16-bit code map in manual table 2.12,
    pp.50-52. "n" and "m" are the manual's Rn/Rm fields.
    dec means decoded; id is the bank-qualified physical register identity.
    Every unknown pattern remains illegal by default.
*/

logic   [3:0]   dec_n, dec_m;                   //encoded Rn and Rm fields
logic   [4:0]   dec_n_id, dec_m_id, dec_r0_id; //physical GPR identities
logic   [4:0]   dec_bank_id;                    //inactive R0_BANK-R7_BANK identity

//Hazard-path source identities rebuilt from the PREDECODED selectors (ifid.pd) and the
//current-bank dec_*_id. This 4:1 qualify (one LUT level) replaces the ~3-level decode on
//the hazard/gpr-need cone that fed the decode->issue->cache victim_tag limiter. A sim
//assertion (see below) proves hz_*_id === id_decode.src_*_id every issue.
//REGISTERED at the IF/ID edge from the nx cone (see the capture block below the GPR
//next-read addresses); the old live recompute survives only as the sim reference.
logic   [4:0]   hz_a_id, hz_b_id, hz_st_id;

// synthesis translate_off
//The registered ids must equal the live bank-qualified recompute on every LIVE packet.
//(Not every cycle: a mid-run reset clears these flops while the R7-unreset IF/ID data
//lanes keep pre-reset garbage - harmless, every consumer is valid/we-gated.)
always_comb begin
    if(ifid.valid && hz_a_id  !== pd_qualify(ifid.pd.a_sel,  dec_n_id, dec_m_id, dec_r0_id, dec_bank_id))
        $fatal(1, "hz_a_id stale: inst=%04x", ifid.inst);
    if(ifid.valid && hz_b_id  !== pd_qualify(ifid.pd.b_sel,  dec_n_id, dec_m_id, dec_r0_id, dec_bank_id))
        $fatal(1, "hz_b_id stale: inst=%04x", ifid.inst);
    if(ifid.valid && hz_st_id !== pd_qualify(ifid.pd.st_sel, dec_n_id, dec_m_id, dec_r0_id, dec_bank_id))
        $fatal(1, "hz_st_id stale: inst=%04x", ifid.inst);
end
// synthesis translate_on

assign  dec_n        = ifid.inst[11:8];
assign  dec_m        = ifid.inst[7:4];
//Bank select = the REGISTERED local mirror (see r_bank1), not the live cross-module o_SR.
assign  dec_n_id     = active_gpr_id(dec_n, r_bank1);
assign  dec_m_id     = active_gpr_id(dec_m, r_bank1);
assign  dec_r0_id    = active_gpr_id(4'd0, r_bank1);
assign  dec_bank_id  = inactive_bank_id(ifid.inst[6:4], r_bank1);

always_comb begin
    //Defaults describe an illegal, side-effect-free instruction packet.
    id_decode = '0;
    id_decode.valid         = ifid.valid;
    id_decode.pc            = ifid.pc;
    id_decode.inst          = ifid.inst;
    id_decode.delay_slot    = ifid.delay_slot;
    id_decode.alu_op        = ALU_PASS_A;
    id_decode.mem_op        = MEM_NONE;
    id_decode.mem_size      = SIZE_LONG;
    id_decode.addr_op       = ADDR_NONE;
    id_decode.load_signed   = 1'b1;
    id_decode.branch_op     = BR_NONE;
    id_decode.illegal       = ifid.valid;
    id_decode.fetch_fault   = ifid.fetch_fault;

    //The high nibble selects the major family in table 2.12.
    case(ifid.inst[15:12])
        4'h0: begin
            //System, indexed-memory, multiply, and register-branch encodings.
            case(ifid.inst[3:0])
                4'h3: begin
                    if(ifid.inst[7:4] == 4'h0) begin
                        id_decode.illegal        = 1'b0;
                        id_decode.branch_op      = BR_BSRF;
                        id_decode.branch_delayed = 1'b1;
                        id_decode.pr_link        = 1'b1;
                        id_decode.src_a_used     = 1'b1;
                        id_decode.src_a_id       = dec_n_id;
                        id_decode.immediate      = ifid.pc + 32'd4;  //pc+4 base; EX adds Rn (2-input)
                    end
                    else if(ifid.inst[7:4] == 4'h2) begin
                        id_decode.illegal        = 1'b0;
                        id_decode.branch_op      = BR_BRAF;
                        id_decode.branch_delayed = 1'b1;
                        id_decode.src_a_used     = 1'b1;
                        id_decode.src_a_id       = dec_n_id;
                        id_decode.immediate      = ifid.pc + 32'd4;  //pc+4 base; EX adds Rn (2-input)
                    end
                    else if(ifid.inst[7:4] == 4'h8) begin
                        //PREF @Rn: allocate the cache line containing Rn. No data is
                        //returned, no register is written, and it never faults; the
                        //cache is told via the o_D_PREF sideband. See sw manual 8.2.48.
                        id_decode.illegal    = 1'b0;
                        id_decode.mem_op     = MEM_PREF;
                        id_decode.mem_size   = SIZE_LONG;
                        id_decode.addr_op    = ADDR_REG;
                        id_decode.src_a_used = 1'b1;
                        id_decode.src_a_id   = dec_n_id;
                    end
                end
                4'h2: begin
                    //STC selectors follow software manual section 8.2.67, pp.261-263.
                    if(ifid.inst[7:4] <= 4'h4) begin
                        id_decode.illegal    = 1'b0;
                        id_decode.privileged = ifid.inst[7:4] != 4'h1;
                        id_decode.alu_op     = ALU_PASS_B;
                        id_decode.immediate  = control_read_value(ifid.inst[6:4],
                                                                  i_SR, i_GBR, i_VBR, i_SSR, i_SPC);
                        id_decode.gpr0_we    = 1'b1;
                        id_decode.gpr0_dst   = dec_n_id;
                    end
                    else if(ifid.inst[7]) begin
                        id_decode.illegal    = 1'b0;
                        id_decode.privileged = 1'b1;
                        id_decode.src_a_used = 1'b1;
                        id_decode.src_a_id   = dec_bank_id;
                        id_decode.alu_op     = ALU_PASS_A;
                        id_decode.gpr0_we    = 1'b1;
                        id_decode.gpr0_dst   = dec_n_id;
                    end
                end
                4'h4, 4'h5, 4'h6: begin
                    id_decode.illegal     = 1'b0;
                    id_decode.mem_op      = MEM_STORE;
                    id_decode.mem_size    = mem_size_t'(ifid.inst[1:0]);
                    id_decode.addr_op     = ADDR_INDEX;
                    id_decode.src_a_used  = 1'b1;
                    id_decode.src_a_id    = dec_n_id;
                    id_decode.src_b_used  = 1'b1;
                    id_decode.src_b_id    = dec_r0_id;
                    id_decode.store_used  = 1'b1;
                    id_decode.store_id    = dec_m_id;
                end
                4'h7: begin
                    id_decode.illegal     = 1'b0;
                    id_decode.mac_cmd     = MAC_MULL;
                    id_decode.src_a_used  = 1'b1;
                    id_decode.src_a_id    = dec_n_id;
                    id_decode.src_b_used  = 1'b1;
                    id_decode.src_b_id    = dec_m_id;
                end
                4'h8: begin
                    case(ifid.inst)
                        16'h0008: begin id_decode.illegal = 1'b0; id_decode.t_write_decode = 1'b1; id_decode.t_decode_value = 1'b0; end
                        16'h0018: begin id_decode.illegal = 1'b0; id_decode.t_write_decode = 1'b1; id_decode.t_decode_value = 1'b1; end
                        16'h0028: begin id_decode.illegal = 1'b0; id_decode.mac_cmd = MAC_CLEAR; end
                        16'h0038: begin id_decode.illegal = 1'b0; id_decode.event_ldtlb = 1'b1; id_decode.privileged = 1'b1; end
                        16'h0048: begin id_decode.illegal = 1'b0; id_decode.s_write_decode = 1'b1; id_decode.s_decode_value = 1'b0; end
                        16'h0058: begin id_decode.illegal = 1'b0; id_decode.s_write_decode = 1'b1; id_decode.s_decode_value = 1'b1; end
                        default: begin end
                    endcase
                end
                4'h9: begin
                    if(ifid.inst == 16'h0019) begin
                        //DIV0U clears M/Q/T; software manual section 8.2.18, p.172.
                        id_decode.illegal = 1'b0;
                        id_decode.div_op  = DIV_INIT_UNSIGNED;
                    end
                    else if(ifid.inst == 16'h0009) begin
                        id_decode.illegal = 1'b0;
                    end
                    else if(ifid.inst[7:0] == 8'h29) begin
                        id_decode.illegal  = 1'b0;
                        id_decode.alu_op   = ALU_MOVT;
                        id_decode.gpr0_we  = 1'b1;
                        id_decode.gpr0_dst = dec_n_id;
                    end
                end
                4'hA: begin
                    if(ifid.inst[7:0] == 8'h0A || ifid.inst[7:0] == 8'h1A || ifid.inst[7:0] == 8'h2A) begin
                        id_decode.illegal  = 1'b0;
                        id_decode.alu_op   = ALU_PASS_B;
                        id_decode.gpr0_we  = 1'b1;
                        id_decode.gpr0_dst = dec_n_id;
                        case(ifid.inst[7:4])
                            4'h0: id_decode.immediate = mach;
                            4'h1: id_decode.immediate = macl;
                            default: id_decode.immediate = pr;
                        endcase
                    end
                end
                4'hB: begin
                    case(ifid.inst)
                        16'h000B: begin id_decode.illegal = 1'b0; id_decode.branch_op = BR_RTS; id_decode.branch_delayed = 1'b1; end
                        16'h001B: begin id_decode.illegal = 1'b0; id_decode.event_sleep = 1'b1; id_decode.privileged = 1'b1; end
                        16'h002B: begin id_decode.illegal = 1'b0; id_decode.branch_op = BR_RTE; id_decode.branch_delayed = 1'b1; id_decode.event_rte = 1'b1; id_decode.privileged = 1'b1; end
                        default: begin end
                    endcase
                end
                4'hC, 4'hD, 4'hE: begin
                    id_decode.illegal     = 1'b0;
                    id_decode.mem_op      = MEM_LOAD;
                    id_decode.mem_size    = mem_size_t'(ifid.inst[1:0]);
                    id_decode.addr_op     = ADDR_INDEX;
                    id_decode.src_a_used  = 1'b1;
                    id_decode.src_a_id    = dec_m_id;
                    id_decode.src_b_used  = 1'b1;
                    id_decode.src_b_id    = dec_r0_id;
                    id_decode.gpr0_we     = 1'b1;
                    id_decode.gpr0_dst    = dec_n_id;
                end
                4'hF: begin
                    //Source: hardware manual table 2.7, pp.40-42.
                    //MAC.L reads @Rn then @Rm, increments both by four, and accumulates.
                    id_decode.illegal     = 1'b0;
                    id_decode.mem_op      = MEM_MAC;
                    id_decode.mem_size    = SIZE_LONG;
                    id_decode.addr_op     = ADDR_MAC;
                    id_decode.mac_cmd     = MAC_ACCUM_L;
                    id_decode.src_a_used  = 1'b1;
                    id_decode.src_a_id    = dec_n_id;
                    id_decode.src_b_used  = 1'b1;
                    id_decode.src_b_id    = dec_m_id;
                    id_decode.gpr0_we     = 1'b1;
                    id_decode.gpr0_dst    = dec_n_id;
                    id_decode.gpr1_we     = dec_n_id != dec_m_id;
                    id_decode.gpr1_dst    = dec_m_id;
                end
                default: begin end
            endcase
        end
        4'h1: begin
            //MOV.L Rm,@(disp,Rn); displacement is scaled by four.
            id_decode.illegal     = 1'b0;
            id_decode.mem_op      = MEM_STORE;
            id_decode.mem_size    = SIZE_LONG;
            id_decode.addr_op     = ADDR_DISP;
            id_decode.immediate   = {26'd0, ifid.inst[3:0], 2'b00};
            id_decode.src_a_used  = 1'b1;
            id_decode.src_a_id    = dec_n_id;
            id_decode.store_used  = 1'b1;
            id_decode.store_id    = dec_m_id;
        end
        4'h2: begin
            //Register stores, logical operations, XTRCT, and word multiplies.
            case(ifid.inst[3:0])
                4'h0, 4'h1, 4'h2: begin
                    id_decode.illegal     = 1'b0;
                    id_decode.mem_op      = MEM_STORE;
                    id_decode.mem_size    = mem_size_t'(ifid.inst[1:0]);
                    id_decode.addr_op     = ADDR_REG;
                    id_decode.src_a_used  = 1'b1;
                    id_decode.src_a_id    = dec_n_id;
                    id_decode.store_used  = 1'b1;
                    id_decode.store_id    = dec_m_id;
                end
                4'h4, 4'h5, 4'h6: begin
                    id_decode.illegal     = 1'b0;
                    id_decode.mem_op      = MEM_STORE;
                    id_decode.mem_size    = mem_size_t'(ifid.inst[1:0]);
                    id_decode.addr_op     = ADDR_PREDEC;
                    id_decode.src_a_used  = 1'b1;
                    id_decode.src_a_id    = dec_n_id;
                    id_decode.store_used  = 1'b1;
                    id_decode.store_id    = dec_m_id;
                    id_decode.gpr1_we     = 1'b1;
                    id_decode.gpr1_dst    = dec_n_id;
                end
                4'h8, 4'h9, 4'hA, 4'hB: begin
                    id_decode.illegal     = 1'b0;
                    id_decode.src_a_used  = 1'b1;
                    id_decode.src_a_id    = dec_n_id;
                    id_decode.src_b_used  = 1'b1;
                    id_decode.src_b_id    = dec_m_id;
                    id_decode.gpr0_we     = ifid.inst[3:0] != 4'h8;
                    id_decode.gpr0_dst    = dec_n_id;
                    case(ifid.inst[3:0])
                        4'h8: id_decode.alu_op = ALU_TST;
                        4'h9: id_decode.alu_op = ALU_AND;
                        4'hA: id_decode.alu_op = ALU_XOR;
                        default: id_decode.alu_op = ALU_OR;
                    endcase
                end
                4'hC: begin
                    id_decode.illegal = 1'b0; id_decode.alu_op = ALU_CMP_STR;
                    id_decode.src_a_used = 1'b1; id_decode.src_a_id = dec_n_id;
                    id_decode.src_b_used = 1'b1; id_decode.src_b_id = dec_m_id;
                end
                4'hD: begin
                    id_decode.illegal = 1'b0; id_decode.alu_op = ALU_XTRCT; id_decode.gpr0_we = 1'b1; id_decode.gpr0_dst = dec_n_id;
                    id_decode.src_a_used = 1'b1; id_decode.src_a_id = dec_n_id;
                    id_decode.src_b_used = 1'b1; id_decode.src_b_id = dec_m_id;
                end
                4'h7: begin
                    //DIV0S initializes M/Q/T; software manual section 8.2.17, p.171.
                    id_decode.illegal    = 1'b0;
                    id_decode.div_op     = DIV_INIT_SIGNED;
                    id_decode.src_a_used = 1'b1;
                    id_decode.src_a_id   = dec_n_id;
                    id_decode.src_b_used = 1'b1;
                    id_decode.src_b_id   = dec_m_id;
                end
                4'hE, 4'hF: begin
                    id_decode.illegal = 1'b0; id_decode.mac_cmd = ifid.inst[0] ? MAC_MULS_W : MAC_MULU_W;
                    id_decode.src_a_used = 1'b1; id_decode.src_a_id = dec_n_id;
                    id_decode.src_b_used = 1'b1; id_decode.src_b_id = dec_m_id;
                end
                default: begin end
            endcase
        end
        4'h3: begin
            //Two-register arithmetic and comparisons; see manual table 2.7.
            id_decode.src_a_used  = 1'b1;
            id_decode.src_a_id    = dec_n_id;
            id_decode.src_b_used  = 1'b1;
            id_decode.src_b_id    = dec_m_id;
            id_decode.gpr0_dst    = dec_n_id;
            case(ifid.inst[3:0])
                4'h0: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_CMP_EQ; end
                4'h2: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_CMP_HS; end
                4'h3: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_CMP_GE; end
                4'h4: begin
                    //DIV1 follows software manual section 8.2.19, pp.173-175.
                    id_decode.illegal = 1'b0;
                    id_decode.alu_op  = ALU_DIV1;
                    id_decode.div_op  = DIV_STEP;
                    id_decode.gpr0_we = 1'b1;
                end
                4'h5: begin
                    //DMULU.L encoding and result follow hardware manual p.41.
                    id_decode.illegal = 1'b0;
                    id_decode.mac_cmd = MAC_DMULU_L;
                end
                4'h6: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_CMP_HI; end
                4'h7: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_CMP_GT; end
                4'h8: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SUB;  id_decode.gpr0_we = 1'b1; end
                4'hA: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SUBC; id_decode.gpr0_we = 1'b1; end
                4'hB: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SUBV; id_decode.gpr0_we = 1'b1; end
                4'hC: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_ADD;  id_decode.gpr0_we = 1'b1; end
                4'hD: begin
                    //DMULS.L encoding and result follow hardware manual p.41.
                    id_decode.illegal = 1'b0;
                    id_decode.mac_cmd = MAC_DMULS_L;
                end
                4'hE: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_ADDC; id_decode.gpr0_we = 1'b1; end
                4'hF: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_ADDV; id_decode.gpr0_we = 1'b1; end
                default: begin end
            endcase
        end
        4'h4: begin
            //Shifts, register jumps, and system-register transfers.
            id_decode.src_a_used  = 1'b1;
            id_decode.src_a_id    = dec_n_id;
            id_decode.gpr0_dst    = dec_n_id;
            case(ifid.inst[7:0])
                8'h00: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SHLL;   id_decode.gpr0_we = 1'b1; end
                8'h01: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SHLR;   id_decode.gpr0_we = 1'b1; end
                8'h04: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_ROTL;   id_decode.gpr0_we = 1'b1; end
                8'h05: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_ROTR;   id_decode.gpr0_we = 1'b1; end
                8'h08: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SHLL2;  id_decode.gpr0_we = 1'b1; end
                8'h09: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SHLR2;  id_decode.gpr0_we = 1'b1; end
                8'h0A, 8'h1A, 8'h2A: begin
                    id_decode.illegal = 1'b0; id_decode.alu_op = ALU_PASS_A;
                    case(ifid.inst[7:4])
                        4'h0: id_decode.mac_cmd = MAC_LOAD_MACH;
                        4'h1: id_decode.mac_cmd = MAC_LOAD_MACL;
                        default: id_decode.pr_we = 1'b1;
                    endcase
                end
                8'h0B: begin id_decode.illegal = 1'b0; id_decode.branch_op = BR_JSR; id_decode.branch_delayed = 1'b1; id_decode.pr_link = 1'b1; end
                8'h06, 8'h16, 8'h26: begin
                    //LDS.L follows software manual section 8.2.30, pp.199-202.
                    id_decode.illegal  = 1'b0;
                    id_decode.mem_op   = MEM_LOAD;
                    id_decode.mem_size = SIZE_LONG;
                    id_decode.addr_op  = ADDR_POSTINC;
                    id_decode.gpr1_we  = 1'b1;
                    id_decode.gpr1_dst = dec_n_id;
                    case(ifid.inst[7:4])
                        4'h0: id_decode.mac_cmd = MAC_LOAD_MACH;
                        4'h1: id_decode.mac_cmd = MAC_LOAD_MACL;
                        default: id_decode.pr_we = 1'b1;
                    endcase
                end
                8'h02, 8'h12, 8'h22: begin
                    //STS.L follows software manual section 8.2.68, pp.267-269.
                    id_decode.illegal  = 1'b0;
                    id_decode.mem_op   = MEM_STORE;
                    id_decode.mem_size = SIZE_LONG;
                    id_decode.addr_op  = ADDR_PREDEC;
                    id_decode.gpr1_we  = 1'b1;
                    id_decode.gpr1_dst = dec_n_id;
                    case(ifid.inst[7:4])
                        4'h0: id_decode.immediate = mach;
                        4'h1: id_decode.immediate = macl;
                        default: id_decode.immediate = pr;
                    endcase
                end
                8'h10: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_DT;     id_decode.gpr0_we = 1'b1; end
                8'h11: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_CMP_PZ; end
                8'h15: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_CMP_PL; end
                8'h18: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SHLL8;  id_decode.gpr0_we = 1'b1; end
                8'h19: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SHLR8;  id_decode.gpr0_we = 1'b1; end
                8'h20: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SHLL;   id_decode.gpr0_we = 1'b1; end
                8'h21: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SHAR;   id_decode.gpr0_we = 1'b1; end
                8'h24: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_ROTCL;  id_decode.gpr0_we = 1'b1; end
                8'h25: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_ROTCR;  id_decode.gpr0_we = 1'b1; end
                8'h28: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SHLL16; id_decode.gpr0_we = 1'b1; end
                8'h29: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SHLR16; id_decode.gpr0_we = 1'b1; end
                8'h2B: begin id_decode.illegal = 1'b0; id_decode.branch_op = BR_JMP; id_decode.branch_delayed = 1'b1; end
                8'h1B: begin
                    //TAS.B holds lock across both phases; section 8.2.73, p.278.
                    id_decode.illegal    = 1'b0;
                    id_decode.mem_op     = MEM_RMW;
                    id_decode.mem_size   = SIZE_BYTE;
                    id_decode.addr_op    = ADDR_REG;
                    id_decode.byte_op    = BYTE_TAS;
                end
                8'h0E, 8'h1E, 8'h2E, 8'h3E, 8'h4E: begin
                    //Direct LDC follows section 8.2.27, pp.189-190.
                    id_decode.illegal    = 1'b0;
                    id_decode.privileged = ifid.inst[7:4] != 4'h1;
                    id_decode.ctrl_dst   = control_destination(ifid.inst[6:4]);
                end
                default: begin
                    if(ifid.inst[3:0] == 4'hE && ifid.inst[7]) begin
                        //Bank LDC follows section 8.2.27, pp.189-190.
                        id_decode.illegal = 1'b0;
                        id_decode.privileged = 1'b1;
                        id_decode.gpr0_we  = 1'b1;
                        id_decode.gpr0_dst = dec_bank_id;
                    end
                    else if(ifid.inst[3:0] == 4'h7 &&
                            (ifid.inst[7] || ifid.inst[7:4] <= 4'h4)) begin
                        //LDC.L forms follow section 8.2.27, pp.191-194.
                        id_decode.illegal     = 1'b0;
                        id_decode.privileged  = ifid.inst[7:4] != 4'h1;
                        id_decode.mem_op      = MEM_LOAD;
                        id_decode.mem_size    = SIZE_LONG;
                        id_decode.addr_op     = ADDR_POSTINC;
                        id_decode.gpr1_we     = 1'b1;
                        id_decode.gpr1_dst    = dec_n_id;
                        if(ifid.inst[7]) begin
                            id_decode.gpr0_we  = 1'b1;
                            id_decode.gpr0_dst = dec_bank_id;
                        end
                        else begin
                            id_decode.ctrl_dst = control_destination(ifid.inst[6:4]);
                        end
                    end
                    else if(ifid.inst[3:0] == 4'h3 &&
                            (ifid.inst[7] || ifid.inst[7:4] <= 4'h4)) begin
                        //STC.L forms follow section 8.2.67, pp.263-266.
                        id_decode.illegal     = 1'b0;
                        id_decode.privileged  = ifid.inst[7:4] != 4'h1;
                        id_decode.mem_op      = MEM_STORE;
                        id_decode.mem_size    = SIZE_LONG;
                        id_decode.addr_op     = ADDR_PREDEC;
                        id_decode.gpr1_we     = 1'b1;
                        id_decode.gpr1_dst    = dec_n_id;
                        if(ifid.inst[7]) begin
                            id_decode.store_used = 1'b1;
                            id_decode.store_id   = dec_bank_id;
                        end
                        else begin
                            id_decode.immediate = control_read_value(ifid.inst[6:4],
                                                                     i_SR, i_GBR, i_VBR, i_SSR, i_SPC);
                        end
                    end
                    else if(ifid.inst[3:0] == 4'hF) begin
                        //Source: hardware manual table 2.7, pp.40-42.
                        //MAC.W uses signed words and advances both source pointers by two.
                        id_decode.illegal     = 1'b0;
                        id_decode.mem_op      = MEM_MAC;
                        id_decode.mem_size    = SIZE_WORD;
                        id_decode.addr_op     = ADDR_MAC;
                        id_decode.mac_cmd     = MAC_ACCUM_W;
                        id_decode.src_b_used  = 1'b1;
                        id_decode.src_b_id    = dec_m_id;
                        id_decode.gpr0_we     = 1'b1;
                        id_decode.gpr1_we     = dec_n_id != dec_m_id;
                        id_decode.gpr1_dst    = dec_m_id;
                    end
                    else if(ifid.inst[3:0] == 4'hC || ifid.inst[3:0] == 4'hD) begin
                        id_decode.illegal     = 1'b0;
                        id_decode.alu_op      = ifid.inst[0] ? ALU_SHLD : ALU_SHAD;
                        id_decode.gpr0_we     = 1'b1;
                        id_decode.src_b_used  = 1'b1;
                        id_decode.src_b_id    = dec_m_id;
                    end
                end
            endcase
        end
        4'h5: begin
            //MOV.L @(disp,Rm),Rn; displacement is scaled by four.
            id_decode.illegal     = 1'b0;
            id_decode.mem_op      = MEM_LOAD;
            id_decode.mem_size    = SIZE_LONG;
            id_decode.addr_op     = ADDR_DISP;
            id_decode.immediate   = {26'd0, ifid.inst[3:0], 2'b00};
            id_decode.src_a_used  = 1'b1;
            id_decode.src_a_id    = dec_m_id;
            id_decode.gpr0_we     = 1'b1;
            id_decode.gpr0_dst    = dec_n_id;
        end
        4'h6: begin
            //Register moves, loads, unary operations, swap, and extension.
            id_decode.src_a_used  = 1'b1;
            id_decode.src_a_id    = dec_m_id;
            id_decode.gpr0_we     = 1'b1;
            id_decode.gpr0_dst    = dec_n_id;
            case(ifid.inst[3:0])
                4'h0, 4'h1, 4'h2: begin id_decode.illegal = 1'b0; id_decode.mem_op = MEM_LOAD; id_decode.mem_size = mem_size_t'(ifid.inst[1:0]); id_decode.addr_op = ADDR_REG; end
                4'h3: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_PASS_A; end
                4'h4, 4'h5, 4'h6: begin
                    id_decode.illegal = 1'b0; id_decode.mem_op = MEM_LOAD; id_decode.mem_size = mem_size_t'(ifid.inst[1:0]); id_decode.addr_op = ADDR_POSTINC;
                    if(dec_m_id != dec_n_id) begin id_decode.gpr1_we = 1'b1; id_decode.gpr1_dst = dec_m_id; end
                end
                4'h7: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_NOT; end
                4'h8: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SWAP_B; end
                4'h9: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_SWAP_W; end
                4'hA: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_NEGC; end
                4'hB: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_NEG; end
                4'hC: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_EXTU_B; end
                4'hD: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_EXTU_W; end
                4'hE: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_EXTS_B; end
                4'hF: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_EXTS_W; end
            endcase
        end
        4'h7: begin
            //ADD #imm,Rn uses an eight-bit sign-extended immediate.
            id_decode.illegal     = 1'b0;
            id_decode.alu_op      = ALU_ADD;
            id_decode.src_a_used  = 1'b1;
            id_decode.src_a_id    = dec_n_id;
            id_decode.immediate   = {{24{ifid.inst[7]}}, ifid.inst[7:0]};
            id_decode.gpr0_we     = 1'b1;
            id_decode.gpr0_dst    = dec_n_id;
        end
        4'h8: begin
            //R0 displacement accesses and eight-bit conditional branches.
            case(ifid.inst[11:8])
                4'h0, 4'h1: begin
                    id_decode.illegal = 1'b0; id_decode.mem_op = MEM_STORE; id_decode.mem_size = ifid.inst[8] ? SIZE_WORD : SIZE_BYTE; id_decode.addr_op = ADDR_DISP;
                    id_decode.immediate = ifid.inst[8] ? {27'd0, ifid.inst[3:0], 1'b0} : {28'd0, ifid.inst[3:0]};
                    id_decode.src_a_used = 1'b1; id_decode.src_a_id = dec_m_id;
                    id_decode.store_used = 1'b1; id_decode.store_id = dec_r0_id;
                end
                4'h4, 4'h5: begin
                    id_decode.illegal = 1'b0; id_decode.mem_op = MEM_LOAD; id_decode.mem_size = ifid.inst[8] ? SIZE_WORD : SIZE_BYTE; id_decode.addr_op = ADDR_DISP;
                    id_decode.immediate = ifid.inst[8] ? {27'd0, ifid.inst[3:0], 1'b0} : {28'd0, ifid.inst[3:0]};
                    id_decode.src_a_used = 1'b1; id_decode.src_a_id = dec_m_id;
                    id_decode.gpr0_we = 1'b1; id_decode.gpr0_dst = dec_r0_id;
                end
                4'h8: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_CMP_EQ; id_decode.src_a_used = 1'b1; id_decode.src_a_id = dec_r0_id; id_decode.immediate = {{24{ifid.inst[7]}}, ifid.inst[7:0]}; end
                4'h9: begin id_decode.illegal = 1'b0; id_decode.branch_op = BR_BT; id_decode.immediate = ifid.pc + 32'd4 + {{23{ifid.inst[7]}}, ifid.inst[7:0], 1'b0}; end //precompute full BF/BT target in ID (pc+4+disp), so EX needs no branch adder
                4'hB: begin id_decode.illegal = 1'b0; id_decode.branch_op = BR_BF; id_decode.immediate = ifid.pc + 32'd4 + {{23{ifid.inst[7]}}, ifid.inst[7:0], 1'b0}; end //precompute full BF/BT target in ID (pc+4+disp), so EX needs no branch adder
                4'hD: begin id_decode.illegal = 1'b0; id_decode.branch_op = BR_BT; id_decode.branch_delayed = 1'b1; id_decode.immediate = ifid.pc + 32'd4 + {{23{ifid.inst[7]}}, ifid.inst[7:0], 1'b0}; end //precompute full BF/BT target in ID (pc+4+disp), so EX needs no branch adder
                4'hF: begin id_decode.illegal = 1'b0; id_decode.branch_op = BR_BF; id_decode.branch_delayed = 1'b1; id_decode.immediate = ifid.pc + 32'd4 + {{23{ifid.inst[7]}}, ifid.inst[7:0], 1'b0}; end //precompute full BF/BT target in ID (pc+4+disp), so EX needs no branch adder
                default: begin end
            endcase
        end
        4'h9: begin
            //PC-relative word load; architectural PC is instruction PC plus four.
            id_decode.illegal   = 1'b0;
            id_decode.mem_op    = MEM_LOAD;
            id_decode.mem_size  = SIZE_WORD;
            id_decode.addr_op   = ADDR_PC_WORD;
            id_decode.immediate = ifid.pc + 32'd4 + {23'd0, ifid.inst[7:0], 1'b0};
            id_decode.gpr0_we   = 1'b1;
            id_decode.gpr0_dst  = dec_n_id;
        end
        4'hA, 4'hB: begin
            //BRA and BSR use signed twelve-bit displacement and one delay slot.
            id_decode.illegal        = 1'b0;
            id_decode.branch_op      = ifid.inst[15:12] == 4'hA ? BR_BRA : BR_BSR;
            id_decode.branch_delayed = 1'b1;
            id_decode.pr_link        = ifid.inst[15:12] == 4'hB;
            //precompute full BRA/BSR target in ID (pc+4+disp); EX branch needs no adder
            id_decode.immediate      = ifid.pc + 32'd4 + {{19{ifid.inst[11]}}, ifid.inst[11:0], 1'b0};
        end
        4'hC: begin
            //GBR accesses, TRAPA, MOVA, and R0 immediate logical operations.
            case(ifid.inst[11:8])
                4'h0, 4'h1, 4'h2: begin
                    id_decode.illegal = 1'b0; id_decode.mem_op = MEM_STORE; id_decode.mem_size = mem_size_t'(ifid.inst[9:8]); id_decode.addr_op = ADDR_GBR_DISP;
                    id_decode.immediate = {24'd0, ifid.inst[7:0]} << ifid.inst[9:8];
                    id_decode.store_used = 1'b1; id_decode.store_id = dec_r0_id;
                end
                4'h3: begin id_decode.illegal = 1'b0; id_decode.event_trapa = 1'b1; id_decode.trapa_imm = ifid.inst[7:0]; end
                4'h4, 4'h5, 4'h6: begin
                    id_decode.illegal = 1'b0; id_decode.mem_op = MEM_LOAD; id_decode.mem_size = mem_size_t'(ifid.inst[9:8]); id_decode.addr_op = ADDR_GBR_DISP;
                    id_decode.immediate = {24'd0, ifid.inst[7:0]} << ifid.inst[9:8];
                    id_decode.gpr0_we = 1'b1; id_decode.gpr0_dst = dec_r0_id;
                end
                4'h7: begin id_decode.illegal = 1'b0; id_decode.alu_op = ALU_PASS_B; id_decode.immediate = (ifid.pc + 32'd4 & 32'hFFFF_FFFC) + {22'd0, ifid.inst[7:0], 2'b00}; id_decode.gpr0_we = 1'b1; id_decode.gpr0_dst = dec_r0_id; end
                4'h8, 4'h9, 4'hA, 4'hB: begin
                    id_decode.illegal = 1'b0; id_decode.src_a_used = 1'b1; id_decode.src_a_id = dec_r0_id; id_decode.immediate = {24'd0, ifid.inst[7:0]}; id_decode.gpr0_dst = dec_r0_id;
                    case(ifid.inst[9:8])
                        2'd0: begin id_decode.alu_op = ALU_TST; end
                        2'd1: begin id_decode.alu_op = ALU_AND; id_decode.gpr0_we = 1'b1; end
                        2'd2: begin id_decode.alu_op = ALU_XOR; id_decode.gpr0_we = 1'b1; end
                        default: begin id_decode.alu_op = ALU_OR; id_decode.gpr0_we = 1'b1; end
                    endcase
                end
                4'hC, 4'hD, 4'hE, 4'hF: begin
                    //GBR byte logic: sections 8.2.4/47/75/76, pp.148, 233-234, 279-283.
                    id_decode.illegal    = 1'b0;
                    id_decode.mem_op     = MEM_RMW;
                    id_decode.mem_size   = SIZE_BYTE;
                    id_decode.addr_op    = ADDR_GBR_INDEX;
                    id_decode.src_a_used = 1'b1;
                    id_decode.src_a_id   = dec_r0_id;
                    id_decode.immediate  = {24'd0, ifid.inst[7:0]};
                    case(ifid.inst[9:8])
                        2'd0: id_decode.byte_op = BYTE_TST;
                        2'd1: id_decode.byte_op = BYTE_AND;
                        2'd2: id_decode.byte_op = BYTE_XOR;
                        default: id_decode.byte_op = BYTE_OR;
                    endcase
                end
                default: begin end
            endcase
        end
        4'hD: begin
            //PC-relative longword load aligns the architectural PC to four bytes.
            id_decode.illegal   = 1'b0;
            id_decode.mem_op    = MEM_LOAD;
            id_decode.mem_size  = SIZE_LONG;
            id_decode.addr_op   = ADDR_PC_LONG;
            id_decode.immediate = ((ifid.pc + 32'd4) & 32'hFFFF_FFFC) +
                                  {22'd0, ifid.inst[7:0], 2'b00};
            id_decode.gpr0_we   = 1'b1;
            id_decode.gpr0_dst  = dec_n_id;
        end
        4'hE: begin
            //MOV #imm,Rn sign-extends the encoded eight-bit immediate.
            id_decode.illegal     = 1'b0;
            id_decode.alu_op      = ALU_PASS_B;
            id_decode.immediate   = {{24{ifid.inst[7]}}, ifid.inst[7:0]};
            id_decode.gpr0_we     = 1'b1;
            id_decode.gpr0_dst    = dec_n_id;
        end
        default: begin end
    endcase

    //When load destination equals post-increment base, the loaded value wins.
    if(id_decode.gpr0_we && id_decode.gpr1_we && id_decode.gpr0_dst == id_decode.gpr1_dst) begin
        id_decode.gpr1_we = 1'b0;
    end

    case(id_decode.mem_size)
        SIZE_BYTE: id_mem_step = 32'd1;
        SIZE_WORD: id_mem_step = 32'd2;
        default:   id_mem_step = 32'd4;
    endcase
    //Route the decoded operation to one of the four EX result units.
    id_decode.alu_class = alu_class_of(id_decode.alu_op);
end


///////////////////////////////////////////////////////////
//////  Hazard And Stage Control
////

/*
    A stage advances only when its work is complete and the next stage accepts.
    EX launches primary data requests before EX/MA captures.
    MA waits after request acceptance until the matching response arrives.
    A load dependency holds ID because EX/MA has no load value yet.
    Delayed branches hold EX until one sequential instruction can enter ID/EX.
    Control-register changes serialize younger instructions to preserve bank state.
*/

logic           ma_complete;
logic           ma_cpl_now;       //completes with no response (early registered products)
logic           ma_cpl_on_rsp;    //completes when the D response arrives (early product)
logic           ma_cpl_on_frsp;   //aborts on a FAULTING D response - MAC/RMW (early product)
logic           wb_valid;         //registered WB packet commits on this i_CLK_p edge
//Force the packet-register load enables onto the DFF dedicated clock-enable pin
//instead of a LUT feedback mux; see Quartus direct_enable. The loads above are now
//clean "if(allow) reg <= data" forms so the attribute can bind.
(* direct_enable *) logic exma_allow;       //EX/MA can accept a replacement packet
(* direct_enable *) logic idex_allow;       //ID/EX can accept a replacement packet
logic           id_hazard;        //decoded instruction must remain in IF/ID
logic           id_hazard_early;  //hazard terms free of ma_complete (rail-invariant)
logic           hz_ld_exma_hit;   //exma-load source match, completion NOT yet applied
logic           btbf_cancel_base; //taken BT/BF wrong-path issue cancel, exma_allow applied per rail
logic           state_hazard;     //special-register dependency requires serialization
logic           ex_complete;      //EX work and delay-slot requirement are complete
logic           id_issue;         //decoded packet transfers into ID/EX
logic           ex_advance;       //executed packet transfers into EX/MA
logic           data_response;    //accepted MA request returned data or a fault
logic           ex_data_req_valid;//EX AGU request presented before EX/MA captures
logic           ex_data_req_accept;//EX AGU request accepted (driven by u_ma_seq)
logic           early_d_req_valid;//EX primary (first) data request is meaningful
logic           wb_fault_pending; //registered fault will flush younger work this edge
logic           mac_dsp_operation;//current EX/MA packet needs the pipelined DSP
logic           id_uses_mac_state;//decoded instruction reads or writes MACH/MACL
logic           id_reads_sr;      //STC/STC.L needs every older SR bit update committed
logic           id_reads_ctrl_misc;//decoded instruction reads GBR/VBR/SSR/SPC/PR
logic           sr_write_pending;  //an older LDC ...,SR may change the active bank
logic           rte_pending;       //an older RTE restores SR and PC
logic           ctrl_misc_write_pending;//older GBR/VBR/SSR/SPC/PR write still uncommitted

logic  retire_int_defer;   //retiree was a delayed branch: slot still owed (pair atomicity)

assign data_response = data_req_sent && d_rsp_valid;
assign wb_valid      = mawb.valid;
assign wb_fault_pending = wb_valid && mawb.fault;
//Interrupt-acceptance defer (exc_handler): killing an ACCEPTED D access orphans its
//response - the held rsp_valid_d wedges the shared L-bus response channel and starves
//every later fetch - and killing between the legs of a locked RMW / MAC pair splits an
//indivisible sequence (dangling bus lock). Acceptance waits until the op leaves MA; a
//not-yet-granted request stays killable (L-bus withdrawal is legal). Registered terms.
wire   ma_inflight   = data_req_sent || (exma.valid && !exma.fault && ma_second_access);
//The pipe OWNS the whole acceptance-boundary invariant and exports ONE bit; the
//exception handler no longer reassembles it from three raw pipeline signals.
assign o_INT_BOUNDARY = o_RETIRE_VALID && !retire_int_defer && !ma_inflight;
//!i_REDIRECT_VALID: the packet in WB at an interrupt-redirect edge is KILLED (its
//retirement and every other commit lane are suppressed) - without this gate its GPR
//write leaked and the resumed instruction ran twice (interrupt-sweep golden).
assign gpr_wb0_we    = wb_valid && !i_REDIRECT_VALID && !mawb.fault && mawb.gpr0_we;
assign gpr_wb1_we    = wb_valid && !i_REDIRECT_VALID && !mawb.fault && mawb.gpr1_we;
assign gpr_wb0_dst   = mawb.gpr0_dst;
assign gpr_wb1_dst   = mawb.gpr1_dst;
assign gpr_wb0_data  = mawb.gpr0_data;
assign gpr_wb1_data  = mawb.gpr1_data;

always_comb begin
    mac_dsp_operation = (exma.mac_cmd == MAC_MULL || exma.mac_cmd == MAC_MULS_W ||
                         exma.mac_cmd == MAC_MULU_W || exma.mac_cmd == MAC_DMULS_L ||
                         exma.mac_cmd == MAC_DMULU_L || exma.mac_cmd == MAC_ACCUM_L ||
                         exma.mac_cmd == MAC_ACCUM_W);

    //FLATTENED completion: the late cen_n terms (data_response, d_rsp_fault) enter at ONE
    //final OR plane; every product below is a registered-field compare (early). Exactly the
    //old priority tree: TST.B retires on its evaluate slot (Fig 10.36, MA EX MA); other RMW
    //ops need the phase-two write response; a faulting response aborts MAC/RMW immediately;
    //DSP ops hold MA for the multiply; a store is a NOTIFY (commits at accept, no response).
    ma_cpl_now    = !exma.valid || exma.fault ||
                    (exma.mem_op == MEM_RMW && exma.byte_op == BYTE_TST && ma_second_access) ||
                    (exma.mem_op != MEM_RMW && mac_dsp_operation && mac_dsp_done) ||
                    (exma.mem_op != MEM_RMW && !mac_dsp_operation &&
                     (exma.mem_op == MEM_NONE || exma.mem_op == MEM_STORE));
    ma_cpl_on_rsp = (exma.mem_op == MEM_RMW && exma.byte_op != BYTE_TST && ma_second_access) ||
                    (exma.mem_op != MEM_RMW && !mac_dsp_operation &&
                     exma.mem_op != MEM_NONE && exma.mem_op != MEM_STORE);
    ma_cpl_on_frsp= (exma.mem_op == MEM_MAC || exma.mem_op == MEM_RMW);
end

//Completion: the FLATTENED early products above, one final OR plane with the live
//response terms (a hit response is combinational off the cache resolve this cycle).
assign  ma_complete = ma_cpl_now || (ma_cpl_on_rsp  && data_response) ||
                      (ma_cpl_on_frsp && data_response && d_rsp_fault);
assign  exma_allow  = !exma.valid || ma_complete;

always_comb begin
    //Only two cases serialize every younger instruction: an LDC ...,SR (it may
    //change SR.MD/RB and thus the active register bank, see int_pipe.md sec 2.1)
    //and an RTE (it restores SR and PC). Software manual 10.2.3, p.432 lists no
    //general stall behind GBR/VBR/SSR/SPC/PR writes, so those hold only the rare
    //instructions that actually read that state. The idex branch-delayed packet
    //is excluded: its delay slot must always issue and cannot serialize on it.
    sr_write_pending = (idex.valid && !idex.branch_delayed && idex.ctrl_dst == CTRL_SR) ||
                       (exma.valid && exma.ctrl_dst == CTRL_SR) ||
                       (mawb.valid && mawb.ctrl_dst == CTRL_SR);
    rte_pending      = (idex.valid && !idex.branch_delayed && idex.event_rte) ||
                       (exma.valid && exma.event_rte) ||
                       (mawb.valid && mawb.event_rte);
    //A non-NONE, non-SR ctrl_dst is one of GBR/VBR/SSR/SPC; pr_we is the PR write.
    ctrl_misc_write_pending =
        (idex.valid && !idex.branch_delayed &&
            ((idex.ctrl_dst != CTRL_NONE && idex.ctrl_dst != CTRL_SR) || idex.pr_we)) ||
        (exma.valid &&
            ((exma.ctrl_dst != CTRL_NONE && exma.ctrl_dst != CTRL_SR) || exma.pr_we)) ||
        (mawb.valid &&
            ((mawb.ctrl_dst != CTRL_NONE && mawb.ctrl_dst != CTRL_SR) || mawb.pr_we));
    //Readers of GBR/VBR/SSR/SPC/PR: PRE-DECODED at fetch time (pd.reads_cmisc; the
    //live ifid.inst decode sat on the id_hazard -> id_issue -> IF/ID CE cone).
    id_reads_ctrl_misc = ifid.pd.reads_cmisc;
    state_hazard = sr_write_pending || rte_pending ||
                   (id_reads_ctrl_misc && ctrl_misc_write_pending);

    //Check all semantic sources: arithmetic A/B and independent store data.
    //SR-state and MAC-state interlock conditions are decoded first, then every
    //hazard term is OR-reduced in one flat expression so the tool builds a
    //balanced reduction tree rather than the 9-deep mux chain a sequential
    //id_hazard |= ... waterfall infers. id_hazard gates id_issue and ex_advance,
    //which sit on the global EX/MA register critical path, so depth here matters.
    id_reads_sr       = ifid.pd.reads_sr;      //pre-decoded at fetch (see pd_route)
    id_uses_mac_state = ifid.pd.uses_mac;      //pre-decoded at fetch (see pd_route)
    //Rail split: the exma-load interlock is the ONLY hazard term carrying ma_complete (the
    //cache late bits). Its registered-compare product (hz_ld_exma_hit) is factored out here;
    //the rail block ANDs it with !ma_complete per rail. Everything in id_hazard_early is a
    //registered-field product - identical Boolean, one distribution step.
    //The old (ifid.valid && !gpr_read_ready) term is DELETED as provably constant-0: after
    //the clear-path removal, ready was a bare cen_n resample of ifid.valid, and with strict
    //cen_p/cen_n alternation every cen_p consumer edge sees them equal (the cen_n reload
    //between any two cen_p edges reads the already-updated ifid.valid). Its live content
    //(ports_ready) died when the two-line read-port count guard proved constant.
    id_hazard_early = |{
        state_hazard,
        id_reads_sr && ((idex.valid && (idex.t_write_decode || idex.s_write_decode ||
                                        idex.div_op != DIV_NONE)) ||
                        (exma.valid && (exma.t_we || exma.s_we || |exma.mq_we)) ||
                        (mawb.valid && (mawb.t_we || mawb.s_we || |mawb.mq_we))),
        id_uses_mac_state && ((idex.valid && idex.mac_cmd != MAC_NONE) ||
                              (exma.valid && exma.mac_cmd != MAC_NONE) ||
                              (mawb.valid && mawb.mac_cmd != MAC_NONE)),
        //Load-use interlock reads the PREDECODED source identities (ifid.pd/hz_*_id), so the
        //~3-level ID decode leaves this cone - the decode->issue->cache victim_tag limiter.
        source_matches_load(ifid.pd.a_used, hz_a_id,
                            idex.valid && idex.mem_op == MEM_LOAD,
                            idex.gpr0_we, idex.gpr0_dst),
        source_matches_load(ifid.pd.b_used, hz_b_id,
                            idex.valid && idex.mem_op == MEM_LOAD,
                            idex.gpr0_we, idex.gpr0_dst),
        source_matches_load(ifid.pd.st_used, hz_st_id,
                            idex.valid && idex.mem_op == MEM_LOAD,
                            idex.gpr0_we, idex.gpr0_dst)
    };
    hz_ld_exma_hit = |{
        source_matches_load(ifid.pd.a_used, hz_a_id,
                            exma.valid && exma.mem_op == MEM_LOAD,
                            exma.gpr0_we, exma.gpr0_dst),
        source_matches_load(ifid.pd.b_used, hz_b_id,
                            exma.valid && exma.mem_op == MEM_LOAD,
                            exma.gpr0_we, exma.gpr0_dst),
        source_matches_load(ifid.pd.st_used, hz_st_id,
                            exma.valid && exma.mem_op == MEM_LOAD,
                            exma.gpr0_we, exma.gpr0_dst)
    };
    //Taken BT/BF cancels the wrong-path issue; the exma_allow qualifier is applied per rail.
    btbf_cancel_base = idex.valid && (idex.branch_op == BR_BT || idex.branch_op == BR_BF) &&
                       !idex.branch_delayed && branch_taken;
end

//A delayed branch cannot leave EX without securing its sequential packet; a memory op
//cannot leave EX until the cache accepts the live AGU request (ex_data_req_accept).
assign  id_hazard  = id_hazard_early || (hz_ld_exma_hit && !ma_complete);
assign  ex_complete= (!idex.valid || !idex.branch_delayed ||
                      (ifid.valid && !id_hazard)) &&
                     (!early_d_req_valid || ex_data_req_accept);
assign  idex_allow = !idex.valid || (ex_complete && exma_allow);
//Per-cluster duplicates of idex_allow: the single net fans to ~266 loads (every idex
//bit) and its one LUT paid ~2 ns of cross-die routing into the operand captures. keep
//blocks the merge so the fitter places one copy at each 32-bit capture cluster.
//D-cones are identical, so the split loads below stay bit-exact.
(* keep *) wire idex_allow_opa = !idex.valid || (ex_complete && exma_allow);
(* keep *) wire idex_allow_opb = !idex.valid || (ex_complete && exma_allow);
//Kill-product duplicate for the issue/advance cluster: wb_fault_pending is one merged
//LUT with ~1060 loads placed at the whole-pipe centroid (fit4 fanout table).
(* keep *) wire wb_kill_issue = wb_valid && mawb.fault;
assign  id_issue   = ifid.valid && idex_allow && !id_hazard &&
                     !fault_hold && !wb_kill_issue &&
                     !(btbf_cancel_base && exma_allow);
assign  ex_advance = idex.valid && ex_complete && exma_allow && !wb_kill_issue;

logic   [31:0]  id_src_a_value, id_src_b_value, id_store_value;
logic   [31:0]  id_mem_step;     //decoded transfer byte count for predecrement


//===========================================================================================
//  Operand select - LAST-LEVEL flat mux: every LATE word crosses exactly ONE level
//===========================================================================================
//TWO late data sources remain: the GPR BRAM read words (DOA/DOB). The old third late
//word (the MA load ld_word) and the LIVE EX-result tail no longer enter this mux at
//all - they moved to the EX-head lanes (fwd_lane_*): the load word DEPOSITS into a
//shadow register at its completion edge, and the EX producer's result is read from
//the EX/MA packet one cycle later. Every early candidate here (MA forward, WB lanes,
//R0 mirrors, GBR/PC/imm/PREDEC overrides) is a REGISTERED field, so this cone carries
//no ALU or cache term. Final mux per source: {DOA, DOB, early} - one ALM level;
//the DOA/DOB routing compares hz ids against the live read addresses (1 LUT).

//MA (non-load) forward shadows - the "did the MA producer already win" selects.
//The EX live tail and the MA load word are GONE from this mux (EX-head lanes).
wire        ma_take_a  = ma_take_only(ifid.pd.a_used,  hz_a_id,  exma);
wire        ma_take_b  = ma_take_only(ifid.pd.b_used,  hz_b_id,  exma);
wire        ma_take_st = ma_take_only(ifid.pd.st_used, hz_st_id, exma);

//EX-head lane picks (registered compares; latched into fwd_lane_* at issue). The
//address-op overrides mirror the sel_* steering below: GBR/PC bases never patch
//(source a), GBR-INDEX rides source A's RAW pick (the R0 forward), and the
//PREDEC / immediate legs never patch.
fwd_lane_t  id_lane_a_raw, id_lane_a, id_lane_b, id_lane_st;
assign  id_lane_a_raw = fwd_lane_pick(ifid.pd.a_used, hz_a_id, idex, exma);
assign  id_lane_a  = (ifid.pd.agbr || ifid.pd.apc) ? FWD_NONE : id_lane_a_raw;
assign  id_lane_b  = ifid.pd.pdec ? FWD_NONE :
                     ifid.pd.gbrx ? id_lane_a_raw :
                                    fwd_lane_pick(ifid.pd.b_used, hz_b_id, idex, exma);
assign  id_lane_st = (ifid.pd.gbrx || !ifid.pd.st_used) ? FWD_NONE :
                     fwd_lane_pick(ifid.pd.st_used, hz_st_id, idex, exma);

//WB-lane in-flight overrides: the LIVE lanes (writing the file at the coming edge) plus
//the one-cycle SHADOW lanes (wrote at the last edge - the same edge this read's address
//was captured, so the RAM q excludes them). Any hit here must steer the source OFF the
//RAM legs onto the early residue below.
//PER-PORT (* keep *) lane-enable duplicates (idex_allow_op* pattern): the shared
//gpr_wb*_we LUTs served the RAM write ports, the shadow captures, and all three
//hit/base clusters from one placement centroid (fit4 mawb.valid -> src_b class); one
//pair per port keeps each cluster's enables local. D-cones identical = bit-exact.
(* keep *) wire wb0_we_opa  = wb_valid && !mawb.fault && mawb.gpr0_we;
(* keep *) wire wb1_we_opa  = wb_valid && !mawb.fault && mawb.gpr1_we;
(* keep *) wire wb0_we_opb  = wb_valid && !mawb.fault && mawb.gpr0_we;
(* keep *) wire wb1_we_opb  = wb_valid && !mawb.fault && mawb.gpr1_we;
(* keep *) wire wb0_we_opst = wb_valid && !mawb.fault && mawb.gpr0_we;
(* keep *) wire wb1_we_opst = wb_valid && !mawb.fault && mawb.gpr1_we;
wire        wb_hit_a   = (ifid.pd.a_used  && wb0_we_opa  && gpr_wb0_dst == hz_a_id) ||
                         (ifid.pd.a_used  && wb1_we_opa  && gpr_wb1_dst == hz_a_id) ||
                         (ifid.pd.a_used  && wb0z_we     && wb0z_dst    == hz_a_id) ||
                         (ifid.pd.a_used  && wb1z_we     && wb1z_dst    == hz_a_id);
wire        wb_hit_b   = (ifid.pd.b_used  && wb0_we_opb  && gpr_wb0_dst == hz_b_id) ||
                         (ifid.pd.b_used  && wb1_we_opb  && gpr_wb1_dst == hz_b_id) ||
                         (ifid.pd.b_used  && wb0z_we     && wb0z_dst    == hz_b_id) ||
                         (ifid.pd.b_used  && wb1z_we     && wb1z_dst    == hz_b_id);
wire        wb_hit_st  = (ifid.pd.st_used && wb0_we_opst && gpr_wb0_dst == hz_st_id) ||
                         (ifid.pd.st_used && wb1_we_opst && gpr_wb1_dst == hz_st_id) ||
                         (ifid.pd.st_used && wb0z_we     && wb0z_dst    == hz_st_id) ||
                         (ifid.pd.st_used && wb1z_we     && wb1z_dst    == hz_st_id);

//EARLY base residue: WB lanes newest-first (live wb1 > live wb0 > shadow wb1 > shadow
//wb0), then the R0 bank mirrors; 0 when nothing carries the id. Lane enables ride the
//same per-port duplicates as the hit selects (one cluster per port).
wire [31:0] early_base_a  = (ifid.pd.a_used  && wb1_we_opa  && gpr_wb1_dst == hz_a_id)  ? gpr_wb1_data :
                            (ifid.pd.a_used  && wb0_we_opa  && gpr_wb0_dst == hz_a_id)  ? gpr_wb0_data :
                            (ifid.pd.a_used  && wb1z_we     && wb1z_dst    == hz_a_id)  ? wb1z_data :
                            (ifid.pd.a_used  && wb0z_we     && wb0z_dst    == hz_a_id)  ? wb0z_data :
                            (hz_a_id  == 5'd0) ? gpr_r0_bank0 :
                            (hz_a_id  == 5'd8) ? gpr_r0_bank1 : 32'd0;
wire [31:0] early_base_b  = (ifid.pd.b_used  && wb1_we_opb  && gpr_wb1_dst == hz_b_id)  ? gpr_wb1_data :
                            (ifid.pd.b_used  && wb0_we_opb  && gpr_wb0_dst == hz_b_id)  ? gpr_wb0_data :
                            (ifid.pd.b_used  && wb1z_we     && wb1z_dst    == hz_b_id)  ? wb1z_data :
                            (ifid.pd.b_used  && wb0z_we     && wb0z_dst    == hz_b_id)  ? wb0z_data :
                            (hz_b_id  == 5'd0) ? gpr_r0_bank0 :
                            (hz_b_id  == 5'd8) ? gpr_r0_bank1 : 32'd0;
wire [31:0] early_base_st = (ifid.pd.st_used && wb1_we_opst && gpr_wb1_dst == hz_st_id) ? gpr_wb1_data :
                            (ifid.pd.st_used && wb0_we_opst && gpr_wb0_dst == hz_st_id) ? gpr_wb0_data :
                            (ifid.pd.st_used && wb1z_we     && wb1z_dst    == hz_st_id) ? wb1z_data :
                            (ifid.pd.st_used && wb0z_we     && wb0z_dst    == hz_st_id) ? wb0z_data :
                            (hz_st_id == 5'd0) ? gpr_r0_bank0 :
                            (hz_st_id == 5'd8) ? gpr_r0_bank1 : 32'd0;

//EARLY value per source: MA > base residue. The EX live tail is GONE from here
//(EX-head lanes read the registered exma packet at the consumer's own EX cycle),
//so every input of this mux is a REGISTERED field - no ALU/cache cone remains.
wire [31:0] early_src_a  = ma_forward(early_base_a,  ifid.pd.a_used,  hz_a_id,  exma);
wire [31:0] early_src_b  = ma_forward(early_base_b,  ifid.pd.b_used,  hz_b_id,  exma);
wire [31:0] early_src_st = ma_forward(early_base_st, ifid.pd.st_used, hz_st_id, exma);

//BRAM word routing: which read port carries this source id (port A wins on a double
//match, = decoded_gpr_value's order). Compares run against the LIVE read addresses, NOT
//a cen_n context capture: the old gpr_read_ctx regs resampled gpr_read0/1_address at
//cen_n from an ifid/SR that cannot change again before the consuming cen_p edge, so
//ctx == live there ALWAYS (the gpr_read_ready argument). Dropping the regs moves these
//selects onto the full 10 ns cen_p budget and deletes the launch class they anchored.
wire        doa_hit_a  = hz_a_id  == gpr_read0_address;
wire        dob_hit_a  = hz_a_id  == gpr_read1_address;
wire        doa_hit_b  = hz_b_id  == gpr_read0_address;
wire        dob_hit_b  = hz_b_id  == gpr_read1_address;
wire        doa_hit_st = hz_st_id == gpr_read0_address;
wire        dob_hit_st = hz_st_id == gpr_read1_address;

//Source resolution WITHOUT the addr_op overrides (doa > dob > early; an MA-forward
//or WB hit falls through to the early leg). An EX-forward or load hit no longer
//steers here: the stale RAM/early word lands in idex.src_* and the EX-head lane
//overrides it at the consumer's EX, so this cone carries no idex/ex compare.
wire        src_doa_a  = !ma_take_a  && !wb_hit_a  && doa_hit_a;
wire        src_dob_a  = !ma_take_a  && !wb_hit_a  && !doa_hit_a  && dob_hit_a;
wire        src_doa_b  = !ma_take_b  && !wb_hit_b  && doa_hit_b;
wire        src_dob_b  = !ma_take_b  && !wb_hit_b  && !doa_hit_b  && dob_hit_b;
wire        src_doa_st = !ma_take_st && !wb_hit_st && doa_hit_st;
wire        src_dob_st = !ma_take_st && !wb_hit_st && !doa_hit_st && dob_hit_st;

//addr_op overrides. GBR-INDEX redirects src_b onto SOURCE A's resolution (the R0 forward,
//old ovr_b_val=fwd_result_a) - done by steering b's SELECTS to a's, so no extra level.
//All REGISTERED pd bits (fetch-time addr-op class, asserted above): the id_decode.addr_op
//forms kept the live ifid.inst case cone on every sel_a/sel_b/sel_st leg (fit4).
wire        addr_a_gbr  = ifid.pd.agbr;
wire        addr_a_pc   = ifid.pd.apc;
wire        addr_a_ovr  = addr_a_gbr || addr_a_pc;
wire        addr_b_gbrx = ifid.pd.gbrx;
wire        addr_b_pdec = ifid.pd.pdec;
wire        st_use_imm  = ifid.pd.gbrx || !ifid.pd.st_used;

//Final-mux selects (2-bit priority encoders, EARLY) and the collapsed early legs.
//The ld_word input is GONE (2'd0 unreachable): the load word deposits into the
//EX-head shadow register instead, so only the two GPR BRAM words remain late.
wire [1:0]  sel_a  = (!addr_a_ovr && src_doa_a) ? 2'd1 :
                     (!addr_a_ovr && src_dob_a) ? 2'd2 : 2'd3;
wire [31:0] early_a_final  = addr_a_gbr ? i_GBR :
                             addr_a_pc  ? 32'd0 : early_src_a;    //PC-rel addr rides the immediate

wire [1:0]  sel_b  = addr_b_pdec                                          ? 2'd3 :
                     (addr_b_gbrx ? src_doa_a : ifid.pd.b_used && src_doa_b) ? 2'd1 :
                     (addr_b_gbrx ? src_dob_a : ifid.pd.b_used && src_dob_b) ? 2'd2 : 2'd3;
wire [31:0] early_b_final  = addr_b_pdec    ? (~id_mem_step + 32'd1) :    //-step for @-Rn
                             addr_b_gbrx    ? early_src_a :
                             ifid.pd.b_used ? early_src_b : id_decode.immediate;

wire [1:0]  sel_st = (!st_use_imm && src_doa_st) ? 2'd1 :
                     (!st_use_imm && src_dob_st) ? 2'd2 : 2'd3;
wire [31:0] early_st_final = st_use_imm ? id_decode.immediate : early_src_st;

//Flat muxes as EXPLICIT case statements (one always_comb per source) so each maps to a
//single ALM level per bit. BOTH late words (the GPR BRAM ports) are direct data inputs.
always_comb begin
    case(sel_a)
        2'd1:    id_src_a_value = gpr_read_data_a;  //GPR BRAM port A (late)
        2'd2:    id_src_a_value = gpr_read_data_b;  //GPR BRAM port B (late)
        default: id_src_a_value = early_a_final;    //MA-forward/WB/mirrors/GBR/PC (early)
    endcase
end
always_comb begin
    case(sel_b)
        2'd1:    id_src_b_value = gpr_read_data_a;  //GPR BRAM port A (late)
        2'd2:    id_src_b_value = gpr_read_data_b;  //GPR BRAM port B (late)
        default: id_src_b_value = early_b_final;    //PREDEC step/MA-forward/imm (early)
    endcase
end
always_comb begin
    case(sel_st)
        2'd1:    id_store_value = gpr_read_data_a;  //GPR BRAM port A (late)
        2'd2:    id_store_value = gpr_read_data_b;  //GPR BRAM port B (late)
        default: id_store_value = early_st_final;   //MA-forward/imm (early)
    endcase
end


///////////////////////////////////////////////////////////
//////  Execute
////

/*
    EX consumes operand values registered in ID/EX on E2.
    ALU means arithmetic logic unit; A/B are its semantic operand buses.
    T forwarding supports compare-to-branch and carry-dependent instructions.
    Instruction operations follow manual tables 2.6-2.10, pp.38-46.
*/

logic   [31:0]  ex_a, ex_b, ex_store; //registered operands and store data
logic           ex_t;                 //newest visible SR.T value (= r_t, the running flag)
logic           r_t;                  //running SR.T register read by EX; latched at EX/MA
logic           ex_s;                 //newest visible SR.S saturation control
logic           r_s;                  //running SR.S register read by EX/MAC; latched at EX/MA
logic           ex_m, ex_q;           //newest visible SR.M/Q division state
logic           r_m, r_q;             //running SR.M/Q registers read by EX/DIV1; latched at EX/MA

//Four parallel DEU units; a 4-to-1 mux on alu_class drives the captured result.
logic   [31:0]  arith_result;         //adder/subtractor group; shares one carry chain
logic   [31:0]  logic_result;         //bitwise group
logic   [31:0]  shift_result;         //constant shifts plus the SHAD/SHLD barrel shifter
logic   [31:0]  misc_result;          //bypass and byte-steering group
logic   [31:0]  alu_result;           //selected EX result captured into EX/MA

//T candidates from each producing unit; the T selector picks the active one.
logic           arith_t_we, arith_t_value; //carry, overflow, or zero from the adder
logic           shift_t_we, shift_t_value; //bit shifted out of the shifter
logic           ceu_t_we, ceu_t_value;     //compare and TST results
logic           div_t_we, div_t_value;     //DIV initialization or step result
logic           alu_t_we, alu_t_value;     //final SR.T update for this instruction
logic   [32:0]  alu_wide;             //extra bit captures addition carry
logic   [31:0]  div_result;            //DIV1 shifted add/subtract result
logic   [32:0]  div_wide;              //carry or no-borrow from DIV1 carry chain
logic   [32:0]  div_wide_t0, div_wide_t1; //duplicated DIV1 sums; area for late T mux
logic   [32:0]  addc_wide_t0, addc_wide_t1; //duplicated ADDC sums; area for late T mux
logic   [32:0]  subc_wide_t0, subc_wide_t1; //duplicated SUBC sums; area for late T mux
logic   [32:0]  negc_wide_t0, negc_wide_t1; //duplicated NEGC sums; area for late T mux
logic   [1:0]   div_mq_we, div_mq_data;//SR.M/Q updates from divide instructions
logic           div_add, div_event;    //DIV1 operation direction and carry/borrow event

logic   [31:0]  effective_addr;        //first byte address sent to MA
logic   [31:0]  effective_addr_second; //second MAC.W/MAC.L byte address
logic   [31:0]  address_update;        //first post-increment result
logic   [31:0]  address_update_second; //second post-increment result
logic           address_error;        //word or longword alignment violation
logic   [31:0]  aligned_store_data;   //store value replicated or lane-aligned
logic   [3:0]   store_strobe;         //one bit enables each byte lane
logic   [1:0]   store_lane;           //low address bits used for store strobes
logic   [31:0]  ea_step;              //transfer byte count: 1, 2, or 4
logic   [31:0]  ea_addr_base;         //selected base for the primary cache address
logic   [31:0]  ea_addr_addend;       //selected addend for the primary cache address
logic   [31:0]  ea_addr_sum;          //single primary-address carry chain
logic   [31:0]  ea_update_addend;     //post/pre update increment for writeback
logic           early_ex_fault;       //fault condition known before request capture

logic           early_d_req_base;     //ditto without the exma_allow leg (rail-invariant)
dbus_req_pkt_t  ex_req;               //EX primary request handed to the MA sequencer
dbus_req_pkt_t  early_bus_d_req;      //unified D request from u_ma_seq (EX or second access)
logic           ma_flush;             //redirect or recognized WB fault clears the sequencer
logic           ma_active;            //held MA packet may drive a second access
logic   [31:0]  ma_capture_value;     //first-response value latched by the sequencer
//SHAD/SHLD dynamic shifter; one logarithmic right barrel shared via bit reversal.
logic   [4:0]   shdyn_amt;            //right-shift magnitude 0..31 driving the barrel
logic           shdyn_left;           //Rm>=0 selects a left shift (reversed operand)
logic           shdyn_fill;           //bit shifted in: sign for SHAD right, else zero
logic           shdyn_full32;         //Rm<0 with Rm[4:0]==0 means a full 32-bit shift
logic   [31:0]  shdyn_in;             //operand entering the right barrel, pre-reversed
logic   [31:0]  shdyn_s0, shdyn_s1, shdyn_s2, shdyn_s3, shdyn_s4; //barrel stages
logic   [31:0]  shdyn_shifted;        //barrel output before output reversal
logic   [31:0]  shdyn_result;         //final SHAD/SHLD result in natural bit order

//EX-head operand patch - the registered forward lanes replace the old ID-mux LIVE
//legs (the ex_result ALU tail and the ld_word aligner).
//WB-view leg per port: the producer's word read from the registered mawb packet on
//its one live cycle (lane FWD_WB at issue = load release; or a G0/G1 producer that
//drained under a held consumer - fwd_wbsel_* REGISTERS that decision at the drain
//edge), then from the deposited shadow. Selects are single FFs and the data legs
//are FFs, so the hot G0/G1/idex legs cross ONE 4:1 level and the WB leg two.
wire [31:0] fwd_mawb_word_a  = fwd_lane_a  == FWD_EXMA_G1 ? mawb.gpr1_data : mawb.gpr0_data;
wire [31:0] fwd_mawb_word_b  = fwd_lane_b  == FWD_EXMA_G1 ? mawb.gpr1_data : mawb.gpr0_data;
wire [31:0] fwd_mawb_word_st = fwd_lane_st == FWD_EXMA_G1 ? mawb.gpr1_data : mawb.gpr0_data;
wire [31:0] fwd_wb_a  = fwd_dep_a  ? fwd_shadow_a  : fwd_mawb_word_a;
wire [31:0] fwd_wb_b  = fwd_dep_b  ? fwd_shadow_b  : fwd_mawb_word_b;
wire [31:0] fwd_wb_st = fwd_dep_st ? fwd_shadow_st : fwd_mawb_word_st;
//AGU-cluster twins of the a/b WB folds, from the (* preserve *) duplicates, so
//the address mux places at the adder (32-bit data legs are shared nets).
wire [31:0] agu_wb_a = fwd_dep_a_agu ? fwd_shadow_a :
                       (fwd_lane_a_agu == FWD_EXMA_G1 ? mawb.gpr1_data : mawb.gpr0_data);
wire [31:0] agu_wb_b = fwd_dep_b_agu ? fwd_shadow_b :
                       (fwd_lane_b_agu == FWD_EXMA_G1 ? mawb.gpr1_data : mawb.gpr0_data);
always_comb begin
    case(fwd_wbsel_a ? FWD_WB : fwd_lane_a)
        FWD_EXMA_G0: ex_a = exma.gpr0_data;      //producer result, one stage ahead
        FWD_EXMA_G1: ex_a = exma.gpr1_data;      //producer address update
        FWD_WB:      ex_a = fwd_wb_a;            //mawb word / deposited shadow
        default:     ex_a = idex.src_a_value;    //ID-resolved operand
    endcase
    case(fwd_wbsel_b ? FWD_WB : fwd_lane_b)
        FWD_EXMA_G0: ex_b = exma.gpr0_data;
        FWD_EXMA_G1: ex_b = exma.gpr1_data;
        FWD_WB:      ex_b = fwd_wb_b;
        default:     ex_b = idex.src_b_value;
    endcase
    case(fwd_wbsel_st ? FWD_WB : fwd_lane_st)
        FWD_EXMA_G0: ex_store = exma.gpr0_data;
        FWD_EXMA_G1: ex_store = exma.gpr1_data;
        FWD_WB:      ex_store = fwd_wb_st;
        default:     ex_store = idex.store_value;
    endcase
end

always_comb begin
    //Forward the older EX/MA T value, including a completing byte-memory test.
    //SR.T for EX is a registered running flag (r_t), latched at the EX/MA boundary as each
    //T-writing instruction leaves EX (see the pipeline state block). The next EX reads it as
    //a plain register - no forward mux on the EX T path / DIV1 carry chain - and it persists
    //across bubbles, the case the old i_SR+forward missed (the cacheable DT;BF off-by-one).
    ex_t = r_t;
    //Byte test/TAS forward the registered original byte during the evaluate slot
    //(ma_second_access), never the live cache read. This keeps the cache way-select
    //cone off the EX T path (and off the DIV1 carry chain); T still commits in WB.
    //See Fig 10.36/10.37: the loaded byte is captured, then evaluated one slot later.
    if(exma.valid && !exma.fault && exma.mem_op == MEM_RMW && ma_second_access) begin
        if(exma.byte_op == BYTE_TST) ex_t = (ma_first_value[7:0] & exma.mac_b[7:0]) == 8'd0;
        if(exma.byte_op == BYTE_TAS) ex_t = ma_first_value[7:0] == 8'd0;
    end
end

always_comb begin
    //SR.M/Q for EX are running registers (r_m/r_q), latched at the EX/MA boundary like r_t -
    //no forward mux on the DIV1 carry-chain input (ex_m/ex_q pick add-vs-subtract via div_add)
    //and they hold across bubbles, the 2-ahead hole the old i_SR+exma forward missed.
    //DIV0S/DIV0U initialize M/Q/T; DIV1 is one shifted add/subtract carry chain (compute below).
    ex_m = r_m;
    ex_q = r_q;

    div_result  = ex_a;
    div_add     = ex_q ^ ex_m;
    div_wide_t0 = div_add ? {1'b0, ex_a[30:0], 1'b0} + {1'b0, ex_b} :
                            {1'b0, ex_a[30:0], 1'b0} + {1'b0, ~ex_b} + 33'd1;
    div_wide_t1 = div_add ? {1'b0, ex_a[30:0], 1'b1} + {1'b0, ex_b} :
                            {1'b0, ex_a[30:0], 1'b1} + {1'b0, ~ex_b} + 33'd1;
    div_wide    = ex_t ? div_wide_t1 : div_wide_t0;
    div_event   = div_add ? div_wide[32] : !div_wide[32];
    div_mq_we   = 2'b00;
    div_mq_data = {ex_m, ex_q};
    div_t_we    = 1'b0;
    div_t_value = ex_t;

    case(idex.div_op)
        DIV_INIT_UNSIGNED: begin
            div_mq_we   = 2'b11;
            div_mq_data = 2'b00;
            div_t_we    = 1'b1;
            div_t_value = 1'b0;
        end
        DIV_INIT_SIGNED: begin
            div_mq_we   = 2'b11;
            div_mq_data = {ex_b[31], ex_a[31]};
            div_t_we    = 1'b1;
            div_t_value = ex_b[31] ^ ex_a[31];
        end
        DIV_STEP: begin
            div_result     = div_wide[31:0];
            div_mq_we      = 2'b01;
            //Manual p.174 captures Q from Rn[31] before shifting Rn.
            div_mq_data[0] = div_event ^ ex_a[31] ^ ex_m;
            div_t_we       = 1'b1;
            div_t_value    = div_mq_data[0] == ex_m;
        end
        default: begin end
    endcase
end

always_comb begin
    //SR.S for EX is a running register (r_s), latched at the EX/MA boundary like r_t; MAC.W/
    //MAC.L read it with no forward mux (keeps the {i_SR,exma} select off the saturation path).
    ex_s = r_s;
end

always_comb begin
    //ARITH unit: the ADD/SUB family, NEG, and DT all reuse one carry chain.
    //ADDC/SUBC/NEGC fold SR.T into the carry; ADDV/SUBV detect signed overflow.
    arith_result  = ex_a + ex_b;
    arith_t_we    = 1'b0;
    arith_t_value = 1'b0;
    alu_wide      = 33'd0;
    addc_wide_t0  = {1'b0, ex_a} + {1'b0, ex_b};
    addc_wide_t1  = {1'b0, ex_a} + {1'b0, ex_b} + 33'd1;
    subc_wide_t0  = {1'b0, ex_a} + {1'b0, ~ex_b} + 33'd1;
    subc_wide_t1  = {1'b0, ex_a} + {1'b0, ~ex_b};
    negc_wide_t0  = {1'b0, 32'd0} + {1'b0, ~ex_a} + 33'd1;
    negc_wide_t1  = {1'b0, 32'd0} + {1'b0, ~ex_a};
    case(idex.alu_op)
        ALU_ADD: arith_result = ex_a + ex_b;
        ALU_ADDC: begin
            //The 33rd sum bit becomes T; see ADDC in manual table 2.7.
            alu_wide      = ex_t ? addc_wide_t1 : addc_wide_t0;
            arith_result  = alu_wide[31:0];
            arith_t_we    = 1'b1;
            arith_t_value = alu_wide[32];
        end
        ALU_ADDV: begin
            arith_result  = ex_a + ex_b;
            arith_t_we    = 1'b1;
            arith_t_value = (~(ex_a[31] ^ ex_b[31])) & (ex_a[31] ^ arith_result[31]);
        end
        ALU_SUB: arith_result = ex_a - ex_b;
        ALU_SUBC: begin
            alu_wide      = ex_t ? subc_wide_t1 : subc_wide_t0;
            arith_result  = alu_wide[31:0];
            arith_t_we    = 1'b1;
            arith_t_value = !alu_wide[32];
        end
        ALU_SUBV: begin
            arith_result  = ex_a - ex_b;
            arith_t_we    = 1'b1;
            arith_t_value = (ex_a[31] ^ ex_b[31]) & (ex_a[31] ^ arith_result[31]);
        end
        ALU_NEG:  arith_result = 32'd0 - ex_a;
        ALU_NEGC: begin
            alu_wide      = ex_t ? negc_wide_t1 : negc_wide_t0;
            arith_result  = alu_wide[31:0];
            arith_t_we    = 1'b1;
            arith_t_value = ex_a != 32'd0 || ex_t;
        end
        ALU_DT: begin
            arith_result  = ex_a - 32'd1;
            arith_t_we    = 1'b1;
            arith_t_value = arith_result == 32'd0;
        end
        ALU_DIV1: arith_result = div_result;
        default: arith_result = ex_a + ex_b;
    endcase
end

always_comb begin
    //LOGIC unit: each bitwise op is one 6-LUT deep with no carry chain.
    case(idex.alu_op)
        ALU_OR:  logic_result = ex_a | ex_b;
        ALU_XOR: logic_result = ex_a ^ ex_b;
        ALU_NOT: logic_result = ~ex_a;
        default: logic_result = ex_a & ex_b; //ALU_AND
    endcase
end

/*
    SHAD/SHLD dynamic shifter. A naive case infers four separate 32-bit barrels
    (<<<, >>>, <<, >>) and muxes them, the dominant EX critical path. Instead one
    logarithmic right barrel serves every case: left shifts feed the bit-reversed
    operand and reverse the result, so only five 2:1 mux stages sit on the path.
    Negative Rm right-shifts by magnitude (~Rm[4:0]+1); Rm<0 with Rm[4:0]==0 is a
    full 32-bit shift producing pure fill. See software manual pp.244 (SHAD), 248.
*/
always_comb begin
    shdyn_left   = ~ex_b[31];                       //Rm>=0 selects a left shift
    shdyn_full32 = ex_b[31] & (ex_b[4:0] == 5'd0);  //Rm<0, low bits zero -> shift by 32
    shdyn_amt    = ex_b[31] ? ((~ex_b[4:0]) + 5'd1) : ex_b[4:0];
    //SHAD right shift fills the sign bit; left shifts and SHLD always fill zero.
    shdyn_fill   = (idex.alu_op == ALU_SHAD) & ex_b[31] & ex_a[31];

    //Left shifts reuse the right barrel by reversing the operand into it.
    shdyn_in = shdyn_left ? shift_reverse32(ex_a) : ex_a;

    shdyn_s0 = shdyn_amt[0] ? {{1{shdyn_fill}},  shdyn_in[31:1]}  : shdyn_in;
    shdyn_s1 = shdyn_amt[1] ? {{2{shdyn_fill}},  shdyn_s0[31:2]}  : shdyn_s0;
    shdyn_s2 = shdyn_amt[2] ? {{4{shdyn_fill}},  shdyn_s1[31:4]}  : shdyn_s1;
    shdyn_s3 = shdyn_amt[3] ? {{8{shdyn_fill}},  shdyn_s2[31:8]}  : shdyn_s2;
    shdyn_s4 = shdyn_amt[4] ? {{16{shdyn_fill}}, shdyn_s3[31:16]} : shdyn_s3;

    shdyn_shifted = shdyn_full32 ? {32{shdyn_fill}} : shdyn_s4;
    //Reverse left-shift results back to natural order; right shifts pass through.
    shdyn_result  = shdyn_left ? shift_reverse32(shdyn_shifted) : shdyn_shifted;
end

always_comb begin
    //SHIFT unit: constant shifts are wiring; only SHAD/SHLD need the barrel
    //shifter, computed once above and selected here for both directions.
    shift_t_we    = 1'b0;
    shift_t_value = 1'b0;
    case(idex.alu_op)
        ALU_SHLL:  begin shift_result = {ex_a[30:0], 1'b0};   shift_t_we = 1'b1; shift_t_value = ex_a[31]; end
        ALU_SHLR:  begin shift_result = {1'b0, ex_a[31:1]};   shift_t_we = 1'b1; shift_t_value = ex_a[0]; end
        ALU_SHAR:  begin shift_result = {ex_a[31], ex_a[31:1]}; shift_t_we = 1'b1; shift_t_value = ex_a[0]; end
        ALU_ROTL:  begin shift_result = {ex_a[30:0], ex_a[31]}; shift_t_we = 1'b1; shift_t_value = ex_a[31]; end
        ALU_ROTR:  begin shift_result = {ex_a[0], ex_a[31:1]};  shift_t_we = 1'b1; shift_t_value = ex_a[0]; end
        ALU_ROTCL: begin shift_result = {ex_a[30:0], ex_t};   shift_t_we = 1'b1; shift_t_value = ex_a[31]; end
        ALU_ROTCR: begin shift_result = {ex_t, ex_a[31:1]};   shift_t_we = 1'b1; shift_t_value = ex_a[0]; end
        ALU_SHLL2:  shift_result = ex_a << 2;
        ALU_SHLL8:  shift_result = ex_a << 8;
        ALU_SHLL16: shift_result = ex_a << 16;
        ALU_SHLR2:  shift_result = ex_a >> 2;
        ALU_SHLR8:  shift_result = ex_a >> 8;
        ALU_SHLR16: shift_result = ex_a >> 16;
        ALU_SHAD:   shift_result = shdyn_result; //arithmetic dynamic shift; fill via shdyn_fill
        ALU_SHLD:   shift_result = shdyn_result; //logical dynamic shift; zero fill
        default: shift_result = ex_a;
    endcase
end

always_comb begin
    //MISC unit: bypass, MOVT, sign/zero extension, byte/word swap, and extract.
    case(idex.alu_op)
        ALU_PASS_B: misc_result = ex_b;
        ALU_MOVT:   misc_result = {31'd0, ex_t};
        ALU_EXTU_B: misc_result = {24'd0, ex_a[7:0]};
        ALU_EXTU_W: misc_result = {16'd0, ex_a[15:0]};
        ALU_EXTS_B: misc_result = {{24{ex_a[7]}}, ex_a[7:0]};
        ALU_EXTS_W: misc_result = {{16{ex_a[15]}}, ex_a[15:0]};
        ALU_SWAP_B: misc_result = {ex_a[31:16], ex_a[7:0], ex_a[15:8]};
        ALU_SWAP_W: misc_result = {ex_a[15:0], ex_a[31:16]};
        ALU_XTRCT:  misc_result = {ex_b[15:0], ex_a[31:16]};
        default:    misc_result = ex_a; //ALU_PASS_A and operand-A default
    endcase
end

always_comb begin
    //CEU: condition evaluation produces SR.T only and writes no register result.
    //CMP_STR sets T when any byte pair matches; TST tests a bitwise AND for zero.
    ceu_t_we    = 1'b0;
    ceu_t_value = 1'b0;
    case(idex.alu_op)
        ALU_TST:    begin ceu_t_we = 1'b1; ceu_t_value = (ex_a & ex_b) == 32'd0; end
        ALU_CMP_EQ: begin ceu_t_we = 1'b1; ceu_t_value = ex_a == ex_b; end
        ALU_CMP_HS: begin ceu_t_we = 1'b1; ceu_t_value = ex_a >= ex_b; end
        ALU_CMP_GE: begin ceu_t_we = 1'b1; ceu_t_value = $signed(ex_a) >= $signed(ex_b); end
        ALU_CMP_HI: begin ceu_t_we = 1'b1; ceu_t_value = ex_a > ex_b; end
        ALU_CMP_GT: begin ceu_t_we = 1'b1; ceu_t_value = $signed(ex_a) > $signed(ex_b); end
        ALU_CMP_PZ: begin ceu_t_we = 1'b1; ceu_t_value = !ex_a[31]; end
        ALU_CMP_PL: begin ceu_t_we = 1'b1; ceu_t_value = !ex_a[31] && ex_a != 32'd0; end
        ALU_CMP_STR: begin
            ceu_t_we    = 1'b1;
            ceu_t_value = (ex_a[31:24] == ex_b[31:24]) || (ex_a[23:16] == ex_b[23:16]) ||
                          (ex_a[15:8] == ex_b[15:8]) || (ex_a[7:0] == ex_b[7:0]);
        end
        default: begin end
    endcase
end

always_comb begin
    //Final 4-to-1 result mux; alu_class is the two-bit selector decoded in ID.
    //Four 32-bit inputs and two selects map to one 6-LUT per result bit.
    case(idex.alu_class)
        ALU_CLASS_ARITH: alu_result = arith_result;
        ALU_CLASS_LOGIC: alu_result = logic_result;
        ALU_CLASS_SHIFT: alu_result = shift_result;
        default:         alu_result = misc_result;
    endcase
end

always_comb begin
    //T selector: SETT/CLRT from decode, otherwise the active unit's flag.
    //Exactly one source asserts its write enable for any given instruction.
    alu_t_we = idex.t_write_decode | arith_t_we | shift_t_we | ceu_t_we | div_t_we;
    if(arith_t_we)      alu_t_value = arith_t_value;
    else if(shift_t_we) alu_t_value = shift_t_value;
    else if(ceu_t_we)   alu_t_value = ceu_t_value;
    else if(div_t_we)   alu_t_value = div_t_value;
    else                alu_t_value = idex.t_decode_value;
end

always_comb begin
    //Address modes implement manual table 2.2, pp.28-31.
    case(idex.mem_size)
        SIZE_BYTE: ea_step = 32'd1;
        SIZE_WORD: ea_step = 32'd2;
        default:   ea_step = 32'd4;
    endcase

    //AGU base: agu_base_q keeps the idex.src_a leg role (same D/CE plus the MAC/RMW
    //second-access hold-load); the EX-head lanes patch it exactly like ex_a. A
    //second-access DISPATCH forces the held address via ma_second_pending_agu
    //(= second_access && !req_sent, both FFs), so no live handshake rides this
    //select; the lane/dep terms read the AGU-cluster (* preserve *) duplicates.
    case(ma_second_pending_agu ? FWD_NONE : (fwd_wbsel_a_agu ? FWD_WB : fwd_lane_a_agu))
        FWD_EXMA_G0: ea_addr_base = exma.gpr0_data;
        FWD_EXMA_G1: ea_addr_base = exma.gpr1_data;
        FWD_WB:      ea_addr_base = agu_wb_a;
        default:     ea_addr_base = agu_base_q;   //ID-resolved base / held 2nd-access addr
    endcase
    //Addend: same lane patch on the b port, from its own AGU-cluster duplicates
    //(ea_addr_addend used to alias ex_b; a shared net dragged the ALU cluster's
    //placement onto the adder). The en-gate (PRE-DECODED idex.agu_en_mode) still
    //NULLs it for REG/POSTINC/MAC, so no EX addr_op decode sits on the carry cone.
    case(fwd_wbsel_b_agu ? FWD_WB : fwd_lane_b_agu)
        FWD_EXMA_G0: ea_addr_addend = exma.gpr0_data;
        FWD_EXMA_G1: ea_addr_addend = exma.gpr1_data;
        FWD_WB:      ea_addr_addend = agu_wb_b;
        default:     ea_addr_addend = idex.src_b_value;
    endcase
    ea_update_addend = ea_step;

    //effective_addr is exactly that one sum (= ex_a when the addend is zero), so no
    //post-adder select sits on the half-cycle address path to the cache. ea_addr_sum is
    //driven by the shared AGU instance (u_agu_d) below - the addend is already nulled in
    //the case above, so the AGU runs en_mode=FORCE (i_USE_BASE=1, no PC+2 carry).
    effective_addr        = ea_addr_sum;
    //Equal MAC pointer registers expose the first increment to the second read.
    effective_addr_second = (idex.addr_op == ADDR_MAC &&
                             idex.src_a_id == idex.src_b_id) ? ex_a + ea_step : ex_b;
    //Address updates: ONE bare adder each, no addr_op decode in EX. au_addend_q is the
    //ID-pre-decoded addend (PREDEC's old effective_addr leg == ex_a - step, base = src_a).
    //update_second is consumed only by a MAC gpr1 writeback, so it is unconditional.
    address_update        = ex_a + au_addend_q;
    address_update_second = ex_b + ea_step;

    //Byte accesses are always aligned; word and longword accesses are constrained.
    //PREF addresses the line containing Rn, so it never raises an address error.
    address_error = 1'b0;
    if(idex.mem_op != MEM_NONE && idex.mem_op != MEM_PREF) begin
        if(idex.mem_size == SIZE_WORD) address_error = effective_addr[0];
        if(idex.mem_size == SIZE_LONG) address_error = |effective_addr[1:0];
        if(idex.mem_op == MEM_MAC && idex.mem_size == SIZE_WORD) address_error |= effective_addr_second[0];
        if(idex.mem_op == MEM_MAC && idex.mem_size == SIZE_LONG) address_error |= |effective_addr_second[1:0];
    end
end

//The MAC/RMW second access presents its address from the AGU only while it is being DISPATCHED
//(second_access && !req_sent). Keying on !req_sent - not the whole MA residency - is what frees
//the AGU on the completion cycle (req_sent is still 1 when the 2nd response lands), so the next
//EX instruction gets the AGU that cycle instead of being forced to the second address.
//Built from the preserved *_agu duplicates: the AGU is its only consumer.
assign  ma_second_pending_agu = ma_second_access_agu && !data_req_sent_agu;

//SHALLOW "data access presented this cycle" (was the o_D_REQ_RAW sideband). Drives the L bus
//req_fetch: a fetch is presented only when this is 0 (DATA priority). The AGU time-share select
//rides l_is_data_agu below (same expression off the preserved low-fanout duplicates).
//idex.is_data = (valid && mem_op!=NONE) is PRE-DECODED in ID (a flop), so the AGU base-select sees
//flop + one OR, NOT the mem_op decode - keeps the mem_op->l_is_data->agu_x cone off the 5 ns cen_n
//bram_addr path. ma_second_access is live MA-seq state (MAC/RMW 2nd access) and cannot pre-register.
wire    l_is_data     = idex.is_data || ma_second_access;
wire    l_is_data_agu = idex_is_data_agu || ma_second_access_agu; //AGU-only copy (i_USE_BASE/i_EN_MODE)

//Shared time-shared AGU (agu.sv) - the SINGLE address source for the L bus. On a data cycle
//(l_is_data=1) i_USE_BASE=1 selects agu_base_q (EA base, or the held EA2/write addr while a 2nd
//access dispatches) and en_mode=FORCE/NULL adds the offset. On a fetch cycle (l_is_data=0)
//i_USE_BASE=0 selects i_FETCH_PC=fetch_pc with en_mode=NULL, so o_ADDR = fetch_pc: the cache reads
//the fetch address off the SAME adder, no separate I address.
//NOTE (1a): folding the fetch PC INTO agu_base_q (to delete this per-bit fetch mux and pack each AGU
//bit into 1 ALM) was attempted and REVERTED - the shared base register clobbers the held second-access
//address on MAC/RMW/TAS.B cycles that branch1 does not preload (BYTE_TST is excluded). Kept the mux.
//effective_addr = ea_addr_sum is only consumed by the data path when idex has a mem op, so the
//fetch_pc value leaking onto it on a fetch cycle is masked (address_error etc. gate on mem_op).
agu u_agu_d (
    .i_AGU_A    (ea_addr_base                  ),  //pre-selected base: EA base / EA2 (agu_base_q)
    .i_AGU_B    (ea_addr_addend                ),
    .i_FETCH_PC (fetch_pc                      ),  //IF sequential PC (data/fetch time-share mux)
    .i_USE_BASE (l_is_data_agu                 ),  //1: EX/2nd base (MA); 0: fetch PC (IF)
    .i_EN_MODE  ((l_is_data_agu && !ma_second_pending_agu) ? idex.agu_en_mode : 2'd0),  //ID-decoded gate; NULL on 2nd/fetch
    .i_R_T      (1'b0                          ),
    .i_PC_INC   (1'b0                          ),
    .o_ADDR     (ea_addr_sum                   )
);

always_comb begin
    //Convert the register value into normalized 32-bit memory lanes.
    aligned_store_data = ex_store;
    store_strobe       = 4'b0000;
    store_lane         = ea_addr_sum[1:0];
    case(idex.mem_size)
        SIZE_BYTE: begin
            if(BIG_ENDIAN) begin
                aligned_store_data = {4{ex_store[7:0]}};
                store_strobe       = 4'b1000 >> store_lane;
            end
            else begin
                aligned_store_data = {4{ex_store[7:0]}};
                store_strobe       = 4'b0001 << store_lane;
            end
        end
        SIZE_WORD: begin
            aligned_store_data = {2{ex_store[15:0]}};
            if(BIG_ENDIAN) store_strobe = store_lane[1] ? 4'b0011 : 4'b1100;
            else           store_strobe = store_lane[1] ? 4'b1100 : 4'b0011;
        end
        default: begin
            aligned_store_data = ex_store;
            store_strobe       = 4'b1111;
        end
    endcase
end

//Kill-product duplicate for the EX request cluster (see wb_kill_issue).
(* keep *) wire wb_kill_ex = wb_valid && mawb.fault;
always_comb begin
    //EX assembles the primary (first) data request. u_ma_seq phase-selects it
    //against its own second access onto the single D bus.
    early_ex_fault = idex.fetch_fault || idex.illegal ||
                     (idex.privileged && !i_SR[30]) || address_error;
    //Request-valid base: everything except the exma_allow qualifier, which carries the
    //cache late bits and is ANDed in per rail (see the 4-rail block below u_ma_seq).
    //!i_REDIRECT_VALID: a D request granted AT a kill edge runs wrong-path and its
    //response is orphaned (the fetch drop_d leak's D-side twin). !(data_response &&
    //d_rsp_fault): the elder MA op's SAME-EDGE fault response must block this fire -
    //wb_kill_ex sees the fault one cycle too late; the unfired op stalls in MA until
    //the fault redirect kills it. Both found by the exception/interrupt sweeps.
    early_d_req_base = idex.valid &&
                        !fault_hold && !wb_kill_ex &&
                        !early_ex_fault && idex.mem_op != MEM_NONE &&
                        !i_REDIRECT_VALID &&
                        !(data_response && d_rsp_fault);
    ex_req       = '0;
    ex_req.valid = early_d_req_valid;
    ex_req.write = idex.mem_op == MEM_STORE;
    ex_req.size  = idex.mem_size;
    ex_req.addr  = effective_addr;
    ex_req.wdata = aligned_store_data;
    ex_req.wstrb = idex.mem_op == MEM_STORE ? store_strobe : 4'b0000;
    ex_req.lock  = idex.mem_op == MEM_RMW && idex.byte_op != BYTE_TST;
    ex_req.pref  = idex.mem_op == MEM_PREF;
end

always_comb begin
    //PC-relative targets (BT/BF/BRA/BSR) are precomputed in ID as idex.immediate = pc+4+disp,
    //so EX needs NO branch adder for them. BRAF/BSRF carry pc+4 in immediate and add Rn here
    //(one 2-input add). Register/return targets (JMP/JSR/RTS/RTE) are plain register reads.
    branch_taken   = 1'b0;
    branch_delayed = idex.branch_delayed;
    branch_target  = idex.immediate;            //PC-relative full target, precomputed in ID
    case(idex.branch_op)
        BR_BT:   branch_taken = ex_t;
        BR_BF:   branch_taken = !ex_t;
        BR_BRA:  branch_taken = 1'b1;
        BR_BSR:  branch_taken = 1'b1;
        BR_BRAF: begin branch_taken = 1'b1; branch_target = idex.immediate + ex_a; end  //pc+4 + Rn
        BR_BSRF: begin branch_taken = 1'b1; branch_target = idex.immediate + ex_a; end  //pc+4 + Rn
        BR_JMP:  begin branch_taken = 1'b1; branch_target = ex_a; end
        BR_JSR:  begin branch_taken = 1'b1; branch_target = ex_a; end
        BR_RTS:  begin branch_taken = 1'b1; branch_target = pr; end
        BR_RTE:  begin branch_taken = 1'b1; branch_target = i_SPC; end
        default: begin branch_taken = 1'b0; branch_target = idex.pc + 32'd4; end
    endcase
end

assign branch_event    = ex_advance && idex.branch_op != BR_NONE;
assign branch_redirect = branch_event && branch_taken;

always_comb begin
    //Build one registered EX result; fault handling below removes every side effect.
    ex_result = '0;
    ex_result.valid         = idex.valid;
    ex_result.pc            = idex.pc;
    ex_result.inst          = idex.inst;
    ex_result.delay_slot    = idex.delay_slot;
    ex_result.gpr0_we       = idex.gpr0_we;
    ex_result.gpr0_dst      = idex.gpr0_dst;
    ex_result.gpr0_data     = idex.mem_op == MEM_MAC ? address_update : alu_result;
    ex_result.gpr1_we       = idex.gpr1_we;
    ex_result.gpr1_dst      = idex.gpr1_dst;
    ex_result.gpr1_data     = idex.mem_op == MEM_MAC ? address_update_second : address_update;
    ex_result.mem_op        = idex.mem_op;
    ex_result.dbr           = idex.branch_delayed;  //interrupt-defer marker (pair atomicity)
    ex_result.nd_taken      = idex.branch_op != BR_NONE && !idex.branch_delayed &&
                              branch_taken;         //commit successor = redirect target
    ex_result.mem_size      = idex.mem_size;
    ex_result.load_signed   = idex.load_signed;
    ex_result.byte_op       = idex.byte_op;
    ex_result.mem_addr      = effective_addr;
    ex_result.mem_addr_second = effective_addr_second;
    ex_result.store_wdata   = aligned_store_data;
    ex_result.store_wstrb   = store_strobe;
    ex_result.mac_cmd       = idex.mac_cmd;
    ex_result.mac_a         = ex_a; //multiplicand for MUL; load data for LDS to MACH/MACL
    ex_result.mac_b         = idex.addr_op == ADDR_GBR_INDEX ? ex_store : ex_b; //RMW immediate
    ex_result.mac_saturate  = ex_s;
    ex_result.pr_we         = idex.pr_we || idex.pr_link;
    ex_result.pr_data       = idex.pr_link ? idex.pc + 32'd4 : alu_result;
    ex_result.ctrl_dst      = idex.ctrl_dst;
    ex_result.ctrl_data     = alu_result;
    ex_result.t_we          = alu_t_we;
    ex_result.t_data        = alu_t_value;
    ex_result.s_we          = idex.s_write_decode;
    ex_result.s_data        = idex.s_decode_value;
    ex_result.mq_we         = div_mq_we;
    ex_result.mq_data       = div_mq_data;
    ex_result.event_trapa   = idex.event_trapa;
    ex_result.trapa_imm     = idex.trapa_imm;
    ex_result.event_rte     = idex.event_rte;
    ex_result.event_sleep   = idex.event_sleep;
    ex_result.event_ldtlb   = idex.event_ldtlb;
    // Address-error events need read/write and faulting address metadata.
    ex_result.fault_write   = idex.mem_op == MEM_STORE;
    ex_result.fault_addr    = idex.mem_op == MEM_NONE ? idex.pc : effective_addr;

    //Fault priority follows pipeline age: fetch, decode legality, privilege, address.
    if(idex.fetch_fault) begin
        ex_result.fault       = 1'b1;
        ex_result.fault_cause = EXC_IFETCH;
        ex_result.fault_write = 1'b0;
        ex_result.fault_addr  = idex.pc;
    end
    else if(idex.illegal) begin
        ex_result.fault       = 1'b1;
        ex_result.fault_cause = EXC_ILLEGAL;
        ex_result.fault_write = 1'b0;
        ex_result.fault_addr  = idex.pc;
    end
    else if(idex.privileged && !i_SR[30]) begin
        ex_result.fault       = 1'b1;
        ex_result.fault_cause = EXC_PRIVILEGE;
        ex_result.fault_write = 1'b0;
        ex_result.fault_addr  = idex.pc;
    end
    else if(address_error) begin
        ex_result.fault       = 1'b1;
        ex_result.fault_cause = EXC_ADDRESS;
    end

    //A faulting instruction may report metadata but cannot change architectural state.
    if(ex_result.fault) begin
        ex_result.gpr0_we    = 1'b0;
        ex_result.gpr1_we    = 1'b0;
        ex_result.mem_op     = MEM_NONE;
        ex_result.mac_cmd    = MAC_NONE;
        ex_result.pr_we      = 1'b0;
        ex_result.ctrl_dst   = CTRL_NONE;
        ex_result.t_we       = 1'b0;
        ex_result.s_we       = 1'b0;
        ex_result.mq_we      = 2'b00;
    end
end



///////////////////////////////////////////////////////////
//////  Memory Access
////

/*
    EX issues the primary ordered request before EX/MA captures context.
    MA waits for the matching completion response.
    The response contains the aligned 32-bit word surrounding the byte address.
    Byte/word selection and sign extension follow manual section 2.2,
    p.25. Stores also wait for acknowledgement before retirement.
*/

logic   [31:0]  load_value;
logic   [7:0]   selected_byte; //addressed byte from the returned longword
logic   [31:0]  memory_read_addr;
logic   [7:0]   sel_byte_hit,  sel_byte_miss;  //per-source addressed byte
logic   [15:0]  sel_word_hit,  sel_word_miss;  //per-source addressed halfword
logic   [31:0]  load_hit,      load_miss;      //per-source aligned/extended load word

//DUAL ALIGNER: lane-select + sign-extend computed ONCE PER SOURCE (the CEN_n-late cache
//hit word and the CEN_p-early registered miss word), and the hit select re-applied at the
//END. Every aligner select (address low bits, size, sign) is an early registered field,
//so the late word crosses the same two mux levels the early word does - in parallel -
//instead of stacking the cache 2:1 UNDER the aligner. Boolean-identical: the aligner is
//pure per-bit muxing, so align(hit ? A : B) == hit ? align(A) : align(B).
function automatic logic [7:0] pick_byte(input logic [31:0] w, input logic [1:0] a);
    case(a)
        2'd0: pick_byte = BIG_ENDIAN ? w[31:24] : w[7:0];
        2'd1: pick_byte = BIG_ENDIAN ? w[23:16] : w[15:8];
        2'd2: pick_byte = BIG_ENDIAN ? w[15:8]  : w[23:16];
        default: pick_byte = BIG_ENDIAN ? w[7:0] : w[31:24];
    endcase
endfunction
function automatic logic [15:0] pick_word(input logic [31:0] w, input logic a1);
    pick_word = (BIG_ENDIAN ^ a1) ? w[31:16] : w[15:0];
endfunction
function automatic logic [31:0] ld_extend(
    input logic [31:0] w,       //full source word
    input logic [7:0]  b,       //its addressed byte
    input logic [15:0] h,       //its addressed halfword
    input mem_size_t   sz,
    input logic        sgn
);
    //SH register loads sign-extend byte and word operands; see p.25.
    case(sz)
        SIZE_BYTE: ld_extend = sgn ? {{24{b[7]}},  b} : {24'd0, b};
        SIZE_WORD: ld_extend = sgn ? {{16{h[15]}}, h} : {16'd0, h};
        default:   ld_extend = w;
    endcase
endfunction

always_comb begin
    memory_read_addr = exma.mem_op == MEM_MAC && ma_second_access ?
                       exma.mem_addr_second : exma.mem_addr;
    //Address bits select lanes differently for big-endian and little-endian memory.
    sel_byte_hit  = pick_byte(L_BUS.rsp_rdata_hit,  memory_read_addr[1:0]);
    sel_byte_miss = pick_byte(L_BUS.rsp_rdata_miss, memory_read_addr[1:0]);
    sel_word_hit  = pick_word(L_BUS.rsp_rdata_hit,  memory_read_addr[1]);
    sel_word_miss = pick_word(L_BUS.rsp_rdata_miss, memory_read_addr[1]);
    load_hit  = ld_extend(L_BUS.rsp_rdata_hit,  sel_byte_hit,  sel_word_hit,
                          exma.mem_size, exma.load_signed);
    load_miss = ld_extend(L_BUS.rsp_rdata_miss, sel_byte_miss, sel_word_miss,
                          exma.mem_size, exma.load_signed);
    load_value    = L_BUS.rsp_hit_d ? load_hit     : load_miss;
    selected_byte = L_BUS.rsp_hit_d ? sel_byte_hit : sel_byte_miss;
end

//MAC-leg twin of load_value, word/long ONLY: MAC.W/MAC.L never issue byte reads, so the
//shared aligner's byte-extend stage is unreachable on this leg (= load_value whenever
//exma.mem_op==MEM_MAC, the only condition both consumers test). keep hands the fitter a
//copy it can place at the DSP columns (fit8: byp_q/bram_addr -> dsp_b, 13/20 top paths).
wire    [15:0]  mac_word_sel = L_BUS.rsp_hit_d ? sel_word_hit : sel_word_miss;
(* keep *) wire [31:0] mac_load_value =
    exma.mem_size == SIZE_WORD
        ? (exma.load_signed ? {{16{mac_word_sel[15]}}, mac_word_sel} : {16'd0, mac_word_sel})
        : (L_BUS.rsp_hit_d ? L_BUS.rsp_rdata_hit : L_BUS.rsp_rdata_miss);

//MA sequencer: owns the second-access phase and builds the one D request. The
//first access of every memory op is the EX primary (ex_req); the sequencer issues
//the MAC second read or the RMW write. EX and second access are mutually
//exclusive, so the request is a parallel phase-select, not a priority overlay.
//Kill-product duplicate for the MA cluster (see wb_kill_issue).
(* keep *) wire wb_kill_ma = wb_valid && mawb.fault;
assign  ma_flush         = i_REDIRECT_VALID || wb_kill_ma;
assign  ma_active        = exma.valid && !exma.fault && !fault_hold && !wb_kill_ma;
assign  ma_capture_value = exma.mem_op == MEM_MAC ? mac_load_value : {24'd0, selected_byte};

ma_seq #(.BIG_ENDIAN(BIG_ENDIAN)) u_ma_seq (
    .i_CLK          (i_CLK                ),
    .i_RST_n        (i_RST_n              ),
    .i_CEN          (cen                  ),
    .i_advance      (exma_allow           ),
    .i_flush        (ma_flush             ),
    .i_ex_req       (ex_req               ),
    .i_ex_valid     (early_d_req_valid    ),
    .o_ex_accept    (ex_data_req_accept   ),
    .i_ma_active    (ma_active            ),
    .i_ma_op        (exma.mem_op          ),
    .i_ma_byte_op   (exma.byte_op         ),
    .i_ma_size      (exma.mem_size        ),
    .i_ma_addr      (exma.mem_addr        ),
    .i_ma_addr2     (exma.mem_addr_second ),
    .i_ma_imm       (exma.mac_b[7:0]      ),
    .i_ma_capture   (ma_capture_value     ),
    .i_req_ready    (L_BUS.req_ready      ),  //D ready whenever a D request is up (l_is_data)
    .i_rsp_valid    (d_rsp_valid          ),
    .i_rsp_fault    (d_rsp_fault          ),
    .o_req          (early_bus_d_req      ),
    .o_second       (ma_second_access     ),
    .o_second_agu   (ma_second_access_agu ),
    .o_req_sent     (data_req_sent        ),
    .o_req_sent_agu (data_req_sent_agu    ),
    .o_first_value  (ma_first_value       )
);

///////////////////////////////////////////////////////////
//////  Handshake Tail (scalar; the dual-CEN 4-rail machinery is deleted)
////

//AGU base-shadow hold-load, early factors (see the agu_base_q capture): a held MA
//MAC / byte-RMW (not TAS) packet reloads the base with its 2nd-access address.
wire            agu_hold_base = exma.valid && !exma.fault &&
                                (exma.mem_op == MEM_MAC ||
                                 (exma.mem_op == MEM_RMW && exma.byte_op != BYTE_TST));
wire            agu_hold_mac  = exma.mem_op == MEM_MAC;   //2nd-addr source select (early)

//WB-recognized fault this edge - kills younger packets (see the fault block).
//keep: identical product to wb_fault_pending/wb_kill_* - blocks the merge so the fetch
//cluster owns a local copy (fit4: one merged node, 1060 loads, whole-pipe centroid).
(* keep *) wire wb_fault_kill = wb_valid && mawb.fault;

//IF/ID + fetch-slot capture compositions. Each is the register's whole CE (or data
//select) with every term folded in - external redirect, WB fault kill, branch clear,
//drop/pending flags - priorities preserved exactly from the pre-rail procedural code:
//redirect > WB fault kill > branch clear > response insert > issue clear (ifid);
//fault > accept > branch redirect (fetch_drop, last-write-wins order).
//Insert leg gated on !pair_ready: the held sibling is OLDER than any waiting response,
//so it must reach IF/ID first (the response holds loss-free in the cache meanwhile).
//The drop/redirect consume legs stay pair-blind.
assign  i_rsp_ready = fetch_pending &&
                      (fetch_drop || ((!ifid.valid || id_issue) && !pair_ready) ||
                       i_REDIRECT_VALID || branch_redirect);
assign  if_accept   = i_rsp_valid && i_rsp_ready;
//A usable pair rides the live fetch response: the same-edge request fire MUST be
//suppressed - fetch_pc is the sibling's own PC, which the pair slot satisfies.
//Kill-gated only (NOT insert-gated: an unconsumed paired response keeps suppressing);
//under a redirect the fire is the TARGET fetch and proceeds (drop_d marks wrong-path).
wire    rsp_pair_ok = i_rsp_valid && L_BUS.rsp_pair && !i_rsp_fault &&
                      fetch_pending && !fetch_drop &&
                      !i_REDIRECT_VALID && !branch_redirect && !wb_fault_kill;
//Pipelined fetch "want to issue". A wrong-path fetch is marked via fetch_drop and its
//line fill aborted (o_I_SQUASH).
assign  early_i_req_raw_valid = (!fetch_pending || if_accept) && !rsp_pair_ok &&
                                !i_REDIRECT_VALID && !fault_hold && !wb_fault_kill;
assign  i_req_fire  = early_i_req_raw_valid && !l_is_data && L_BUS.req_ready;

wire    ifid_clr  = branch_redirect ||
                    (branch_event && !branch_delayed && branch_taken);
wire    ifid_ld   = !i_REDIRECT_VALID && !wb_fault_kill && !ifid_clr &&
                    if_accept && !fetch_drop;               //response insert into IF/ID
(* keep *) wire ifid_ld_dat = !i_REDIRECT_VALID && !wb_fault_kill && !ifid_clr &&
                    if_accept && !fetch_drop;   //data-cluster CE duplicate (placement-local)
//Pair slot events. Capture = the response inserts into IF/ID this edge AND pairs
//(ifid_ld already folds every kill term); at that edge fetch_pc == the sibling's PC
//(it only advances on fire/capture, and any redirect in between killed the capture).
//Serve = IF/ID can take the held sibling (kill terms mirror ifid_ld; exclusive with
//ifid_ld by the i_rsp_ready gate above).
wire    pair_capture = ifid_ld && L_BUS.rsp_pair;
wire    pair_serve   = pair_ready && !i_REDIRECT_VALID && !wb_fault_kill && !ifid_clr &&
                       (!ifid.valid || id_issue);
wire    ifid_zero = i_REDIRECT_VALID || wb_fault_kill || ifid_clr ||
                    (id_issue && !ifid_ld && !pair_serve);  //every '0 load of the IF/ID packet
wire    fpc_ce    = i_REDIRECT_VALID || branch_redirect || i_req_fire ||
                    pair_capture;             //capture advances fetch_pc PAST the sibling
wire    fpc_selbr = branch_redirect;      //fetch_pc source: branch target (over pc+2)
wire    fp_ce     = i_req_fire || if_accept;                //fetch_pending capture
wire    drop_ce   = i_REDIRECT_VALID || wb_fault_kill || branch_redirect || if_accept;
//accept arm: a request FIRED at a branch-redirect edge still carries the old fetch_pc
//(the target loads at this edge) - mark it dropped, else its response inserts as if it
//were the branch target and a wrong-path instruction RETIRES (found by the cacheable
//squash sweep: the +3-ahead slot leaked on every full-speed taken branch).
wire    drop_d    = i_REDIRECT_VALID ? (if_accept ? 1'b0 : fetch_pending) :
                    wb_fault_kill    ? fetch_pending :
                    if_accept        ? (branch_redirect && i_req_fire) :
                                       (fetch_pending || i_req_fire);   //fetch_drop next value
wire    agu_hold_sel = agu_hold_base && !ma_complete; //agu_base_q source: MA 2nd-access addr
wire    agu_ce    = agu_hold_sel || idex_allow;       //agu_base_q capture (hold-load | advance)

//GPR NEXT-cycle read addresses (declared at the GPR file section). Bank select: the one
//writer that can change the active bank without a pipeline redirect is a retiring
//LDC ...,SR (RTE and exception entry both redirect, flushing IF/ID); its new bank bits
//must address the read captured at its own commit edge, one cycle before i_SR shows them.
wire            sr_wr_wb  = wb_valid && !i_REDIRECT_VALID && !mawb.fault &&
                            mawb.ctrl_dst == CTRL_SR;
//An RTE restores SR from SSR with NO external redirect (its PC redirect is
//internal, and the serialized target WAITS live in IF/ID), so the mirror must
//snoop the restore like the LDC arm - found by the RB-flip SR-race sweep and the
//random-interrupt oracle (an RTE back into the OTHER bank read the target's
//operands from the handler bank). TWO phases are needed: the WB-cycle arm
//(rte_wr_wb) gives the one-cycle LOOKAHEAD the read-address capture needs - the
//serialized target can issue on the cycle right after the pulse, and its GPR
//BRAM read is addressed AT the pulse edge; the pulse arm (o_RTE_VALID) then
//holds the value through the cycle where ctrl_reg commits SR<=SSR. A general
//exception cannot share the pulse (one commit point), and the RTE's own dbr
//defer blocks an interrupt redirect there, so both arms always mean "the
//restore commits". SSR is stable across both cycles (nothing else commits).
wire            rte_wr_wb = wb_valid && !i_REDIRECT_VALID && !mawb.fault &&
                            mawb.event_rte;
wire            bank1_nx  = (rte_wr_wb || o_RTE_VALID)
                                        ? (i_SSR[30] && i_SSR[29]) :
                            sr_wr_wb    ? (mawb.ctrl_data[30] && mawb.ctrl_data[29]) :
                                          gpr_active_bank1;

//r_bank1 declared at the GPR section; SR resets to MD=1/RB=1 (ctrl_reg 32'h7000_00F0).
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) r_bank1 <= 1'b1;
    else begin if(cen) begin
        r_bank1 <= bank1_nx;
    end end
end
//NEXT-ifid instruction/predecode: what IF/ID will hold during the read-data cycle.
//Pair-serve leads: pair_inst/pair_pd are REGISTERS, so that arm is shallower than
//the live response arm (exclusive with ifid_ld by construction). Feeds the hz_*
//captures; the GPR read ADDRESSES use the per-arm late-select tail below instead.
wire    [15:0]  nx_inst = pair_serve ? pair_inst :
                          ifid_ld    ? i_rsp_inst : ifid.inst;
pd_route_t      nx_pd;
assign  nx_pd   = pair_serve ? pair_pd :
                  ifid_ld    ? pd_fetch : ifid.pd;
wire    [4:0]   nx_n_id    = active_gpr_id(nx_inst[11:8], bank1_nx);
wire    [4:0]   nx_m_id    = active_gpr_id(nx_inst[7:4],  bank1_nx);
wire    [4:0]   nx_bank_id = inactive_bank_id(nx_inst[6:4], bank1_nx);
wire    [4:0]   nx_r0_id   = active_gpr_id(4'd0, bank1_nx);

//GPR read-address TAIL LATE-SELECT (advance-loop/Wall A headline class, see
//eval_ooc/cache_wall_campaign.md): pair_serve/ifid_ld carry the whole advance loop
//(idex_allow, fo 346) and the cache hit resolve (if_accept), yet used to cross the
//id-map LUTs plus the rib/need_a muxes before the M10K address pin. Each arm's
//addresses now come from its OWN inst/pd (pair/hold arms fully registered; rsp arm
//is the live response = Wall A data leg, unavoidable), so the late selects cross
//exactly ONE 3:1 mux level at the pin (2 sel + 3 data = 5 inputs/bit, one ALM).
//readb = read port B (port 1) address; read0 = read port 0 (need_a folds port A).
wire    [4:0]   nx_pair_readb = pair_pd.rib ? inactive_bank_id(pair_inst[6:4], bank1_nx) :
                                              active_gpr_id(pair_inst[7:4], bank1_nx);
wire    [4:0]   nx_pair_read0 = pair_pd.need_a ? active_gpr_id(pair_inst[11:8], bank1_nx) :
                                                 nx_pair_readb;
wire    [4:0]   nx_rsp_readb  = pd_fetch.rib ? inactive_bank_id(i_rsp_inst[6:4], bank1_nx) :
                                               active_gpr_id(i_rsp_inst[7:4], bank1_nx);
wire    [4:0]   nx_rsp_read0  = pd_fetch.need_a ? active_gpr_id(i_rsp_inst[11:8], bank1_nx) :
                                                  nx_rsp_readb;
wire    [4:0]   nx_hold_readb = ifid.pd.rib ? inactive_bank_id(ifid.inst[6:4], bank1_nx) :
                                              active_gpr_id(ifid.inst[7:4], bank1_nx);
wire    [4:0]   nx_hold_read0 = ifid.pd.need_a ? active_gpr_id(ifid.inst[11:8], bank1_nx) :
                                                 nx_hold_readb;

//Select twins for the GPR M10K cluster (merge-blocked, like idex_allow_opa/opb and
//ifid_ld_dat): id_issue re-rooted on a private idex_allow copy; the ifid_ld twin
//folds its own !drop/!redirect/!clr terms into i_rsp_ready, whose drop/redirect/
//branch arms then drop out dead (!ifid_clr implies !branch_redirect). Sim-checked
//against the shared-mux original below.
(* keep *) wire idex_allow_gpr = !idex.valid || (ex_complete && exma_allow);
(* keep *) wire id_issue_gpr   = ifid.valid && idex_allow_gpr && !id_hazard &&
                                 !fault_hold && !wb_kill_issue &&
                                 !(btbf_cancel_base && exma_allow);
(* keep *) wire pair_serve_gpr = pair_ready && !i_REDIRECT_VALID && !wb_fault_kill &&
                                 !ifid_clr && (!ifid.valid || id_issue_gpr);
(* keep *) wire ifid_ld_gpr    = !i_REDIRECT_VALID && !wb_fault_kill && !ifid_clr &&
                                 !fetch_drop && i_rsp_valid && fetch_pending &&
                                 (!ifid.valid || id_issue_gpr) && !pair_ready;

assign  nx_read0 = pair_serve_gpr ? nx_pair_read0 :
                   ifid_ld_gpr    ? nx_rsp_read0  : nx_hold_read0;
assign  nx_read1 = pair_serve_gpr ? nx_pair_readb :
                   ifid_ld_gpr    ? nx_rsp_readb  : nx_hold_readb;

// synthesis translate_off
//Late-select equivalence: the per-arm composition must equal the shared-mux original
//(nx_pd/nx_*_id above) every cycle - proves the twins and the dead-arm reduction.
always_comb begin
    logic [4:0] ref_readb, ref_read0;
    ref_readb = nx_pd.rib ? nx_bank_id : nx_m_id;
    ref_read0 = nx_pd.need_a ? nx_n_id : ref_readb;
    if(nx_read0 !== ref_read0 || nx_read1 !== ref_readb)
        $fatal(1, "nx_read late-select mismatch: r0=%02x ref=%02x r1=%02x refb=%02x",
               nx_read0, ref_read0, nx_read1, ref_readb);
end
// synthesis translate_on

//NEXT-cycle hazard identities, captured with the read addresses: every input's next
//value is what this cone computes (nx_inst/nx_pd = the coming IF/ID data, bank1_nx =
//the coming r_bank1), so the registered ids equal the old live pd_qualify(ifid, r_bank1)
//recompute EVERY cycle - the ifid.inst -> id -> dst-compare -> sel legs (fit9/10
//ifid.inst -> idex.src class) now launch from these flops. Sim-asserted below.
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        //= pd_qualify of the all-zero IF/ID under the reset bank (MD=RB=1): R0_BANK1.
        hz_a_id  <= 5'd8;
        hz_b_id  <= 5'd8;
        hz_st_id <= 5'd8;
    end
    else begin if(cen) begin
        hz_a_id  <= pd_qualify(nx_pd.a_sel,  nx_n_id, nx_m_id, nx_r0_id, nx_bank_id);
        hz_b_id  <= pd_qualify(nx_pd.b_sel,  nx_n_id, nx_m_id, nx_r0_id, nx_bank_id);
        hz_st_id <= pd_qualify(nx_pd.st_sel, nx_n_id, nx_m_id, nx_r0_id, nx_bank_id);
    end end
end

//Request-valid for the EX primary access (the exma_allow leg carries the live response).
assign  early_d_req_valid  = early_d_req_base && exma_allow;

always_comb begin
    ex_data_req_valid = early_d_req_valid;

    //L bus request: ONE address (the AGU output effective_addr = ea_addr_sum), req_fetch selects
    //the role (DATA priority). The data control/data fields come straight from the sequencer
    //descriptor; they are inert on a fetch (the cache gates them on !req_fetch), so no address or
    //data multiplexer is needed - only req_valid / req_fetch arbitrate IF vs MA.
    L_BUS.req_fetch = !l_is_data;
    L_BUS.req_valid = l_is_data ? early_bus_d_req.valid : early_i_req_raw_valid;
    L_BUS.req_addr  = effective_addr;                  //= ea_addr_sum: data EA (MA) or fetch PC (IF)
    L_BUS.req_write = early_bus_d_req.write;
    L_BUS.req_size  = early_bus_d_req.size;
    L_BUS.req_wdata = early_bus_d_req.wdata;
    L_BUS.req_wstrb = early_bus_d_req.wstrb;
    L_BUS.req_lock  = early_bus_d_req.lock;

    //Response ready routed by the type the cache is delivering (rsp_fetch): IF vs MA consume.
    d_rsp_ready     = exma.valid && data_req_sent;
    L_BUS.rsp_ready = L_BUS.rsp_fetch ? i_rsp_ready : d_rsp_ready;
end

//Sideband prefetch qualifier for PREF. SHALLOW: rides only the registered request descriptor
//(early_bus_d_req.pref = second_access ? 0 : idex PREF-decode), NOT early_bus_d_req.valid. The
//deep valid used to drag mawb.fault -> wb_fault_pending -> early valid onto the cache's bram_pref
//cen_n latch (the 5 ns -1.471 path). The cache ANDs this with grant_d (data cycle), and every D
//response the pipe consumes is gated by data_req_sent, so a squashed-PREF transient here is inert
//(no accept, no bus cycle, no cache state change) - see the bram_pref benign-spurious analysis.
assign  o_D_PREF = early_bus_d_req.pref;

//Sideband squash qualifier - the one outstanding fetch is being discarded, so the
//cache may abort its line fill instead of allocating wrong-path instructions.
assign  o_I_SQUASH = fetch_drop;

///////////////////////////////////////////////////////////
//////  Interrupt Restart PC (commit-time architectural register)
////

//o_INT_NEXT_PC = the PC of the next instruction to execute after the last COMMIT,
//maintained AT the commit point instead of scanning the pipe for its oldest live
//PC. The old scan needed one patched arm per frontier state (taken-branch loss,
//dropped-fetch, held pair - two of those shipped as bugs); the register makes the
//whole class unreachable. Update per retiring packet X:
//  X.nd_taken                  -> the EX redirect target (taken non-delayed BT/BF)
//  X.delay_slot && pair_taken  -> the EX redirect target (slot of a TAKEN pair)
//  otherwise                   -> X.pc + 2
//rdir_target_q holds the ONE outstanding EX redirect target: in-order EX with the
//1-deep fetch cannot resolve a second taken branch before the first's consumer
//commits (the target's first instruction reaches EX no earlier than that edge; a
//same-edge re-arm is read-old/write-new safe). pair_taken_q marks a taken DELAYED
//pair in flight; kills clear it (the pair re-executes and re-arms). Acceptance is
//legal only on a retire pulse, so the register is always fresh at a boundary, and
//interrupts never split a pair, so a mid-pair value is never consumed.
logic   [31:0]  arch_next_pc;   //next PC to execute, as of the last commit
logic   [31:0]  rdir_target_q;  //last EX branch-redirect target (single outstanding)
logic           pair_taken_q;   //taken delayed pair in flight (slot commit consumes)

wire            commit_fire = wb_valid && !i_REDIRECT_VALID && !mawb.fault;
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        arch_next_pc  <= RESET_PC;
        rdir_target_q <= RESET_PC;
        pair_taken_q  <= 1'b0;
    end
    else begin if(cen) begin
        if(branch_redirect) rdir_target_q <= branch_target;
        if(i_REDIRECT_VALID || wb_fault_kill)            pair_taken_q <= 1'b0; //pair re-executes
        else if(branch_redirect && idex.branch_delayed)  pair_taken_q <= 1'b1; //arm beats consume
        else if(commit_fire && mawb.delay_slot)          pair_taken_q <= 1'b0;
        if(commit_fire) begin
            arch_next_pc <= (mawb.nd_taken || (mawb.delay_slot && pair_taken_q))
                            ? rdir_target_q : mawb.pc + 32'd2;
        end
    end end
end

assign  o_INT_NEXT_PC = arch_next_pc;

//The former o_D_REQ_RAW sideband is now L_BUS.req_fetch (= !l_is_data), driven above; the
//former o_I_REQ_RAW cold-accept sideband died with the cache's reqn machinery (the single
//clock accepts live - accept and fire are the same edge decision, no phase race remains).

always_comb begin
    //MA substitutes formatted load data, then copies all other pending commit fields.
    ma_result = '0;
    ma_result.valid         = exma.valid;
    ma_result.pc            = exma.pc;
    ma_result.inst          = exma.inst;
    ma_result.delay_slot    = exma.delay_slot;
    ma_result.dbr           = exma.dbr;
    ma_result.nd_taken      = exma.nd_taken;
    ma_result.gpr0_we       = exma.gpr0_we;
    ma_result.gpr0_dst      = exma.gpr0_dst;
    ma_result.gpr0_data     = exma.mem_op == MEM_LOAD ? load_value : exma.gpr0_data;
    ma_result.gpr1_we       = exma.gpr1_we;
    ma_result.gpr1_dst      = exma.gpr1_dst;
    ma_result.gpr1_data     = exma.gpr1_data;
    ma_result.mac_cmd       = exma.mac_cmd;
    ma_result.mac_a         = exma.mem_op == MEM_LOAD ? load_value : exma.mac_a;
    ma_result.mac_b         = exma.mac_b;
    ma_result.mac_saturate  = exma.mac_saturate;
    //PR/ctrl memory sources are LDS.L / LDC.L ONLY (longword, manual sec 6: no .B/.W memory
    //forms), so the raw response word == load_value here - skipping the byte/word aligner
    //takes these mawb captures off the deep align cone. A non-ctrl load leaves garbage in
    //these fields, but WB ignores them unless pr_we/ctrl_dst is set (long-only ops).
    ma_result.pr_we         = exma.pr_we;
    ma_result.pr_data       = exma.mem_op == MEM_LOAD ? d_rsp_rdata : exma.pr_data;
    ma_result.ctrl_dst      = exma.ctrl_dst;
    ma_result.ctrl_data     = exma.mem_op == MEM_LOAD ? d_rsp_rdata : exma.ctrl_data;
    ma_result.t_we          = exma.t_we;
    ma_result.t_data        = exma.t_data;
    ma_result.s_we          = exma.s_we;
    ma_result.s_data        = exma.s_data;
    ma_result.mq_we         = exma.mq_we;
    ma_result.mq_data       = exma.mq_data;
    ma_result.event_trapa   = exma.event_trapa;
    ma_result.trapa_imm     = exma.trapa_imm;
    ma_result.event_rte     = exma.event_rte;
    ma_result.event_sleep   = exma.event_sleep;
    ma_result.event_ldtlb   = exma.event_ldtlb;
    ma_result.fault         = exma.fault;
    ma_result.fault_cause   = exma.fault_cause;
    ma_result.fault_write   = exma.fault_write;
    ma_result.fault_addr    = exma.fault_addr;

    //T commits in WB from the byte captured during the read slot (ma_first_value),
    //matching Fig 10.36/10.37 and keeping the live cache read off the commit path.
    if(exma.byte_op == BYTE_TST) begin
        ma_result.t_we   = 1'b1;
        ma_result.t_data = (ma_first_value[7:0] & exma.mac_b[7:0]) == 8'd0;
    end
    else if(exma.byte_op == BYTE_TAS) begin
        ma_result.t_we   = 1'b1;
        ma_result.t_data = ma_first_value[7:0] == 8'd0;
    end

    //A memory fault suppresses both possible GPR writes from this instruction.
    //PREF never faults (the cache suppresses it), so it is excluded here too.
    if(exma.mem_op != MEM_NONE && exma.mem_op != MEM_PREF && data_response && d_rsp_fault) begin
        ma_result.fault       = 1'b1;
        ma_result.fault_cause = EXC_DATA;
        ma_result.fault_write = exma.mem_op == MEM_STORE ||
                                (exma.mem_op == MEM_RMW && ma_second_access);
        ma_result.fault_addr  = memory_read_addr;
        ma_result.gpr0_we     = 1'b0;
        ma_result.gpr1_we     = 1'b0;
    end
end

///////////////////////////////////////////////////////////
//////  MAC DSP Unit
////

/*
    The multiply/MAC datapath lives in u_mac_dsp so the product maps to an FPGA
    DSP block instead of the EX result mux. MACH/MACL are owned there.
    A 16x16 operation completes after one architectural clock; 32x32 completes
    after two. The accumulator addition shares the final multiply cycle.
    Results remain pending until precise WB commits them into MACH and MACL.
*/

logic           mac_capture;     //operands are ready; latch them into the DSP pipeline stage (selected rail)
logic           mac_cap_base;    //ditto without the response leg (rail-invariant)
logic           mac_start;       //launch one multiply or multiply-accumulate operation
logic           mac_commit;      //retire one completed MACH/MACL command
logic   [31:0]  mac_operand_a;   //register operand or first formatted memory operand
logic   [31:0]  mac_operand_b;   //register operand or second formatted memory operand

//The multiplicand path mac_operand_a -> DSP was the sole setup-violating endpoint:
//the exma.mem_op select is fed by the deep decode/advance cone and retimed into the
//DSP input register. Software manual p.103 guarantees a following instruction cannot
//read MACH/MACL for two (16x16) or three (32x32) extra cycles, and id_uses_mac_state
//already stalls such a reader. That budget lets the operands take one cycle in a
//dedicated register before the multiply launches, so the DSP input becomes a clean
//register read. mac_capture latches the operands; mac_start fires one cycle later.
assign  mac_cap_base = !mac_started && !mac_armed && exma.valid && !exma.fault &&
                       mac_dsp_operation;                //response leg applied below
assign  mac_capture  = mac_cap_base &&
                       (exma.mem_op != MEM_MAC ||
                        (ma_second_access && data_response && !d_rsp_fault));
assign  mac_start     = mac_armed && !mac_started;
assign  mac_operand_a = exma.mem_op == MEM_MAC ? ma_first_value : exma.mac_a;
assign  mac_operand_b = exma.mem_op == MEM_MAC ? mac_load_value : exma.mac_b;

assign  mac_commit = !i_REDIRECT_VALID && wb_valid && !mawb.fault &&
                     mawb.mac_cmd != MAC_NONE;

mac_dsp u_mac_dsp (
    .i_CLK                  (i_CLK                                          ),
    .i_RST_n                (i_RST_n                                        ),
    .i_CEN                  (cen                                            ),
    .i_CANCEL               (i_REDIRECT_VALID                               ),
    .i_START                (mac_start                                      ),
    .i_COMMIT               (mac_commit                                     ),
    .i_CLEAR                (mawb.mac_cmd == MAC_CLEAR                      ),
    .i_LOAD_MACH            (mawb.mac_cmd == MAC_LOAD_MACH                  ),
    .i_LOAD_MACL            (mawb.mac_cmd == MAC_LOAD_MACL                  ),
    .i_ACCUMULATE           ((exma.mac_cmd == MAC_ACCUM_L || exma.mac_cmd == MAC_ACCUM_W) ),
    .i_FULL_LONG_RESULT     ((exma.mac_cmd == MAC_DMULS_L || exma.mac_cmd == MAC_DMULU_L) ),
    .i_WORD                 ((exma.mac_cmd == MAC_MULS_W || exma.mac_cmd == MAC_MULU_W || exma.mac_cmd == MAC_ACCUM_W)),
    .i_SIGNED               (!(exma.mac_cmd == MAC_MULU_W || exma.mac_cmd == MAC_DMULU_L)),
    .i_SATURATE             (exma.mac_saturate                              ),
    .i_LOAD_DATA            (mawb.mac_a                                     ),
    .i_A                    (dsp_a                                          ),
    .i_B                    (dsp_b                                          ),
    .o_DONE                 (mac_dsp_done                                   ),
    .o_MACH                 (mach                                           ),
    .o_MACL                 (macl                                           )
);


//The whole GPR read-context register group is gone. gpr_read_ready: with its clear and
//ports_ready both proven away it was a bare cen_n resample of ifid.valid, equal to it at
//every cen_p consumer edge, its one hazard term constant-0. gpr_read_ctx_address_a/b:
//cen_n resamples of gpr_read0/1_address off an ifid/SR that cannot change again before
//the consuming cen_p edge - the routing compares (doa/dob_hit_*) now use the live
//addresses on the 10 ns budget. The read-context guarantee is structural: every cen_n
//relaunches the read for whatever IF/ID holds, so data always matches by the next cen_p.


///////////////////////////////////////////////////////////
//////  Pipeline State Update
////

/*
    This block owns architectural-edge advancement and WB commit.
    MA completion loads mawb first; only the registered WB packet commits.
    WB samples architectural state and writes GPR storage on the next edge.
    External redirect flushes all packets and restarts IF at the supplied PC.
    A precise fault flushes younger work and waits for that redirect.
    Reset values follow manual table 2.1, p.22.
*/

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        //R7 reset strip: pure VALUE lanes (pc/inst/operand/data/address words) carry NO
        //reset arm - every consumer is gated by a valid/we/armed control bit that IS
        //reset, so pre-first-load garbage is never architecturally consumed. Dropping
        //the async-clear pin off the wide lanes shrinks the core_rst_n tree (3291 loads)
        //and unblocks physical-synthesis retiming across the data registers.
        ifid.valid       <= 1'b0;
        ifid.fetch_fault <= 1'b0;
        ifid.delay_slot  <= 1'b0;
        ifid.pd          <= '0;
        idex.valid          <= 1'b0;
        idex.delay_slot     <= 1'b0;
        fwd_lane_a          <= FWD_NONE;   //EX-head lane picks (shadow words = R7 strip)
        fwd_lane_b          <= FWD_NONE;
        fwd_lane_st         <= FWD_NONE;
        fwd_lane_a_agu      <= FWD_NONE;
        fwd_lane_b_agu      <= FWD_NONE;
        fwd_wbsel_a         <= 1'b0;
        fwd_wbsel_b         <= 1'b0;
        fwd_wbsel_st        <= 1'b0;
        fwd_wbsel_a_agu     <= 1'b0;
        fwd_wbsel_b_agu     <= 1'b0;
        fwd_dep_a           <= 1'b0;
        fwd_dep_b           <= 1'b0;
        fwd_dep_st          <= 1'b0;
        fwd_dep_a_agu       <= 1'b0;
        fwd_dep_b_agu       <= 1'b0;
        idex.src_a_used     <= 1'b0;
        idex.src_a_id       <= 5'd0;
        idex.src_b_used     <= 1'b0;
        idex.src_b_id       <= 5'd0;
        idex.store_used     <= 1'b0;
        idex.store_id       <= 5'd0;
        idex.gpr0_we        <= 1'b0;
        idex.gpr0_dst       <= 5'd0;
        idex.gpr1_we        <= 1'b0;
        idex.gpr1_dst       <= 5'd0;
        idex.alu_op         <= alu_op_t'(0);
        idex.alu_class      <= alu_class_t'(0);
        idex.mem_op         <= mem_op_t'(0);
        idex.mem_size       <= mem_size_t'(0);
        idex.addr_op        <= addr_op_t'(0);
        idex.load_signed    <= 1'b0;
        idex.byte_op        <= byte_op_t'(0);
        idex.is_data        <= 1'b0;
        idex.agu_en_mode    <= 2'd0;
        idex.branch_op      <= branch_op_t'(0);
        idex.branch_delayed <= 1'b0;
        idex.pr_link        <= 1'b0;
        idex.pr_we          <= 1'b0;
        idex.ctrl_dst       <= ctrl_dst_t'(0);
        idex.mac_cmd        <= mac_cmd_t'(0);
        idex.div_op         <= div_op_t'(0);
        idex.t_write_decode <= 1'b0;
        idex.t_decode_value <= 1'b0;
        idex.s_write_decode <= 1'b0;
        idex.s_decode_value <= 1'b0;
        idex.event_trapa    <= 1'b0;
        idex.trapa_imm      <= 8'd0;
        idex.event_rte      <= 1'b0;
        idex.event_sleep    <= 1'b0;
        idex.event_ldtlb    <= 1'b0;
        idex.fetch_fault    <= 1'b0;
        idex.illegal        <= 1'b0;
        idex.privileged     <= 1'b0;
        idex_is_data_agu <= 1'b0;
        exma.valid        <= 1'b0;
        exma.delay_slot   <= 1'b0;
        exma.gpr0_we      <= 1'b0;
        exma.gpr0_dst     <= 5'd0;
        exma.gpr1_we      <= 1'b0;
        exma.gpr1_dst     <= 5'd0;
        exma.mem_op       <= mem_op_t'(0);
        exma.mem_size     <= mem_size_t'(0);
        exma.load_signed  <= 1'b0;
        exma.byte_op      <= byte_op_t'(0);
        exma.store_wstrb  <= 4'd0;
        exma.mac_cmd      <= mac_cmd_t'(0);
        exma.mac_saturate <= 1'b0;
        exma.pr_we        <= 1'b0;
        exma.ctrl_dst     <= ctrl_dst_t'(0);
        exma.t_we         <= 1'b0;
        exma.t_data       <= 1'b0;
        exma.s_we         <= 1'b0;
        exma.s_data       <= 1'b0;
        exma.mq_we        <= 2'd0;
        exma.mq_data      <= 2'd0;
        exma.event_trapa  <= 1'b0;
        exma.trapa_imm    <= 8'd0;
        exma.event_rte    <= 1'b0;
        exma.event_sleep  <= 1'b0;
        exma.event_ldtlb  <= 1'b0;
        exma.fault        <= 1'b0;
        exma.fault_cause  <= 3'd0;
        exma.fault_write  <= 1'b0;
        mawb.valid        <= 1'b0;
        mawb.delay_slot   <= 1'b0;
        mawb.gpr0_we      <= 1'b0;
        mawb.gpr0_dst     <= 5'd0;
        mawb.gpr1_we      <= 1'b0;
        mawb.gpr1_dst     <= 5'd0;
        mawb.mac_cmd      <= mac_cmd_t'(0);
        mawb.mac_saturate <= 1'b0;
        mawb.pr_we        <= 1'b0;
        mawb.ctrl_dst     <= ctrl_dst_t'(0);
        mawb.t_we         <= 1'b0;
        mawb.t_data       <= 1'b0;
        mawb.s_we         <= 1'b0;
        mawb.s_data       <= 1'b0;
        mawb.mq_we        <= 2'd0;
        mawb.mq_data      <= 2'd0;
        mawb.event_trapa  <= 1'b0;
        mawb.trapa_imm    <= 8'd0;
        mawb.event_rte    <= 1'b0;
        mawb.event_sleep  <= 1'b0;
        mawb.event_ldtlb  <= 1'b0;
        mawb.fault        <= 1'b0;
        mawb.fault_cause  <= 3'd0;
        mawb.fault_write  <= 1'b0;
        fetch_pending   <= 1'b0;
        fetch_drop      <= 1'b0;
        fault_hold      <= 1'b0;
        fetch_pc        <= RESET_PC;
        fetch_pending_pc<= 32'd0;
        pair_ready      <= 1'b0;
        pair_pc         <= 32'd0;
        pair_inst       <= 16'd0;
        pair_pd         <= '0;
        mac_started       <= 1'b0;
        mac_armed         <= 1'b0;
        agu_base_q        <= 32'd0;
        au_addend_q       <= 32'd0;
        r_t             <= 1'b0;   //running SR.T flag; reset matches SR.T=0
        r_s             <= 1'b0;   //running SR.S flag; resyncs to committed SR on first drain
        r_m             <= 1'b0;   //running SR.M flag; resyncs to committed SR on first drain
        r_q             <= 1'b0;   //running SR.Q flag; resyncs to committed SR on first drain
        pr              <= 32'd0; //MACH/MACL reset inside u_mac_dsp
        o_EXC_VALID          <= 1'b0;
        o_EXC_CAUSE          <= EXC_NONE;
        o_EXC_PC             <= 32'd0;
        o_EXC_IN_DELAY_SLOT  <= 1'b0;
        o_EXC_ACCESS_WRITE   <= 1'b0;
        o_EXC_ACCESS_ADDR    <= 32'd0;
        o_TRAPA_VALID        <= 1'b0;
        o_TRAPA_IMM          <= 8'd0;
        o_RTE_VALID          <= 1'b0;
        o_SLEEP_VALID        <= 1'b0;
        o_LDTLB_VALID        <= 1'b0;
        o_RETIRE_VALID       <= 1'b0;
        retire_int_defer     <= 1'b0;
        o_RETIRE_PC          <= 32'd0;
        o_RETIRE_INST        <= 16'd0;

        o_RETIRE_GPR_WE      <= 1'b0;
        o_RETIRE_GPR         <= 5'd0;
        o_RETIRE_GPR_DATA    <= 32'd0;
    end
    else begin if(cen) begin
        //Event and trace outputs pulse for one enabled clock.
        o_EXC_VALID         <= 1'b0;
        o_EXC_ACCESS_WRITE  <= 1'b0;
        o_TRAPA_VALID       <= 1'b0;
        o_RTE_VALID         <= 1'b0;
        o_SLEEP_VALID       <= 1'b0;
        o_LDTLB_VALID       <= 1'b0;
        o_RETIRE_VALID      <= 1'b0;
        retire_int_defer    <= 1'b0;
        o_RETIRE_GPR_WE     <= 1'b0;

        //External reset/exception control has priority over all internal advancement.
        //ifid / fetch_pc / fetch_pending / fetch_drop moved to the railed capture block
        //at the tail of this process (their redirect terms are folded into the rails).
        if(i_REDIRECT_VALID) begin
            idex          <= '0;
            idex_is_data_agu <= 1'b0;
            fwd_lane_a    <= FWD_NONE;
            fwd_lane_b    <= FWD_NONE;
            fwd_lane_st   <= FWD_NONE;
            fwd_lane_a_agu<= FWD_NONE;
            fwd_lane_b_agu<= FWD_NONE;
            fwd_wbsel_a   <= 1'b0;
            fwd_wbsel_b   <= 1'b0;
            fwd_wbsel_st  <= 1'b0;
            fwd_wbsel_a_agu <= 1'b0;
            fwd_wbsel_b_agu <= 1'b0;
            fwd_dep_a     <= 1'b0;
            fwd_dep_b     <= 1'b0;
            fwd_dep_st    <= 1'b0;
            fwd_dep_a_agu <= 1'b0;
            fwd_dep_b_agu <= 1'b0;
            exma          <= '0;
            mawb          <= '0;
            fault_hold    <= 1'b0;
            r_t           <= i_SR[0];   //exception/RTE entry: resync running flags to committed SR
            r_s           <= i_SR[1];   //SR.S
            r_m           <= i_SR[9];   //SR.M
            r_q           <= i_SR[8];   //SR.Q
            mac_started       <= 1'b0;
            mac_armed         <= 1'b0;
        end
        else begin
            //WB is the sole normal commit point for registers and state events.
            if(wb_valid) begin
                if(mawb.fault) begin
                    //Fault metadata identifies the instruction and delay-slot state.
                    o_EXC_VALID         <= 1'b1;
                    o_EXC_CAUSE         <= mawb.fault_cause;
                    o_EXC_PC            <= mawb.pc;
                    o_EXC_IN_DELAY_SLOT <= mawb.delay_slot;
                    o_EXC_ACCESS_WRITE  <= mawb.fault_write;
                    o_EXC_ACCESS_ADDR   <= mawb.fault_addr;
                end
                else begin
                    //Write lane zero normally carries the instruction's main result.
                    if(mawb.gpr0_we) begin
                        if(mawb.gpr0_dst == 5'd0) gpr_r0_bank0 <= mawb.gpr0_data;
                        if(mawb.gpr0_dst == 5'd8) gpr_r0_bank1 <= mawb.gpr0_data;
                        o_RETIRE_GPR_WE   <= 1'b1;
                        o_RETIRE_GPR      <= mawb.gpr0_dst;
                        o_RETIRE_GPR_DATA <= mawb.gpr0_data;
                    end
                    //Write lane one carries address updates from pre/post addressing.
                    if(mawb.gpr1_we) begin
                        if(mawb.gpr1_dst == 5'd0) gpr_r0_bank0 <= mawb.gpr1_data;
                        if(mawb.gpr1_dst == 5'd8) gpr_r0_bank1 <= mawb.gpr1_data;
                        if(!mawb.gpr0_we) begin
                            o_RETIRE_GPR_WE   <= 1'b1;
                            o_RETIRE_GPR      <= mawb.gpr1_dst;
                            o_RETIRE_GPR_DATA <= mawb.gpr1_data;
                        end
                    end
                    //MACH/MACL commit inside u_mac_dsp, driven by mac_commit below.
                    if(mawb.pr_we) pr <= mawb.pr_data;
                    o_TRAPA_VALID <= mawb.event_trapa;
                    o_TRAPA_IMM   <= mawb.trapa_imm;
                    o_RTE_VALID   <= mawb.event_rte;
                    o_SLEEP_VALID <= mawb.event_sleep;
                    o_LDTLB_VALID <= mawb.event_ldtlb;
                end

                if(!mawb.fault) begin
                    o_RETIRE_VALID <= 1'b1;
                    o_RETIRE_PC    <= mawb.pc;
                    o_RETIRE_INST  <= mawb.inst;
                    retire_int_defer <= mawb.dbr;    //slot still owed: defer acceptance
                end
            end

            //MA/WB is a real stage register; WB commits the previous packet.
            if(ma_complete) begin
                mawb       <= ma_result;
                mawb.valid <= exma.valid;
            end
            else begin
                mawb <= '0;
            end

            //EX/MA advances after non-memory work or a completed memory response.
            //EX/MA uses exma_allow as a direct clock-enable and loads ex_result
            //unconditionally; only valid is gated by ex_advance, keeping the deep
            //ex_advance cone off the exma data paths (notably exma.mem_op, which
            //selects the DSP operand). A non-advancing slot is valid=0; the else
            //memory state machine only runs when exma.valid is set, and every other
            //consumer gates on exma.valid, so the data fields are don't-care.
            //u_ma_seq owns data_req_sent / ma_second_access / ma_first_value and the
            //phase-two request; this block keeps only the DSP arming state.
            if(exma_allow) begin
                exma             <= ex_result;
                exma.valid       <= ex_advance;
                mac_started       <= 1'b0;
                mac_armed         <= 1'b0;
            end
            else begin
                //Latch operands one cycle before launch (p.103 latency budget); the
                //multiply then reads the registered dsp_a/dsp_b instead of the
                //deep-cone-fed operand mux.
                if(mac_capture) begin
                    dsp_a     <= mac_operand_a;
                    dsp_b     <= mac_operand_b;
                    mac_armed <= 1'b1;
                end
                if(mac_start) mac_started <= 1'b1;
            end

            //Running SR.T latch. Each leaving instruction deposits its T at its own boundary
            //so the next EX reads r_t as a plain register (no forward) and it holds across
            //bubbles. A drained pipeline (nothing in MA/WB) resyncs to the committed SR.T;
            //ALU/compare T is known leaving EX; byte-RMW T only when leaving MA. Listed
            //oldest-to-newest so the newest write (EX) wins when several land the same edge.
            if(!exma.valid && !mawb.valid) r_t <= i_SR[0];
            if(ma_complete && exma.valid && !exma.fault && !wb_fault_pending &&
               exma.mem_op == MEM_RMW && ma_result.t_we) r_t <= ma_result.t_data;
            if(ex_advance && alu_t_we) r_t <= alu_t_value;

            //Running SR.S/M/Q latches - same model as r_t (deposit leaving EX, hold across
            //bubbles, resync to committed SR on drain). SETS/CLRS (S) and DIV0S/DIV0U/DIV1
            //(M/Q) all produce in EX, so unlike r_t there is no byte-RMW (MA) producer. Drain
            //listed first so an EX write the same edge wins (newest). Closes the 2-ahead hole
            //that the old i_SR+exma forward still left open for S/M/Q (would mis-run a cacheable
            //DIV1 or MAC chain at 2-apart spacing - the same flavor as the cached DT;BF bug).
            if(!exma.valid && !mawb.valid) begin r_s <= i_SR[1]; r_m <= i_SR[9]; r_q <= i_SR[8]; end
            if(ex_advance && idex.s_write_decode) r_s <= idex.s_decode_value;
            if(ex_advance && div_mq_we[1])        r_m <= div_mq_data[1];
            if(ex_advance && div_mq_we[0])        r_q <= div_mq_data[0];

            //ID/EX uses idex_allow as a direct clock-enable and loads operands
            //unconditionally; only valid is gated by id_issue. This keeps the deep
            //id_issue cone off the 32-bit operand data paths (it previously selected
            //the bubble-clear mux). An unissued slot is valid=0, and every consumer
            //gates on idex.valid, so its data fields are don't-care.
            if(idex_allow) begin
                idex             <= id_decode;
                idex.valid       <= id_issue;
                idex.is_data     <= id_issue && (id_decode.mem_op != MEM_NONE);  //AGU select, pre-decoded in ID (mirrors valid)
                //AGU addend gate, pre-decoded in ID: base-only modes (REG/POSTINC/MAC, and NONE)
                //NULL the addend, all others FORCE it. Registered so the EX addr_op decode leaves the
                //idex.addr_op -> AGU i_AGU_B/o_ADDR carry cone (the 5 ns bram_addr path). See 1a/1b.
                idex.agu_en_mode <= (id_decode.addr_op == ADDR_REG     ||
                                     id_decode.addr_op == ADDR_POSTINC ||
                                     id_decode.addr_op == ADDR_MAC     ||
                                     id_decode.addr_op == ADDR_NONE) ? 2'd0 : 2'd1;
                //preserved AGU-only duplicate: same cone as idex.is_data above
                idex_is_data_agu <= id_issue && (id_decode.mem_op != MEM_NONE);
                //Pre-decoded address-update addend (see au_addend_q declaration).
                au_addend_q <= (id_decode.addr_op == ADDR_PREDEC)  ? (~id_mem_step + 32'd1) :
                               (id_decode.addr_op == ADDR_POSTINC) ? id_mem_step :
                               (id_decode.addr_op == ADDR_MAC)     ?
                                   ((id_decode.src_a_id == id_decode.src_b_id)
                                        ? {id_mem_step[30:0], 1'b0} : id_mem_step) :
                               32'd0;
                if(id_issue && branch_event && branch_delayed) idex.delay_slot <= 1'b1;
            end
            //Operand captures on the PER-CLUSTER idex_allow duplicates (identical value,
            //placement-local enables). Written after the main load so they own the field.
            if(idex_allow_opa) idex.src_a_value <= id_src_a_value;
            if(idex_allow_opb) begin
                idex.src_b_value <= id_src_b_value;
                idex.store_value <= id_store_value;
            end

            //AGU base shadow load. agu_base_q is the registered i_AGU_A base (the fetch PC rides the
            //separate i_FETCH_PC mux, so this carries ONLY the data EA base / MAC/RMW 2nd-access addr):
            //  1. MAC/RMW 2nd access (EA2 / RMW write addr) - rides the REGISTERED exma field, NOT the
            //     live AGU output (keeps idex.addr_op->o_ADDR off this load, Route A). That cycle is
            //     always a stall (exma incomplete -> idex_allow=0), so it is caught here first. NOTE:
            //     TAS.B (BYTE_TST) is excluded here and instead HOLDS its base (else branch below).
            //  2. Otherwise, on an advance, load id_src_a_value (the fresh EA base); a stalled data op
            //     holds (idex_allow=0). Non-data instrs load a harmless base (unused, i_USE_BASE=0).
            //CE/select are the railed picks (lp_agu_ce / rl_agu_hold): hold-load wins,
            //else an idex_allow advance loads the fresh EA base - same priority as the
            //old if/else-if chain, with do_*_valid on ONE final LUT of the CE.
            if(agu_ce)
                agu_base_q <= agu_hold_sel ? (agu_hold_mac ? exma.mem_addr_second
                                                           : exma.mem_addr)
                                           : id_src_a_value;

            //EX-head forward state. Lanes latch at issue. fwd_wbsel_* REGISTERS the
            //"read the WB view" pick so the operand/AGU mux selects are single FFs:
            //set at issue for a load release (id_lane == FWD_WB), or at the
            //producer's drain edge (exma_allow under a held consumer; self-holds).
            //The shadow captures the mawb word on the first wbsel cycle (reg->reg,
            //CE from FFs only); fwd_dep_* = wbsel one held-cycle later flips the
            //fold from the one-live-cycle mawb word to the parked shadow. The deep
            //idex_allow/exma_allow cones ride ONLY these 1-bit Ds.
            if(!fwd_dep_a  && fwd_wbsel_a ) fwd_shadow_a  <= fwd_mawb_word_a;
            if(!fwd_dep_b  && fwd_wbsel_b ) fwd_shadow_b  <= fwd_mawb_word_b;
            if(!fwd_dep_st && fwd_wbsel_st) fwd_shadow_st <= fwd_mawb_word_st;
            if(idex_allow) begin
                fwd_lane_a      <= id_lane_a;
                fwd_lane_b      <= id_lane_b;
                fwd_lane_st     <= id_lane_st;
                fwd_lane_a_agu  <= id_lane_a;
                fwd_lane_b_agu  <= id_lane_b;
                fwd_wbsel_a     <= id_lane_a  == FWD_WB;
                fwd_wbsel_b     <= id_lane_b  == FWD_WB;
                fwd_wbsel_st    <= id_lane_st == FWD_WB;
                fwd_wbsel_a_agu <= id_lane_a  == FWD_WB;
                fwd_wbsel_b_agu <= id_lane_b  == FWD_WB;
                fwd_dep_a       <= 1'b0;
                fwd_dep_b       <= 1'b0;
                fwd_dep_st      <= 1'b0;
                fwd_dep_a_agu   <= 1'b0;
                fwd_dep_b_agu   <= 1'b0;
            end
            else begin
                //held consumer: the drain edge switches the port to the WB view
                if(fwd_lane_a  != FWD_NONE && exma_allow) begin
                    fwd_wbsel_a <= 1'b1; fwd_wbsel_a_agu <= 1'b1; end
                if(fwd_lane_b  != FWD_NONE && exma_allow) begin
                    fwd_wbsel_b <= 1'b1; fwd_wbsel_b_agu <= 1'b1; end
                if(fwd_lane_st != FWD_NONE && exma_allow) fwd_wbsel_st <= 1'b1;
                if(fwd_wbsel_a ) begin fwd_dep_a  <= 1'b1; fwd_dep_a_agu <= 1'b1; end
                if(fwd_wbsel_b ) begin fwd_dep_b  <= 1'b1; fwd_dep_b_agu <= 1'b1; end
                if(fwd_wbsel_st) fwd_dep_st <= 1'b1;
            end

            //Recognized WB fault kills younger packets and blocks further requests.
            //u_ma_seq clears its own state from ma_flush (= this same condition).
            //ifid / fetch_drop moved to the railed capture block below (folded in).
            if(wb_valid && mawb.fault) begin
                idex          <= '0;
                idex_is_data_agu <= 1'b0;
                fwd_lane_a    <= FWD_NONE;
                fwd_lane_b    <= FWD_NONE;
                fwd_lane_st   <= FWD_NONE;
                fwd_lane_a_agu<= FWD_NONE;
                fwd_lane_b_agu<= FWD_NONE;
                fwd_wbsel_a   <= 1'b0;
                fwd_wbsel_b   <= 1'b0;
                fwd_wbsel_st  <= 1'b0;
                fwd_wbsel_a_agu <= 1'b0;
                fwd_wbsel_b_agu <= 1'b0;
                fwd_dep_a     <= 1'b0;
                fwd_dep_b     <= 1'b0;
                fwd_dep_st    <= 1'b0;
                fwd_dep_a_agu <= 1'b0;
                fwd_dep_b_agu <= 1'b0;
                exma          <= '0;
                mawb          <= '0;
                mac_started       <= 1'b0;
                fault_hold    <= 1'b1;
            end
        end

        //IF/ID + fetch-slot captures, RAILED. One flat CE / data select per register,
        //picked from the rl_* composition rails in two LUT levels (lp_* nets) - the old
        //redirect-branch / branch-clear / accept / issue / fault writes above are ALL
        //folded into these rails with their priorities preserved. Placed last in the
        //process, replacing the last-write-wins overrides it absorbs.
        //ifid data fields ride ifid_ld only (zero loads '0); fetch_pc selects external
        //redirect > branch target > pc+2; fetch_pending_pc pipelines the OLD fetch_pc
        //this same edge (NBA), as before.
        //IF/ID valid/data SPLIT: only the valid bit needs the zero arm (redirect / WB
        //fault / branch clear / issue drain) - every data-field consumer is gated on
        //ifid.valid (or reads junk harmlessly: the GPR read-ahead, hazard terms under
        //id_issue). The 89-bit data cluster rides the load-only enable (a placement-
        //local keep duplicate), halving the deep CE net's fanout.
        if(ifid_zero)    ifid.valid <= 1'b0;
        else if(ifid_ld || pair_serve) ifid.valid <= 1'b1;
        if(pair_serve) begin
            //Held sibling insert: register-sourced, exclusive with the response insert.
            ifid.pc          <= pair_pc;
            ifid.inst        <= pair_inst;
            ifid.fetch_fault <= 1'b0;     //a pair is never captured from a faulted response
            ifid.delay_slot  <= 1'b0;
            ifid.pd          <= pair_pd;
        end
        else if(ifid_ld_dat) begin
            ifid.pc          <= fetch_pending_pc;
            ifid.inst        <= i_rsp_inst;
            ifid.fetch_fault <= i_rsp_fault;
            ifid.delay_slot  <= 1'b0;
            ifid.pd          <= pd_fetch; //predecoded source routing rides the instruction
        end

        //Fetch-pair slot: kill > capture > serve-consume.
        if(i_REDIRECT_VALID || wb_fault_kill || ifid_clr) pair_ready <= 1'b0;
        else if(pair_capture)                             pair_ready <= 1'b1;
        else if(pair_serve)                               pair_ready <= 1'b0;
        if(pair_capture) begin
            pair_pc   <= fetch_pc;        //= the sibling's PC (see the rails invariant)
            pair_inst <= i_rsp_sib;
            pair_pd   <= pd_fetch_sib;
        end

        if(fpc_ce)  fetch_pc <= i_REDIRECT_VALID ? i_REDIRECT_PC :
                                fpc_selbr        ? branch_target : fetch_pc + 32'd2;
        if(fp_ce)   fetch_pending <= i_req_fire;
        if(i_req_fire) fetch_pending_pc <= fetch_pc;
        if(drop_ce) fetch_drop <= drop_d;
    end end
end

endmodule

/* verilator lint_off DECLFILENAME */

/*
    MA-stage memory-access sequencer.

    Owns the second-access phase and drives the single D bus request port. Every
    memory op issues its FIRST access from EX (i_ex_req). For MAC.W/MAC.L and the
    byte RMW ops this sequencer then issues the SECOND access from the held MA
    packet: the MAC second read (mem_addr_second) or the RMW write (mem_addr).
    TST.B has no write, so it captures the byte and retires with no second request.

    The EX first access and the MA second access are mutually exclusive - EX is
    blocked (exma_allow = 0) while this packet holds the stage - so the request is
    a parallel phase-select, never a priority overlay. See SH3 Fig 10.29/10.35/10.37.
*/

module ma_seq import int_pipe_pkg::*; #(
    parameter        BIG_ENDIAN = 1'b1
) (
    /* CLOCK AND ENABLE */
    input   wire            i_CLK,
    input   wire            i_RST_n,
    input   wire            i_CEN,

    /* PIPELINE CONTROL */
    input   wire            i_advance,      //EX/MA loads a new packet this edge (exma_allow)
    input   wire            i_flush,        //redirect or recognized WB fault: drop state

    /* EX PRIMARY (FIRST) REQUEST */
    input   dbus_req_pkt_t  i_ex_req,       //assembled in EX; .valid gates the first access
    input   wire            i_ex_valid,     //i_ex_req.valid (early_d_req_valid)
    output  wire            o_ex_accept,    //first access accepted with the EX/MA edge

    /* HELD MA PACKET CONTEXT (SECOND ACCESS) */
    input   wire            i_ma_active,    //exma valid, fault-free, not flushing
    input   mem_op_t        i_ma_op,
    input   byte_op_t       i_ma_byte_op,
    input   mem_size_t      i_ma_size,
    input   wire    [31:0]  i_ma_addr,      //RMW write / MAC first-read address
    input   wire    [31:0]  i_ma_addr2,     //MAC second-read address
    input   wire    [7:0]   i_ma_imm,       //RMW immediate byte (exma.mac_b[7:0])
    input   wire    [31:0]  i_ma_capture,   //value latched on the first response

    /* D-BUS HANDSHAKE / RESPONSE */
    input   wire            i_req_ready,    //cache accept (the muxed L-bus ready; D whenever
                                            //a D request is up - l_is_data holds then)
    input   wire            i_rsp_valid,    //cache D response delivered this cycle
    input   wire            i_rsp_fault,

    /* OUTPUTS */
    output  dbus_req_pkt_t  o_req,          //unified request to the single D bus port
    output  wire            o_second,       //second access in progress (phase)
    output  wire            o_second_agu,   //AGU-only preserved duplicate of o_second
    output  wire            o_req_sent,     //request accepted, response still pending
    output  wire            o_req_sent_agu, //AGU-only preserved duplicate of o_req_sent
    output  wire    [31:0]  o_first_value   //captured first operand / original RMW byte
);

logic           second_access;  //0 = first access, 1 = second access (MAC read2 / RMW write)
logic           req_sent;       //a request has been accepted; response is pending
logic   [31:0]  first_value;    //first MAC operand or original RMW byte

//Manual register duplication for the AGU control cone. second_access fans out to ~112
//loads; preserve stops Quartus merging these copies back into the originals, so the
//fitter can place them beside the AGU half-cycle (cen_p->cen_n) i_USE_BASE / i_EN_MODE
//nets. D-cones mirror the originals exactly; only the AGU loads their Q outputs.
(* preserve *) logic    second_access_agu;
(* preserve *) logic    req_sent_agu;

//Scalar handshake products. Each next-state below composes the ORIGINAL nested-if
//priority exactly: capture beats fire beats hold, advance reloads, flush clears.
logic           data_resp, ex_accept, fire, cap;
logic           req_sent_nx, second_nx, fv_ce;

//Second access is valid only in phase two, for MAC reads or non-TST RMW writes.
wire    ma_req_valid = i_ma_active && !req_sent && second_access &&
                       (i_ma_op == MEM_MAC ||
                        (i_ma_op == MEM_RMW && i_ma_byte_op != BYTE_TST));

//RMW phase-two write data: modify the captured byte, replicate, enable one lane.
logic   [7:0]   rmw_byte;
always_comb begin
    case(i_ma_byte_op)
        BYTE_AND: rmw_byte = first_value[7:0] & i_ma_imm;
        BYTE_OR:  rmw_byte = first_value[7:0] | i_ma_imm;
        BYTE_XOR: rmw_byte = first_value[7:0] ^ i_ma_imm;
        default:  rmw_byte = first_value[7:0] | 8'h80; //TAS.B sets bit seven
    endcase
end
wire    [31:0]  rmw_wdata = {4{rmw_byte}};
wire    [3:0]   rmw_wstrb = BIG_ENDIAN ? (4'b1000 >> i_ma_addr[1:0])
                                       : (4'b0001 << i_ma_addr[1:0]);

//One request descriptor. The data fields (address, size, store data/strobe, lock)
//select on the REGISTERED phase bit second_access, NOT on the request-valid - so the
//deep valid cone (exma_allow, alignment, idex decode) stays off the half-cycle address
//and store paths. Only o_req.valid carries the qualification, which it must: it gates
//actually issuing the access. EX first access and MA second access never overlap
//(EX is blocked while MA holds the stage), so the valid is simply their OR.
dbus_req_pkt_t  d_req;
always_comb begin
    if(second_access) begin
        d_req.write = i_ma_op == MEM_RMW;
        d_req.size  = i_ma_size;
        //The second-access address now rides the AGU (i_ex_req.addr = agu_base_q during the
        //dispatch), so no second-access address mux sits on the cen_n path. i_ma_addr/i_ma_addr2
        //remain for fault_addr reporting and the RMW write-strobe lane below.
        d_req.addr  = i_ex_req.addr;
        d_req.wdata = rmw_wdata;
        d_req.wstrb = i_ma_op == MEM_RMW ? rmw_wstrb : 4'b0000;
        d_req.lock  = i_ma_op == MEM_RMW && i_ma_byte_op != BYTE_TST;
        d_req.pref  = 1'b0;
    end
    else begin
        d_req.write = i_ex_req.write;
        d_req.size  = i_ex_req.size;
        d_req.addr  = i_ex_req.addr;
        d_req.wdata = i_ex_req.wdata;
        d_req.wstrb = i_ex_req.wstrb;
        d_req.lock  = i_ex_req.lock;
        d_req.pref  = i_ex_req.pref;
    end
    d_req.valid = (i_ex_req.valid && !second_access) | ma_req_valid;  //phase gate (see ex_val)
end

//EX-valid phase gate: on the phase-two COMPLETION-response cycle exma_allow opens
//combinationally, so the next instruction's EX request raises valid while the
//field mux (registered second_access) still presents the stale phase-two write -
//a one-cycle PHANTOM the bus could accept as a spurious extra locked write.
//Found by the random-interrupt oracle's D-request stability checker. Gating with
//the registered phase bit costs nothing legitimate: any acceptance on that cycle
//would carry wrong fields by construction; the request presents cleanly one
//cycle later when second_access has cleared.
wire    ex_val = i_ex_valid && !second_access;

//Next-state tail: original priority "flush > advance(reload) > {fire sets, capture
//clears}", with capture written LAST in the old block so it beats a same-edge fire.
always_comb begin
    data_resp   = req_sent && i_rsp_valid;
    ex_accept   = ex_val && i_req_ready;
    fire        = (ex_val | ma_req_valid) && i_req_ready;
    cap         = data_resp && !second_access && !i_rsp_fault &&
                  (i_ma_op == MEM_MAC || i_ma_op == MEM_RMW);
    req_sent_nx = i_flush ? 1'b0 :
                  i_advance ? ex_accept :
                  cap ? 1'b0 : (fire ? 1'b1 : req_sent);
    second_nx   = i_flush ? 1'b0 :
                  i_advance ? 1'b0 :
                  cap ? 1'b1 : second_access;
    fv_ce       = !i_flush && !i_advance && cap;
end

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        req_sent      <= 1'b0;
        second_access <= 1'b0;
        //first_value: R7 strip - MAC capture word, consumed only under exma MEM_MAC
        req_sent_agu      <= 1'b0;
        second_access_agu <= 1'b0;
    end
    else if(i_CEN) begin
        req_sent          <= req_sent_nx;
        second_access     <= second_nx;
        req_sent_agu      <= req_sent_nx;
        second_access_agu <= second_nx;
        if(fv_ce) first_value <= i_ma_capture;
    end
end

assign  o_req          = d_req;
assign  o_ex_accept    = ex_accept;
assign  o_second       = second_access;
assign  o_second_agu   = second_access_agu;
assign  o_req_sent     = req_sent;
assign  o_req_sent_agu = req_sent_agu;
assign  o_first_value  = first_value;

endmodule

/* verilator lint_on DECLFILENAME */

/*
    Multiply / multiply-accumulate unit owning the MACH and MACL registers.
    Registered '*' results infer FPGA DSP hardware. Word multiplication finishes
    in one architectural clock; long multiplication finishes in two clocks.
    Accumulation shares the final clock and adds no extra operation latency.

    SR.S selects saturation defined by the SH programming manual, pp.308-314:
    MAC.L clamps the valid low 48-bit accumulator and preserves MACH[31:16].
    MAC.W clamps signed MACL to 32 bits and sets MACH[0] on overflow.
    Pending results update architectural MACH/MACL only when WB asserts commit.
*/

/* verilator lint_off DECLFILENAME */
module mac_dsp (
    input   wire            i_CLK,
    input   wire            i_RST_n,
    input   wire            i_CEN,         //architectural clock enable from int_pipe
    input   wire            i_CANCEL,      //discard an uncommitted operation after redirect
    input   wire            i_START,       //capture one multiply operation
    input   wire            i_COMMIT,      //retire one MACH/MACL command this enabled cycle
    input   wire            i_CLEAR,       //CLRMAC clears both registers
    input   wire            i_LOAD_MACH,   //LDS Rm,MACH writes i_LOAD_DATA
    input   wire            i_LOAD_MACL,   //LDS Rm,MACL writes i_LOAD_DATA
    input   wire            i_ACCUMULATE,  //add product to the linked MACH:MACL value
    input   wire            i_FULL_LONG_RESULT, //write both halves for DMULS.L/DMULU.L
    input   wire            i_WORD,        //16x16 operation when set; otherwise 32x32
    input   wire            i_SIGNED,      //select signed multiplication for word or long input
    input   wire            i_SATURATE,    //captured SR.S for MAC.W or MAC.L
    input   wire    [31:0]  i_LOAD_DATA,   //LDS source value applied at precise WB
    input   wire    [31:0]  i_A,           //multiplicand
    input   wire    [31:0]  i_B,           //multiplier
    output  logic           o_DONE,        //pending multiply result is ready to commit
    output  logic   [31:0]  o_MACH,
    output  logic   [31:0]  o_MACL
);

import int_pipe_pkg::*;

logic signed [32:0] multiplicand_long;  //zero-extended unsigned or sign-extended signed input
logic signed [32:0] multiplier_long;    //zero-extended unsigned or sign-extended signed input
logic signed [65:0] product_long_wide;  //33x33 product contains every signed/unsigned result bit
logic        [63:0] product_long_input; //low 64 architectural product bits
logic        [63:0] product_long_z;
logic        [31:0] product_word_u;
logic signed [31:0] product_word_s;
logic signed [63:0] product_word_extended;
logic        [63:0] accumulator_z;
logic                long_busy;
logic                long_accumulate_z;
logic                long_full_result_z;
logic                long_saturate_z;
logic        [63:0] pending_result;

assign  multiplicand_long   = $signed({i_SIGNED & i_A[31], i_A});
assign  multiplier_long     = $signed({i_SIGNED & i_B[31], i_B});
assign  product_long_wide   = multiplicand_long * multiplier_long;
assign  product_long_input  = product_long_wide[63:0];
assign  product_word_u      = i_A[15:0] * i_B[15:0];
assign  product_word_s      = $signed(i_A[15:0]) * $signed(i_B[15:0]);
assign  product_word_extended = i_SIGNED ? {{32{product_word_s[31]}}, product_word_s}
                                         : {32'd0, product_word_u};

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        o_DONE            <= 1'b0;
        o_MACH            <= 32'd0;
        o_MACL            <= 32'd0;
        product_long_z     <= 64'sd0;
        accumulator_z      <= 64'd0;
        long_busy          <= 1'b0;
        long_accumulate_z  <= 1'b0;
        long_full_result_z <= 1'b0;
        long_saturate_z    <= 1'b0;
        pending_result     <= 64'd0;
    end
    else begin if(i_CEN) begin
        if(i_CANCEL) begin
            o_DONE   <= 1'b0;
            long_busy<= 1'b0;
        end
        else begin
            if(i_COMMIT) begin
                if(i_CLEAR) begin
                    o_MACH <= 32'd0;
                    o_MACL <= 32'd0;
                end
                else if(i_LOAD_MACH) o_MACH <= i_LOAD_DATA;
                else if(i_LOAD_MACL) o_MACL <= i_LOAD_DATA;
                else if(o_DONE) begin
                    o_MACH <= pending_result[63:32];
                    o_MACL <= pending_result[31:0];
                end
                o_DONE <= 1'b0;
            end

            //The registered long product becomes ready during the second cycle.
            if(long_busy) begin
                if(long_accumulate_z) begin
                    pending_result <= accumulated_result(accumulator_z, product_long_z,
                                                         1'b0, long_saturate_z);
                end
                else if(long_full_result_z) begin
                    pending_result <= product_long_z;
                end
                else begin
                    pending_result <= {accumulator_z[63:32], product_long_z[31:0]};
                end
                long_busy <= 1'b0;
                o_DONE    <= 1'b1;
            end

            if(i_START) begin
                accumulator_z <= {o_MACH, o_MACL};
                o_DONE       <= 1'b0;
                if(i_WORD) begin
                    pending_result <= i_ACCUMULATE ?
                                      accumulated_result({o_MACH, o_MACL},
                                                         product_word_extended,
                                                         1'b1, i_SATURATE) :
                                      {o_MACH, product_word_extended[31:0]};
                    o_DONE <= 1'b1;
                end
                else begin
                    product_long_z    <= product_long_input;
                    long_accumulate_z <= i_ACCUMULATE;
                    long_full_result_z <= i_FULL_LONG_RESULT;
                    long_saturate_z   <= i_SATURATE;
                    long_busy         <= 1'b1;
                end
            end
        end
    end end
end

endmodule
/* verilator lint_on DECLFILENAME */

`default_nettype none
