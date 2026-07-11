`default_nettype wire

/*
    Direct memory access controller DMAC (SH7709S section 11, pp.327-387).

    This block is the DMAC's register face, the on-chip compare match
    timer CMT (section 11.4 - a 16-bit up-counter whose compare match is
    a DMA request source; it has NO INTC line on the SH7709S), and the
    transfer engine (request priority + start-up control + bus interface
    of Fig 11.1). The engine masters IBus_1 through the on-chip arbiter
    and calls the BSC like a function - external bus cycles are shaped
    by the BSC "in the same way as when the CPU is the bus master"
    (p.363). Scope: auto / CMT / external-DREQ requests, dual-direct +
    dual-indirect (ch3) + single-address units, byte/word/long + 16-byte
    (4-longword) sizes, cycle-steal + burst, fixed + round-robin
    priority, ch2 source reload, DACK/DRAK, NMIF/AE aborts (NMI edge
    from the INTC; alignment checks + bus faults). Illegal setups
    (section 11.6: 16-byte combined with dec/indirect/reload/on-chip
    RS) are NOT guarded - silicon says "operation not guaranteed".
    16-byte units tag their beats req_burst: the BSC chains the 4
    longwords as one back-to-back run (fig 11.11; burst-ROM envelope
    fig 23.19; SDRAM line ops) and ignores WAIT on the write runs per
    p.304 / 11.6 note 12 (dual 16-byte writes, single dev->mem).
    Deviations: 11.6 notes 4/13 (standby/sleep restrictions) are
    software rules - no standby machinery exists in this SoC yet.

    External request (ch0/1, section 11.3.2): DREQ is sampled on the
    CKIO falling edge (i_CKIO_NCEN); DS selects low-level or falling-
    edge detection (edge needs DREQ high at the previous sample, fig
    11.19). Cycle-steal withdraws the request at the FIRST transfer,
    burst-edge holds it to the LAST (fig 11.21: sample once, run to
    DMATCR=0); burst-level follows the live level per unit boundary.
    Deviation: sampling runs every CKIO fall instead of the 2-cycle
    one-step-ahead cadence - at least as responsive, laws locked in tb.
    DACK rides the sideband (AM resolved here, window framed in the
    BSC, AL pad polarity applied here); DRAK pulses one CKIO cycle at
    the accepting grant, RL polarity (p.337).

    Register window (tables 11.2 + 11.7): channel quads at 0x04000020 +
    0x10*n {SAR, DAR, DMATCR, CHCR}, DMAOR at 0x60, CMT at 0x70-76. The
    0x62-6F hole is never decoded (section 11.6 note 11) - the BSC's
    front-end excludes it. P2 aliases (0xA4000020+) land here through the
    BSC's shadow-invariant area-1 decode.

    Access laws: writes arrive right-justified from the P bridge and are
    lane-aligned here (big-endian, matching the chip top); a 16-bit
    access to a 32-bit register keeps the untouched half (p.332 note 2).
    TE/NMIF/AE/CMF are write-0-only flags (hardware set outranks a
    same-edge clear). Size legality (section 11.6 note 1: DMAOR 8/16,
    others 16/32) is NOT enforced - the TMU precedent.

    Reset: CHCR/DMAOR/CMT clear on power-on AND manual reset (p.332);
    SAR/DAR/DMATCR are architecturally undefined - reset-to-0 here.

    DEI0-3 transfer-end interrupt requests are LEVELS (TE AND IE),
    dropped by the handler's TE write-0 (INTC codes 0x800-0x860, IPRE).
*/

module dmac (
    /* CLOCK AND RESET - clears on any reset flavor (p.332) */
    input   wire            i_RST_n,
    input   wire            i_CLK,
    input   wire            i_CEN,
    input   wire            i_PCEN,         //P-phi enable (i_CEN-qualified) for the CMT prescaler
    input   wire            i_CKIO_NCEN,    //CKIO falling-edge enable: DREQ sample phase (p.363)

    /* INTERFACES */
    IBus_2.slave            REG_BUS,        //P bus window 0x04000020-77 (behind the BSC)
    IBus_1.master           I_BUS,          //transfer engine -> ibus_arb DMA leg

    /* BUS ARBITER HOOK */
    output  wire            o_BUS_HOLD,     //transfer-unit / burst bus hold (arb i_DMA_HOLD)

    /* ABORT HOOK - INTC NMI edge sets NMIF even while idle (11.6 note 3) */
    input   wire            i_NMI_SET,

    /* DREQ/DACK/DRAK - Port D pads (table 18.1); DACK windows from the BSC */
    input   wire    [1:0]   i_DREQ_n,       //DREQ0/1 pad levels (PTD4/PTD6, active-low)
    input   wire    [1:0]   i_DACK_WIN,     //BSC: CSn-framed active-high DACK windows
    output  wire    [1:0]   o_DACK,         //DACK0/1 pads (PTD5/PTD7, AL polarity)
    output  wire    [1:0]   o_DRAK,         //DRAK0/1 pads (PTD1/PTD0, RL polarity)

    /* INTERRUPT REQUESTS - levels, table 6.4 order */
    output  wire    [3:0]   o_DEI           //{DEI3, DEI2, DEI1, DEI0} = per-channel TE & IE
);

///////////////////////////////////////////////////////////
//////  Register Storage (shared: DMAOR + CMT)
////

logic   [1:0]   pr;                         //DMAOR[9:8]: channel priority mode (p.343)
logic           ae;                         //DMAOR[2]: address error flag, write-0-only
logic           nmif;                       //DMAOR[1]: NMI flag, write-0-only
logic           dme;                        //DMAOR[0]: DMA master enable
logic           cmstr_rsv;                  //CMSTR[1]: R/W spare, write 0 (p.377)
logic           cmstr_str0;                 //CMSTR[0]: CMCNT0 count start
logic           cmf;                        //CMCSR0[7]: compare match flag, write-0-only
logic           cmcsr_rsv;                  //CMCSR0[6]: R/W spare, write 0 (p.378)
logic   [1:0]   cks;                        //CMCSR0[1:0]: clock select P-phi/4/8/16/64
logic   [15:0]  cmcnt;                      //16-bit up-counter (p.379)
logic   [15:0]  cmcor;                      //compare match constant, resets H'FFFF (p.380)

wire    [15:0]  dmaor = {6'd0, pr, 5'd0, ae, nmif, dme};
wire    [15:0]  cmstr = {14'd0, cmstr_rsv, cmstr_str0};
wire    [15:0]  cmcsr = {8'd0, cmf, cmcsr_rsv, 4'd0, cks};



///////////////////////////////////////////////////////////
//////  Write Lane Alignment (big-endian)
////

//the P bridge right-justifies payloads; spread them onto the 32-bit register
//lanes so partial accesses mask cleanly: byte offset a lands in lane 3-a
logic   [31:0]  wd_lane;                    //write data replicated onto its lanes
logic   [3:0]   wm_lane;                    //byte-lane mask, [3] = bits 31:24
always_comb begin
    unique case(REG_BUS.size)
        2'd0: begin                         //byte
            wd_lane = {4{REG_BUS.wdata[7:0]}};
            wm_lane = 4'b1000 >> REG_BUS.addr[1:0];
        end
        2'd1: begin                         //word (16-bit)
            wd_lane = {2{REG_BUS.wdata[15:0]}};
            wm_lane = REG_BUS.addr[1] ? 4'b0011 : 4'b1100;
        end
        default: begin                      //longword
            wd_lane = REG_BUS.wdata;
            wm_lane = 4'b1111;
        end
    endcase
end



///////////////////////////////////////////////////////////
//////  Register Access Decode
////

wire            reg_wr = REG_BUS.stb && REG_BUS.we;

//channel quad select: 0x20 + 0x10*n, +0 SAR +4 DAR +8 DMATCR +C CHCR
logic   [3:0]   wr_sar, wr_dar, wr_tcr, wr_chcr;
always_comb begin
    for(int c = 0; c < 4; c++) begin
        logic sel;
        sel = reg_wr && (REG_BUS.addr[7:4] == 4'(c + 2));
        wr_sar[c]  = sel && (REG_BUS.addr[3:2] == 2'd0);
        wr_dar[c]  = sel && (REG_BUS.addr[3:2] == 2'd1);
        wr_tcr[c]  = sel && (REG_BUS.addr[3:2] == 2'd2);
        wr_chcr[c] = sel && (REG_BUS.addr[3:2] == 2'd3);
    end
end

wire            wr_dmaor = reg_wr && (REG_BUS.addr[7:2] == 6'b01_1000);     //0x60
wire            wr_cmt_a = reg_wr && (REG_BUS.addr[7:2] == 6'b01_1100);     //0x70: CMSTR+CMCSR0
wire            wr_cmt_b = reg_wr && (REG_BUS.addr[7:2] == 6'b01_1101);     //0x74: CMCNT0+CMCOR0



///////////////////////////////////////////////////////////
//////  Channel Register Quads (the Fig 11.1 x4 block)
////

wire    [31:0]  ch_sar  [0:3];
wire    [31:0]  ch_dar  [0:3];
wire    [23:0]  ch_tcr  [0:3];
wire    [31:0]  ch_chcr [0:3];

logic   [3:0]   ch_upd;                     //sequencer: unit completed on channel c
logic   [3:0]   ch_en;                      //live channel enables (request comb below);
                                            //fed back as i_EN for the ch2 reload counter

//feature asymmetry per pp.336-342: DREQ/DACK bits on ch0/1, reload on
//ch2, indirect on ch3; i_UPD drives each channel's iteration datapath
dmac_channel #(.CH_ID(0), .HAS_EXT(1'b1)) u_ch0 (
    .i_RST_n(i_RST_n), .i_CLK(i_CLK), .i_CEN(i_CEN),
    .i_WR_SAR(wr_sar[0]), .i_WR_DAR(wr_dar[0]), .i_WR_TCR(wr_tcr[0]), .i_WR_CHCR(wr_chcr[0]),
    .i_WDATA(wd_lane), .i_WMASK(wm_lane),
    .i_UPD(ch_upd[0]), .i_UPD_MASK(upd_mask), .i_EN(ch_en[0]),
    .o_SAR(ch_sar[0]), .o_DAR(ch_dar[0]), .o_TCR(ch_tcr[0]), .o_CHCR(ch_chcr[0])
);
dmac_channel #(.CH_ID(1), .HAS_EXT(1'b1)) u_ch1 (
    .i_RST_n(i_RST_n), .i_CLK(i_CLK), .i_CEN(i_CEN),
    .i_WR_SAR(wr_sar[1]), .i_WR_DAR(wr_dar[1]), .i_WR_TCR(wr_tcr[1]), .i_WR_CHCR(wr_chcr[1]),
    .i_WDATA(wd_lane), .i_WMASK(wm_lane),
    .i_UPD(ch_upd[1]), .i_UPD_MASK(upd_mask), .i_EN(ch_en[1]),
    .o_SAR(ch_sar[1]), .o_DAR(ch_dar[1]), .o_TCR(ch_tcr[1]), .o_CHCR(ch_chcr[1])
);
dmac_channel #(.CH_ID(2), .HAS_RELOAD(1'b1)) u_ch2 (
    .i_RST_n(i_RST_n), .i_CLK(i_CLK), .i_CEN(i_CEN),
    .i_WR_SAR(wr_sar[2]), .i_WR_DAR(wr_dar[2]), .i_WR_TCR(wr_tcr[2]), .i_WR_CHCR(wr_chcr[2]),
    .i_WDATA(wd_lane), .i_WMASK(wm_lane),
    .i_UPD(ch_upd[2]), .i_UPD_MASK(upd_mask), .i_EN(ch_en[2]),
    .o_SAR(ch_sar[2]), .o_DAR(ch_dar[2]), .o_TCR(ch_tcr[2]), .o_CHCR(ch_chcr[2])
);
dmac_channel #(.CH_ID(3), .HAS_INDIRECT(1'b1)) u_ch3 (
    .i_RST_n(i_RST_n), .i_CLK(i_CLK), .i_CEN(i_CEN),
    .i_WR_SAR(wr_sar[3]), .i_WR_DAR(wr_dar[3]), .i_WR_TCR(wr_tcr[3]), .i_WR_CHCR(wr_chcr[3]),
    .i_WDATA(wd_lane), .i_WMASK(wm_lane),
    .i_UPD(ch_upd[3]), .i_UPD_MASK(upd_mask), .i_EN(ch_en[3]),
    .o_SAR(ch_sar[3]), .o_DAR(ch_dar[3]), .o_TCR(ch_tcr[3]), .o_CHCR(ch_chcr[3])
);



///////////////////////////////////////////////////////////
//////  DMAOR and CMT Registers
////

/*
    DMAOR lives in word-view lanes 3:2 (16-bit register at 0x60, big-
    endian): PR = view bits 25:24, AE/NMIF/DME = view bits 18:16. AE and
    NMIF are write-0-only; NMIF sets on the INTC's NMI edge (i_NMI_SET),
    AE on the engine's address-error checks (ae_set, engine section).

    CMT (section 11.4): CMCNT0 counts up on the CKS-selected P-phi tap
    while STR0 = 1; at CMCNT0 == CMCOR0 the counter clears and CMF sets
    in the same tick (fig 11.27 - one request per period, no re-fire
    without a fresh input clock). A bus write to CMCNT0 wins over a
    same-edge tick; the CMF set outranks a same-edge write-0 clear
    (the TMU UNF pattern).
*/

//free-running P-phi prescaler; a tap fires when its divided count wraps
logic   [5:0]   psc;
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) psc <= 6'd0;
    else begin if(i_CEN) begin
        if(i_PCEN) psc <= psc + 6'd1;
    end end
end

logic           cmt_tick;                   //CKS-selected count enable (p.379)
always_comb begin
    unique case(cks)
        2'd0:    cmt_tick = i_PCEN & (psc[1:0] == 2'h3);    //P-phi/4
        2'd1:    cmt_tick = i_PCEN & (psc[2:0] == 3'h7);    //P-phi/8
        2'd2:    cmt_tick = i_PCEN & (psc[3:0] == 4'hF);    //P-phi/16
        default: cmt_tick = i_PCEN & (psc[5:0] == 6'h3F);   //P-phi/64
    endcase
end

wire            wr_cmcnt = wr_cmt_b && (wm_lane[3] || wm_lane[2]);  //CMCNT0 = view lanes 3:2
wire            cmt_match = cmstr_str0 && cmt_tick && (cmcnt == cmcor);
wire            cmt_fire  = cmt_match && !wr_cmcnt; //CMF set + DMA request (fig 11.27)

logic           ae_set;                     //engine: address-error abort (defined below)

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        pr    <= 2'd0;
        ae    <= 1'b0;
        nmif  <= 1'b0;
        dme   <= 1'b0;
        cmstr_rsv  <= 1'b0;
        cmstr_str0 <= 1'b0;
        cmf   <= 1'b0;
        cmcsr_rsv  <= 1'b0;
        cks   <= 2'd0;
        cmcnt <= 16'd0;
        cmcor <= 16'hFFFF;
    end
    else begin if(i_CEN) begin
        if(wr_dmaor) begin
            if(wm_lane[3]) pr <= wd_lane[25:24];
            if(wm_lane[2]) dme <= wd_lane[16];
        end

        //write-0-only flags (p.343): the hardware set outranks a same-edge
        //clear; either flag high drops every ch_en = all channels suspend
        if(i_NMI_SET)                   nmif <= 1'b1;
        else if(wr_dmaor && wm_lane[2]) nmif <= nmif & wd_lane[17];
        if(ae_set)                      ae   <= 1'b1;
        else if(wr_dmaor && wm_lane[2]) ae   <= ae & wd_lane[18];

        if(wr_cmt_a && wm_lane[2]) begin        //CMSTR = view lanes 3:2
            cmstr_rsv  <= wd_lane[17];
            cmstr_str0 <= wd_lane[16];
        end
        if(wr_cmt_a && wm_lane[0]) begin        //CMCSR0 = view lanes 1:0
            cmcsr_rsv <= wd_lane[6];
            cks       <= wd_lane[1:0];
        end

        if(wr_cmcnt) begin                      //bus write wins over the tick
            if(wm_lane[3]) cmcnt[15:8] <= wd_lane[31:24];
            if(wm_lane[2]) cmcnt[7:0]  <= wd_lane[23:16];
        end
        else if(cmstr_str0 && cmt_tick) begin
            if(cmcnt == cmcor) cmcnt <= 16'd0;  //match: clear and count on (fig 11.25)
            else               cmcnt <= cmcnt + 16'd1;
        end
        if(wr_cmt_b && wm_lane[1]) cmcor[15:8] <= wd_lane[15:8];    //CMCOR0 = view lanes 1:0
        if(wr_cmt_b && wm_lane[0]) cmcor[7:0]  <= wd_lane[7:0];

        //CMF: match tick sets (unless the same-edge write replaced the count),
        //else CMCSR0 write-0 clears (write-1 holds)
        if(cmt_fire)                       cmf <= 1'b1;
        else if(wr_cmt_a && wm_lane[0])    cmf <= cmf & wd_lane[7];
    end end
end



///////////////////////////////////////////////////////////
//////  Transfer Engine (start-up + request priority + bus interface)
////

/*
    One transfer unit at a time: the dual-direct pair = read at SAR then
    write at DAR (figs 11.5/11.6). Per-edge dataflow (core clock, i_CEN):

      IDLE     ch_req (pending&enable regs, ~2 lvl) -> priority win (~2
               lvl: fixed orders p.349, round-robin rotation fig 11.3) ->
               latch grant_q/addr_q(=SAR)/sarlo_q/size_q/beat_q=0, clear
               a cycle-steal pending (request withdrawn at the FIRST
               transfer, p.348); ch3-DI -> PT_REQ, dev->mem single ->
               WR_REQ, else RD_REQ
      PT_REQ/  ch3 indirect pointer fetch at SAR3, always LONG (p.339);
      PT_WAIT  rsp: addr_q <= the pointer = the data read address, size_q
               <= TS, go RD_REQ (fig 11.7; 32-bit bus, so no split reads
               and no NOP alignment cycle of the 16-bit fig 11.8 case)
      RD_REQ   req_valid high, addr/size straight from regs (flat cone
               through the arb 2:1); accept -> RD_WAIT
      RD_WAIT  rsp_valid: extract the SAR-lane datum (shift ~2 lvl),
               replicate onto lanes, latch wdata_q/wstrb_q(DAR lane)/
               addr_q(=DAR), go WR_REQ; a 16-byte unit gathers 4 longword
               beats into buf_q first (addr +4 per beat, fig 11.11)
      WR_REQ   req_valid+write; accept -> WR_WAIT
      WR_WAIT  rsp_valid (posted-write ack): 16-byte plays 4 beats from
               buf_q, then ch_upd strobe (channel steps SAR/DAR/DMATCR,
               sets TE on the last unit); burst -> IDLE (priority re-
               resolved EVERY unit boundary: a higher-priority channel
               preempts between units, fig 11.14, but o_BUS_HOLD keeps
               the CPU off); cycle-steal -> GAP
      GAP      one request-free cycle so the arb owner returns to the CPU
               (the fig 11.12 cycle-steal boundary), then IDLE

    Ending laws (p.374): enables are checked at grant only - clearing
    DE/DME (or an NMI setting NMIF) mid-unit lets the unit complete
    with registers updated (law d + p.375), the channel then stops with
    TE unset. A DMAC address error instead sets AE and never runs (or
    abandons) the offending unit - the block before unit_done below.
*/

//DREQ pin sampler: 2FF sync at core rate, then the CKIO-falling-edge sample
//(p.363). Edge detection compares the new sample against the previous one,
//so a falling edge needs DREQ high at the prior sample point (fig 11.19).
logic   [1:0]   dreq_ff, dreq_sync, dreq_smp;
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        dreq_ff   <= 2'b11;                 //negated idle (active-low pins)
        dreq_sync <= 2'b11;
        dreq_smp  <= 2'b11;
    end
    else begin if(i_CEN) begin
        dreq_ff   <= i_DREQ_n;
        dreq_sync <= dreq_ff;
        if(i_CKIO_NCEN) dreq_smp <= dreq_sync;
    end end
end
wire    [1:0]   dreq_lvl = ~dreq_smp;       //DS=0: low-level detection (p.347)

//request sources: auto = level while enabled (p.347); CMT = compare-match
//pulse latched until served (p.348); external (ch0/1 only) = DREQ level or
//the pend_ext edge latch per DS. Unimplemented RS codes (IrDA/SCIF/A-D) inert.
logic   [3:0]   pend;                       //CMT request latch per channel
logic   [1:0]   pend_ext;                   //DREQ falling-edge latch (DS=1)
logic   [3:0]   ch_req, ch_tm_v, ch_rs_cmt, ch_rs_ext, ch_rs_sgr, ch_rs_sgw;
always_comb begin
    for(int c = 0; c < 4; c++) begin
        logic rs_auto, ext_line;
        rs_auto      = (ch_chcr[c][11:8] == 4'b0100);
        ch_rs_cmt[c] = (ch_chcr[c][11:8] == 4'b1111);
        ch_rs_sgr[c] = (ch_chcr[c][11:8] == 4'b0010);   //single: memory -> device w/ DACK
        ch_rs_sgw[c] = (ch_chcr[c][11:8] == 4'b0011);   //single: device w/ DACK -> memory
        //external request exists on ch0/1 only (table 11.3)
        ch_rs_ext[c] = ((ch_chcr[c][11:8] == 4'b0000) | ch_rs_sgr[c] | ch_rs_sgw[c])
                       && (c < 2);
        ext_line     = ch_chcr[c][6] ? pend_ext[c[0]] : dreq_lvl[c[0]];     //DS edge/level
        //enable = DE & ~TE & DME & ~NMIF & ~AE (p.342)
        ch_en[c]     = ch_chcr[c][0] & ~ch_chcr[c][1] & dme & ~nmif & ~ae;
        ch_req[c]    = ch_en[c] & (rs_auto | (ch_rs_cmt[c] & pend[c]) |
                                   (ch_rs_ext[c] & ext_line));
        ch_tm_v[c]   = ch_chcr[c][5];       //TM: burst flag per channel
    end
end

//channel priority (p.349): three fixed orders, or PR=11 round-robin. The
//served channel dropping to the bottom keeps the order a PURE ROTATION
//(check fig 11.3's worked cases) - rr_head names the current top channel
logic   [1:0]   rr_head;                    //round-robin top = last served + 1
logic   [1:0]   win;
always_comb begin
    unique case(pr)
        2'b01:   win = ch_req[0] ? 2'd0 : ch_req[2] ? 2'd2 : ch_req[3] ? 2'd3 : 2'd1;
        2'b10:   win = ch_req[2] ? 2'd2 : ch_req[0] ? 2'd0 : ch_req[1] ? 2'd1 : 2'd3;
        2'b11:   win = ch_req[rr_head        ] ? rr_head :
                       ch_req[rr_head + 2'd1] ? rr_head + 2'd1 :
                       ch_req[rr_head + 2'd2] ? rr_head + 2'd2 : rr_head + 2'd3;
        default: win = ch_req[0] ? 2'd0 : ch_req[1] ? 2'd1 : ch_req[2] ? 2'd2 : 2'd3;
    endcase
end
wire            win_v = |ch_req;

//sequencer state ("seq"): the unit pipeline above; PT = the ch3 indirect
//pointer fetch prologue (fig 11.7)
localparam logic [2:0] S_IDLE = 3'd0, S_RD_REQ = 3'd1, S_RD_WAIT = 3'd2,
                       S_WR_REQ = 3'd3, S_WR_WAIT = 3'd4, S_GAP = 3'd5,
                       S_PT_REQ = 3'd6, S_PT_WAIT = 3'd7;
//unit shape: dual R->W, or ONE single-address cycle (read for mem->dev,
//write-with-external-drive for dev->mem, figs 11.9-11.10); bit1 = single
localparam logic [1:0] M_DUAL = 2'b00, M_SGR = 2'b10, M_SGW = 2'b11;
logic   [2:0]   seq;
logic   [1:0]   mode_q;                     //granted unit shape (M_*)
logic   [1:0]   grant_q;                    //granted channel (registered mux select)
logic   [1:0]   sarlo_q;                    //granted SAR[1:0]: read-lane pick
logic   [1:0]   size_q;                     //bus access size (16-byte moves as long beats)
logic           sz16_q;                     //granted unit is 16-byte: 4 longword beats
logic   [1:0]   beat_q;                     //longword beat index within a 16-byte unit
logic   [31:0]  buf_q [0:3];                //16-byte unit gather buffer (fig 11.11)
logic   [31:0]  addr_q;                     //read address, then write address
logic   [31:0]  wdata_q;                    //lane-replicated write data
logic   [3:0]   wstrb_q;                    //DAR-lane strobes
logic           dack_en_q;                  //granted unit outputs DACK (ext-request ch0/1)
logic           dack_rd_q;                  //DACK on the read(1)/write(0) access - AM
                                            //resolved at grant; single always on its access
logic   [1:0]   drak_q, drak_vis;           //DRAK pulse + its seen-one-CKIO-fall marker

//granted-channel views (4:1 muxes, registered grant_q select)
wire    [31:0]  dar_g = ch_dar[grant_q];
wire            tm_g  = ch_tm_v[grant_q];
wire    [1:0]   ts_g  = {ch_chcr[grant_q][4], ch_chcr[grant_q][3]};

//read-lane extract: right-justify the SAR-addressed datum (big-endian,
//(3-a)*8 = {~a,000}), then replicate - wstrb picks the DAR lane, so no
//DAR-dependent shift is ever needed
wire    [31:0]  rd_sh = I_BUS.rsp_rdata >> {~sarlo_q, 3'b000};
logic   [31:0]  wr_rep;
logic   [3:0]   wr_stb;
always_comb begin
    unique case(size_q)
        2'd0: begin                         //byte
            wr_rep = {4{rd_sh[7:0]}};
            wr_stb = 4'b1000 >> dar_g[1:0];
        end
        2'd1: begin                         //word
            wr_rep = {2{sarlo_q[1] ? I_BUS.rsp_rdata[15:0] : I_BUS.rsp_rdata[31:16]}};
            wr_stb = dar_g[1] ? 4'b0011 : 4'b1100;
        end
        default: begin                      //longword
            wr_rep = I_BUS.rsp_rdata;
            wr_stb = 4'b1111;
        end
    endcase
end

//single-write strobes need the DAR lane at grant time (size_q registers on
//the same edge): a small win-muxed duplicate of the wr_stb cone
wire    [1:0]   ts_w = {ch_chcr[win][4], ch_chcr[win][3]};
logic   [3:0]   sgw_stb;
always_comb begin
    unique case(ts_w)
        2'd0:    sgw_stb = 4'b1000 >> ch_dar[win][1:0];
        2'd1:    sgw_stb = ch_dar[win][1] ? 4'b0011 : 4'b1100;
        default: sgw_stb = 4'b1111;
    endcase
end

/*
    DMAC address error -> AE (fig 11.2 p.346, p.343): a misaligned
    SAR/DAR (word=2n, long=4n, 16-byte=16n) is caught at grant BEFORE
    the offending bus cycle fires; the ch3 pointer TABLE needs 4n (the
    pointer fetch is LONG) and the FETCHED pointer is re-checked against
    the data size at its return. A bus-reported fault (rsp_fault, e.g. a
    reserved region) abandons the unit in flight with NO register step.
    AE high drops every ch_en (all channels suspended), TE stays clear
    (p.374); the handler clears AE by write-0 after read-1.
*/

logic   [3:0]   amask_w, amask_g;           //offending low addr bits per TS
always_comb begin
    unique case(ts_w)
        2'd1:    amask_w = 4'b0001;
        2'd2:    amask_w = 4'b0011;
        2'd3:    amask_w = 4'b1111;
        default: amask_w = 4'b0000;         //byte never misaligns
    endcase
    unique case(ts_g)
        2'd1:    amask_g = 4'b0001;
        2'd2:    amask_g = 4'b0011;
        2'd3:    amask_g = 4'b1111;
        default: amask_g = 4'b0000;
    endcase
end
//single-address units only own their memory-side address (fig 11.10)
wire    [3:0]   amask_s   = ch_chcr[win][20] ? 4'b0011 : amask_w;
wire            align_bad = (~ch_rs_sgw[win] & |(ch_sar[win][3:0] & amask_s)) |
                            (~ch_rs_sgr[win] & |(ch_dar[win][3:0] & amask_w));
wire            ptr_bad   = |(I_BUS.rsp_rdata[3:0] & amask_g);

wire            ae_fault  = I_BUS.rsp_valid && I_BUS.rsp_fault &&
                            ((seq == S_RD_WAIT) || (seq == S_WR_WAIT) || (seq == S_PT_WAIT));
assign  ae_set = ((seq == S_IDLE) && win_v && align_bad) || ae_fault ||
                 ((seq == S_PT_WAIT) && I_BUS.rsp_valid && !I_BUS.rsp_fault && ptr_bad);

wire            beat_last  = !sz16_q || (beat_q == 2'd3);   //16-byte: 4th beat ends the unit
wire            unit_done  = (I_BUS.rsp_valid) && !I_BUS.rsp_fault && beat_last &&
                             ((seq == S_WR_WAIT) ||
                              (seq == S_RD_WAIT && mode_q == M_SGR));
wire            grant_fire = (seq == S_IDLE) && win_v && !align_bad;

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        seq      <= S_IDLE;
        mode_q   <= M_DUAL;
        grant_q  <= 2'd0;
        sarlo_q  <= 2'd0;
        size_q   <= 2'd0;
        sz16_q   <= 1'b0;
        beat_q   <= 2'd0;
        rr_head  <= 2'd0;                   //round-robin reset order 0>1>2>3 (p.349)
        addr_q   <= 32'd0;
        wdata_q  <= 32'd0;
        wstrb_q  <= 4'd0;
        dack_en_q<= 1'b0;
        dack_rd_q<= 1'b0;
        pend     <= 4'd0;
        pend_ext <= 2'd0;
        drak_q   <= 2'd0;
        drak_vis <= 2'd0;
        buf_q[0] <= 32'd0;
        buf_q[1] <= 32'd0;
        buf_q[2] <= 32'd0;
        buf_q[3] <= 32'd0;
    end
    else begin if(i_CEN) begin
        unique case(seq)
            S_IDLE: begin
                //a misaligned winner never launches: ae_set raises AE on this
                //edge, ch_en (and win_v) drop on the next - no bus cycle fires
                if(win_v && !align_bad) begin   //start-up: latch the winner's unit
                    grant_q <= win;
                    size_q  <= (ts_w == 2'b11) ? 2'd2 : ts_w;
                    sz16_q  <= (ts_w == 2'b11);
                    beat_q  <= 2'd0;
                    rr_head <= win + 2'd1;  //served channel to the bottom (fig 11.3)
                    //DACK tag: dual-ext per AM's cycle (p.337); single always
                    dack_en_q <= ch_rs_ext[win];
                    dack_rd_q <= ch_rs_sgr[win] |
                                 (~ch_rs_sgw[win] & ~ch_chcr[win][17]);     //AM=0: read cycle
                    if(ch_rs_sgw[win]) begin            //dev->mem: lone external-drive WRITE
                        mode_q  <= M_SGW;
                        addr_q  <= ch_dar[win];
                        wstrb_q <= sgw_stb;
                        wdata_q <= 32'd0;               //don't-care: D left undriven
                        seq     <= S_WR_REQ;
                    end
                    else if(ch_chcr[win][20]) begin     //ch3 DI: pointer fetch prologue
                        mode_q  <= M_DUAL;
                        addr_q  <= ch_sar[win];
                        size_q  <= 2'd2;                //pointer is always LONG (p.339)
                        sz16_q  <= 1'b0;
                        seq     <= S_PT_REQ;
                    end
                    else begin                          //dual, or single mem->dev (lone read)
                        mode_q  <= ch_rs_sgr[win] ? M_SGR : M_DUAL;
                        addr_q  <= ch_sar[win];
                        sarlo_q <= ch_sar[win][1:0];
                        seq     <= S_RD_REQ;
                    end
                end
            end
            S_PT_REQ:  if(I_BUS.req_ready) seq <= S_PT_WAIT;
            S_PT_WAIT: begin
                if(I_BUS.rsp_valid) begin   //the fetched pointer IS the data read address
                    if(I_BUS.rsp_fault || ptr_bad)  //address error: abandon the unit
                        seq <= S_IDLE;
                    else begin
                        addr_q  <= I_BUS.rsp_rdata;
                        sarlo_q <= I_BUS.rsp_rdata[1:0];
                        size_q  <= (ts_g == 2'b11) ? 2'd2 : ts_g;   //back to the data size
                        seq     <= S_RD_REQ;
                    end
                end
            end
            S_RD_REQ:  if(I_BUS.req_ready) seq <= S_RD_WAIT;
            S_RD_WAIT: begin
                if(I_BUS.rsp_valid) begin
                    if(I_BUS.rsp_fault)     //bus fault: abandon, no write, no step
                        seq <= S_IDLE;
                    else if(sz16_q) begin   //16-byte: gather 4 longwords, then turn
                        buf_q[beat_q] <= I_BUS.rsp_rdata;
                        if(beat_q != 2'd3) begin
                            addr_q <= addr_q + 32'd4;   //source, +4, +8, +12 (fig 11.11)
                            beat_q <= beat_q + 2'd1;
                            seq    <= S_RD_REQ;
                        end
                        else if(mode_q == M_SGR)        //single: 4 lone reads end the unit
                            seq <= tm_g ? S_IDLE : S_GAP;
                        else begin                      //dual: 4 write beats follow
                            addr_q  <= dar_g;
                            wdata_q <= buf_q[0];
                            wstrb_q <= 4'b1111;
                            beat_q  <= 2'd0;
                            seq     <= S_WR_REQ;
                        end
                    end
                    else if(mode_q == M_SGR)    //single-read unit: device latched off the bus
                        seq <= tm_g ? S_IDLE : S_GAP;
                    else begin              //dual: buffer the datum, turn the pair around
                        wdata_q <= wr_rep;
                        wstrb_q <= wr_stb;
                        addr_q  <= dar_g;
                        seq     <= S_WR_REQ;
                    end
                end
            end
            S_WR_REQ:  if(I_BUS.req_ready) seq <= S_WR_WAIT;
            S_WR_WAIT: begin
                if(I_BUS.rsp_valid) begin
                    if(I_BUS.rsp_fault)     //bus fault: abandon, no step
                        seq <= S_IDLE;
                    else if(sz16_q && beat_q != 2'd3) begin  //next longword of the 16-byte unit
                        addr_q  <= addr_q + 32'd4;
                        wdata_q <= buf_q[beat_q + 2'd1];
                        beat_q  <= beat_q + 2'd1;
                        seq     <= S_WR_REQ;
                    end
                    else                    //burst re-arbitrates at once (fig 11.14);
                        seq <= tm_g ? S_IDLE : S_GAP;   //cycle-steal yields a CPU boundary
                end
            end
            default: seq <= S_IDLE;         //S_GAP: one request-free cycle
        endcase

        //CMT pending: match sets (outranks the clears), a cycle-steal grant
        //withdraws at the FIRST transfer, burst at the LAST (p.348)
        for(int c = 0; c < 4; c++) begin
            if(cmt_fire && ch_rs_cmt[c])                             pend[c] <= 1'b1;
            else if(grant_fire && win == c[1:0] && !ch_tm_v[c])      pend[c] <= 1'b0;
            else if(ch_upd[c] && ch_tcr[c] == 24'd1)                 pend[c] <= 1'b0;
        end

        //DREQ edge pending: same withdraw laws (first transfer / fig 11.21
        //burst-edge runs to DMATCR=0); set needs DREQ high at the prior sample
        for(int k = 0; k < 2; k++) begin
            if(i_CKIO_NCEN && dreq_smp[k] && !dreq_sync[k])          pend_ext[k] <= 1'b1;
            else if(grant_fire && win == k[1:0] && !ch_tm_v[k])      pend_ext[k] <= 1'b0;
            else if(ch_upd[k] && ch_tcr[k] == 24'd1)                 pend_ext[k] <= 1'b0;
        end

        //DRAK: one-CKIO-cycle request-accepted pulse at an external grant
        for(int k = 0; k < 2; k++) begin
            if(grant_fire && win == k[1:0] && ch_rs_ext[k]) begin
                drak_q[k]   <= 1'b1;
                drak_vis[k] <= 1'b0;
            end
            else if(i_CKIO_NCEN && drak_q[k]) begin
                if(drak_vis[k]) drak_q[k] <= 1'b0;      //seen one full CKIO fall
                else            drak_vis[k] <= 1'b1;
            end
        end
    end end
end

//unit-completion strobes into the channel iteration datapaths; single-address
//units step only their memory-side register ({DAR, SAR} mask, fig 11.10)
wire    [1:0]   upd_mask = (mode_q == M_SGR) ? 2'b01 :
                           (mode_q == M_SGW) ? 2'b10 : 2'b11;
always_comb begin
    for(int c = 0; c < 4; c++) ch_upd[c] = unit_done && (grant_q == c[1:0]);
end

//bus master drive: request fields straight from registers - one flat level
//into the arb's 2:1 (the reqn/addr 5 ns class stays shallow)
assign  I_BUS.req_valid   = (seq == S_RD_REQ) || (seq == S_WR_REQ) || (seq == S_PT_REQ);
assign  I_BUS.req_write   = (seq == S_WR_REQ);
assign  I_BUS.req_size    = size_q;
//16-byte unit beats carry the line-burst tag (the cache's fill/drain
//argument): the BSC chains the 4 longwords back-to-back - fig 11.11
//shape on ordinary areas, CSn-held envelope on burst ROM (fig 23.19),
//SDRAM engine line ops on areas 2/3 - and applies the p.304 WAIT-ignore
assign  I_BUS.req_burst   = sz16_q && ((seq == S_RD_REQ) || (seq == S_WR_REQ));
assign  I_BUS.req_addr    = addr_q;
assign  I_BUS.req_wdata   = wdata_q;
assign  I_BUS.req_wstrb   = (seq == S_WR_REQ) ? wstrb_q : 4'd0;
assign  I_BUS.req_lock    = 1'b0;
//DACK/single-address tags: the read or write access per the grant-resolved
//AM (dual) or the lone access (single); the BSC frames the CSn window
assign  I_BUS.req_dack    = dack_en_q && ((seq == S_RD_REQ &&  dack_rd_q) ||
                                          (seq == S_WR_REQ && !dack_rd_q));
assign  I_BUS.req_dack_ch = grant_q[0];
assign  I_BUS.req_saddr   = mode_q[1] && ((seq == S_RD_REQ) || (seq == S_WR_REQ));
assign  I_BUS.rsp_ready   = (seq == S_RD_WAIT) || (seq == S_WR_WAIT) || (seq == S_PT_WAIT);

//pads: AL/RL polarity from the live CHCR bits (ch0/1 only, pp.337-338);
//negated idle when active-low (the default)
assign  o_DACK[0] = ch_chcr[0][16] ? i_DACK_WIN[0] : ~i_DACK_WIN[0];
assign  o_DACK[1] = ch_chcr[1][16] ? i_DACK_WIN[1] : ~i_DACK_WIN[1];
assign  o_DRAK[0] = ch_chcr[0][18] ? drak_q[0]     : ~drak_q[0];
assign  o_DRAK[1] = ch_chcr[1][18] ? drak_q[1]     : ~drak_q[1];

//bus hold: through the unit (the R->W pair is indivisible vs the CPU,
//fig 11.12 tier 1) and across burst units (CPU locked out, fig 11.13/11.14)
assign  o_BUS_HOLD = ((seq != S_IDLE) && (seq != S_GAP)) || (|(ch_req & ch_tm_v));



///////////////////////////////////////////////////////////
//////  Register Read Mux
////

//32-bit word view per addr[7:2]; 16-bit registers pack big-endian
//({reg @+0, reg @+2}); undecoded offsets read 0
logic   [31:0]  rword;
always_comb begin
    unique case(REG_BUS.addr[7:2])
        6'h08:   rword = ch_sar[0];                 //0x20
        6'h09:   rword = ch_dar[0];                 //0x24
        6'h0A:   rword = {8'd0, ch_tcr[0]};         //0x28
        6'h0B:   rword = ch_chcr[0];                //0x2C
        6'h0C:   rword = ch_sar[1];                 //0x30
        6'h0D:   rword = ch_dar[1];                 //0x34
        6'h0E:   rword = {8'd0, ch_tcr[1]};         //0x38
        6'h0F:   rword = ch_chcr[1];                //0x3C
        6'h10:   rword = ch_sar[2];                 //0x40
        6'h11:   rword = ch_dar[2];                 //0x44
        6'h12:   rword = {8'd0, ch_tcr[2]};         //0x48
        6'h13:   rword = ch_chcr[2];                //0x4C
        6'h14:   rword = ch_sar[3];                 //0x50
        6'h15:   rword = ch_dar[3];                 //0x54
        6'h16:   rword = {8'd0, ch_tcr[3]};         //0x58
        6'h17:   rword = ch_chcr[3];                //0x5C
        6'h18:   rword = {dmaor, 16'd0};            //0x60 (16-bit, upper lanes)
        6'h1C:   rword = {cmstr, cmcsr};            //0x70, 0x72
        6'h1D:   rword = {cmcnt, cmcor};            //0x74, 0x76
        default: rword = 32'd0;
    endcase
end

//right-justify the addressed lane for the bridge's size replication:
//(3 - a)*8 = {~a, 3'b000} for the big-endian byte pick
wire    [31:0]  rw_byte = rword >> {~REG_BUS.addr[1:0], 3'b000};
always_comb begin
    unique case(REG_BUS.size)
        2'd0:    REG_BUS.rdata = {24'd0, rw_byte[7:0]};
        2'd1:    REG_BUS.rdata = REG_BUS.addr[1] ? {16'd0, rword[15:0]}
                                                 : {16'd0, rword[31:16]};
        default: REG_BUS.rdata = rword;
    endcase
end



///////////////////////////////////////////////////////////
//////  Interrupt Requests
////

//levels: TE AND IE per channel (p.372); the handler's TE write-0 drops them
logic   [3:0]   dei;
always_comb begin
    for(int c = 0; c < 4; c++) dei[c] = ch_chcr[c][1] & ch_chcr[c][2];
end
assign  o_DEI = dei;

endmodule

`default_nettype none
