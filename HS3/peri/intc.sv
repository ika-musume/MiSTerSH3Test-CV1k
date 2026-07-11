`default_nettype wire

/*
    Interrupt controller INTC (SH7709S section 6, pp.117-148).

    Ascertains the priority of every interrupt source and presents ONE
    pre-prioritized request to the CPU: (valid, level, INTEVT code); the
    SR.I3-I0 comparison itself lives in exc_handler (functionally the
    comparator of Fig 6.1). INTEVT2 is owned HERE (I-bus register, phys
    0x04000000, Appendix B p.741) and is latched from this module's own
    registered winner when the CPU pulses i_INT_ACK/i_NMI_ACK - coherent by
    construction with the code exc_handler consumed that same cycle.

    The priority resolver is a 2-stage REGISTERED pipeline (interrupt latency
    is architecturally free): stage A picks per-cluster winners from the
    entry list ordered exactly as tables 6.4/6.5 (strict > scans keep the
    earlier entry on ties = the default priority), stage B runs the cluster
    tournament and the code decode. Consequence (matches silicon, p.121):
    an IPR/ICR/IRR0 write takes bridge + resolver cycles before the CPU-side
    request reflects it - rewrite masks with interrupts blocked.

    Documented deviations: no IRQOUT pin; register writes are not
    size-checked.
*/

module intc (
    /* CLOCK AND RESET - cleared by every reset flavor (table 6.2 note 1) */
    input   wire            i_RST_n,
    input   wire            i_CLK,
    input   wire            i_CEN,
    input   wire            i_PCEN,         //P-phi enable (i_CEN-qualified): IRQ edge / IRL samplers

    /* INTERFACES */
    IBus_2.slave            REG_HI,         //0xFFFFFEE0-EF: ICR0/IPRA/IPRB
    IBus_2.slave            REG_LO,         //0xA4000000-1F: INTEVT2/IRR0-2/ICR1/ICR2/PINTER/IPRC-E

    /* PINS */
    input   wire            i_NMI,
    input   wire    [5:0]   i_IRQ,          //IRQ5-0; IRQ3-0 double as IRL3-0 when ICR1.IRQLVL=1
    input   wire    [3:0]   i_IRLS,         //IRLS3-0, enabled by ICR1.IRLSEN (p.122)
    input   wire    [15:0]  i_PINT,

    /* ON-CHIP MODULE REQUESTS - only the WDT is wired this session */
    input   wire            i_ITI_REQ,      //WDT interval, code 0x560, IPRB[15:12]
    input   wire    [3:0]   i_TMU_REQ,      //{TICPI2,TUNI2,TUNI1,TUNI0}
    input   wire    [2:0]   i_RTC_REQ,      //{CUI,PRI,ATI}
    input   wire    [3:0]   i_SCI_REQ,      //{TEI,TXI,RXI,ERI}
    input   wire    [3:0]   i_SCIF_REQ,     //{TXI2,BRI2,RXI2,ERI2}
    input   wire    [3:0]   i_IRDA_REQ,     //{TXI1,BRI1,RXI1,ERI1}
    input   wire    [3:0]   i_DMAC_REQ,     //{DEI3,DEI2,DEI1,DEI0}
    input   wire    [1:0]   i_REF_REQ,      //{ROVI,RCMI}
    input   wire            i_ADC_REQ,      //ADI
    input   wire            i_UDI_REQ,      //UDI, fixed priority 15

    /* CPU CONTRACT - pre-prioritized request + accept strobes back */
    output  logic           o_NMI_VALID,
    output  wire            o_NMI_BLMSK,    //ICR1.BLMSK: NMI accepted even under SR.BL (p.133)
    output  wire            o_NMI_EDGE,     //accepted NMI edge pulse -> DMAOR.NMIF (p.344)
    output  logic           o_INT_VALID,
    output  logic   [3:0]   o_INT_LEVEL,
    output  logic   [11:0]  o_INT_CODE,     //INTEVT code (level code or source code, tables 6.4/6.5)
    input   wire            i_INT_ACK,      //accepted: latch INTEVT2 from the presented winner
    input   wire            i_NMI_ACK       //accepted: clear NMI edge-pending, INTEVT2 <= 0x1C0
);

///////////////////////////////////////////////////////////
//////  Register Storage
////

logic           icr0_nmie;                  //ICR0[8]: 0 falling / 1 rising NMI edge (p.132)
logic   [15:0]  icr1;                       //init 0x4000 = IRL mode (p.133)
logic   [15:0]  icr2;                       //PINT sense: 1 = detect high (p.136)
logic   [15:0]  pinter;                     //PINT enables (p.137)
logic   [15:0]  ipra, iprb, iprc, iprd, ipre;
logic   [31:0]  intevt2;

wire            icr1_mai    = icr1[15];
wire            icr1_irqlvl = icr1[14];
wire            icr1_irlsen = icr1[12];
assign          o_NMI_BLMSK = icr1[13];



///////////////////////////////////////////////////////////
//////  Pin Synchronizers
////

logic           nmi_ff, nmi_sync, nmi_z;
logic   [5:0]   irq_ff, irq_sync;
logic   [3:0]   irls_ff, irls_sync;
logic   [15:0]  pint_ff, pint_sync;

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        nmi_ff  <= 1'b0; nmi_sync  <= 1'b0; nmi_z <= 1'b0;
        irq_ff  <= '1;   irq_sync  <= '1;               //IRQ/IRL pins idle HIGH
        irls_ff <= '1;   irls_sync <= '1;
        pint_ff <= '0;   pint_sync <= '0;
    end
    else begin if(i_CEN) begin
        nmi_ff    <= i_NMI;   nmi_sync  <= nmi_ff;   nmi_z <= nmi_sync;
        irq_ff    <= i_IRQ;   irq_sync  <= irq_ff;
        irls_ff   <= i_IRLS;  irls_sync <= irls_ff;
        pint_ff   <= i_PINT;  pint_sync <= pint_ff;
    end end
end



///////////////////////////////////////////////////////////
//////  NMI
////

//edge-detected, held pending until the CPU accepts; MAI masks EVERY request
//while the NMI pin is low (p.133, flowchart fig 6.3 p.144)
wire            nmi_edge  = icr0_nmie ? (nmi_sync & ~nmi_z) : (~nmi_sync & nmi_z);
wire            mai_block = icr1_mai & ~nmi_sync;

//20-cycle NMIE-change lockout (p.121): flipping the NMIE edge polarity can
//look like a live edge if the pin already sits in the new active state, so
//NMI detection is suppressed for 20 core cycles after ICR0.NMIE is changed.
//icr0_nmie holds its OLD value on the write edge (non-blocking), so the
//compare below is a true change-detect.
logic   [4:0]   nmie_lock;                             //down-counts 20..0; NMI masked while !=0
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) nmie_lock <= 5'd0;
    else begin if(i_CEN) begin
        if(REG_HI.stb && REG_HI.we && REG_HI.addr == 8'hE0 && REG_HI.wdata[8] != icr0_nmie)
                                nmie_lock <= 5'd20;    //ICR0.NMIE changed: arm the lockout
        else if(nmie_lock != 0) nmie_lock <= nmie_lock - 5'd1;
    end end
end

logic           nmi_pend;
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) nmi_pend <= 1'b0;
    else begin if(i_CEN) begin
        if(nmi_edge && nmie_lock == 5'd0) nmi_pend <= 1'b1;   //new edge outranks a same-cycle ack
        else if(i_NMI_ACK)                nmi_pend <= 1'b0;    //(edge ignored during the lockout)
    end end
end

assign  o_NMI_VALID = nmi_pend & ~mai_block;
//the DMAC's NMIF hook: the same lockout-qualified edge the pend latch
//trusts, exported raw - it must set NMIF even while the DMAC idles and
//independently of the CPU's accept handshake (11.6 note 3)
assign  o_NMI_EDGE  = nmi_edge && (nmie_lock == 5'd0);



///////////////////////////////////////////////////////////
//////  IRQ0-5 Sense and Pending
////

//per-pin sense from ICR1 bit pairs (p.133): 00 falling / 01 rising / 10 low level.
//Edge modes sample on P-phi: two-sample detection gives the 2-Pcyc minimum pulse
//width of p.121 naturally. IRQ0-3 act as IRQ pins only when ICR1.IRQLVL = 0.
logic   [5:0]   irq_smp;                    //previous P-phi sample
logic   [5:0]   irq_pend;                   //edge-mode pending flags (IRR0[5:0], write-0-clear)
logic   [5:0]   irq_lvl_req, irq_edge_set, irq_req, irq_ent_en;

always_comb begin
    for(int n = 0; n < 6; n++) begin
        logic [1:0] sense;
        sense             = icr1[2*n +: 2];
        irq_lvl_req[n]    = (sense == 2'b10) & ~irq_sync[n];                    //low-level detect
        irq_edge_set[n]   = (sense == 2'b00) ? (irq_smp[n] & ~irq_sync[n]) :    //falling
                            (sense == 2'b01) ? (~irq_smp[n] & irq_sync[n]) :    //rising
                            1'b0;
        irq_req[n]        = (sense == 2'b10) ? irq_lvl_req[n] : irq_pend[n];
        irq_ent_en[n]     = (n < 4) ? ~icr1_irqlvl : 1'b1;                      //IRQ0-3 vs IRL pins
    end
end

//IRR0 write: bits [5:0] are write-0-to-clear, write-1 holds (p.138)
wire            irr0_wr = REG_LO.stb && REG_LO.we && (REG_LO.addr == 8'h04);

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        irq_smp  <= '1;
        irq_pend <= '0;
    end
    else begin if(i_CEN) begin
        if(i_PCEN) irq_smp <= irq_sync;
        for(int n = 0; n < 6; n++) begin
            if(i_PCEN && irq_edge_set[n]) irq_pend[n] <= 1'b1;  //edge outranks a same-cycle clear
            else if(irr0_wr && !REG_LO.wdata[n]) irq_pend[n] <= 1'b0;
        end
    end end
end



///////////////////////////////////////////////////////////
//////  IRL Level Decode
////

//IRL3-0 (= IRQ3-0 pins) encode an active-low level: 0000 -> 15, 1111 -> none
//(table 6.3, p.123). Noise cancel = two consecutive equal P-phi samples.
logic   [3:0]   irl_smp, irl_cln, irls_smp, irls_cln;
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        irl_smp <= '1; irl_cln <= '1; irls_smp <= '1; irls_cln <= '1;
    end
    else begin if(i_CEN) begin
        if(i_PCEN) begin
            irl_smp  <= irq_sync[3:0];
            irls_smp <= irls_sync;
            if(irl_smp  == irq_sync[3:0]) irl_cln  <= irq_sync[3:0];
            if(irls_smp == irls_sync    ) irls_cln <= irls_sync;
        end
    end end
end

//the higher level of the two pin groups wins when IRLS is enabled (p.122)
wire    [3:0]   irl_lvl_a  = ~irl_cln;
wire    [3:0]   irl_lvl_b  = icr1_irlsen ? ~irls_cln : 4'd0;
wire    [3:0]   irl_level  = icr1_irqlvl ? ((irl_lvl_a >= irl_lvl_b) ? irl_lvl_a : irl_lvl_b)
                                         : 4'd0;
//both event codes are the level code 0x200 + (15-level)*0x20 (table 6.5)
wire    [11:0]  irl_code   = {3'b001, ~irl_level, 5'd0};



///////////////////////////////////////////////////////////
//////  PINT
////

//sense: 1 = detect high (p.136); gated by PINTER enables; two group sources
wire    [15:0]  pint_req = (pint_sync ~^ icr2) & pinter;
wire            pint07_req  = |pint_req[7:0];
wire            pint815_req = |pint_req[15:8];



///////////////////////////////////////////////////////////
//////  Priority Resolver - Entry Table
////

/*
    Entries follow the DEFAULT PRIORITY order of tables 6.4/6.5 (pp.126-128):
    UDI, IRL, IRQ0-5, PINT groups, DMAC, IrDA, SCIF, ADC, TMU, RTC, SCI, WDT,
    REF. kind=1 puts the source code in INTEVT as well (IPRA/IPRB sources +
    UDI + IRL); kind=0 sources (IPRC/D/E) get the level code in INTEVT.
    A non-requesting entry holds level 0 = masked.
*/

localparam int N_ENT = 37;
logic   [3:0]   ent_lvl  [0:N_ENT-1];
logic           ent_kind [0:N_ENT-1];
logic   [11:0]  ent_code2[0:N_ENT-1];

always_comb begin
    //UDI: fixed priority 15 (table 6.4)
    ent_lvl[0]  = i_UDI_REQ ? 4'd15 : 4'd0;               ent_kind[0]  = 1'b1; ent_code2[0]  = 12'h5E0;
    //IRL pseudo-entry (active only when ICR1.IRQLVL=1); level=0 means no request
    ent_lvl[1]  = irl_level;                              ent_kind[1]  = 1'b1; ent_code2[1]  = irl_code;
    //IRQ0-5 (IRQ0-3 gated by !IRQLVL): IPRC/IPRD nibbles
    ent_lvl[2]  = (irq_req[0] & irq_ent_en[0]) ? iprc[3:0]   : 4'd0; ent_kind[2] = 1'b0; ent_code2[2] = 12'h600;
    ent_lvl[3]  = (irq_req[1] & irq_ent_en[1]) ? iprc[7:4]   : 4'd0; ent_kind[3] = 1'b0; ent_code2[3] = 12'h620;
    ent_lvl[4]  = (irq_req[2] & irq_ent_en[2]) ? iprc[11:8]  : 4'd0; ent_kind[4] = 1'b0; ent_code2[4] = 12'h640;
    ent_lvl[5]  = (irq_req[3] & irq_ent_en[3]) ? iprc[15:12] : 4'd0; ent_kind[5] = 1'b0; ent_code2[5] = 12'h660;
    ent_lvl[6]  = irq_req[4] ? iprd[3:0]  : 4'd0;         ent_kind[6]  = 1'b0; ent_code2[6]  = 12'h680;
    ent_lvl[7]  = irq_req[5] ? iprd[7:4]  : 4'd0;         ent_kind[7]  = 1'b0; ent_code2[7]  = 12'h6A0;
    //PINT groups: IPRD
    ent_lvl[8]  = pint07_req  ? iprd[15:12] : 4'd0;       ent_kind[8]  = 1'b0; ent_code2[8]  = 12'h700;
    ent_lvl[9]  = pint815_req ? iprd[11:8]  : 4'd0;       ent_kind[9]  = 1'b0; ent_code2[9]  = 12'h720;
    //DMAC DEI0-3: IPRE[15:12]
    ent_lvl[10] = i_DMAC_REQ[0] ? ipre[15:12] : 4'd0;     ent_kind[10] = 1'b0; ent_code2[10] = 12'h800;
    ent_lvl[11] = i_DMAC_REQ[1] ? ipre[15:12] : 4'd0;     ent_kind[11] = 1'b0; ent_code2[11] = 12'h820;
    ent_lvl[12] = i_DMAC_REQ[2] ? ipre[15:12] : 4'd0;     ent_kind[12] = 1'b0; ent_code2[12] = 12'h840;
    ent_lvl[13] = i_DMAC_REQ[3] ? ipre[15:12] : 4'd0;     ent_kind[13] = 1'b0; ent_code2[13] = 12'h860;
    //IrDA ERI1/RXI1/BRI1/TXI1: IPRE[11:8]
    ent_lvl[14] = i_IRDA_REQ[0] ? ipre[11:8] : 4'd0;      ent_kind[14] = 1'b0; ent_code2[14] = 12'h880;
    ent_lvl[15] = i_IRDA_REQ[1] ? ipre[11:8] : 4'd0;      ent_kind[15] = 1'b0; ent_code2[15] = 12'h8A0;
    ent_lvl[16] = i_IRDA_REQ[2] ? ipre[11:8] : 4'd0;      ent_kind[16] = 1'b0; ent_code2[16] = 12'h8C0;
    ent_lvl[17] = i_IRDA_REQ[3] ? ipre[11:8] : 4'd0;      ent_kind[17] = 1'b0; ent_code2[17] = 12'h8E0;
    //SCIF ERI2/RXI2/BRI2/TXI2: IPRE[7:4]
    ent_lvl[18] = i_SCIF_REQ[0] ? ipre[7:4] : 4'd0;       ent_kind[18] = 1'b0; ent_code2[18] = 12'h900;
    ent_lvl[19] = i_SCIF_REQ[1] ? ipre[7:4] : 4'd0;       ent_kind[19] = 1'b0; ent_code2[19] = 12'h920;
    ent_lvl[20] = i_SCIF_REQ[2] ? ipre[7:4] : 4'd0;       ent_kind[20] = 1'b0; ent_code2[20] = 12'h940;
    ent_lvl[21] = i_SCIF_REQ[3] ? ipre[7:4] : 4'd0;       ent_kind[21] = 1'b0; ent_code2[21] = 12'h960;
    //ADC ADI: IPRE[3:0]
    ent_lvl[22] = i_ADC_REQ ? ipre[3:0] : 4'd0;           ent_kind[22] = 1'b0; ent_code2[22] = 12'h980;
    //TMU0-2 + input capture: IPRA (source code in BOTH events)
    ent_lvl[23] = i_TMU_REQ[0] ? ipra[15:12] : 4'd0;      ent_kind[23] = 1'b1; ent_code2[23] = 12'h400;
    ent_lvl[24] = i_TMU_REQ[1] ? ipra[11:8]  : 4'd0;      ent_kind[24] = 1'b1; ent_code2[24] = 12'h420;
    ent_lvl[25] = i_TMU_REQ[2] ? ipra[7:4]   : 4'd0;      ent_kind[25] = 1'b1; ent_code2[25] = 12'h440;
    ent_lvl[26] = i_TMU_REQ[3] ? ipra[7:4]   : 4'd0;      ent_kind[26] = 1'b1; ent_code2[26] = 12'h460;
    //RTC ATI/PRI/CUI: IPRA[3:0]
    ent_lvl[27] = i_RTC_REQ[0] ? ipra[3:0] : 4'd0;        ent_kind[27] = 1'b1; ent_code2[27] = 12'h480;
    ent_lvl[28] = i_RTC_REQ[1] ? ipra[3:0] : 4'd0;        ent_kind[28] = 1'b1; ent_code2[28] = 12'h4A0;
    ent_lvl[29] = i_RTC_REQ[2] ? ipra[3:0] : 4'd0;        ent_kind[29] = 1'b1; ent_code2[29] = 12'h4C0;
    //SCI ERI/RXI/TXI/TEI: IPRB[7:4]
    ent_lvl[30] = i_SCI_REQ[0] ? iprb[7:4] : 4'd0;        ent_kind[30] = 1'b1; ent_code2[30] = 12'h4E0;
    ent_lvl[31] = i_SCI_REQ[1] ? iprb[7:4] : 4'd0;        ent_kind[31] = 1'b1; ent_code2[31] = 12'h500;
    ent_lvl[32] = i_SCI_REQ[2] ? iprb[7:4] : 4'd0;        ent_kind[32] = 1'b1; ent_code2[32] = 12'h520;
    ent_lvl[33] = i_SCI_REQ[3] ? iprb[7:4] : 4'd0;        ent_kind[33] = 1'b1; ent_code2[33] = 12'h540;
    //WDT ITI: IPRB[15:12]
    ent_lvl[34] = i_ITI_REQ ? iprb[15:12] : 4'd0;         ent_kind[34] = 1'b1; ent_code2[34] = 12'h560;
    //REF RCMI/ROVI: IPRB[11:8]
    ent_lvl[35] = i_REF_REQ[0] ? iprb[11:8] : 4'd0;       ent_kind[35] = 1'b1; ent_code2[35] = 12'h580;
    ent_lvl[36] = i_REF_REQ[1] ? iprb[11:8] : 4'd0;       ent_kind[36] = 1'b1; ent_code2[36] = 12'h5A0;
end



///////////////////////////////////////////////////////////
//////  Priority Resolver - 2-Stage Tournament
////

//stage A: four cluster winners cut along the entry order; a strict > scan keeps
//the EARLIER entry on level ties = the default priority of tables 6.4/6.5
localparam int CL_LO [0:3] = '{0, 10, 20, 30};
localparam int CL_HI [0:3] = '{9, 19, 29, 36};

logic   [3:0]   a_lvl_nx  [0:3];
logic           a_kind_nx [0:3];
logic   [11:0]  a_code2_nx[0:3];
logic   [3:0]   a_lvl_q   [0:3];
logic           a_kind_q  [0:3];
logic   [11:0]  a_code2_q [0:3];

always_comb begin
    for(int c = 0; c < 4; c++) begin
        a_lvl_nx[c]   = 4'd0;
        a_kind_nx[c]  = 1'b0;
        a_code2_nx[c] = 12'd0;
        for(int i = CL_LO[c]; i <= CL_HI[c]; i++) begin
            if(ent_lvl[i] > a_lvl_nx[c]) begin
                a_lvl_nx[c]   = ent_lvl[i];
                a_kind_nx[c]  = ent_kind[i];
                a_code2_nx[c] = ent_code2[i];
            end
        end
    end
end

//stage B: cluster tournament (earlier cluster keeps ties) + registered winner
logic   [3:0]   b_lvl_nx;
logic           b_kind_nx;
logic   [11:0]  b_code2_nx;
logic   [3:0]   b_lvl_q;
logic           b_kind_q;
logic   [11:0]  b_code2_q;

always_comb begin
    b_lvl_nx   = 4'd0;
    b_kind_nx  = 1'b0;
    b_code2_nx = 12'd0;
    for(int c = 0; c < 4; c++) begin
        if(a_lvl_q[c] > b_lvl_nx) begin
            b_lvl_nx   = a_lvl_q[c];
            b_kind_nx  = a_kind_q[c];
            b_code2_nx = a_code2_q[c];
        end
    end
end

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        for(int c = 0; c < 4; c++) begin
            a_lvl_q[c] <= 4'd0; a_kind_q[c] <= 1'b0; a_code2_q[c] <= 12'd0;
        end
        b_lvl_q <= 4'd0; b_kind_q <= 1'b0; b_code2_q <= 12'd0;
    end
    else begin if(i_CEN) begin
        for(int c = 0; c < 4; c++) begin
            a_lvl_q[c] <= a_lvl_nx[c]; a_kind_q[c] <= a_kind_nx[c]; a_code2_q[c] <= a_code2_nx[c];
        end
        b_lvl_q <= b_lvl_nx; b_kind_q <= b_kind_nx; b_code2_q <= b_code2_nx;
    end end
end

//INTEVT: level code 0x200 + (15-level)*0x20 for IPRC/D/E sources, else the source code
wire    [11:0]  level_code = {3'b001, ~b_lvl_q, 5'd0};
assign  o_INT_VALID = (b_lvl_q != 4'd0) & ~mai_block;
assign  o_INT_LEVEL = b_lvl_q;
assign  o_INT_CODE  = b_kind_q ? b_code2_q : level_code;



///////////////////////////////////////////////////////////
//////  INTEVT2 Register
////

//latched from the SAME registered winner the CPU consumed at the ack cycle;
//an NMI acceptance writes its fixed code 0x1C0 (tables 6.4/6.5)
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) intevt2 <= 32'd0;
    else begin if(i_CEN) begin
        if(i_NMI_ACK)      intevt2 <= 32'h0000_01C0;
        else if(i_INT_ACK) intevt2 <= {20'd0, b_code2_q};
    end end
end



///////////////////////////////////////////////////////////
//////  Register File Access
////

//IRR0/IRR1/IRR2 read images (pp.138-141): IRR0[5:0] shows pend (edge) or the live
//request (level); IRR1/IRR2 mirror the DMAC/IrDA and SCIF/ADC request inputs
wire    [7:0]   irr0 = {pint07_req, pint815_req, irq_req[5:0]};
wire    [7:0]   irr1 = {i_IRDA_REQ[3], i_IRDA_REQ[2], i_IRDA_REQ[1], i_IRDA_REQ[0],
                        i_DMAC_REQ[3], i_DMAC_REQ[2], i_DMAC_REQ[1], i_DMAC_REQ[0]};
wire    [7:0]   irr2 = {3'd0, i_ADC_REQ,
                        i_SCIF_REQ[3], i_SCIF_REQ[2], i_SCIF_REQ[1], i_SCIF_REQ[0]};

//window reads, right-justified (the bridge replicates lanes)
always_comb begin
    unique case(REG_HI.addr)
        8'hE0:   REG_HI.rdata = {16'd0, nmi_sync, 6'd0, icr0_nmie, 8'd0};   //ICR0: NMIL RO + NMIE
        8'hE2:   REG_HI.rdata = {16'd0, ipra};
        8'hE4:   REG_HI.rdata = {16'd0, iprb};
        default: REG_HI.rdata = 32'd0;
    endcase
    unique case(REG_LO.addr)
        8'h00:   REG_LO.rdata = intevt2;
        8'h04:   REG_LO.rdata = {24'd0, irr0};
        8'h06:   REG_LO.rdata = {24'd0, irr1};
        8'h08:   REG_LO.rdata = {24'd0, irr2};
        8'h10:   REG_LO.rdata = {16'd0, icr1};
        8'h12:   REG_LO.rdata = {16'd0, icr2};
        8'h14:   REG_LO.rdata = {16'd0, pinter};
        8'h16:   REG_LO.rdata = {16'd0, iprc};
        8'h18:   REG_LO.rdata = {16'd0, iprd};
        8'h1A:   REG_LO.rdata = {16'd0, ipre};
        default: REG_LO.rdata = 32'd0;
    endcase
end

//window writes (IRR0's write-0-clear is handled in the IRQ pending block above)
wire            hi_wr = REG_HI.stb && REG_HI.we;
wire            lo_wr = REG_LO.stb && REG_LO.we;

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        icr0_nmie <= 1'b0;
        icr1      <= 16'h4000;                          //IRL mode out of reset (p.133)
        icr2      <= 16'd0;
        pinter    <= 16'd0;
        ipra      <= 16'd0;
        iprb      <= 16'd0;
        iprc      <= 16'd0;
        iprd      <= 16'd0;
        ipre      <= 16'd0;
    end
    else begin if(i_CEN) begin
        if(hi_wr) begin
            unique case(REG_HI.addr)
                8'hE0:   icr0_nmie <= REG_HI.wdata[8];
                8'hE2:   ipra      <= REG_HI.wdata[15:0];
                8'hE4:   iprb      <= REG_HI.wdata[15:0] & 16'hFFF0;    //IPRB[3:0] reserved
                default: begin end
            endcase
        end
        if(lo_wr) begin
            unique case(REG_LO.addr)
                8'h10:   icr1   <= REG_LO.wdata[15:0];
                8'h12:   icr2   <= REG_LO.wdata[15:0];
                8'h14:   pinter <= REG_LO.wdata[15:0];
                8'h16:   iprc   <= REG_LO.wdata[15:0];
                8'h18:   iprd   <= REG_LO.wdata[15:0];
                8'h1A:   ipre   <= REG_LO.wdata[15:0];
                default: begin end
            endcase
        end
    end end
end

endmodule

`default_nettype none
