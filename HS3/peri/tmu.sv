`default_nettype wire

/*
    Timer unit TMU (SH7709S section 12, pp.389-406).

    Three 32-bit auto-reload down counters. A channel ticks on one of the
    table-12.2.3 TPSC selections: the shared P-phi prescaler taps (P/4, P/16,
    P/64, P/256), the on-chip RTC output clock, or TCLK pin edges per CKEG.
    On the tick that finds TCNT at 0 the counter reloads from TCOR and UNF
    sets (fig 12.8) - the period is TCOR+1 ticks (p.400). Channel 2 adds
    input capture: a TCLK edge (per CKEG) copies TCNT2 into TCPR2 and sets
    ICPF when ICPE1 enables the function (fig 12.7).

    Interrupt requests are LEVELS (flag AND enable, section 12.4.3), cleared
    by the handler's write-0 to the flag: o_TMU_REQ = {TICPI2, TUNI2, TUNI1,
    TUNI0}, matching the INTC entry order (IPRA, codes 0x400-0x460).

    Reset: TOCR/TSTR/TCOR/TCNT/TCR initialize on power-on AND manual resets
    (table 12.2 note); TCPR2 is never initialized (p.399).

    TCLK pin: 2FF-synced then P-phi-sampled (two-sample edge detect = the
    1.5-Pcyc minimum pulse width of p.403, the INTC IRQ sampler pattern).
    RTC hookup (session 4): i_RTC_TICK is the RTC's 16.384 kHz output clock
    as a bus-domain tick (TPSC=100 count source); i_RTCCLK is the same
    divider output as a LEVEL, driven onto the TCLK pad when TOCR.TCOE=1
    (p.392). Deviations: no standby coupling; writes not size-checked.
*/

module tmu (
    /* CLOCK AND RESET - initialized by power-on and manual resets (p.391) */
    input   wire            i_RST_n,
    input   wire            i_CLK,
    input   wire            i_CEN,
    input   wire            i_PCEN,         //P-phi enable (i_CEN-qualified) from the CPG

    /* INTERFACES */
    IBus_2.slave            REG_BUS,        //P bus window 0xFFFFFE90-B8 (behind the BSC)

    /* PINS / MODULE CONNECTIONS */
    input   wire            i_TCLK,         //TCLK pad level (PTH7, table 18.1)
    output  wire            o_TCLK_O,       //TCOE=1: RTC output clock onto the pad (p.392)
    output  wire            o_TCLK_OE,
    input   wire            i_RTC_TICK,     //RTC output clock tick (i_CEN-qualified)
    input   wire            i_RTCCLK,       //RTC output clock level (16.384 kHz, async)

    /* INTERRUPT REQUESTS - levels, table 12.3 order */
    output  wire    [3:0]   o_TMU_REQ       //{TICPI2, TUNI2, TUNI1, TUNI0}
);

///////////////////////////////////////////////////////////
//////  Register Storage
////

logic           tocr_tcoe;                  //TOCR[0]: TCLK pin direction (p.392)
logic   [2:0]   tstr;                       //TSTR[2:0]: STR2-0 count enables (p.393)
logic   [31:0]  tcor [0:2];                 //timer constant (reload) registers
logic   [31:0]  tcnt [0:2];                 //timer counters
logic           unf  [0:2];                 //TCR[8]: underflow flags
logic           unie [0:2];                 //TCR[5]: underflow interrupt enables
logic   [1:0]   ckeg [0:2];                 //TCR[4:3]: external/capture edge select
logic   [2:0]   tpsc [0:2];                 //TCR[2:0]: count clock select
logic   [1:0]   icpe;                       //TCR2[7:6]: input capture control (ch 2 only)
logic           icpf;                       //TCR2[9]: input capture flag
logic   [31:0]  tcpr2;                      //input capture register (never reset, p.399)

assign  o_TCLK_O  = i_RTCCLK;               //RTC output clock rides the pad when TCOE=1
assign  o_TCLK_OE = tocr_tcoe;



///////////////////////////////////////////////////////////
//////  Prescaler and TCLK Sampler (P-phi domain)
////

//shared free-running prescaler on P-phi; a tap fires when its divided count
//wraps (the cpg_wdt WDT prescaler idiom)
logic   [7:0]   psc;
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) psc <= 8'd0;
    else begin if(i_CEN) begin
        if(i_PCEN) psc <= psc + 8'd1;
    end end
end

wire            tick4   = i_PCEN & (psc[1:0] == 2'h3);
wire            tick16  = i_PCEN & (psc[3:0] == 4'hF);
wire            tick64  = i_PCEN & (psc[5:0] == 6'h3F);
wire            tick256 = i_PCEN & (psc[7:0] == 8'hFF);

//TCLK: 2FF sync at core rate, then P-phi two-sample edge detect (the INTC
//IRQ sampler pattern = the 1.5-Pcyc minimum pulse width of p.403)
logic           tclk_ff, tclk_sync, tclk_smp;
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        tclk_ff <= 1'b0; tclk_sync <= 1'b0; tclk_smp <= 1'b0;
    end
    else begin if(i_CEN) begin
        tclk_ff   <= i_TCLK;
        tclk_sync <= tclk_ff;
        if(i_PCEN) tclk_smp <= tclk_sync;
    end end
end

wire            tclk_rise = i_PCEN & ~tclk_smp &  tclk_sync;
wire            tclk_fall = i_PCEN &  tclk_smp & ~tclk_sync;



///////////////////////////////////////////////////////////
//////  Channel Tick Select
////

//per-channel count tick: TPSC clock select (p.396) with the TCLK edge per
//CKEG (p.396: 00 rising / 01 falling / 1x both); reserved codes never tick
logic   [2:0]   ch_tick;
always_comb begin
    for(int c = 0; c < 3; c++) begin
        logic tclk_edge;
        tclk_edge  = ckeg[c][1] ? (tclk_rise | tclk_fall) :
                     ckeg[c][0] ?  tclk_fall : tclk_rise;
        unique case(tpsc[c])
            3'b000:  ch_tick[c] = tick4;
            3'b001:  ch_tick[c] = tick16;
            3'b010:  ch_tick[c] = tick64;
            3'b011:  ch_tick[c] = tick256;
            3'b100:  ch_tick[c] = i_RTC_TICK;
            3'b101:  ch_tick[c] = tclk_edge;
            default: ch_tick[c] = 1'b0;                 //11x reserved (p.396)
        endcase
    end
end

//channel-2 capture strobe: TCLK edge per CKEG2 while ICPE1 enables the
//function (fig 12.7); interrupt additionally needs ICPE0 (p.395)
wire            cap_edge = ckeg[2][1] ? (tclk_rise | tclk_fall) :
                           ckeg[2][0] ?  tclk_fall : tclk_rise;
wire            cap_tick = icpe[1] & cap_edge;



///////////////////////////////////////////////////////////
//////  Register Access Decode
////

wire            reg_wr   = REG_BUS.stb && REG_BUS.we;
wire            wr_tocr  = reg_wr && (REG_BUS.addr == 8'h90);
wire            wr_tstr  = reg_wr && (REG_BUS.addr == 8'h92);
//per-channel register strobes: base 0x94 + 12*c + {0 TCOR, 4 TCNT, 8 TCR}
logic   [2:0]   wr_tcor, wr_tcnt, wr_tcr;
always_comb begin
    wr_tcor[0] = reg_wr && (REG_BUS.addr == 8'h94);
    wr_tcnt[0] = reg_wr && (REG_BUS.addr == 8'h98);
    wr_tcr [0] = reg_wr && (REG_BUS.addr == 8'h9C);
    wr_tcor[1] = reg_wr && (REG_BUS.addr == 8'hA0);
    wr_tcnt[1] = reg_wr && (REG_BUS.addr == 8'hA4);
    wr_tcr [1] = reg_wr && (REG_BUS.addr == 8'hA8);
    wr_tcor[2] = reg_wr && (REG_BUS.addr == 8'hAC);
    wr_tcnt[2] = reg_wr && (REG_BUS.addr == 8'hB0);
    wr_tcr [2] = reg_wr && (REG_BUS.addr == 8'hB4);
end

//right-justified read mux (the P bridge replicates lanes)
always_comb begin
    unique case(REG_BUS.addr)
        8'h90:   REG_BUS.rdata = {31'd0, tocr_tcoe};
        8'h92:   REG_BUS.rdata = {29'd0, tstr};
        8'h94:   REG_BUS.rdata = tcor[0];
        8'h98:   REG_BUS.rdata = tcnt[0];
        8'h9C:   REG_BUS.rdata = {23'd0, unf[0], 2'd0, unie[0], ckeg[0], tpsc[0]};
        8'hA0:   REG_BUS.rdata = tcor[1];
        8'hA4:   REG_BUS.rdata = tcnt[1];
        8'hA8:   REG_BUS.rdata = {23'd0, unf[1], 2'd0, unie[1], ckeg[1], tpsc[1]};
        8'hAC:   REG_BUS.rdata = tcor[2];
        8'hB0:   REG_BUS.rdata = tcnt[2];
        8'hB4:   REG_BUS.rdata = {22'd0, icpf, unf[2], icpe, unie[2], ckeg[2], tpsc[2]};
        8'hB8:   REG_BUS.rdata = tcpr2;
        default: REG_BUS.rdata = 32'd0;                 //reserved offsets read 0
    endcase
end



///////////////////////////////////////////////////////////
//////  Counters and Flags
////

/*
    A software TCNT write wins over a same-cycle count tick (the manual
    demands counting be halted around writes anyway, p.406). An underflow
    set outranks a same-cycle TCR write-0 clear (the INTC pending pattern);
    a plain write-1 never sets a flag (p.394-395).
*/

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        tocr_tcoe <= 1'b0;
        tstr      <= 3'd0;
        icpe      <= 2'd0;
        icpf      <= 1'b0;
        for(int c = 0; c < 3; c++) begin
            tcor[c] <= 32'hFFFF_FFFF;
            tcnt[c] <= 32'hFFFF_FFFF;
            unf[c]  <= 1'b0;
            unie[c] <= 1'b0;
            ckeg[c] <= 2'd0;
            tpsc[c] <= 3'd0;
        end
    end
    else begin if(i_CEN) begin
        if(wr_tocr) tocr_tcoe <= REG_BUS.wdata[0];
        if(wr_tstr) tstr      <= REG_BUS.wdata[2:0];

        for(int c = 0; c < 3; c++) begin
            if(wr_tcor[c]) tcor[c] <= REG_BUS.wdata;

            if(wr_tcnt[c]) tcnt[c] <= REG_BUS.wdata;    //bus write wins over the tick
            else if(tstr[c] && ch_tick[c]) begin
                if(tcnt[c] == 32'd0) tcnt[c] <= tcor[c];    //underflow: auto-reload (p.397)
                else                 tcnt[c] <= tcnt[c] - 32'd1;
            end

            //UNF: set on the underflow tick, else write-0-clear (write-1 holds)
            if(tstr[c] && ch_tick[c] && tcnt[c] == 32'd0 && !wr_tcnt[c])
                 unf[c] <= 1'b1;
            else if(wr_tcr[c])
                 unf[c] <= unf[c] & REG_BUS.wdata[8];

            if(wr_tcr[c]) begin
                unie[c] <= REG_BUS.wdata[5];
                ckeg[c] <= REG_BUS.wdata[4:3];
                tpsc[c] <= REG_BUS.wdata[2:0];
            end
        end

        if(wr_tcr[2]) icpe <= REG_BUS.wdata[7:6];

        //ICPF: capture edge sets, TCR2 write-0 clears (set outranks clear)
        if(cap_tick)       icpf <= 1'b1;
        else if(wr_tcr[2]) icpf <= icpf & REG_BUS.wdata[9];
    end end
end

//TCPR2: never reset (p.399); copies TCNT2 at the capture edge (fig 12.7)
always_ff @(posedge i_CLK) begin if(i_CEN) begin
    if(cap_tick) tcpr2 <= tcnt[2];
end end



///////////////////////////////////////////////////////////
//////  Interrupt Requests
////

//levels: flag AND enable (section 12.4.3); the handler's write-0 drops them
assign  o_TMU_REQ = {icpf & icpe[1] & icpe[0],
                     unf[2] & unie[2],
                     unf[1] & unie[1],
                     unf[0] & unie[0]};

endmodule

`default_nettype none
