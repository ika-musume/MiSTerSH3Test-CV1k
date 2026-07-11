`default_nettype wire

/*
    CPG + WDT (SH7709S section 9, pp.203-218; STBCR/STBCR2 section 8.2,
    pp.183-186). FPGA adaptation: one architectural clock, no PLLs - FRQCR is
    bookkeeping plus a REAL peripheral-clock divider: o_PCEN pulses at the
    I:P ratio implied by the IFC/PFC fields (every legal table-9.4 pair gives
    an integer ratio in {1,2,3,4,6}). The CPG also owns the bus clock: o_BCEN
    paces the BSC's SDRAM engine and the o_CKIO pin drives the board (p.207:
    modes 0-2 generate the system clock from the chip's CKIO output). The WDT
    counts on o_PCEN through the CKS prescaler taps and implements both
    modes: interval (ITI interrupt, code 0x560) and watchdog (internal reset
    request per RSTS).

    Reset domains (pp.211,214-215): WTCNT/WTCSR and the counters reset ONLY by
    the pin POR (RESETP) and are retained across WDT-caused internal resets;
    FRQCR/STBCR/STBCR2 also clear on a WDT power-on reset (sync term) but
    survive manual resets. This module therefore takes the RAW reset pins.

    Documented deviations: the WDT-timed clock-freeze on FRQCR multiplier
    writes and the standby-cancel count (9.5.1/9.8) are not emulated; no
    STATUS/RESETOUT pins; wrong-size register reads return the value.
*/

module cpg_wdt (
    /* CLOCK AND RESET - raw pins, see reset-domain note above */
    input   wire            i_POR_n,        //RESETP pin
    input   wire            i_RST_n,        //manual-reset pin (unused: no register clears on it)
    input   wire            i_CLK,
    input   wire            i_CEN,

    /* INTERFACES */
    IBus_2.slave            REG_BUS,        //window 0xFFFFFF80-8F

    /* CLOCK/INTERRUPT/RESET FANOUT */
    output  wire            o_PCEN,         //peripheral clock (P-phi) enable, i_CEN-qualified
    output  wire            o_BCEN,         //bus clock (B-phi) enable - the SDRAM engine pace
    output  wire            o_CKIO,         //bus clock output pin (B-phi square wave)
    output  wire            o_CKIO_PCEN,
    output  wire            o_CKIO_NCEN,
    output  wire            o_ITI_REQ,      //WDT interval interrupt request (level = IOVF)
    output  wire            o_WDT_RST_POR_n,//watchdog reset request, RSTS=0 (16-cycle pulse)
    output  wire            o_WDT_RST_MAN_n //watchdog reset request, RSTS=1
);

///////////////////////////////////////////////////////////
//////  Register Storage
////

logic   [15:0]  frqcr;                      //0x80, word access only, init 0x0102 (p.211)
logic   [7:0]   stbcr;                      //0x82 (p.183)
logic   [7:0]   stbcr2;                     //0x88 (p.185)
logic   [7:0]   wtcnt;                      //0x84, keyed word write 0x5A (p.217)
logic           wt_tme, wt_it, wt_rsts;     //WTCSR 0x86 control bits (p.215)
logic           wt_wovf, wt_iovf;           //WTCSR overflow flags
logic   [2:0]   wt_cks;                     //WTCSR count clock select
wire    [7:0]   wtcsr = {wt_tme, wt_it, wt_rsts, wt_wovf, wt_iovf, wt_cks};

wire            reg_wr      = REG_BUS.stb && REG_BUS.we;
wire            reg_wr_word = reg_wr && (REG_BUS.size == 2'd1);
wire            reg_wr_byte = reg_wr && (REG_BUS.size == 2'd0);

//keyed word writes (fig 9.3, p.217): upper byte is the key, lower byte the data
wire            wr_wtcnt = reg_wr_word && (REG_BUS.addr == 8'h84) && (REG_BUS.wdata[15:8] == 8'h5A);
wire            wr_wtcsr = reg_wr_word && (REG_BUS.addr == 8'h86) && (REG_BUS.wdata[15:8] == 8'hA5);

//right-justified read mux; WTCNT/WTCSR are byte reads per p.214
always_comb begin
    unique case(REG_BUS.addr)
        8'h80:   REG_BUS.rdata = {16'd0, frqcr};
        8'h82:   REG_BUS.rdata = {24'd0, stbcr};
        8'h84:   REG_BUS.rdata = {24'd0, wtcnt};
        8'h86:   REG_BUS.rdata = {24'd0, wtcsr};
        8'h88:   REG_BUS.rdata = {24'd0, stbcr2};
        default: REG_BUS.rdata = 32'd0;
    endcase
end



///////////////////////////////////////////////////////////
//////  Peripheral Clock Divider
////

//IFC/PFC field decode to integer dividers (p.211-212); reserved codes fall to 1
logic   [2:0]   div_i, div_p;
always_comb begin
    unique case({frqcr[14], frqcr[3:2]})            //IFC2,IFC1,IFC0
        3'b000:  div_i = 3'd1;
        3'b001:  div_i = 3'd2;
        3'b100:  div_i = 3'd3;
        3'b010:  div_i = 3'd4;
        default: div_i = 3'd1;
    endcase
    unique case({frqcr[13], frqcr[1:0]})            //PFC2,PFC1,PFC0
        3'b000:  div_p = 3'd1;
        3'b001:  div_p = 3'd2;
        3'b100:  div_p = 3'd3;
        3'b010:  div_p = 3'd4;
        3'b101:  div_p = 3'd6;
        default: div_p = 3'd1;
    endcase
end

//I:P ratio - integer for every legal table-9.4 combination; anything else runs 1:1
logic   [2:0]   pdiv_n;
always_comb begin
    if     (div_p == div_i           ) pdiv_n = 3'd1;
    else if(div_p == {div_i[1:0],1'b0}) pdiv_n = 3'd2;  //div_p == 2*div_i
    else if(div_p == 3'd3 && div_i == 3'd1) pdiv_n = 3'd3;
    else if(div_p == 3'd4 && div_i == 3'd1) pdiv_n = 3'd4;
    else if(div_p == 3'd6 && div_i == 3'd1) pdiv_n = 3'd6;
    else if(div_p == 3'd6 && div_i == 3'd2) pdiv_n = 3'd3;
    else if(div_p == 3'd6 && div_i == 3'd3) pdiv_n = 3'd2;
    else                                    pdiv_n = 3'd1;
end

logic   [2:0]   pdiv_cnt;
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) pdiv_cnt <= 3'd0;
    else begin if(i_CEN) begin
        pdiv_cnt <= (pdiv_cnt >= pdiv_n - 3'd1) ? 3'd0 : pdiv_cnt + 3'd1;
    end end
end

wire            pcen = (pdiv_cnt == 3'd0);          //reset default: /4 (FRQCR 0x0102)
assign  o_PCEN = i_CEN & pcen;



///////////////////////////////////////////////////////////
//////  Bus Clock / CKIO Pin
////

/*
    B-phi = CKIO = core/2, FIXED: on silicon the crystal + PLL2 set the CKIO
    rate and FRQCR only re-paces I-phi/P-phi around it (table 9.4); with no
    FPGA PLLs the /2 is wired. The SH7709S FRQCR has NO CKOEN bit (bit 8 is
    reserved-1, p.212), so CKIO always drives in the output modes 0-2
    (p.207). DATASHEET PHASE: the pin RISES at the BCEN-enabled command
    edges - every bus pin changes at the CKIO rise and the mid-state shapes
    (RD/WEn, WAIT sampling) sit at the fall, as figs 10.14/23.16 draw them.
    Board note: a synchronous device clocked straight off this pin samples
    at the same edge the pins change; the board must phase-shift/delay the
    device clock (output-delay constraint or a small trace/clock-tree skew,
    ~1/4 cycle) exactly as with the real chip's tOD window. Raw-pin POR
    domain: the SDRAM engine phase must survive manual resets (p.297).
*/

logic           ckio_ph;
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) ckio_ph <= 1'b0;
    else begin if(i_CEN) begin
        ckio_ph <= ~ckio_ph;
    end end
end

assign  o_BCEN = i_CEN & ckio_ph;
assign  o_CKIO = ~ckio_ph;      //rises at the command edges (datasheet phase)
assign  o_CKIO_PCEN = ~o_CKIO & i_CEN;
assign  o_CKIO_NCEN =  o_CKIO & i_CEN;



///////////////////////////////////////////////////////////
//////  WDT Counter
////

//free-running prescaler on P-phi; the CKS tap fires when the divided count wraps (p.217)
logic   [11:0]  presc;
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) presc <= 12'd0;
    else begin if(i_CEN) begin
        if(pcen) presc <= presc + 12'd1;
    end end
end

logic           tap;
always_comb begin
    unique case(wt_cks)
        3'b000:  tap = 1'b1;                        //x1
        3'b001:  tap = (presc[1:0]  == 2'h3);       //x1/4
        3'b010:  tap = (presc[3:0]  == 4'hF);       //x1/16
        3'b011:  tap = (presc[4:0]  == 5'h1F);      //x1/32
        3'b100:  tap = (presc[5:0]  == 6'h3F);      //x1/64
        3'b101:  tap = (presc[7:0]  == 8'hFF);      //x1/256
        3'b110:  tap = (presc[9:0]  == 10'h3_FF);   //x1/1024
        default: tap = (presc[11:0] == 12'hFFF);    //x1/4096
    endcase
end

wire            wt_count    = i_CEN & pcen & tap & wt_tme & ~wr_wtcnt;  //software write wins
wire            wt_overflow = wt_count & (wtcnt == 8'hFF);

//WTCNT/WTCSR: pin-POR domain ONLY - retained across WDT-caused internal resets (p.215)
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) begin
        wtcnt   <= 8'd0;
        wt_tme  <= 1'b0;
        wt_it   <= 1'b0;
        wt_rsts <= 1'b0;
        wt_wovf <= 1'b0;
        wt_iovf <= 1'b0;
        wt_cks  <= 3'd0;
    end
    else begin if(i_CEN) begin
        if(wr_wtcnt)      wtcnt <= REG_BUS.wdata[7:0];
        else if(wt_count) wtcnt <= wtcnt + 8'd1;                //wraps 0xFF -> 0x00

        if(wr_wtcsr) begin
            wt_tme  <= REG_BUS.wdata[7];
            wt_it   <= REG_BUS.wdata[6];
            wt_rsts <= REG_BUS.wdata[5];
            wt_wovf <= REG_BUS.wdata[4] & wt_wovf;              //flags are clear-only
            wt_iovf <= REG_BUS.wdata[3] & wt_iovf;
            wt_cks  <= REG_BUS.wdata[2:0];
        end
        else if(wt_overflow) begin
            if(wt_it) wt_wovf <= 1'b1;                          //watchdog mode
            else      wt_iovf <= 1'b1;                          //interval mode -> ITI
        end
    end end
end

assign  o_ITI_REQ = wt_iovf;                        //level-held until software clears IOVF



///////////////////////////////////////////////////////////
//////  Watchdog Reset Request
////

//16-cycle stretched request per RSTS; pin-POR domain so the reset it CAUSES
//cannot clear it mid-pulse (this module never receives the folded resets)
logic   [4:0]   rst_cnt;
logic           rst_is_man;
wire            wdt_por_fire = wt_overflow & wt_it & ~wt_rsts;  //sync-clears FRQCR/STBCR below

always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) begin
        rst_cnt    <= 5'd0;
        rst_is_man <= 1'b0;
    end
    else begin if(i_CEN) begin
        if(wt_overflow && wt_it) begin
            rst_cnt    <= 5'd16;
            rst_is_man <= wt_rsts;
        end
        else if(rst_cnt != 5'd0) rst_cnt <= rst_cnt - 5'd1;
    end end
end

assign  o_WDT_RST_POR_n = ~((rst_cnt != 5'd0) & ~rst_is_man);
assign  o_WDT_RST_MAN_n = ~((rst_cnt != 5'd0) &  rst_is_man);



///////////////////////////////////////////////////////////
//////  CPG Registers
////

//FRQCR/STBCR/STBCR2: cleared by ANY power-on reset (pin async + WDT-POR sync
//term), retained through manual resets (p.211, p.183)
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) begin
        frqcr  <= 16'h0102;
        stbcr  <= 8'd0;
        stbcr2 <= 8'd0;
    end
    else begin if(i_CEN) begin
        if(wdt_por_fire) begin
            frqcr  <= 16'h0102;
            stbcr  <= 8'd0;
            stbcr2 <= 8'd0;
        end
        else begin
            //FRQCR is word-access-only (p.211); reserved bit 8 always reads 1
            if(reg_wr_word && REG_BUS.addr == 8'h80)
                frqcr  <= (REG_BUS.wdata[15:0] & 16'hE03F) | 16'h0100;
            if(reg_wr_byte && REG_BUS.addr == 8'h82)
                stbcr  <= REG_BUS.wdata[7:0] & 8'h97;           //STBY,STBXTL,MSTP2-0
            if(reg_wr_byte && REG_BUS.addr == 8'h88)
                stbcr2 <= REG_BUS.wdata[7:0] & 8'h7F;           //MDCHG,MSTP8-3
        end
    end end
end

endmodule

`default_nettype none
