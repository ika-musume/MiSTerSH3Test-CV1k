`default_nettype wire

/*
    Pin function controller + I/O ports (SH7709S sections 18/19, pp.565-611).

    Twelve ports (A-L, SCP), each pin owning a 2-bit PFC mode field in its
    PnCR control register (p.570): 00 other function / 01 port output /
    10 port input pull-up on / 11 port input pull-up off. The PnDR data
    registers follow the uniform table-19.x rule: input modes read the PIN,
    other-function/output modes read the REGISTER for writable bits and low
    for read-only bits; writes land only in writable bits and only drive
    the pad in output mode.

    Per-port capability masks below encode the datasheet matrix: input-only
    pins (all of F/G/L + SCPT7, D4/D6) never drive and their DR bits are
    read-only; port L has no pull-up MOS (p.607). PTG0's mode is controlled
    by PGCR bit 3, not bit 1 (silicon quirk, p.577 note).

    Reset: control + data registers initialize on POWER-ON reset only and
    hold through manual resets (pp.570-585, 588+). ASE mode is not
    implemented: E/F/G/H reset to their ASEMD0=1 (normal mode) values.

    FPGA adaptation (deviations): pads are dedicated split i/o/oe vectors -
    the "other function" muxing with the BSC/DMAC/UDI pins of table 18.1
    does not exist here (those pins are dedicated on this chip top). The
    implemented shares: the INTC PINT/IRQ/IRLS pins tap the C/F/H/SCPT pad
    inputs at the top level, and PTH7 hands its pad to the TMU's TCLK when
    its mode is 00 (o_PH7_FN). o_Px_PU exports the pull-up MOS state for
    the pad ring. Reserved mode combos store and decode as written.
*/

module ioport (
    /* CLOCK AND RESET - power-on domain only (registers hold through manual reset) */
    input   wire            i_POR_n,
    input   wire            i_CLK,
    input   wire            i_CEN,

    /* INTERFACES */
    IBus_2.slave            REG_BUS,        //P bus window 0x04000100-137 (behind the BSC)

    /* PADS - split input / output / output-enable / pull-up state */
    input   wire    [7:0]   i_PTA_I,
    output  wire    [7:0]   o_PTA_O,
    output  wire    [7:0]   o_PTA_OE,
    output  wire    [7:0]   o_PTA_PU,
    input   wire    [7:0]   i_PTB_I,
    output  wire    [7:0]   o_PTB_O,
    output  wire    [7:0]   o_PTB_OE,
    output  wire    [7:0]   o_PTB_PU,
    input   wire    [7:0]   i_PTC_I,
    output  wire    [7:0]   o_PTC_O,
    output  wire    [7:0]   o_PTC_OE,
    output  wire    [7:0]   o_PTC_PU,
    input   wire    [7:0]   i_PTD_I,
    output  wire    [7:0]   o_PTD_O,
    output  wire    [7:0]   o_PTD_OE,
    output  wire    [7:0]   o_PTD_PU,
    input   wire    [7:0]   i_PTE_I,
    output  wire    [7:0]   o_PTE_O,
    output  wire    [7:0]   o_PTE_OE,
    output  wire    [7:0]   o_PTE_PU,
    input   wire    [7:0]   i_PTF_I,        //input-only port
    output  wire    [7:0]   o_PTF_PU,
    input   wire    [7:0]   i_PTG_I,        //input-only port
    output  wire    [7:0]   o_PTG_PU,
    input   wire    [7:0]   i_PTH_I,
    output  wire    [7:0]   o_PTH_O,
    output  wire    [7:0]   o_PTH_OE,
    output  wire    [7:0]   o_PTH_PU,
    input   wire    [7:0]   i_PTJ_I,
    output  wire    [7:0]   o_PTJ_O,
    output  wire    [7:0]   o_PTJ_OE,
    output  wire    [7:0]   o_PTJ_PU,
    input   wire    [7:0]   i_PTK_I,
    output  wire    [7:0]   o_PTK_O,
    output  wire    [7:0]   o_PTK_OE,
    output  wire    [7:0]   o_PTK_PU,
    input   wire    [7:0]   i_PTL_I,        //input-only port, no pull-up MOS
    input   wire    [7:0]   i_SCPT_I,
    output  wire    [7:0]   o_SCPT_O,
    output  wire    [7:0]   o_SCPT_OE,
    output  wire    [7:0]   o_SCPT_PU,

    /* PFC FANOUT */
    output  wire            o_PH7_FN,       //PTH7 mode 00: pad belongs to TCLK (TMU, p.392)
    output  wire    [7:0]   o_PC_FN,        //PTC mode 00: pad belongs to MCS (BSC, p.323)
    output  wire    [7:0]   o_PD_FN         //PTD mode 00: pad belongs to DACK/DREQ/DRAK (DMAC)
);

///////////////////////////////////////////////////////////
//////  Capability Masks and Reset Values
////

//port index 0-11 = A,B,C,D,E,F,G,H,J,K,L,SCP (the register map order)
localparam int NP = 12;

//DRV: pin can drive AND its DR bit is writable (the two capabilities
//coincide on every port of the datasheet matrix, sections 19.2-19.13)
localparam logic [7:0] DRV [0:NP-1] = '{
    8'hFF, 8'hFF, 8'hFF,        //A, B, C
    8'hAF,                      //D: PTD4/PTD6 input-only (p.594)
    8'hFF,                      //E
    8'h00, 8'h00,               //F, G: input-only (pp.598-600)
    8'h80,                      //H: only PTH7 drives (p.602)
    8'hFF, 8'hFF,               //J, K
    8'h00,                      //L: input-only (p.608)
    8'h7F};                     //SCP: SCPT7 input-only (p.611)

//PUE: pin has an input pull-up MOS (port L is analog-shared, p.607)
localparam logic [7:0] PUE [0:NP-1] = '{
    8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF,
    8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'h00, 8'hFF};

//control register power-on values, ASEMD0=1 normal mode (table 18.2)
localparam logic [15:0] CRI [0:NP-1] = '{
    16'h0000, 16'h0000, 16'hAAAA, 16'hAA8A, 16'hAAAA, 16'hAAAA,
    16'hAAAA, 16'hAAAA, 16'h0000, 16'h0000, 16'h0000, 16'hA888};



///////////////////////////////////////////////////////////
//////  Register Storage
////

logic   [15:0]  pcr [0:NP-1];               //PnCR pin mode fields
logic   [7:0]   pdr [0:NP-1];               //PnDR data registers (undefined bits reset 0)

//pad input gather
wire    [7:0]   pin_i [0:NP-1];
assign  pin_i[0]  = i_PTA_I;
assign  pin_i[1]  = i_PTB_I;
assign  pin_i[2]  = i_PTC_I;
assign  pin_i[3]  = i_PTD_I;
assign  pin_i[4]  = i_PTE_I;
assign  pin_i[5]  = i_PTF_I;
assign  pin_i[6]  = i_PTG_I;
assign  pin_i[7]  = i_PTH_I;
assign  pin_i[8]  = i_PTJ_I;
assign  pin_i[9]  = i_PTK_I;
assign  pin_i[10] = i_PTL_I;
assign  pin_i[11] = i_SCPT_I;



///////////////////////////////////////////////////////////
//////  Pin Mode Decode
////

//per-pin mode bits; PTG0 quirk: its MD1 is PGCR bit 3 (shared with PTG1),
//not bit 1 (p.577 note)
logic   [7:0]   md1 [0:NP-1];
logic   [7:0]   md0 [0:NP-1];
always_comb begin
    for(int p = 0; p < NP; p++) begin
        for(int n = 0; n < 8; n++) begin
            md1[p][n] = pcr[p][2*n+1];
            md0[p][n] = pcr[p][2*n];
        end
    end
    md1[6][0] = pcr[6][3];
end

//pad drive / pull-up / DR read value per the uniform table-19.x rule
logic   [7:0]   pin_oe [0:NP-1];
logic   [7:0]   pin_pu [0:NP-1];
logic   [7:0]   pdr_rd [0:NP-1];
always_comb begin
    for(int p = 0; p < NP; p++) begin
        pin_oe[p] = ~md1[p] &  md0[p] & DRV[p];         //mode 01 on a driving pin
        pin_pu[p] =  md1[p] & ~md0[p] & PUE[p];         //mode 10: pull-up MOS on
        for(int n = 0; n < 8; n++)                      //input modes read the pin;
            pdr_rd[p][n] = md1[p][n] ? pin_i[p][n]      //others read DR (RO bits: low)
                                     : (DRV[p][n] & pdr[p][n]);
    end
end



///////////////////////////////////////////////////////////
//////  Register File Access
////

//window offsets: PnCR at 0x00+2n (0x04000100+), PnDR at 0x20+2n (0x04000120+)
wire            adr_ctl  = (REG_BUS.addr[7:5] == 3'b000) && (REG_BUS.addr[4:1] < 4'(NP));
wire            adr_dat  = (REG_BUS.addr[7:5] == 3'b001) && (REG_BUS.addr[4:1] < 4'(NP));
wire    [3:0]   adr_idx  = REG_BUS.addr[4:1];

always_comb begin
    if     (adr_ctl) REG_BUS.rdata = {16'd0, pcr[adr_idx]};
    else if(adr_dat) REG_BUS.rdata = {24'd0, pdr_rd[adr_idx]};
    else             REG_BUS.rdata = 32'd0;             //reserved offsets read 0
end

wire            reg_wr = REG_BUS.stb && REG_BUS.we;

always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) begin
        for(int p = 0; p < NP; p++) begin
            pcr[p] <= CRI[p];
            pdr[p] <= 8'd0;
        end
    end
    else begin if(i_CEN) begin
        if(reg_wr && adr_ctl) pcr[adr_idx] <= REG_BUS.wdata[15:0];
        if(reg_wr && adr_dat)                           //RO bits ignore writes (table 19.x)
            pdr[adr_idx] <= (pdr[adr_idx] & ~DRV[adr_idx]) |
                            (REG_BUS.wdata[7:0] & DRV[adr_idx]);
    end end
end



///////////////////////////////////////////////////////////
//////  Pad Fanout
////

assign  o_PTA_O   = pdr[0];     assign  o_PTA_OE  = pin_oe[0];  assign  o_PTA_PU  = pin_pu[0];
assign  o_PTB_O   = pdr[1];     assign  o_PTB_OE  = pin_oe[1];  assign  o_PTB_PU  = pin_pu[1];
assign  o_PTC_O   = pdr[2];     assign  o_PTC_OE  = pin_oe[2];  assign  o_PTC_PU  = pin_pu[2];
assign  o_PTD_O   = pdr[3];     assign  o_PTD_OE  = pin_oe[3];  assign  o_PTD_PU  = pin_pu[3];
assign  o_PTE_O   = pdr[4];     assign  o_PTE_OE  = pin_oe[4];  assign  o_PTE_PU  = pin_pu[4];
assign  o_PTF_PU  = pin_pu[5];
assign  o_PTG_PU  = pin_pu[6];
assign  o_PTH_O   = pdr[7];     assign  o_PTH_OE  = pin_oe[7];  assign  o_PTH_PU  = pin_pu[7];
assign  o_PTJ_O   = pdr[8];     assign  o_PTJ_OE  = pin_oe[8];  assign  o_PTJ_PU  = pin_pu[8];
assign  o_PTK_O   = pdr[9];     assign  o_PTK_OE  = pin_oe[9];  assign  o_PTK_PU  = pin_pu[9];
assign  o_SCPT_O  = pdr[11];    assign  o_SCPT_OE = pin_oe[11]; assign  o_SCPT_PU = pin_pu[11];

//PTH7 mode 00 hands the pad to the TMU's TCLK (table 18.1; merge at the top)
assign  o_PH7_FN  = (pcr[7][15:14] == 2'b00);

//PTC mode 00 hands each pad to its MCS output (table 18.1; merge at the top)
assign  o_PC_FN   = ~md1[2] & ~md0[2];

//PTD mode 00 hands each pad to the DMAC's DACK/DREQ/DRAK (table 18.1)
assign  o_PD_FN   = ~md1[3] & ~md0[3];

endmodule

`default_nettype none
