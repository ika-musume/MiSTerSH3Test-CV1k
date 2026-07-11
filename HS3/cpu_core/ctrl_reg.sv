`default_nettype wire

/*
    SH7709S control-register file.

    This block owns SR, GBR, VBR, SSR, and SPC. Instruction execution, exception
    entry, and RTE restore all update these registers through one arbiter.
*/

module ctrl_reg (
    /* CLOCK AND RESET */
    input   wire            i_RST_n,
    input   wire            i_CLK,
    input   wire            i_CEN,

    /* EXCEPTION OPERATIONS */
    input   wire            i_RESET_LIKE_VALID,
    input   wire            i_EXC_ENTRY_VALID,
    input   wire    [31:0]  i_EXC_ENTRY_SPC,
    input   wire            i_RTE_RESTORE_VALID,

    /* PIPELINE CONTROL-REGISTER WRITEBACK */
    input   wire            i_PIPE_CTRL_WE,
    input   wire    [2:0]   i_PIPE_CTRL_DST,
    input   wire    [31:0]  i_PIPE_CTRL_DATA,
    input   wire            i_PIPE_SR_T_WE,
    input   wire            i_PIPE_SR_T,
    input   wire            i_PIPE_SR_S_WE,
    input   wire            i_PIPE_SR_S,
    input   wire    [1:0]   i_PIPE_SR_MQ_WE,
    input   wire    [1:0]   i_PIPE_SR_MQ,

    /* CONTROL REGISTER OBSERVATION */
    output  logic   [31:0]  o_SR,
    output  logic   [31:0]  o_GBR,
    output  logic   [31:0]  o_SSR,
    output  logic   [31:0]  o_SPC,
    output  logic   [31:0]  o_VBR
);

///////////////////////////////////////////////////////////
//////  Constants
////

/*
    SR reset and writable bit masks follow section 2.1.3, pp.13-14 and
    exception-entry behavior in section 4, pp.93-101.
*/

localparam logic [31:0] SR_RESET_VALUE    = 32'h7000_00F0;
localparam logic [31:0] SR_EXCEPTION_BITS = 32'h7000_0000;
localparam logic [31:0] SR_MASK           = 32'h7000_13F3;

localparam logic [2:0] CTRL_SR            = 3'd1;
localparam logic [2:0] CTRL_GBR           = 3'd2;
localparam logic [2:0] CTRL_VBR           = 3'd3;
localparam logic [2:0] CTRL_SSR           = 3'd4;
localparam logic [2:0] CTRL_SPC           = 3'd5;

///////////////////////////////////////////////////////////
//////  Register Updates
////

/*
    One arbiter keeps exception entry atomic with respect to retired LDC/STC.
    Priority is reset, reset-like exception, exception entry, RTE, then WB.
*/

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        o_SR  <= SR_RESET_VALUE;
        o_GBR <= 32'd0;
        o_SSR <= 32'd0;
        o_SPC <= 32'd0;
        o_VBR <= 32'd0;
    end
    else begin if(i_CEN) begin
        if(i_RESET_LIKE_VALID) begin
            o_SR  <= SR_RESET_VALUE;
            o_VBR <= 32'd0;
        end
        else if(i_EXC_ENTRY_VALID) begin
            o_SSR <= o_SR;
            o_SPC <= i_EXC_ENTRY_SPC;
            o_SR  <= (o_SR | SR_EXCEPTION_BITS) & SR_MASK;
        end
        else if(i_RTE_RESTORE_VALID) begin
            o_SR <= o_SSR & SR_MASK;
        end
        else if(i_PIPE_CTRL_WE) begin
            unique case(i_PIPE_CTRL_DST)
                CTRL_SR:  o_SR  <= i_PIPE_CTRL_DATA & SR_MASK;
                CTRL_GBR: o_GBR <= i_PIPE_CTRL_DATA;
                CTRL_VBR: o_VBR <= i_PIPE_CTRL_DATA;
                CTRL_SSR: o_SSR <= i_PIPE_CTRL_DATA & SR_MASK;
                CTRL_SPC: o_SPC <= i_PIPE_CTRL_DATA;
                default: begin end
            endcase
        end
        else begin
            if(i_PIPE_SR_T_WE)     o_SR[0] <= i_PIPE_SR_T;
            if(i_PIPE_SR_S_WE)     o_SR[1] <= i_PIPE_SR_S;
            if(i_PIPE_SR_MQ_WE[0]) o_SR[8] <= i_PIPE_SR_MQ[0];
            if(i_PIPE_SR_MQ_WE[1]) o_SR[9] <= i_PIPE_SR_MQ[1];
        end
    end end
end

endmodule

`default_nettype none
