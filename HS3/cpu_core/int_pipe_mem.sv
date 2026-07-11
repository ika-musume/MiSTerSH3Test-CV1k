`default_nettype none

/*
    Integer-pipeline memory primitives.

    GPR file, single-clock 2R2W. The dclk TDP time-mux (writes cen_p / reads
    cen_n on the same two physical ports) dies with the half-clock: one edge now
    carries up to two WB writes AND two ID read-address captures. Built as a
    LIVE-VALUE-TABLE bank pair: bank0 holds lane-0 writes, bank1 lane-1 writes,
    each duplicated per read port -> four 32x32 SIMPLE DUAL-PORT M10Ks. A 32x1
    FF table records which bank wrote each register last; the read output is one
    registered-select 2:1 over the two bank words (select tCO << RAM q tCO).

    M10K is the MEASURED choice: an MLAB 2W2R LVT variant was built and fitted
    on the dclk netlist - its read (FF -> 2 MLAB cells -> lvt mux, three routed
    fabric hops) measured ~1 ns SLOWER than the M10K's self-contained ~2.3 ns
    tCO. The LVT concept was fine; the MLAB read path was not.

    Same-edge W/R collision returns OLD data (RAM old-data + pre-edge LVT, a
    consistent pair); the pipeline's WB shadow forward lanes (wb0z/wb1z in
    int_pipe) supply the new value, so no output bypass mux is needed here.
    Lane0/lane1 same-destination double writes are suppressed upstream in ID
    (gpr1_we killed on gpr0_dst == gpr1_dst), so LVT write order never matters.
*/

/* verilator lint_off DECLFILENAME */

///////////////////////////////////////////////////////////
//////  GPR file - 2R2W LVT bank pair (32 x 32)
////

module gpr_2r2w (
    input   wire            i_CLK,
    input   wire            i_EN,

    /* WRITE LANES (WB) */
    input   wire            i_WE0,
    input   wire    [4:0]   i_WADDR0,
    input   wire    [31:0]  i_WDATA0,
    input   wire            i_WE1,
    input   wire    [4:0]   i_WADDR1,
    input   wire    [31:0]  i_WDATA1,

    /* READ PORTS (ID) - addresses captured this edge, data out next cycle */
    input   wire    [4:0]   i_RADDR0,
    input   wire    [4:0]   i_RADDR1,
    output  wire    [31:0]  o_RDATA0,
    output  wire    [31:0]  o_RDATA1
);

//Four SDP banks: bank<lane>_r<port>. no_rw_check: collisions are architected
//around (shadow forward upstream), so the colliding q is never consumed.
(* ramstyle = "M10K, no_rw_check" *) logic [31:0] bank0_r0 [0:31];
(* ramstyle = "M10K, no_rw_check" *) logic [31:0] bank0_r1 [0:31];
(* ramstyle = "M10K, no_rw_check" *) logic [31:0] bank1_r0 [0:31];
(* ramstyle = "M10K, no_rw_check" *) logic [31:0] bank1_r1 [0:31];

logic   [31:0]  lvt;                    //1 = bank1 (lane 1) wrote this register last
logic   [31:0]  q_b0r0, q_b0r1, q_b1r0, q_b1r1;
logic           sel_r0, sel_r1;         //registered LVT picks, in step with the RAM q

always_ff @(posedge i_CLK) if(i_EN) begin
    if(i_WE0) begin
        bank0_r0[i_WADDR0] <= i_WDATA0;
        bank0_r1[i_WADDR0] <= i_WDATA0;
    end
    if(i_WE1) begin
        bank1_r0[i_WADDR1] <= i_WDATA1;
        bank1_r1[i_WADDR1] <= i_WDATA1;
    end
    q_b0r0 <= bank0_r0[i_RADDR0];
    q_b0r1 <= bank0_r1[i_RADDR1];
    q_b1r0 <= bank1_r0[i_RADDR0];
    q_b1r1 <= bank1_r1[i_RADDR1];
    //Pre-edge LVT values: a same-edge write is not visible (old-data, matching the banks).
    sel_r0 <= lvt[i_RADDR0];
    sel_r1 <= lvt[i_RADDR1];
end

always_ff @(posedge i_CLK) if(i_EN) begin
    if(i_WE0) lvt[i_WADDR0] <= 1'b0;
    if(i_WE1) lvt[i_WADDR1] <= 1'b1;
end

assign  o_RDATA0 = sel_r0 ? q_b1r0 : q_b0r0;
assign  o_RDATA1 = sel_r1 ? q_b1r1 : q_b0r1;

// synthesis translate_off
//Behavioral mirror for testbench probing (cpu_core_tb gpr()/bank checks read .ram).
//Not synthesized; the LVT banks above are the authoritative hardware.
logic   [31:0]  ram [0:31];
always_ff @(posedge i_CLK) if(i_EN) begin
    if(i_WE0) ram[i_WADDR0] <= i_WDATA0;
    if(i_WE1) ram[i_WADDR1] <= i_WDATA1;
end
// synthesis translate_on

endmodule

/* verilator lint_on DECLFILENAME */

`default_nettype none
