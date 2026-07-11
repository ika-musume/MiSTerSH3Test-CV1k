`default_nettype none

/*
    Cache memory primitives, kept in dedicated modules so synthesis maps them
    cleanly onto Cyclone V block RAM. All arrays carry an explicit ramstyle
    directive (M10K) and the synchronous registered-read pattern Quartus needs
    to infer BRAM. The valid bit is folded into each tag entry (bit 22) so the
    4-way lookup is a single registered read with no flop-array mux; CCR.CF
    clears validity with a short index walk in the controller (p.106). The
    round-robin replacement pointer lives in its own small block RAM, read in
    step with the tag.

    The data array is split into one bank per way (cache_data_bank_wt x4) so all
    four ways are read in parallel and the hit way is selected after the tag
    compare - this avoids any tag-RAM -> data-RAM combinational chain. Each bank
    is a SIMPLE DUAL-PORT (1R1W) M10K: one read and one independent write address.

    SINGLE-CLOCK write-through (new-data) bypass: both ports now capture on the
    SAME edge, so a write and a read-address capture to one cell collide - the
    M10K mixed-port RDW returns OLD data (Cyclone V has no mixed-port new-data
    mode in silicon; altsyncram offers only OLD_DATA/DONT_CARE there). The
    dclk design dodged this by time-sharing (write cen_p, read cen_n). Here an
    EXPLICIT soft bypass restores write-before-read order: the collision compare
    and the write word are REGISTERED at the capture edge, and o_DO muxes the
    held write data over the RAM q per lane. One 2:1 after the RAM output;
    identical netlist in simulation and synthesis (no sim/synth split needed).
    no_rw_check stays: the colliding RAM lanes are never consumed.
*/

/* verilator lint_off DECLFILENAME */

///////////////////////////////////////////////////////////
//////  Data bank - 1R1W + per-byte write-through bypass, one per way (4 kB)
////

module cache_data_bank_wt (
    input   wire            i_CLK,
    input   wire            i_EN,       //single architectural clock enable
    input   wire    [9:0]   i_RADDR,    //read  port {index[7:0], word[1:0]} = 1024 longwords
    input   wire            i_WE,       //write port enable (this way's bank)
    input   wire    [3:0]   i_BWE,      //per-byte lane enable; lane b <-> i_DI[8b+7:8b]
    input   wire    [9:0]   i_WADDR,    //write port {index[7:0], word[1:0]}
    input   wire    [31:0]  i_DI,
    output  wire    [31:0]  o_DO
);

//Packed lane array = the Quartus byte-enable M10K template. A sub-word store writes
//only its strobed lanes, so the controller needs no read-modify-write merge word.
(* ramstyle = "M10K, no_rw_check" *) logic [3:0][7:0] ram [0:1023];

logic   [31:0]  rd_q;       //RAM read word (old data on a collision)
logic   [3:0]   byp_q;      //per-lane collision: this edge wrote the cell being read
logic   [31:0]  di_q;       //held write word for the bypass lanes

always_ff @(posedge i_CLK) if(i_EN) begin
    if(i_WE) begin                              //write port - independent address
        if(i_BWE[0]) ram[i_WADDR][0] <= i_DI[ 7: 0];
        if(i_BWE[1]) ram[i_WADDR][1] <= i_DI[15: 8];
        if(i_BWE[2]) ram[i_WADDR][2] <= i_DI[23:16];
        if(i_BWE[3]) ram[i_WADDR][3] <= i_DI[31:24];
    end
    rd_q  <= ram[i_RADDR];                      //read port - registered address/old data
    byp_q <= {4{i_WE && (i_WADDR == i_RADDR)}} & i_BWE;   //same-edge RDW, per strobed lane
    di_q  <= i_DI;
end

//Write-through compose: bypassed lanes take the held write byte, others the RAM q.
assign  o_DO[ 7: 0] = byp_q[0] ? di_q[ 7: 0] : rd_q[ 7: 0];
assign  o_DO[15: 8] = byp_q[1] ? di_q[15: 8] : rd_q[15: 8];
assign  o_DO[23:16] = byp_q[2] ? di_q[23:16] : rd_q[23:16];
assign  o_DO[31:24] = byp_q[3] ? di_q[31:24] : rd_q[31:24];

endmodule


///////////////////////////////////////////////////////////
//////  Tag RAM - 1R1W + write-through bypass (256 x 21 = {valid, U, tag[18:0]})
////

/*
    M10K, MEASURED choice: an MLAB variant (async read + fabric rdaddr_q) was built
    and fitted - the 256-deep composition needs 8-deep MLAB banking plus a wide
    output mux, and the depth-mux levels + inter-LAB routing cost MORE than the
    M10K's ~2.3 ns tCO. 32-deep arrays are the profitable MLAB shape; 256-deep is
    not. The bypass here also guards the DIRTY (U) bit: a store-hit U write must be
    visible to the very next lookup of the same set, else a stale-clean victim
    would skip its write-back (lost store).
*/

module cache_tag_ram_wt (
    input   wire            i_CLK,
    input   wire            i_EN,
    input   wire    [7:0]   i_RADDR,
    input   wire            i_WE,
    input   wire    [7:0]   i_WADDR,
    input   wire    [20:0]  i_DI,
    output  wire    [20:0]  o_DO
);

(* ramstyle = "M10K, no_rw_check" *) logic [20:0] ram [0:255];

logic   [20:0]  rd_q;
logic           byp_q;
logic   [20:0]  di_q;

always_ff @(posedge i_CLK) if(i_EN) begin
    if(i_WE) ram[i_WADDR] <= i_DI;
    rd_q  <= ram[i_RADDR];
    byp_q <= i_WE && (i_WADDR == i_RADDR);
    di_q  <= i_DI;
end

assign  o_DO = byp_q ? di_q : rd_q;

endmodule


///////////////////////////////////////////////////////////
//////  LRU RAM - 1R1W + write-through bypass (256 x 6, 6-bit pseudo-LRU; p.104)
////

/*
    The bypass keeps replacement decisions bit-exact with the dclk ordering: a
    same-set access right after an MRU update must see the updated pseudo-LRU,
    else victim choices (and thus external write-back traffic) would diverge.
*/

module cache_lru_ram_wt (
    input   wire            i_CLK,
    input   wire            i_EN,
    input   wire    [7:0]   i_RADDR,
    input   wire            i_WE,
    input   wire    [7:0]   i_WADDR,
    input   wire    [5:0]   i_DI,
    output  wire    [5:0]   o_DO
);

(* ramstyle = "M10K, no_rw_check" *) logic [5:0] ram [0:255];

logic   [5:0]   rd_q;
logic           byp_q;
logic   [5:0]   di_q;

always_ff @(posedge i_CLK) if(i_EN) begin
    if(i_WE) ram[i_WADDR] <= i_DI;
    rd_q  <= ram[i_RADDR];
    byp_q <= i_WE && (i_WADDR == i_RADDR);
    di_q  <= i_DI;
end

assign  o_DO = byp_q ? di_q : rd_q;

endmodule

/* verilator lint_on DECLFILENAME */

`default_nettype none
