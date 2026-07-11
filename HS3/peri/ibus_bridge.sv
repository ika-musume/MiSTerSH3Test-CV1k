`default_nettype wire

/*
    I bus 1 -> I bus 2 BRIDGE.

    The SH7709S manual shows this block only in the block diagram (Fig 1.1,
    p.6) with no textual description; the implementation here follows its
    apparent role - a one-cycle latch stage that adapts the 32-bit lane/wstrb
    I bus 1 onto the slow register tier and performs data alignment once for
    every slave:

      - write payloads are RIGHT-JUSTIFIED by the byte strobes (the strobed
        lane IS the payload lane in either endianness);
      - read values come back right-justified and are REPLICATED across the
        32-bit lanes by access size, so the pipeline load aligner picks the
        correct byte/halfword from any naturally aligned address.

    IDLE -> ACCESS -> RESP: read latency is 2 cycles request-to-response; a
    write is posted from the pipeline's viewpoint (the response only frees the
    cache's mem_pending). Handshake latency is allowed ONLY on this tier.
    Unmapped offsets inside a window read 0 / drop writes in the slave.
*/

module ibus_bridge (
    /* CLOCK AND RESET */
    input   wire            i_RST_n,
    input   wire            i_CLK,
    input   wire            i_CEN,

    /* INTERFACES */
    IBus_1.slave            I_BUS,          //from the splitter (register windows only)
    IBus_2.master           REG_CPG,        //0xFFFFFF80-8F
    IBus_2.master           REG_INTC_HI,    //0xFFFFFEE0-EF
    IBus_2.master           REG_INTC_LO     //0xA4000000-1F
);

///////////////////////////////////////////////////////////
//////  Request Capture
////

localparam logic [1:0] S_IDLE   = 2'd0;
localparam logic [1:0] S_ACCESS = 2'd1;     //IBus_2 strobe cycle: write commits, read sampled
localparam logic [1:0] S_RESP   = 2'd2;     //hold rsp_valid until the master takes it

logic   [1:0]   state;
logic   [2:0]   sel_q;          //captured window one-hot {INTC_LO, INTC_HI, CPG}
logic   [7:0]   addr_q;         //byte address within the window
logic           we_q;
logic   [1:0]   size_q;
logic   [31:0]  wdata_rj_q;     //right-justified write payload
logic   [31:0]  rsp_rdata_q;    //lane-replicated read value for the response

//window re-decode off the live address (the splitter guarantees one matches)
wire    [2:0]   sel_live = {I_BUS.req_addr[31:5] == 27'h520_0000,
                            I_BUS.req_addr[31:4] == 28'hFFF_FFEE,
                            I_BUS.req_addr[31:4] == 28'hFFF_FFF8};

//right-justify the write payload off the strobes: the strobed lane is the payload
//lane regardless of endianness; misaligned accesses fault in MA and never get here
logic   [31:0]  wdata_rj;
always_comb begin
    unique case(I_BUS.req_wstrb)
        4'b1000: wdata_rj = {24'd0, I_BUS.req_wdata[31:24]};
        4'b0100: wdata_rj = {24'd0, I_BUS.req_wdata[23:16]};
        4'b0010: wdata_rj = {24'd0, I_BUS.req_wdata[15:8]};
        4'b0001: wdata_rj = {24'd0, I_BUS.req_wdata[7:0]};
        4'b1100: wdata_rj = {16'd0, I_BUS.req_wdata[31:16]};
        4'b0011: wdata_rj = {16'd0, I_BUS.req_wdata[15:0]};
        default: wdata_rj = I_BUS.req_wdata;                    //long (or read: don't-care)
    endcase
end

assign  I_BUS.req_ready = (state == S_IDLE);



///////////////////////////////////////////////////////////
//////  Access FSM
////

//read value of the selected slave, then replicated onto the lanes by size so the
//pipe's load aligner works from any aligned offset (byte -> x4, word -> x2)
wire    [31:0]  sel_rdata = ({32{sel_q[0]}} & REG_CPG.rdata    ) |
                            ({32{sel_q[1]}} & REG_INTC_HI.rdata) |
                            ({32{sel_q[2]}} & REG_INTC_LO.rdata);
logic   [31:0]  rdata_rep;
always_comb begin
    unique case(size_q)
        2'd0:    rdata_rep = {4{sel_rdata[7:0]}};
        2'd1:    rdata_rep = {2{sel_rdata[15:0]}};
        default: rdata_rep = sel_rdata;
    endcase
end

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        state       <= S_IDLE;
        sel_q       <= 3'd0;
        addr_q      <= 8'd0;
        we_q        <= 1'b0;
        size_q      <= 2'd0;
        wdata_rj_q  <= 32'd0;
        rsp_rdata_q <= 32'd0;
    end
    else begin if(i_CEN) begin
        unique case(state)
            S_IDLE: begin
                if(I_BUS.req_valid) begin                       //accept edge: latch everything
                    sel_q      <= sel_live;
                    addr_q     <= I_BUS.req_addr[7:0];
                    we_q       <= I_BUS.req_write;
                    size_q     <= I_BUS.req_size;
                    wdata_rj_q <= wdata_rj;
                    state      <= S_ACCESS;
                end
            end
            S_ACCESS: begin                                     //slave strobe: sample the read
                rsp_rdata_q <= rdata_rep;
                state       <= S_RESP;
            end
            default: begin                                      //S_RESP
                if(I_BUS.rsp_ready) state <= S_IDLE;
            end
        endcase
    end end
end

//response: fault-free tier (unmapped offsets read 0 in the slaves)
assign  I_BUS.rsp_valid = (state == S_RESP);
assign  I_BUS.rsp_rdata = rsp_rdata_q;
assign  I_BUS.rsp_fault = 1'b0;



///////////////////////////////////////////////////////////
//////  IBus_2 Drive
////

//shared payload fans to all three slaves; only the selected one sees its strobe
assign  REG_CPG.stb       = (state == S_ACCESS) & sel_q[0];
assign  REG_INTC_HI.stb   = (state == S_ACCESS) & sel_q[1];
assign  REG_INTC_LO.stb   = (state == S_ACCESS) & sel_q[2];

assign  REG_CPG.we        = we_q;
assign  REG_CPG.size      = size_q;
assign  REG_CPG.addr      = addr_q;
assign  REG_CPG.wdata     = wdata_rj_q;
assign  REG_INTC_HI.we    = we_q;
assign  REG_INTC_HI.size  = size_q;
assign  REG_INTC_HI.addr  = addr_q;
assign  REG_INTC_HI.wdata = wdata_rj_q;
assign  REG_INTC_LO.we    = we_q;
assign  REG_INTC_LO.size  = size_q;
assign  REG_INTC_LO.addr  = addr_q;
assign  REG_INTC_LO.wdata = wdata_rj_q;

endmodule

`default_nettype none
