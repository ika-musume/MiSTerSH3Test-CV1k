`default_nettype wire

/*
    I bus 1 address splitter.

    One master (the cache's I_BUS) fans out to two slaves: the BRIDGE for the
    on-chip I-bus-2 register windows, and the external leg (BSC in session 2;
    the tb memory model until then). ZERO LATENCY by design - the request and
    response paths are pure muxes, no registration, no added handshake beats.
    A single owner flop captured at the accept edge steers the response;
    one-outstanding is guaranteed by the cache FSM (mem_pending).

    Register windows routed to the bridge (Appendix B, pp.739-741):
        0xFFFFFF80-8F  CPG/WDT   (FRQCR/STBCR/WTCNT/WTCSR/STBCR2)
        0xFFFFFEE0-EF  INTC high (ICR0/IPRA/IPRB)
        0xA4000000-1F  INTC low  (INTEVT2/IRR0-2/ICR1/ICR2/PINTER/IPRC-E, P2 alias)
    CCR2 (0xA40000B0) never appears here - the cache serves it internally.
*/

module ibus_splitter (
    /* CLOCK AND RESET */
    input   wire            i_RST_n,
    input   wire            i_CLK,
    input   wire            i_CEN,

    /* INTERFACES */
    IBus_1.slave            CORE_BUS,   //from cpu_core (cache master)
    IBus_1.master           BRG_BUS,    //to ibus_bridge (register windows above)
    IBus_1.master           EXT_BUS     //everything else (external memory / future BSC)
);

///////////////////////////////////////////////////////////
//////  Request Routing
////

//window decode; comparators sit on registered cache state + the fill adder whose
//low bits never carry into the compared field (fill_base[3:0] is zero)
wire            hit_brg = (CORE_BUS.req_addr[31:4] == 28'hFFF_FFF8) ||  //CPG/WDT
                          (CORE_BUS.req_addr[31:4] == 28'hFFF_FFEE) ||  //INTC high
                          (CORE_BUS.req_addr[31:5] == 27'h520_0000);    //INTC low (0xA4000000)

//request fields fan to both slaves; valid steers by decode, ready muxes back
assign  BRG_BUS.req_valid = CORE_BUS.req_valid & hit_brg;
assign  BRG_BUS.req_write = CORE_BUS.req_write;
assign  BRG_BUS.req_size  = CORE_BUS.req_size;
assign  BRG_BUS.req_burst = CORE_BUS.req_burst;
assign  BRG_BUS.req_addr  = CORE_BUS.req_addr;
assign  BRG_BUS.req_wdata = CORE_BUS.req_wdata;
assign  BRG_BUS.req_wstrb = CORE_BUS.req_wstrb;
assign  BRG_BUS.req_lock  = CORE_BUS.req_lock;
assign  BRG_BUS.req_dack    = CORE_BUS.req_dack;    //DMAC sideband (BSC-only consumer;
assign  BRG_BUS.req_dack_ch = CORE_BUS.req_dack_ch; //the bridge ignores it)
assign  BRG_BUS.req_saddr   = CORE_BUS.req_saddr;

assign  EXT_BUS.req_valid = CORE_BUS.req_valid & ~hit_brg;
assign  EXT_BUS.req_write = CORE_BUS.req_write;
assign  EXT_BUS.req_size  = CORE_BUS.req_size;
assign  EXT_BUS.req_burst = CORE_BUS.req_burst;
assign  EXT_BUS.req_addr  = CORE_BUS.req_addr;
assign  EXT_BUS.req_wdata = CORE_BUS.req_wdata;
assign  EXT_BUS.req_wstrb = CORE_BUS.req_wstrb;
assign  EXT_BUS.req_lock  = CORE_BUS.req_lock;
assign  EXT_BUS.req_dack    = CORE_BUS.req_dack;
assign  EXT_BUS.req_dack_ch = CORE_BUS.req_dack_ch;
assign  EXT_BUS.req_saddr   = CORE_BUS.req_saddr;

assign  CORE_BUS.req_ready = hit_brg ? BRG_BUS.req_ready : EXT_BUS.req_ready;



///////////////////////////////////////////////////////////
//////  Response Routing
////

//owner of the outstanding transaction, captured at the accept edge
logic           owner_brg;
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) owner_brg <= 1'b0;
    else begin if(i_CEN) begin
        if(CORE_BUS.req_valid && CORE_BUS.req_ready) owner_brg <= hit_brg;
    end end
end

assign  CORE_BUS.rsp_valid = owner_brg ? BRG_BUS.rsp_valid : EXT_BUS.rsp_valid;
assign  CORE_BUS.rsp_rdata = owner_brg ? BRG_BUS.rsp_rdata : EXT_BUS.rsp_rdata;
assign  CORE_BUS.rsp_fault = owner_brg ? BRG_BUS.rsp_fault : EXT_BUS.rsp_fault;
assign  BRG_BUS.rsp_ready  = CORE_BUS.rsp_ready &  owner_brg;
assign  EXT_BUS.rsp_ready  = CORE_BUS.rsp_ready & ~owner_brg;

endmodule

`default_nettype none
