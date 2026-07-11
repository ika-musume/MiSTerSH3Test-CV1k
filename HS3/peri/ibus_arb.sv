`default_nettype wire

/*
    I bus 1 on-chip arbiter: CPU (cache) + DMAC -> the splitter.

    The DMAC is the second on-chip bus master (Fig 11.1); it reaches
    memory through the same splitter->BSC path as the CPU, so the BSC
    shapes its external cycles "in the same way as when the CPU is the
    bus master" (p.363). BREQ/BACK stays external-only in the BSC - it
    is NOT this arbiter.

    ZERO-COST idle path (the phase-2 gate): the owner flop parks on the
    CPU, every request field passes through ONE 2:1 mux whose select is
    the REGISTERED owner, and ready/valid gating is a single AND - with
    the DMAC idle the CPU sees a wire, no added beats, cycle-identical.

    Ownership switches ONLY at idle boundaries: never mid transaction
    (one-outstanding fabric), never inside a CPU locked RMW pair (TAS,
    p.320), never inside a 4-beat cache fill/drain burst (the BSC
    assumes a drain completes once started), and never inside a DMAC
    transfer unit (i_DMA_HOLD: the dual-mode R->W pair, indirect
    4-cycle, or 16-byte 4R+4W is indivisible against the CPU, fig
    11.12 - tier 1; BREQ/refresh still split at bus-cycle granularity
    inside the BSC - tier 2). At an idle boundary the DMAC outranks the
    CPU (on-chip priority: DMAC > CPU, section 10).

    Responses broadcast data; per-master valid gates off the response
    owner captured at the accept edge (the splitter owner_brg pattern).
*/

module ibus_arb (
    /* CLOCK AND RESET */
    input   wire            i_RST_n,
    input   wire            i_CLK,
    input   wire            i_CEN,

    /* INTERFACES */
    IBus_1.slave            CPU_BUS,    //from cpu_core (cache master)
    IBus_1.slave            DMA_BUS,    //from the DMAC engine (tied off until phase 3)
    IBus_1.master           CORE_BUS,   //to the splitter

    /* DMAC transfer-unit hold: keep ownership across the unit's accesses */
    input   wire            i_DMA_HOLD
);

///////////////////////////////////////////////////////////
//////  Ownership Tracking
////

logic           own_dma;            //registered owner: 0 = CPU (park), 1 = DMAC
logic           rsp_dma;            //owner of the outstanding response
logic           busy;               //transaction outstanding (accept -> response accept)
logic           cpu_lock_hold;      //CPU locked pair open (read accepted, write pending)
logic   [1:0]   burst_cnt;          //burst beats accepted (a fill/drain is 4 longwords)
logic           burst_open;

wire            req_acc  = CORE_BUS.req_valid && CORE_BUS.req_ready;
wire            rsp_done = CORE_BUS.rsp_valid && CORE_BUS.rsp_ready;

//idle boundary: nothing outstanding (or completing this very edge - every
//BSC/bridge leg's ready is registered, so a same-edge re-accept can't race
//the flip), no atomic sequence open on either side. Deciding at the
//completion edge is what gives the DMAC its priority over a CPU that
//re-requests every cycle (section 10 on-chip order: DMAC > CPU)
wire            at_idle  = (!busy || rsp_done) && !cpu_lock_hold && !burst_open &&
                           !(own_dma && i_DMA_HOLD);

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        own_dma       <= 1'b0;
        rsp_dma       <= 1'b0;
        busy          <= 1'b0;
        cpu_lock_hold <= 1'b0;
        burst_cnt     <= 2'd0;
        burst_open    <= 1'b0;
    end
    else begin if(i_CEN) begin
        if(req_acc)        busy <= 1'b1;
        else if(rsp_done)  busy <= 1'b0;

        if(req_acc) rsp_dma <= own_dma;

        //locked pair (CPU only: TAS/RMW): the read opens, the write closes at
        //its accept; a fault response on the pair unlatches (bsc precedent)
        if(req_acc && !own_dma && CORE_BUS.req_lock)
            cpu_lock_hold <= !CORE_BUS.req_write;
        else if(rsp_done && !rsp_dma && CORE_BUS.rsp_fault)
            cpu_lock_hold <= 1'b0;

        //4-beat fill/drain: count accepts, close on the 4th. A FAULT response
        //aborts the run - both masters abandon the remaining beats (cache
        //fill / DMAC unit p.343) - so the window must close or the other
        //master starves at the idle boundary (and the count stays skewed)
        if(rsp_done && CORE_BUS.rsp_fault) begin
            burst_cnt  <= (req_acc && CORE_BUS.req_burst) ? 2'd1 : 2'd0;
            burst_open <= (req_acc && CORE_BUS.req_burst);
        end
        else if(req_acc && CORE_BUS.req_burst) begin
            burst_cnt  <= burst_cnt + 2'd1;             //wraps 3 -> 0 on the last beat
            burst_open <= (burst_cnt != 2'd3);
        end

        //owner moves only at an idle boundary AND never on an accept edge
        //(downstream one-outstanding is enforced by the masters, not by the
        //per-class readies - a flip at the accept would let the other master
        //slip a second request into a different BSC class); DMAC outranks
        //the CPU at the boundary
        if(at_idle && !req_acc) own_dma <= DMA_BUS.req_valid;
    end end
end



///////////////////////////////////////////////////////////
//////  Request Path (one 2:1 mux level, registered select)
////

assign  CORE_BUS.req_valid = own_dma ? DMA_BUS.req_valid : CPU_BUS.req_valid;
assign  CORE_BUS.req_write = own_dma ? DMA_BUS.req_write : CPU_BUS.req_write;
assign  CORE_BUS.req_size  = own_dma ? DMA_BUS.req_size  : CPU_BUS.req_size;
assign  CORE_BUS.req_burst = own_dma ? DMA_BUS.req_burst : CPU_BUS.req_burst;
assign  CORE_BUS.req_addr  = own_dma ? DMA_BUS.req_addr  : CPU_BUS.req_addr;
assign  CORE_BUS.req_wdata = own_dma ? DMA_BUS.req_wdata : CPU_BUS.req_wdata;
assign  CORE_BUS.req_wstrb = own_dma ? DMA_BUS.req_wstrb : CPU_BUS.req_wstrb;
assign  CORE_BUS.req_lock  = own_dma ? DMA_BUS.req_lock  : CPU_BUS.req_lock;
//sideband is DMAC-only by construction (the cache ties it off)
assign  CORE_BUS.req_dack    = own_dma ? DMA_BUS.req_dack    : CPU_BUS.req_dack;
assign  CORE_BUS.req_dack_ch = own_dma ? DMA_BUS.req_dack_ch : CPU_BUS.req_dack_ch;
assign  CORE_BUS.req_saddr   = own_dma ? DMA_BUS.req_saddr   : CPU_BUS.req_saddr;

//CPU leg is a bare AND (zero-cost idle path; the cache self-serializes);
//the DMA leg additionally waits out its own outstanding response so a
//sloppy engine can never double-issue into the one-outstanding fabric
assign  CPU_BUS.req_ready = CORE_BUS.req_ready & ~own_dma;
assign  DMA_BUS.req_ready = CORE_BUS.req_ready &  own_dma & ~busy;



///////////////////////////////////////////////////////////
//////  Response Path (broadcast data, owner-gated valid)
////

assign  CPU_BUS.rsp_valid = CORE_BUS.rsp_valid & ~rsp_dma;
assign  DMA_BUS.rsp_valid = CORE_BUS.rsp_valid &  rsp_dma;
assign  CPU_BUS.rsp_rdata = CORE_BUS.rsp_rdata;
assign  DMA_BUS.rsp_rdata = CORE_BUS.rsp_rdata;
assign  CPU_BUS.rsp_fault = CORE_BUS.rsp_fault;
assign  DMA_BUS.rsp_fault = CORE_BUS.rsp_fault;

assign  CORE_BUS.rsp_ready = rsp_dma ? DMA_BUS.rsp_ready : CPU_BUS.rsp_ready;

endmodule

`default_nettype none
