`default_nettype wire

/*
    SH7709S exception-entry controller without MMU/TLB exception support.

    Memory faults are treated as bare-metal CPU address/bus faults. Translation
    exceptions and the TLB-miss vector offset are intentionally omitted.
*/

module exc_handler #(
    parameter [31:0] RESET_PC = 32'hA000_0000
) (
    /* CLOCK AND RESET */
    input   wire            i_POR_n,
    input   wire            i_RST_n,
    input   wire            i_CLK,
    input   wire            i_CEN,

    /* CURRENT PIPELINE STATE */
    input   wire    [31:0]  i_SR,
    input   wire    [31:0]  i_SPC,
    input   wire    [31:0]  i_VBR,
    input   wire    [31:0]  i_FETCH_PC,

    /* PRECISE PIPELINE EVENTS */
    input   wire            i_PIPE_EXC_VALID,
    input   wire    [2:0]   i_PIPE_EXC_CAUSE,
    input   wire    [31:0]  i_PIPE_EXC_PC,
    input   wire            i_PIPE_EXC_IN_DELAY_SLOT,
    input   wire            i_PIPE_EXC_ACCESS_WRITE,
    input   wire    [31:0]  i_PIPE_EXC_ACCESS_ADDR,
    input   wire            i_PIPE_TRAPA_VALID,
    input   wire    [7:0]   i_PIPE_TRAPA_IMM,
    input   wire            i_PIPE_RTE_VALID,
    input   wire            i_PIPE_RETIRE_VALID,
    input   wire    [31:0]  i_PIPE_RETIRE_PC,
    input   wire            i_PIPE_INT_BOUNDARY,     //legal acceptance boundary this edge: an
                                                     //instruction retired, no open delayed pair,
                                                     //no in-flight D access (pipe-owned, 4.5.3)
    input   wire    [31:0]  i_PIPE_INT_NEXT_PC,      //oldest instruction the redirect discards

    /* ALREADY-PRIORITIZED EXTERNAL INTERRUPTS - the INTC owns INTEVT2 (I bus, Appendix B
       p.741), so only the INTEVT code arrives; the acks below let it latch/clear. */
    input   wire            i_NMI_VALID,
    input   wire            i_NMI_BLMSK,
    input   wire            i_INT_VALID,
    input   wire    [3:0]   i_INT_LEVEL,
    input   wire    [11:0]  i_INT_CODE,
    output  wire            o_INT_ACK,      //interrupt accepted this cycle (INTC latches INTEVT2)
    output  wire            o_NMI_ACK,      //NMI accepted this cycle (INTC clears edge-pending)

    /* L-BUS SNOOP - live request address/wdata feed this module's own MMIO decode */
    LBus.monitor            L_BUS,

    /* MA-STAGE MMIO - this module OWNS the exc-register decode: the live hit classifies the
       cache's accept-edge dispatch, the pre-latched hit + read line feed the cache's do_d
       output-flop mux, and a software WRITE comes back fire-and-forget (no handshake). */
    input   wire            i_LMMIO_WE,         //commit a software write to an exc register this cycle
    output  wire            o_LMMIO_HIT_LIVE,   //live decode: this D request targets an exc register
    output  logic           o_LMMIO_HIT,        //request-edge pre-latched: resolve cycle reads an exc register
    output  logic   [31:0]  o_LMMIO_RDATA,      //one-hot AND-OR read line into the cache do_d mux

    /* PIPELINE REDIRECT AND STATE UPDATE */
    output  logic           o_REDIRECT_VALID,
    output  logic   [31:0]  o_REDIRECT_PC,
    output  logic           o_RESET_LIKE_VALID,
    output  logic           o_EXC_ENTRY_VALID,
    output  logic   [31:0]  o_EXC_ENTRY_SPC,
    output  logic           o_RTE_RESTORE_VALID,

    /* EXCEPTION REGISTER OBSERVATION */
    output  logic   [31:0]  o_TRA,
    output  logic   [31:0]  o_EXPEVT,
    output  logic   [31:0]  o_INTEVT,
    output  logic   [31:0]  o_TEA
);

///////////////////////////////////////////////////////////
//////  Constants
////

/*
    Event codes and vector offsets follow section 4, pp.85-101.
    This bare-metal handler maps all memory faults to CPU address errors.
*/

localparam logic [2:0] EXC_ILLEGAL    = 3'd1;
localparam logic [2:0] EXC_PRIVILEGE  = 3'd2;
localparam logic [2:0] EXC_IFETCH     = 3'd3;
localparam logic [2:0] EXC_DATA       = 3'd4;
localparam logic [2:0] EXC_ADDRESS    = 3'd5;

localparam logic [11:0] EV_POWER_RESET    = 12'h000;
localparam logic [11:0] EV_MANUAL_RESET   = 12'h020;
localparam logic [11:0] EV_ADDR_READ      = 12'h0E0;
localparam logic [11:0] EV_ADDR_WRITE     = 12'h100;
localparam logic [11:0] EV_TRAPA          = 12'h160;
localparam logic [11:0] EV_ILLEGAL        = 12'h180;
localparam logic [11:0] EV_ILLEGAL_SLOT   = 12'h1A0;
localparam logic [11:0] EV_NMI            = 12'h1C0;

localparam logic [31:0] VECTOR_GENERAL    = 32'h0000_0100;
localparam logic [31:0] VECTOR_INTERRUPT  = 32'h0000_0600;

localparam logic [31:0] ADDR_TRA          = 32'hFFFF_FFD0;
localparam logic [31:0] ADDR_EXPEVT       = 32'hFFFF_FFD4;
localparam logic [31:0] ADDR_INTEVT       = 32'hFFFF_FFD8;
localparam logic [31:0] ADDR_TEA          = 32'hFFFF_FFFC;

///////////////////////////////////////////////////////////
//////  L-Bus MMIO Decode (exc-register group)
////

//LIVE decode off the L-bus address: the cache's accept-edge dispatch classification and
//the exc write forward use the request cycle's own address. INTEVT2 is NOT here - it is
//an INTC register on the I bus (Appendix B p.741); its reads take the cache bypass path.
assign  o_LMMIO_HIT_LIVE = !L_BUS.req_fetch &&
                           (L_BUS.req_addr == ADDR_TRA    || L_BUS.req_addr == ADDR_EXPEVT ||
                            L_BUS.req_addr == ADDR_INTEVT || L_BUS.req_addr == ADDR_TEA);

//Resolve-cycle hit + read select, PRE-LATCHED at the request edge from the live compares
//above (registering the live twin == the old registered-ADDRESS decode, term for term).
//The old form re-ran the 32-bit compares off a latched mmio_addr_q DURING the resolve
//cycle - that decode headed every rsp_rdata -> ld_word -> dsp/idex wall (fit6 worst
//class). Now the resolve cone starts at 1-bit flops; mmio_addr_q itself is deleted.
logic   [3:0]   lmmio_sel_q;       //registered one-hot read select {TEA,INTEVT,EXPEVT,TRA}
wire            lmmio_rst_n = i_POR_n & i_RST_n;
always_ff @(posedge i_CLK or negedge lmmio_rst_n) begin
    if(!lmmio_rst_n) begin
        o_LMMIO_HIT <= 1'b0;
        lmmio_sel_q <= 4'd0;
    end
    else if(i_CEN) begin
        o_LMMIO_HIT <= o_LMMIO_HIT_LIVE;
        lmmio_sel_q <= {L_BUS.req_addr == ADDR_TEA,
                        L_BUS.req_addr == ADDR_INTEVT,
                        L_BUS.req_addr == ADDR_EXPEVT,
                        L_BUS.req_addr == ADDR_TRA};
    end
end

//One-hot AND-OR (literals are non-overlapping); garbage select on a non-hit cycle is
//never consumed (the cache gates the rdata leg on o_LMMIO_HIT), same as the old case.
always_comb begin
    o_LMMIO_RDATA = ({32{lmmio_sel_q[0]}} & o_TRA    ) |
                    ({32{lmmio_sel_q[1]}} & o_EXPEVT ) |
                    ({32{lmmio_sel_q[2]}} & o_INTEVT ) |
                    ({32{lmmio_sel_q[3]}} & o_TEA    );
end

///////////////////////////////////////////////////////////
//////  Event Selection
////

/*
    The handler chooses one architectural event per enabled cycle.
    Priority is general exception, RTE, NMI, then maskable interrupt.
    Interrupt acceptance uses SR.BL and SR.I3-I0; see section 6, pp.143-145.
*/

logic           sr_bl;
logic   [3:0]   sr_imask;
logic           nmi_accept;
logic           int_accept;
logic           interrupt_boundary;
logic           general_accept;
logic           general_reset_like;
logic           rte_accept;
logic   [11:0]  general_code;
logic   [31:0]  general_spc;
logic   [31:0]  interrupt_spc;

assign  sr_bl              = i_SR[28];
assign  sr_imask           = i_SR[7:4];
//The acceptance-boundary invariant (retire edge, no open pair, no in-flight D
//access - section 4.5.3, pp.98-100) is OWNED BY THE PIPE and arrives as one bit.
assign  interrupt_boundary = i_PIPE_INT_BOUNDARY;
assign  general_accept     = i_PIPE_EXC_VALID || i_PIPE_TRAPA_VALID;
assign  general_reset_like = general_accept && sr_bl;
assign  rte_accept         = i_PIPE_RTE_VALID;
//A same-edge synchronous event wins (the redirect chain below): the accepts - and
//with them the ACKs - must yield, or the INTC drops a request that never entered
//(TRAPA retires, so its edge is a genuinely open interrupt boundary). The loser
//stays pending at the INTC and is accepted after the handler. NMI beats INT.
assign  nmi_accept         = interrupt_boundary && i_NMI_VALID && (!sr_bl || i_NMI_BLMSK) &&
                             !general_accept;
assign  int_accept         = interrupt_boundary && i_INT_VALID && !sr_bl && (i_INT_LEVEL > sr_imask) &&
                             !general_accept && !nmi_accept;
//Accept strobes back to the INTC: it latches INTEVT2 from the code it presented this same
//cycle (its own registered output - coherent by construction) and clears NMI edge-pending.
assign  o_INT_ACK          = int_accept;
assign  o_NMI_ACK          = nmi_accept;

always_comb begin
    // EXPEVT code selection; no MMU/TLB codes are generated here.
    general_code = EV_ILLEGAL;
    unique case(i_PIPE_EXC_CAUSE)
        EXC_IFETCH:    general_code = EV_ADDR_READ;
        EXC_DATA:      general_code = i_PIPE_EXC_ACCESS_WRITE ? EV_ADDR_WRITE : EV_ADDR_READ;
        EXC_ADDRESS:   general_code = i_PIPE_EXC_ACCESS_WRITE ? EV_ADDR_WRITE : EV_ADDR_READ;
        EXC_PRIVILEGE: general_code = i_PIPE_EXC_IN_DELAY_SLOT ? EV_ILLEGAL_SLOT : EV_ILLEGAL;
        EXC_ILLEGAL:   general_code = i_PIPE_EXC_IN_DELAY_SLOT ? EV_ILLEGAL_SLOT : EV_ILLEGAL;
        default:       general_code = EV_ILLEGAL;
    endcase
    if(i_PIPE_TRAPA_VALID) general_code = EV_TRAPA;
end

always_comb begin
    // SPC stores the restart PC; delay-slot faults restart at the branch.
    if(i_PIPE_TRAPA_VALID) begin
        general_spc = i_PIPE_RETIRE_PC + 32'd2;
    end
    else if(i_PIPE_EXC_IN_DELAY_SLOT) begin
        general_spc = i_PIPE_EXC_PC - 32'd2;
    end
    else begin
        general_spc = i_PIPE_EXC_PC;
    end
end

always_comb begin
    //Interrupts complete the current instruction; SPC = the oldest instruction the
    //redirect discards (the pipe's mux). The old retire_pc+2 lost a taken branch
    //when the accept landed on the branch's / its slot's retire cycle.
    interrupt_spc = i_PIPE_INT_NEXT_PC;
end

/*
    Exception entry redirects fetch and requests state updates atomically.
    BL=1 on a general exception causes reset-like
    recovery as described in section 4.6, pp.100-101.
*/

always_comb begin
    o_REDIRECT_VALID = 1'b0;
    o_REDIRECT_PC    = 32'd0;
    o_RESET_LIKE_VALID  = 1'b0;
    o_EXC_ENTRY_VALID   = 1'b0;
    o_EXC_ENTRY_SPC     = 32'd0;
    o_RTE_RESTORE_VALID = 1'b0;

    if(general_reset_like) begin
        o_REDIRECT_VALID = 1'b1;
        o_REDIRECT_PC    = RESET_PC;
        o_RESET_LIKE_VALID = 1'b1;
    end
    else if(i_PIPE_EXC_VALID || i_PIPE_TRAPA_VALID) begin
        o_REDIRECT_VALID = 1'b1;
        o_REDIRECT_PC    = i_VBR + VECTOR_GENERAL;
        o_EXC_ENTRY_VALID = 1'b1;
        o_EXC_ENTRY_SPC   = general_spc;
    end
    else if(rte_accept) begin
        o_RTE_RESTORE_VALID = 1'b1;
    end
    else if(nmi_accept) begin
        o_REDIRECT_VALID = 1'b1;
        o_REDIRECT_PC    = i_VBR + VECTOR_INTERRUPT;
        o_EXC_ENTRY_VALID = 1'b1;
        o_EXC_ENTRY_SPC   = interrupt_spc;
    end
    else if(int_accept) begin
        o_REDIRECT_VALID = 1'b1;
        o_REDIRECT_PC    = i_VBR + VECTOR_INTERRUPT;
        o_EXC_ENTRY_VALID = 1'b1;
        o_EXC_ENTRY_SPC   = interrupt_spc;
    end
end

///////////////////////////////////////////////////////////
//////  Exception Registers
////

/*
    TRA, EXPEVT, INTEVT, and TEA are described in section 4.1.2, p.85 and
    section 4.3, pp.91-92. The MA data bus performs software access. INTEVT2
    lives in the INTC (I bus register, Appendix B p.741), fed by the acks above.
*/

//Reads are served by the cache through its shared do_d output flop (this module pre-selects the
//value onto o_LMMIO_RDATA off its own request-edge-latched one-hot). This module OWNS registers
//AND decode; a software write commits on the cache's i_LMMIO_WE pulse fire-and-forget, keyed on
//the live L-bus address (i_LMMIO_WE is an accept-edge signal, same cycle). A write during a
//hardware event is naturally superseded by the event write (higher priority in the commit chain
//below), and the event also flushes the software access in the pipe.

always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) begin
        o_TRA          <= 32'd0;
        o_EXPEVT       <= {20'd0, EV_POWER_RESET};
        o_INTEVT       <= 32'd0;
        o_TEA          <= 32'd0;
    end
    else begin if(i_CEN) begin
        if(!i_RST_n) begin
            //Manual reset is synchronous, not async (section 4.6 pp.100-101). EXPEVT bit[5]
            //is the only bit that differs from power reset (0x020 vs 0x000); an async preset
            //there cannot share one Cyclone V register with the POR async clear, so Quartus
            //emulates it with a register+latch - a combinational loop. The clock always runs
            //during a manual reset, so a synchronous load is equivalent and closes the loop.
            o_TRA          <= 32'd0;
            o_EXPEVT       <= {20'd0, EV_MANUAL_RESET};
            o_INTEVT       <= 32'd0;
            o_TEA          <= 32'd0;
        end
        else if(general_reset_like) begin
            //BL=1 reset-like recovery is a MANUAL reset (section 4.6 pp.100-101).
            o_TRA    <= 32'd0;
            o_EXPEVT <= {20'd0, EV_MANUAL_RESET};
            o_INTEVT <= 32'd0;
            o_TEA    <= 32'd0;
        end
        else if(i_PIPE_EXC_VALID || i_PIPE_TRAPA_VALID) begin
            o_EXPEVT <= {20'd0, general_code};
            if(i_PIPE_TRAPA_VALID) o_TRA <= {22'd0, i_PIPE_TRAPA_IMM, 2'b00};
            if(i_PIPE_EXC_VALID &&
               (i_PIPE_EXC_CAUSE == EXC_IFETCH ||
                i_PIPE_EXC_CAUSE == EXC_DATA ||
                i_PIPE_EXC_CAUSE == EXC_ADDRESS)) begin
                o_TEA <= i_PIPE_EXC_ACCESS_ADDR;
            end
        end
        else if(nmi_accept) begin
            o_INTEVT  <= {20'd0, EV_NMI};
        end
        else if(int_accept) begin
            o_INTEVT  <= {20'd0, i_INT_CODE};
        end
        else if(i_LMMIO_WE) begin
            //Fire-and-forget software write forwarded by the cache (lowest priority: a concurrent
            //hardware event above wins, and the event also flushes this access in the pipe).
            unique case(L_BUS.req_addr)
                ADDR_TRA:    o_TRA    <= {22'd0, L_BUS.req_wdata[9:2], 2'b00};
                ADDR_EXPEVT: o_EXPEVT <= {20'd0, L_BUS.req_wdata[11:0]};
                ADDR_INTEVT: o_INTEVT <= {20'd0, L_BUS.req_wdata[11:0]};
                ADDR_TEA:    o_TEA    <= L_BUS.req_wdata;
                default: begin end
            endcase
        end
    end end
end

endmodule

`default_nettype none
