`default_nettype wire

/*
    SH7709S integer CPU core wrapper.

    This module connects the instruction pipeline to the bare-metal exception
    handler. MMU/TLB exception entry is intentionally not implemented here.
*/

module cpu_core #(
    parameter [31:0] RESET_PC   = 32'hA000_0000,
    parameter        BIG_ENDIAN = 1'b1
) (
    /* CLOCK AND RESET */
    input   wire            i_POR_n,
    input   wire            i_RST_n,
    input   wire            i_CLK,      //single architectural clock
    input   wire            i_CEN,      //architectural clock enable

    /* I BUS 1 - the cache is the single master; BSC/bridge slave it outside (Fig 1.1, p.6) */
    IBus_1.master           I_BUS,

    /* ALREADY-PRIORITIZED EXTERNAL INTERRUPTS */
    // External INTC supplies source code and priority; see section 6, pp.117-148.
    // INTEVT2 lives in the INTC (I bus, Appendix B p.741); the acks drive its latch/clear.
    input   wire            i_NMI_VALID,
    input   wire            i_NMI_BLMSK,
    input   wire            i_INT_VALID,
    input   wire    [3:0]   i_INT_LEVEL,
    input   wire    [11:0]  i_INT_CODE,
    output  wire            o_INT_ACK,
    output  wire            o_NMI_ACK,

    /* DEBUG OBSERVATION */
    output  logic           dbg_o_RETIRE_VALID,
    output  logic   [31:0]  dbg_o_RETIRE_PC,
    output  logic   [15:0]  dbg_o_RETIRE_INST,
    output  logic           dbg_o_RETIRE_GPR_WE,
    output  logic   [4:0]   dbg_o_RETIRE_GPR,
    output  logic   [31:0]  dbg_o_RETIRE_GPR_DATA,
    output  logic   [31:0]  dbg_o_FETCH_PC,
    output  logic   [31:0]  dbg_o_SR,
    output  logic   [31:0]  dbg_o_GBR,
    output  logic   [31:0]  dbg_o_SSR,
    output  logic   [31:0]  dbg_o_SPC,
    output  logic   [31:0]  dbg_o_VBR,
    output  logic   [31:0]  dbg_o_MACH,
    output  logic   [31:0]  dbg_o_MACL,
    output  logic   [31:0]  dbg_o_PR,
    output  logic   [31:0]  dbg_o_TRA,
    output  logic   [31:0]  dbg_o_EXPEVT,
    output  logic   [31:0]  dbg_o_INTEVT,
    output  logic   [31:0]  dbg_o_TEA,
    output  logic           dbg_o_EXC_VALID,
    output  logic   [2:0]   dbg_o_EXC_CAUSE,
    output  logic   [31:0]  dbg_o_EXC_PC,
    output  logic           dbg_o_EXC_IN_DELAY_SLOT,
    output  logic           dbg_o_EXC_ACCESS_WRITE,
    output  logic   [31:0]  dbg_o_EXC_ACCESS_ADDR,
    output  logic           dbg_o_TRAPA_VALID,
    output  logic   [7:0]   dbg_o_TRAPA_IMM,
    output  logic           dbg_o_RTE_VALID,

    /* STATE-CONTROLLER EVENTS */
    output  logic           o_EXCEPTION_ENTRY_VALID,
    output  logic   [31:0]  o_EXCEPTION_ENTRY_PC,
    output  logic           o_SLEEP_VALID,
    output  logic           o_LDTLB_VALID
);

///////////////////////////////////////////////////////////
//////  Pipeline To Handler
////

/*
    The pipeline reports precise events and retired control-register writes.
    ctrl_reg owns architectural control-register storage and arbitration.
*/

logic           redirect_valid;
logic   [31:0]  redirect_pc;
logic           core_rst_n;
logic           reset_like_valid;
logic           exc_entry_valid;
logic   [31:0]  exc_entry_spc;
logic           rte_restore_valid;

logic           pipe_exc_valid;
logic   [2:0]   pipe_exc_cause;
logic   [31:0]  pipe_exc_pc;
logic           pipe_exc_delay_slot;
logic           pipe_exc_write;
logic   [31:0]  pipe_exc_addr;
logic           pipe_trapa_valid;
logic   [7:0]   pipe_trapa_imm;
logic           pipe_rte_valid;
logic           pipe_sleep_valid;
logic           pipe_ldtlb_valid;
logic           pipe_retire_valid;
logic   [31:0]  pipe_retire_pc;
logic           pipe_int_boundary;      //legal interrupt-acceptance boundary (pipe-owned invariant)
logic   [31:0]  pipe_int_next_pc;       //interrupt SPC: oldest instruction the redirect discards
logic   [15:0]  pipe_retire_inst;
logic           pipe_retire_gpr_we;
logic   [4:0]   pipe_retire_gpr;
logic   [31:0]  pipe_retire_gpr_data;
logic           pipe_ctrl_we;
logic   [2:0]   pipe_ctrl_dst;
logic   [31:0]  pipe_ctrl_data;
logic           pipe_sr_t_we;
logic           pipe_sr_t;
logic           pipe_sr_s_we;
logic           pipe_sr_s;
logic   [1:0]   pipe_sr_mq_we;
logic   [1:0]   pipe_sr_mq;
logic   [31:0]  pipe_fetch_pc;
logic   [31:0]  ctrl_sr;
logic   [31:0]  ctrl_gbr;
logic   [31:0]  ctrl_ssr;
logic   [31:0]  ctrl_spc;
logic   [31:0]  ctrl_vbr;
logic   [31:0]  mach;
logic   [31:0]  macl;
logic   [31:0]  pr;
logic   [31:0]  tra;
logic   [31:0]  expevt;
logic   [31:0]  intevt;
logic   [31:0]  tea;
logic           pipe_d_pref;        //PREF sideband: pipeline -> cache data port
logic           pipe_i_squash;      //wrong-path fetch sideband: pipeline -> cache (abort fill)

LBus            PIPE_L_BUS();       //unified IF+MA L bus: int_pipe (master) <-> u_cache (slave), direct

assign  core_rst_n = i_RST_n & i_POR_n;

assign  o_EXCEPTION_ENTRY_VALID = redirect_valid;
assign  o_EXCEPTION_ENTRY_PC    = redirect_pc;
assign  o_SLEEP_VALID           = pipe_sleep_valid;
assign  o_LDTLB_VALID           = pipe_ldtlb_valid;
assign  dbg_o_RETIRE_VALID      = pipe_retire_valid;
assign  dbg_o_RETIRE_PC         = pipe_retire_pc;
assign  dbg_o_RETIRE_INST       = pipe_retire_inst;
assign  dbg_o_RETIRE_GPR_WE     = pipe_retire_gpr_we;
assign  dbg_o_RETIRE_GPR        = pipe_retire_gpr;
assign  dbg_o_RETIRE_GPR_DATA   = pipe_retire_gpr_data;
assign  dbg_o_FETCH_PC          = pipe_fetch_pc;
assign  dbg_o_SR                = ctrl_sr;
assign  dbg_o_GBR               = ctrl_gbr;
assign  dbg_o_SSR               = ctrl_ssr;
assign  dbg_o_SPC               = ctrl_spc;
assign  dbg_o_VBR               = ctrl_vbr;
assign  dbg_o_MACH              = mach;
assign  dbg_o_MACL              = macl;
assign  dbg_o_PR                = pr;
assign  dbg_o_TRA               = tra;
assign  dbg_o_EXPEVT            = expevt;
assign  dbg_o_INTEVT            = intevt;
assign  dbg_o_TEA               = tea;
assign  dbg_o_EXC_VALID         = pipe_exc_valid;
assign  dbg_o_EXC_CAUSE         = pipe_exc_cause;
assign  dbg_o_EXC_PC            = pipe_exc_pc;
assign  dbg_o_EXC_IN_DELAY_SLOT = pipe_exc_delay_slot;
assign  dbg_o_EXC_ACCESS_WRITE  = pipe_exc_write;
assign  dbg_o_EXC_ACCESS_ADDR   = pipe_exc_addr;
assign  dbg_o_TRAPA_VALID       = pipe_trapa_valid;
assign  dbg_o_TRAPA_IMM         = pipe_trapa_imm;
assign  dbg_o_RTE_VALID         = pipe_rte_valid;



///////////////////////////////////////////////////////////
//////  Bus Complex
////

/*
    Bus tiers follow the SH7709S block diagram (Fig 1.1, hw manual p.6): the L bus
    is CPU-direct (fast); the I bus leaves this module through the cache's I_BUS
    master port (the splitter/BRIDGE/BSC slave it outside); the P bus is the
    peripheral tail behind the BSC.

    L-bus MMIO registers live INSIDE their owning module (Appendix B, pp.739-744):
    the cache owns CCR/CCR2 and the mm tag/data windows; exc_handler owns
    TRA/EXPEVT/INTEVT/TEA plus their decode (it snoops PIPE_L_BUS directly). The
    wires below carry the exc-group hit/read line into the cache's do_d
    output-flop mux and the fire-and-forget write pulse back.
*/

wire            lmmio_exc_hit_live; //exc_handler live decode -> cache accept-edge classify
wire            lmmio_exc_hit;      //exc_handler request-edge-latched hit -> cache resolve select
wire    [31:0]  lmmio_exc_rdata;    //exc_handler pre-selected read line -> cache do_d mux
wire            lmmio_exc_we;       //cache accepted an exc-register write this cycle -> exc_handler



///////////////////////////////////////////////////////////
//////  Instruction Pipeline
////

int_pipe #(
    .RESET_PC               (RESET_PC                               ),
    .BIG_ENDIAN             (BIG_ENDIAN                             )
) u_int_pipe (
    .i_RST_n                (core_rst_n                             ),
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (i_CEN                                  ),

    .i_REDIRECT_VALID       (redirect_valid                         ),
    .i_REDIRECT_PC          (redirect_pc                            ),

    .i_SR                   (ctrl_sr                                ),
    .i_GBR                  (ctrl_gbr                               ),
    .i_SSR                  (ctrl_ssr                               ),
    .i_SPC                  (ctrl_spc                               ),
    .i_VBR                  (ctrl_vbr                               ),

    .L_BUS                  (PIPE_L_BUS                             ),
    .o_D_PREF               (pipe_d_pref                            ),
    .o_I_SQUASH             (pipe_i_squash                          ),

    .o_EXC_VALID            (pipe_exc_valid                         ),
    .o_EXC_CAUSE            (pipe_exc_cause                         ),
    .o_EXC_PC               (pipe_exc_pc                            ),
    .o_EXC_IN_DELAY_SLOT    (pipe_exc_delay_slot                    ),
    .o_EXC_ACCESS_WRITE     (pipe_exc_write                         ),
    .o_EXC_ACCESS_ADDR      (pipe_exc_addr                          ),
    .o_TRAPA_VALID          (pipe_trapa_valid                       ),
    .o_TRAPA_IMM            (pipe_trapa_imm                         ),
    .o_RTE_VALID            (pipe_rte_valid                         ),
    .o_SLEEP_VALID          (pipe_sleep_valid                       ),
    .o_LDTLB_VALID          (pipe_ldtlb_valid                       ),

    .o_RETIRE_VALID         (pipe_retire_valid                      ),
    .o_RETIRE_PC            (pipe_retire_pc                         ),
    .o_RETIRE_INST          (pipe_retire_inst                       ),
    .o_INT_BOUNDARY         (pipe_int_boundary                      ),
    .o_INT_NEXT_PC          (pipe_int_next_pc                       ),
    .o_RETIRE_GPR_WE        (pipe_retire_gpr_we                     ),
    .o_RETIRE_GPR           (pipe_retire_gpr                        ),
    .o_RETIRE_GPR_DATA      (pipe_retire_gpr_data                   ),

    .o_CTRL_WE              (pipe_ctrl_we                           ),
    .o_CTRL_DST             (pipe_ctrl_dst                          ),
    .o_CTRL_DATA            (pipe_ctrl_data                         ),
    .o_SR_T_WE              (pipe_sr_t_we                           ),
    .o_SR_T                 (pipe_sr_t                              ),
    .o_SR_S_WE              (pipe_sr_s_we                           ),
    .o_SR_S                 (pipe_sr_s                              ),
    .o_SR_MQ_WE             (pipe_sr_mq_we                          ),
    .o_SR_MQ                (pipe_sr_mq                             ),

    .o_FETCH_PC             (pipe_fetch_pc                          ),
    .o_MACH                 (mach                                   ),
    .o_MACL                 (macl                                   ),
    .o_PR                   (pr                                     )
);


///////////////////////////////////////////////////////////
//////  Cache Controller
////

/*
    The cache owns CCR/CCR2 MMIO and the unified BRAM I/D cache.
*/

cache #(
    .BIG_ENDIAN             (BIG_ENDIAN                             )
) u_cache (
    .i_RST_n                (i_POR_n & i_RST_n                      ),
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (i_CEN                                  ),

    .PIPE_L_BUS             (PIPE_L_BUS                             ),
    .i_I_SQUASH             (pipe_i_squash                          ),
    .i_PIPE_D_PREF          (pipe_d_pref                            ),

    .I_BUS                  (I_BUS                                  ),

    //exc_handler group on the do_d output-flop mux: read line in, fire-and-forget write pulse out
    .i_LMMIO_EXC_HIT        (lmmio_exc_hit                          ),
    .i_LMMIO_EXC_HIT_LIVE   (lmmio_exc_hit_live                     ),
    .i_LMMIO_EXC_RDATA      (lmmio_exc_rdata                        ),
    .o_LMMIO_EXC_WE         (lmmio_exc_we                           )
);



///////////////////////////////////////////////////////////
//////  Control Registers
////

/*
    ctrl_reg is the only storage owner for SR/GBR/VBR/SSR/SPC. It arbitrates
    exception entry, RTE restore, and retired pipeline control-register writes.
*/

ctrl_reg u_ctrl_reg (
    .i_RST_n                (core_rst_n                             ),
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (i_CEN                                  ),

    .i_RESET_LIKE_VALID     (reset_like_valid                       ),
    .i_EXC_ENTRY_VALID      (exc_entry_valid                        ),
    .i_EXC_ENTRY_SPC        (exc_entry_spc                          ),
    .i_RTE_RESTORE_VALID    (rte_restore_valid                      ),

    .i_PIPE_CTRL_WE         (pipe_ctrl_we                           ),
    .i_PIPE_CTRL_DST        (pipe_ctrl_dst                          ),
    .i_PIPE_CTRL_DATA       (pipe_ctrl_data                         ),
    .i_PIPE_SR_T_WE         (pipe_sr_t_we                           ),
    .i_PIPE_SR_T            (pipe_sr_t                              ),
    .i_PIPE_SR_S_WE         (pipe_sr_s_we                           ),
    .i_PIPE_SR_S            (pipe_sr_s                              ),
    .i_PIPE_SR_MQ_WE        (pipe_sr_mq_we                          ),
    .i_PIPE_SR_MQ           (pipe_sr_mq                             ),

    .o_SR                   (ctrl_sr                                ),
    .o_GBR                  (ctrl_gbr                               ),
    .o_SSR                  (ctrl_ssr                               ),
    .o_SPC                  (ctrl_spc                               ),
    .o_VBR                  (ctrl_vbr                               )
);



///////////////////////////////////////////////////////////
//////  Exception Handler
////

/*
    The handler converts precise pipeline events into architectural exception
    entry. MMU/TLB refill is intentionally absent for bare-metal operation.
*/

exc_handler #(
    .RESET_PC               (RESET_PC                               )
) u_exc_handler (
    .i_POR_n                (i_POR_n                                ),
    .i_RST_n                (i_RST_n                                ),
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (i_CEN                                  ),

    .i_SR                   (ctrl_sr                                ),
    .i_SPC                  (ctrl_spc                               ),
    .i_VBR                  (ctrl_vbr                               ),
    .i_FETCH_PC             (pipe_fetch_pc                          ),

    .i_PIPE_EXC_VALID       (pipe_exc_valid                         ),
    .i_PIPE_EXC_CAUSE       (pipe_exc_cause                         ),
    .i_PIPE_EXC_PC          (pipe_exc_pc                            ),
    .i_PIPE_EXC_IN_DELAY_SLOT(pipe_exc_delay_slot                   ),
    .i_PIPE_EXC_ACCESS_WRITE(pipe_exc_write                         ),
    .i_PIPE_EXC_ACCESS_ADDR (pipe_exc_addr                          ),
    .i_PIPE_TRAPA_VALID     (pipe_trapa_valid                       ),
    .i_PIPE_TRAPA_IMM       (pipe_trapa_imm                         ),
    .i_PIPE_RTE_VALID       (pipe_rte_valid                         ),
    .i_PIPE_RETIRE_VALID    (pipe_retire_valid                      ),
    .i_PIPE_RETIRE_PC       (pipe_retire_pc                         ),
    .i_PIPE_INT_BOUNDARY    (pipe_int_boundary                      ),
    .i_PIPE_INT_NEXT_PC     (pipe_int_next_pc                       ),

    .i_NMI_VALID            (i_NMI_VALID                            ),
    .i_NMI_BLMSK            (i_NMI_BLMSK                            ),
    .i_INT_VALID            (i_INT_VALID                            ),
    .i_INT_LEVEL            (i_INT_LEVEL                            ),
    .i_INT_CODE             (i_INT_CODE                             ),
    .o_INT_ACK              (o_INT_ACK                              ),
    .o_NMI_ACK              (o_NMI_ACK                              ),

    .L_BUS                  (PIPE_L_BUS                             ),
    .i_LMMIO_WE             (lmmio_exc_we                           ),
    .o_LMMIO_HIT_LIVE       (lmmio_exc_hit_live                     ),
    .o_LMMIO_HIT            (lmmio_exc_hit                          ),
    .o_LMMIO_RDATA          (lmmio_exc_rdata                        ),

    .o_REDIRECT_VALID       (redirect_valid                         ),
    .o_REDIRECT_PC          (redirect_pc                            ),
    .o_RESET_LIKE_VALID     (reset_like_valid                       ),
    .o_EXC_ENTRY_VALID      (exc_entry_valid                        ),
    .o_EXC_ENTRY_SPC        (exc_entry_spc                          ),
    .o_RTE_RESTORE_VALID    (rte_restore_valid                      ),

    .o_TRA                  (tra                                    ),
    .o_EXPEVT               (expevt                                 ),
    .o_INTEVT               (intevt                                 ),
    .o_TEA                  (tea                                    )
);

endmodule

`default_nettype none
