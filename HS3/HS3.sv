`default_nettype wire

/*
    HS3 chip top - SH7709S subset (block diagram Fig 1.1, hw manual p.6).

    cpu_core (L-bus domain: pipeline + cache + ctrl_reg + exc_handler) masters
    I bus 1; the splitter routes the on-chip register windows through the
    BRIDGE onto I bus 2 (INTC, CPG/WDT) and everything else to the BSC. The
    INTC feeds the core's pre-prioritized interrupt contract and owns INTEVT2;
    the CPG/WDT paces the peripheral clock enable and can raise watchdog
    resets, which fold into the core/fabric resets below.

    The BSC serves its own registers + the SDRAM areas on real SDRAM pins
    (natural latency, 50 MHz bus enable) and exports every other area on the
    flat generic memory port for the SoC's external controller. The P-bus
    register tier (TMU, RTC, PFC/ports) hangs behind the BSC on IBus_2 legs
    (Appendix B placement); port pads are dedicated split i/o/oe vectors,
    and the table-18.1 pin shares implemented here are the INTC inputs
    (PINT = PTC/PTF, IRQ = PTH/SCPT7, IRLS = PTF3-0), the TMU's TCLK on
    PTH7, and the BSC's MCS0-7 outputs on port C in "other function" mode
    (MCS0 also claims the CS0 pad, p.323). NMI stays a dedicated pin.

    The CPG owns the clock pins: o_CKIO is the bus clock output that paces
    the board (SDRAM device clock, p.207); i_EXTAL2 is the RTC's 32.768 kHz
    crystal input - a genuinely asynchronous clock domain that only the RTC
    divider front end lives in.
*/

module HS3 #(
    parameter [31:0] RESET_PC    = 32'hA000_0000,
    parameter        BIG_ENDIAN  = 1'b1,
    parameter        DISABLE_CEN = 1'b1
) (
    /* CLOCK AND RESET */
    input   wire            i_POR_n,    //RESETP pin (power-on reset)
    input   wire            i_RST_n,    //RESETM pin (manual reset)
    input   wire            i_CLK,      //single architectural clock
    input   wire            i_CEN,      //architectural clock enable
    output  wire            o_CKIO,     //bus clock output (B-phi = core/2, p.207)
    output  wire            o_CKIO_PCEN,
    output  wire            o_CKIO_NCEN,
    input   wire            i_EXTAL2,   //RTC 32.768 kHz crystal pad (own clock domain)

    /* BSC PHYSICAL PINS - the real chip's shared external bus (table 10.1):
       ordinary memory / burst ROM and SDRAM ride the same pins, RD/WR is the
       SDRAM WE command bit, WE3-WE0 double as DQMUU-DQMLL. Data bus split
       (o_D_O/o_D_OE/i_D_I); the true inout lives at the board level. PCMCIA
       pins omitted; MCS0-7 ride the PTC pads (and MCS0 the CS0 pad) per the
       PFC grants, as on silicon. Pull-up states (PULA/PULD) and the release
       drive split (HIZCNT) are exported for the pad ring; IRQOUT asks a
       foreign master for the bus back (pp.320-322). */
    output  wire    [25:0]  o_A,
    output  wire    [31:0]  o_D_O,
    output  wire            o_D_OE,
    input   wire    [31:0]  i_D_I,
    output  wire            o_BS_n,
    output  wire            o_CS0_n,
    output  wire            o_CS2_n,
    output  wire            o_CS3_n,
    output  wire            o_CS4_n,
    output  wire            o_CS5_n,
    output  wire            o_CS6_n,
    output  wire            o_RD_WR,
    output  wire            o_RAS3L_n,
    output  wire            o_RAS3U_n,
    output  wire            o_CASL_n,
    output  wire            o_CASU_n,
    output  wire    [3:0]   o_WE_n,
    output  wire            o_RD_n,
    input   wire            i_WAIT_n,
    input   wire            i_MD4,      //area-0 bus width straps (table 10.4)
    input   wire            i_MD3,
    output  wire            o_CKE,
    input   wire            i_BREQ_n,
    output  wire            o_BACK_n,
    output  wire            o_BUS_OE,
    output  wire            o_RASCAS_OE,    //HIZCNT: RAS/CAS drive through a release
    output  wire            o_A_PU,         //PULA: A25-A0 pull-up state (fig 10.41)
    output  wire            o_D_PU,         //PULD: D31-D0 pull-up state (figs 10.42-43)
    output  wire            o_IRQOUT_n,     //bus retrieval request (p.321)

    /* GENERIC MEMORY PORT - mirrors EVERY external access. Generic-class
       accesses may be completed early by i_MEM_RSP_VALID (ORed with the
       i_WAIT_n-timed physical bus cycle); BSC-owned accesses (SDRAM 2/3,
       areas 1/7) are one-cycle accept strobes - observation only, never
       answered. i_MEM_READY is reserved (ignored). */
    output  wire            o_MEM_REQ,
    output  wire            o_MEM_WRITE,
    output  wire            o_MEM_BURST,
    output  wire    [1:0]   o_MEM_SIZE,
    output  wire    [28:0]  o_MEM_ADDR,
    output  wire    [6:0]   o_MEM_CS_n,
    output  wire    [3:0]   o_MEM_WSTRB,
    input   wire            i_MEM_READY,
    input   wire            i_MEM_RSP_VALID,
    input   wire            i_MEM_FAULT,
    output  wire            o_MEM_RSP_READY,

    /* INTERRUPT PINS - IRQ/IRL, IRLS and PINT arrive through the port pads
       below (table 18.1 pin shares); only NMI is dedicated */
    input   wire            i_NMI,

    /* I/O PORT PADS - split input / output / output-enable / pull-up state
       (sections 18/19). F, G, L are input-only; L has no pull-up MOS.
       Shares: PTC = PINT7-0, PTF = PINT15-8 / IRLS3-0, PTH4-0 = IRQ4-0
       (IRL3-0), SCPT7 = IRQ5, PTH7 = TCLK. */
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
    input   wire    [7:0]   i_PTF_I,
    output  wire    [7:0]   o_PTF_PU,
    input   wire    [7:0]   i_PTG_I,
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
    input   wire    [7:0]   i_PTL_I,
    input   wire    [7:0]   i_SCPT_I,
    output  wire    [7:0]   o_SCPT_O,
    output  wire    [7:0]   o_SCPT_OE,
    output  wire    [7:0]   o_SCPT_PU
);

///////////////////////////////////////////////////////////
//////  Reset Glue
////

/*
    WDT watchdog overflow raises an internal reset request per WTCSR.RSTS
    (p.216): POR-class requests fold into the core's power-on reset (EXPEVT
    0x000), manual-class into the manual reset (EXPEVT 0x020). The fabric and
    INTC clear on every flavor (table 6.2 note 1); cpg_wdt takes the raw pins
    so WTCNT/WTCSR survive the reset they caused (p.215).
*/

wire            wdt_rst_por_n;
wire            wdt_rst_man_n;
wire            iti_req;
wire            pcen;
wire            bcen; //bus clock enable (B-phi = CKIO rate) from the CPG

wire            rst_por_n = i_POR_n & wdt_rst_por_n;
wire            rst_man_n = i_RST_n & wdt_rst_man_n;
wire            rst_all_n = rst_por_n & rst_man_n;

//DISABLE_CEN=1: free-run every submodule at i_CLK, ignore the i_CEN pin
wire            cen = DISABLE_CEN ? 1'b1 : i_CEN;



///////////////////////////////////////////////////////////
//////  Bus Fabric
////

IBus_1          CORE_I_BUS();       //I bus 1: cache master -> arbiter
IBus_1          DMA_I_BUS();        //I bus 1: DMAC master -> arbiter (tied off until the engine lands)
IBus_1          ARB_I_BUS();        //arbiter -> splitter (the single downstream master)
IBus_1          BRG_I_BUS();        //splitter -> bridge (register windows)
IBus_1          BSC_I_BUS();        //splitter -> BSC (memory + BSC registers)
IBus_2          REG_CPG();          //bridge -> cpg_wdt   (0xFFFFFF80-8F)
IBus_2          REG_INTC_HI();      //bridge -> intc      (0xFFFFFEE0-EF)
IBus_2          REG_INTC_LO();      //bridge -> intc      (0xA4000000-1F)
IBus_2          PBUS_TMU();         //P bus: BSC -> tmu    (0xFFFFFE90-B8)
IBus_2          PBUS_RTC();         //P bus: BSC -> rtc    (0xFFFFFEC0-DE)
IBus_2          PBUS_PORT();        //P bus: BSC -> ioport (0x04000100-137)
IBus_2          PBUS_DMAC();        //P bus: BSC -> dmac   (0x04000020-77)

wire            dmac_hold;          //DMAC transfer-unit / burst bus hold

ibus_arb u_arb (
    .i_RST_n                (rst_all_n                              ),
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (cen                                    ),

    .CPU_BUS                (CORE_I_BUS                             ),
    .DMA_BUS                (DMA_I_BUS                              ),
    .CORE_BUS               (ARB_I_BUS                              ),

    .i_DMA_HOLD             (dmac_hold                              )
);

ibus_splitter u_split (
    .i_RST_n                (rst_all_n                              ),
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (cen                                    ),

    .CORE_BUS               (ARB_I_BUS                              ),
    .BRG_BUS                (BRG_I_BUS                              ),
    .EXT_BUS                (BSC_I_BUS                              )
);

ibus_bridge u_bridge (
    .i_RST_n                (rst_all_n                              ),
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (cen                                    ),

    .I_BUS                  (BRG_I_BUS                              ),
    .REG_CPG                (REG_CPG                                ),
    .REG_INTC_HI            (REG_INTC_HI                            ),
    .REG_INTC_LO            (REG_INTC_LO                            )
);



///////////////////////////////////////////////////////////
//////  BSC
////

wire            rcmi_req, rovi_req;
wire            ref_pend;           //refresh request pending (IRQOUT, p.321)
wire    [1:0]   dack_win;           //CSn-framed DACK windows (polarity in the DMAC)
wire    [7:0]   mcs_n;              //MCS0-7 selects, merged onto the PTC/CS0 pads
wire            mcs0_cs0;           //MCSCR0 decodes area 0: CS0 pad may switch
wire            cs0_bsc;            //BSC's own CS0 view (pre-MCS0 pad merge)

bsc #(
    .BIG_ENDIAN             (BIG_ENDIAN                             )
) u_bsc (
    .i_POR_n                (rst_por_n                              ),  //regs/engine/refresh survive manual reset (p.297)
    .i_RST_n                (rst_all_n                              ),  //front-end handshake only
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (cen                                    ),
    .i_BCEN                 (bcen                                   ),

    .I_BUS                  (BSC_I_BUS                              ),
    .REG_TMU                (PBUS_TMU                               ),
    .REG_RTC                (PBUS_RTC                               ),
    .REG_PORT               (PBUS_PORT                              ),
    .REG_DMAC               (PBUS_DMAC                              ),

    .o_MEM_REQ              (o_MEM_REQ                              ),
    .o_MEM_WRITE            (o_MEM_WRITE                            ),
    .o_MEM_BURST            (o_MEM_BURST                            ),
    .o_MEM_SIZE             (o_MEM_SIZE                             ),
    .o_MEM_ADDR             (o_MEM_ADDR                             ),
    .o_MEM_CS_n             (o_MEM_CS_n                             ),
    .o_MEM_WSTRB            (o_MEM_WSTRB                            ),
    .i_MEM_READY            (i_MEM_READY                            ),
    .i_MEM_RSP_VALID        (i_MEM_RSP_VALID                        ),
    .i_MEM_FAULT            (i_MEM_FAULT                            ),
    .o_MEM_RSP_READY        (o_MEM_RSP_READY                        ),

    .o_A                    (o_A                                    ),
    .o_D_O                  (o_D_O                                  ),
    .o_D_OE                 (o_D_OE                                 ),
    .i_D_I                  (i_D_I                                  ),
    .o_BS_n                 (o_BS_n                                 ),
    .o_CS0_n                (cs0_bsc                                ),
    .o_CS2_n                (o_CS2_n                                ),
    .o_CS3_n                (o_CS3_n                                ),
    .o_CS4_n                (o_CS4_n                                ),
    .o_CS5_n                (o_CS5_n                                ),
    .o_CS6_n                (o_CS6_n                                ),
    .o_RD_WR                (o_RD_WR                                ),
    .o_RAS3L_n              (o_RAS3L_n                              ),
    .o_RAS3U_n              (o_RAS3U_n                              ),
    .o_CASL_n               (o_CASL_n                               ),
    .o_CASU_n               (o_CASU_n                               ),
    .o_WE_n                 (o_WE_n                                 ),
    .o_RD_n                 (o_RD_n                                 ),
    .i_WAIT_n               (i_WAIT_n                               ),
    .i_MD4                  (i_MD4                                  ),
    .i_MD3                  (i_MD3                                  ),
    .o_CKE                  (o_CKE                                  ),
    .i_BREQ_n               (i_BREQ_n                               ),
    .o_BACK_n               (o_BACK_n                               ),
    .o_BUS_OE               (o_BUS_OE                               ),
    .o_RASCAS_OE            (o_RASCAS_OE                            ),
    .o_A_PU                 (o_A_PU                                 ),
    .o_D_PU                 (o_D_PU                                 ),

    .o_MCS_n                (mcs_n                                  ),
    .o_MCS0_CS0             (mcs0_cs0                               ),
    .o_DACK_WIN             (dack_win                               ),
    .o_REF_PEND             (ref_pend                               ),

    .o_RCMI_REQ             (rcmi_req                               ),
    .o_ROVI_REQ             (rovi_req                               )
);

//CS0 pad merge: with PTC0 granted to its function and MCSCR0 decoding
//area 0, the CS0 pad follows MCS[0] (p.323)
wire    [7:0]   pc_fn;              //PTC pins in "other function" mode (PFC)
assign  o_CS0_n = (pc_fn[0] && mcs0_cs0) ? mcs_n[0] : cs0_bsc;

//IRQOUT (pp.320-321): asserted on a pending-not-run refresh, or on an
//interrupt above the SR.I3-I0 mask (BL-independent; NMI always qualifies) -
//a foreign bus master negates BREQ so the chip can retrieve the bus
wire    [31:0]  core_sr;
assign  o_IRQOUT_n = ~(ref_pend | nmi_valid |
                       (int_valid && (int_level > core_sr[7:4])));



///////////////////////////////////////////////////////////
//////  CPU Core
////

wire            nmi_valid, nmi_blmsk, int_valid;
wire    [3:0]   int_level;
wire    [11:0]  int_code;
wire            int_ack, nmi_ack;

cpu_core #(
    .RESET_PC               (RESET_PC                               ),
    .BIG_ENDIAN             (BIG_ENDIAN                             )
) u_cpu (
    .i_POR_n                (rst_por_n                              ),
    .i_RST_n                (rst_man_n                              ),
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (cen                                    ),

    .I_BUS                  (CORE_I_BUS                             ),

    .i_NMI_VALID            (nmi_valid                              ),
    .i_NMI_BLMSK            (nmi_blmsk                              ),
    .i_INT_VALID            (int_valid                              ),
    .i_INT_LEVEL            (int_level                              ),
    .i_INT_CODE             (int_code                               ),
    .o_INT_ACK              (int_ack                                ),
    .o_NMI_ACK              (nmi_ack                                ),

    .dbg_o_RETIRE_VALID     (), .dbg_o_RETIRE_PC        (), .dbg_o_RETIRE_INST  (),
    .dbg_o_RETIRE_GPR_WE    (), .dbg_o_RETIRE_GPR       (), .dbg_o_RETIRE_GPR_DATA(),
    .dbg_o_FETCH_PC         (), .dbg_o_SR               (core_sr), .dbg_o_GBR   (),
    .dbg_o_SSR              (), .dbg_o_SPC              (), .dbg_o_VBR          (),
    .dbg_o_MACH             (), .dbg_o_MACL             (), .dbg_o_PR           (),
    .dbg_o_TRA              (), .dbg_o_EXPEVT           (), .dbg_o_INTEVT       (),
    .dbg_o_TEA              (), .dbg_o_EXC_VALID        (), .dbg_o_EXC_CAUSE    (),
    .dbg_o_EXC_PC           (), .dbg_o_EXC_IN_DELAY_SLOT(), .dbg_o_EXC_ACCESS_WRITE(),
    .dbg_o_EXC_ACCESS_ADDR  (), .dbg_o_TRAPA_VALID      (), .dbg_o_TRAPA_IMM    (),
    .dbg_o_RTE_VALID        (),

    .o_EXCEPTION_ENTRY_VALID(), .o_EXCEPTION_ENTRY_PC   (),
    .o_SLEEP_VALID          (), .o_LDTLB_VALID          ()
);



///////////////////////////////////////////////////////////
//////  CPG / WDT
////

cpg_wdt u_cpg_wdt (
    .i_POR_n                (i_POR_n                                ),  //raw pins: p.215 retention
    .i_RST_n                (i_RST_n                                ),
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (cen                                    ),

    .REG_BUS                (REG_CPG                                ),

    .o_PCEN                 (pcen                                   ),
    .o_BCEN                 (bcen                                   ),
    .o_CKIO                 (o_CKIO                                 ),
    .o_CKIO_PCEN            (o_CKIO_PCEN                            ),
    .o_CKIO_NCEN            (o_CKIO_NCEN                            ),
    .o_ITI_REQ              (iti_req                                ),
    .o_WDT_RST_POR_n        (wdt_rst_por_n                          ),
    .o_WDT_RST_MAN_n        (wdt_rst_man_n                          )
);



///////////////////////////////////////////////////////////
//////  TMU
////

wire    [3:0]   tmu_req;
wire            tclk_o, tclk_oe;
wire            ph7_fn;
wire            rtcclk, rtc_tick;   //RTC divider output: pad level + bus-domain tick

tmu u_tmu (
    .i_RST_n                (rst_all_n                              ),  //regs init on POR AND manual (p.391)
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (cen                                    ),
    .i_PCEN                 (pcen                                   ),

    .REG_BUS                (PBUS_TMU                               ),

    .i_TCLK                 (i_PTH_I[7]                             ),  //TCLK pad = PTH7 (table 18.1)
    .o_TCLK_O               (tclk_o                                 ),
    .o_TCLK_OE              (tclk_oe                                ),
    .i_RTC_TICK             (rtc_tick                               ),  //16.384 kHz tick (TPSC=100)
    .i_RTCCLK               (rtcclk                                 ),  //same, as the TCOE pad level

    .o_TMU_REQ              (tmu_req                                )
);



///////////////////////////////////////////////////////////
//////  RTC
////

wire    [2:0]   rtc_req;

rtc u_rtc (
    .i_POR_n                (rst_por_n                              ),  //alarm ENB + RTCEN/START (p.410)
    .i_RST_n                (rst_all_n                              ),  //RCR1 + PEF/PES clear on any reset
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (cen                                    ),

    .REG_BUS                (PBUS_RTC                               ),

    .i_EXTAL2               (i_EXTAL2                               ),  //32.768 kHz crystal domain
    .o_RTCCLK               (rtcclk                                 ),
    .o_RTC_TICK             (rtc_tick                               ),

    .o_RTC_REQ              (rtc_req                                )
);



///////////////////////////////////////////////////////////
//////  DMAC (+ CMT)
////

wire    [3:0]   dmac_dei;           //DEI0-3 transfer-end levels (IPRE, codes 0x800-0x860)
wire    [1:0]   dack_pad, drak_pad; //DACK/DRAK pad levels, merged onto Port D below
wire            dmac_nmi_set;       //INTC NMI edge -> DMAOR.NMIF (11.6 note 3)

dmac u_dmac (
    .i_RST_n                (rst_all_n                              ),  //CHCR/DMAOR/CMT clear on any reset (p.332)
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (cen                                    ),
    .i_PCEN                 (pcen                                   ),
    .i_CKIO_NCEN            (o_CKIO_NCEN                            ),  //DREQ sample = CKIO falling edge (p.363)

    .REG_BUS                (PBUS_DMAC                              ),
    .I_BUS                  (DMA_I_BUS                              ),

    .o_BUS_HOLD             (dmac_hold                              ),

    .i_NMI_SET              (dmac_nmi_set                           ),

    //DREQ pins ride the Port D pads as inputs (table 18.1; INTC tap idiom)
    .i_DREQ_n               ({i_PTD_I[6], i_PTD_I[4]}               ),
    .i_DACK_WIN             (dack_win                               ),
    .o_DACK                 (dack_pad                               ),
    .o_DRAK                 (drak_pad                               ),

    .o_DEI                  (dmac_dei                               )
);



///////////////////////////////////////////////////////////
//////  I/O Ports
////

//PTH7 pad merge: mode 00 hands the pad to the TMU's TCLK (output only when
//TOCR.TCOE selects the RTC-clock-out function, p.392)
wire    [7:0]   pth_o_port, pth_oe_port;
assign  o_PTH_O  = {ph7_fn ? tclk_o : pth_o_port[7], pth_o_port[6:0]};
assign  o_PTH_OE = {ph7_fn ? tclk_oe : pth_oe_port[7], pth_oe_port[6:0]};

//PTC pad merge: mode 00 hands each pad to its MCS[n] output (p.323); the
//port's own OE is 0 in that mode, so the OR only ever adds the MCS drive
wire    [7:0]   ptc_o_port, ptc_oe_port;
assign  o_PTC_O  = (pc_fn & mcs_n) | (~pc_fn & ptc_o_port);
assign  o_PTC_OE = pc_fn | ptc_oe_port;

//PTD pad merge (table 18.1): mode 00 output pads = PTD7 DACK1, PTD5 DACK0,
//PTD1 DRAK0, PTD0 DRAK1 (note the swap); PTD6/PTD4 = DREQ inputs (tapped
//raw at u_dmac, no drive - input-only pads)
wire    [7:0]   ptd_o_port, ptd_oe_port;
wire    [7:0]   pd_fn;
assign  o_PTD_O  = {pd_fn[7] ? dack_pad[1] : ptd_o_port[7],
                    ptd_o_port[6],
                    pd_fn[5] ? dack_pad[0] : ptd_o_port[5],
                    ptd_o_port[4:2],
                    pd_fn[1] ? drak_pad[0] : ptd_o_port[1],
                    pd_fn[0] ? drak_pad[1] : ptd_o_port[0]};
assign  o_PTD_OE = {pd_fn[7] | ptd_oe_port[7],
                    ptd_oe_port[6],
                    pd_fn[5] | ptd_oe_port[5],
                    ptd_oe_port[4:2],
                    pd_fn[1] | ptd_oe_port[1],
                    pd_fn[0] | ptd_oe_port[0]};

ioport u_ioport (
    .i_POR_n                (rst_por_n                              ),  //regs hold through manual reset (p.570)
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (cen                                    ),

    .REG_BUS                (PBUS_PORT                              ),

    .i_PTA_I                (i_PTA_I                                ),
    .o_PTA_O                (o_PTA_O                                ),
    .o_PTA_OE               (o_PTA_OE                               ),
    .o_PTA_PU               (o_PTA_PU                               ),
    .i_PTB_I                (i_PTB_I                                ),
    .o_PTB_O                (o_PTB_O                                ),
    .o_PTB_OE               (o_PTB_OE                               ),
    .o_PTB_PU               (o_PTB_PU                               ),
    .i_PTC_I                (i_PTC_I                                ),
    .o_PTC_O                (ptc_o_port                             ),
    .o_PTC_OE               (ptc_oe_port                            ),
    .o_PTC_PU               (o_PTC_PU                               ),
    .i_PTD_I                (i_PTD_I                                ),
    .o_PTD_O                (ptd_o_port                             ),
    .o_PTD_OE               (ptd_oe_port                            ),
    .o_PTD_PU               (o_PTD_PU                               ),
    .i_PTE_I                (i_PTE_I                                ),
    .o_PTE_O                (o_PTE_O                                ),
    .o_PTE_OE               (o_PTE_OE                               ),
    .o_PTE_PU               (o_PTE_PU                               ),
    .i_PTF_I                (i_PTF_I                                ),
    .o_PTF_PU               (o_PTF_PU                               ),
    .i_PTG_I                (i_PTG_I                                ),
    .o_PTG_PU               (o_PTG_PU                               ),
    .i_PTH_I                (i_PTH_I                                ),
    .o_PTH_O                (pth_o_port                             ),
    .o_PTH_OE               (pth_oe_port                            ),
    .o_PTH_PU               (o_PTH_PU                               ),
    .i_PTJ_I                (i_PTJ_I                                ),
    .o_PTJ_O                (o_PTJ_O                                ),
    .o_PTJ_OE               (o_PTJ_OE                               ),
    .o_PTJ_PU               (o_PTJ_PU                               ),
    .i_PTK_I                (i_PTK_I                                ),
    .o_PTK_O                (o_PTK_O                                ),
    .o_PTK_OE               (o_PTK_OE                               ),
    .o_PTK_PU               (o_PTK_PU                               ),
    .i_PTL_I                (i_PTL_I                                ),
    .i_SCPT_I               (i_SCPT_I                               ),
    .o_SCPT_O               (o_SCPT_O                               ),
    .o_SCPT_OE              (o_SCPT_OE                              ),
    .o_SCPT_PU              (o_SCPT_PU                              ),

    .o_PH7_FN               (ph7_fn                                 ),
    .o_PC_FN                (pc_fn                                  ),
    .o_PD_FN                (pd_fn                                  )
);



///////////////////////////////////////////////////////////
//////  INTC
////

intc u_intc (
    .i_RST_n                (rst_all_n                              ),
    .i_CLK                  (i_CLK                                  ),
    .i_CEN                  (cen                                    ),
    .i_PCEN                 (pcen                                   ),

    .REG_HI                 (REG_INTC_HI                            ),
    .REG_LO                 (REG_INTC_LO                            ),

    //interrupt pins ride the port pads (table 18.1): IRQ5 = SCPT7,
    //IRQ4-0 (= IRL3-0) = PTH4-0, IRLS3-0 = PTF3-0, PINT15-0 = PTF/PTC
    .i_NMI                  (i_NMI                                  ),
    .i_IRQ                  ({i_SCPT_I[7], i_PTH_I[4:0]}            ),
    .i_IRLS                 (i_PTF_I[3:0]                           ),
    .i_PINT                 ({i_PTF_I, i_PTC_I}                     ),

    //on-chip sources: WDT + BSC refresh + TMU + RTC + DMAC exist; the rest
    //arrive with their modules (SCI/SCIF/IrDA/ADC later)
    .i_ITI_REQ              (iti_req                                ),
    .i_TMU_REQ              (tmu_req                                ),
    .i_RTC_REQ              (rtc_req                                ),
    .i_SCI_REQ              (4'd0                                   ),
    .i_SCIF_REQ             (4'd0                                   ),
    .i_IRDA_REQ             (4'd0                                   ),
    .i_DMAC_REQ             (dmac_dei                               ),
    .i_REF_REQ              ({rovi_req, rcmi_req}                   ),
    .i_ADC_REQ              (1'b0                                   ),
    .i_UDI_REQ              (1'b0                                   ),

    .o_NMI_VALID            (nmi_valid                              ),
    .o_NMI_BLMSK            (nmi_blmsk                              ),
    .o_NMI_EDGE             (dmac_nmi_set                           ),
    .o_INT_VALID            (int_valid                              ),
    .o_INT_LEVEL            (int_level                              ),
    .o_INT_CODE             (int_code                               ),
    .i_INT_ACK              (int_ack                                ),
    .i_NMI_ACK              (nmi_ack                                )
);

endmodule

`default_nettype none
