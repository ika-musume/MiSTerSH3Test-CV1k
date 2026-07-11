`default_nettype wire

/*
    BSC - bus state controller (SH7709S section 10, pp.223-304).

    Slaves the splitter's external I-bus-1 leg and serves four worlds:

      1. its own register file (0xFFFFFF50-74, Appendix B: I bus) + the SDMR
         address windows (0xFFFFD000/0xFFFFE000, p.252);
      2. the ORIGINAL SDRAM interface - areas 2/3 when BCR1.DRAMTP selects
         synchronous DRAM (p.234). Command timing is register-programmed
         (MCR/WCR2) and runs on the 50 MHz bus enable i_BCEN, reproducing the
         real chip's natural latency (figs 10.14-10.28). Burst-read/single-
         write, BL=1: a cache line moves as 4 pipelined READ/WRIT commands
         (8 half-word beats on a 16-bit BCR2 bus width). Address multiplexing
         follows MCR.AMX per table 10.13; BS marks the Td data cycles.
      3. the ORDINARY MEMORY / BURST ROM controller for every other area
         (p.268, p.304): the access is held on the SHARED physical bus (and
         mirrored on the generic port) for a full register-programmed bus
         cycle - WCR2 first-access waits, the burst pitch of BCR1-enabled
         burst ROM areas, and i_WAIT_n stretching - then read data is
         sampled live from the D bus (i_D_I), like real chip pins. A
         handshake controller may instead complete any access EARLY by
         pulsing i_MEM_RSP_VALID (ORed in, data from i_MEM_RDATA), so tying
         the handshake low (unused) works with only i_WAIT_n; the fast path
         also keeps the IPC parity constants intact.
      4. the P-BUS register tier (Appendix B "P" bus: TMU 0xFFFFFE90-B8,
         RTC 0xFFFFFEC0-DE, PFC/ports 0x04000100-137) - an in-BSC bridge
         with the ibus_bridge contract (IDLE->ACCESS->RESP, right-justified
         writes, lane-replicated reads) driving the REG_TMU/REG_RTC/REG_PORT
         IBus_2 legs. Handshake latency is allowed on this tier only (user
         latency law); P-window accesses never touch the external pins.

    Front-end/register file tick on i_CEN (100 MHz core rate); only the
    PCB-facing SDRAM engine + refresh timer tick on i_BCEN. Same clock, so
    the flag exchange is plain registers - no synchronizers (user decision).

    Reset domains (p.228 note, p.297): ALL BSC registers and the SDRAM
    engine/refresh reset on power-on reset only - refresh keeps running
    through a manual reset. Only the front-end handshake clears on every
    reset flavor so a dying transaction cannot wedge the bus.

    DEVIATIONS (vs real silicon):
      - no PCMCIA (A5PCM/A6PCM/PCR are bookkeeping); no standby coupling -
        the SoC has no standby mode, so HIZMEM (a standby-only pad rule) is
        stored but never observable. BREQ/BACK ARE implemented: grant drains
        the bus, PALLs open bank-active rows, releases the shared pads via
        o_BUS_OE / o_RASCAS_OE (HIZCNT); PULA/PULD export pull-up states and
        o_REF_PEND feeds the chip-level IRQOUT (pp.320-322).
      - WCR1 idles gate only PIN bus cycles (launch/dispatch): an access the
        generic-port handshake completes early never drove the D pins, so it
        pays no idle - this keeps the IPC-parity fast path beat-exact.
      - WAITSEL=0 with WAIT asserted is "operation not guaranteed" (p.241);
        HS3's defined behavior for it is the boundary-edge sample. WAITSEL=1
        is the compliant mid-state (CKIO-fall) sample point.
      - strobe shapes follow fig 23.16: RD/WEn mid-T1 -> mid-T2, CSn T1 ->
        mid-T2, per width-split sub-cycle, read data sampled at the mid-T2
        fall (tRDH1 = 0 ns). A 4-beat line burst (cache fill/drain, DMAC
        16-byte unit) is paced by the CONTROLLER, not by the per-beat bus
        calls: beats chain back-to-back with no idle states (fig 11.11),
        burst-ROM reads hold CSn/RD-WR/DACK low across the whole run with
        A3-A0 stepping at the beat-launch posedges (p.304, figs 10.29/10.30,
        23.19/23.20), and burst WRITE runs ignore the WAIT pin (p.304:
        16-byte DMA writes, single-address dev->mem, cache write-back).
        Write beats always run as basic cycles (figs 10.29/10.30 notes);
        their calls post like SDRAM drain beats - only the 4th call waits
        for the envelope so a unit fault stays visible to the master.
      - a generic-port hsk EARLY completion releases the pins at its core
        edge (off the bus grid) - extension-path behavior; the timed/raw
        path always releases at the T2-close boundary edge.
      - i_MEM_READY and i_MEM_RSP_VALID are equivalent external completions
        (both ORed with the timed path); accept pacing is always internal.
        The generic port carries no data - the physical D pins do.
      - area 1 (internal I/O) and area 7 (reserved) answer locally (read 0,
        posted write) instead of emitting external cycles.
      - the generic port mirrors ALL external areas: BSC-owned accesses
        (SDRAM areas, dummy 1/7) appear as one-cycle accept strobes for
        shadowing; the external controller masks them by its own memory map.

    The CKIO pin is DATASHEET phase (rises at the command edges - cpg_wdt);
    a board clocking a synchronous device straight from it must grant the
    device the real chip's tOD margin (clock-tree skew / PLL phase or an
    output-delay constraint; the TB models it as a transport delay on the
    device clock net). The register file matches section 10.2 exactly:
    POR values, reserved-bit masks, ENDIAN(BCR1[11]) read-only = !MD5,
    fig-10.5 write keys, RFCR clear at the LMTS limit, CMF clear bound to
    the next performed CBR refresh.
*/

module bsc #(
    parameter       BIG_ENDIAN = 1'b1   //BCR1.ENDIAN reads !BIG_ENDIAN (= MD5 pin, p.236)
) (
    /* CLOCK AND RESET */
    input   wire            i_POR_n,    //registers + SDRAM engine + refresh (survives manual reset)
    input   wire            i_RST_n,    //front-end handshake only (all-flavor reset)
    input   wire            i_CLK,
    input   wire            i_CEN,      //core enable - front-end / register file / generic leg
    (* direct_enable *)
    input   wire            i_BCEN,     //bus enable (50 MHz) - SDRAM engine + refresh timer

    /* INTERFACES */
    IBus_1.slave            I_BUS,      //from the splitter's external leg
    IBus_2.master           REG_TMU,    //P bus: TMU window 0xFFFFFE90-B8
    IBus_2.master           REG_RTC,    //P bus: RTC window 0xFFFFFEC0-DE
    IBus_2.master           REG_PORT,   //P bus: PFC/port window 0x04000100-137
    IBus_2.master           REG_DMAC,   //P bus: DMAC+CMT window 0x04000020-77

    /* BSC PHYSICAL PINS - the real chip's external bus, table 10.1
       (pp.226-227). Ordinary memory / burst ROM and SDRAM SHARE these,
       exactly as on silicon: RD/WR doubles as the SDRAM WE command bit and
       WE3-WE0 double as DQMUU-DQMLL. The data bus is split unidirectional
       (o_D_O/o_D_OE/i_D_I); the true inout lives at the board level.
       PCMCIA pins (CE2A/B, ICIORD/ICIOWR, IOIS16) are omitted. */
    output  wire    [25:0]  o_A,            //shared: static ordinary / muxed SDRAM row-col
    output  wire    [31:0]  o_D_O,
    output  wire            o_D_OE,
    input   wire    [31:0]  i_D_I,
    output  wire            o_BS_n,         //bus cycle start
    output  wire            o_CS0_n,
    output  wire            o_CS2_n,
    output  wire            o_CS3_n,
    output  wire            o_CS4_n,
    output  wire            o_CS5_n,
    output  wire            o_CS6_n,
    output  wire            o_RD_WR,        //bus direction / SDRAM WE command bit
    output  wire            o_RAS3L_n,      //lower 32MB row strobe
    output  wire            o_RAS3U_n,      //upper 32MB row strobe
    output  wire            o_CASL_n,
    output  wire            o_CASU_n,
    output  wire    [3:0]   o_WE_n,         //WE3-WE0 write strobes / DQMUU-DQMLL
    output  wire            o_RD_n,         //ordinary read strobe
    input   wire            i_WAIT_n,       //sampled after the WCR2 waits when enabled
    input   wire            i_MD4,          //area-0 bus width straps (table 10.4:
    input   wire            i_MD3,          //10=16-bit, 11=32-bit; 8-bit unsupported)
    output  wire            o_CKE,
    input   wire            i_BREQ_n,       //bus release request (2FF-synced)
    output  wire            o_BACK_n,       //grant: shared bus released
    output  wire            o_BUS_OE,       //board-level pad enable for the shared
                                            //control/address group (D uses o_D_OE;
                                            //CKE stays driven through a release)
    output  wire            o_RASCAS_OE,    //RAS/CAS pad enable: HIZCNT keeps them
                                            //driven through a release (p.236)
    output  wire            o_A_PU,         //PULA: A25-A0 pull-up state (fig 10.41)
    output  wire            o_D_PU,         //PULD: D31-D0 pull-up state (figs 10.42-43)

    /* MCS0-7 MASK-ROM SELECTS (MCSCR0-7, table 10.15) - on silicon these
       ride the PTC pads (and MCS0 the CS0 pad) when the PFC grants them;
       the chip top does that merge */
    output  wire    [7:0]   o_MCS_n,
    output  wire            o_MCS0_CS0,     //MCSCR0.CS2/0 = 0: CS0 pad may switch

    /* IRQOUT contribution (p.321): refresh request pending, cycle not run */
    output  wire            o_REF_PEND,

    /* GENERIC MEMORY PORT - address/control view only: ALL data rides the
       physical D pins (user rule). Every external access is mirrored here;
       ordinary accesses are held for their timed bus cycle. EITHER
       i_MEM_READY or i_MEM_RSP_VALID completes an ordinary access early -
       always as one full 32-bit D-bus transfer, bypassing the width split -
       so tying both low leaves i_WAIT_n in sole control. */
    output  wire            o_MEM_REQ,
    output  wire            o_MEM_WRITE,
    output  wire            o_MEM_BURST,    //beat of a 16-byte line transfer
    output  wire    [1:0]   o_MEM_SIZE,
    output  wire    [28:0]  o_MEM_ADDR,     //physical (A31-29 shadow stripped, p.232)
    output  wire    [6:0]   o_MEM_CS_n,     //area strobes; bit n = area n (bit 1 never asserts)
    output  wire    [3:0]   o_MEM_WSTRB,    //32-bit-lane byte enables (control view)
    input   wire            i_MEM_READY,
    input   wire            i_MEM_RSP_VALID,
    input   wire            i_MEM_FAULT,
    output  wire            o_MEM_RSP_READY,

    /* DMAC SIDEBAND CONSUMER (section 11.3.4-11.3.5): active-high DACK
       window strobes framed on the tagged ordinary cycle's CSn assertion
       ("DACK is output for the same duration as CSn", p.363); the DMAC
       applies the AL pad polarity. Single-address cycles additionally
       tri-state the write data path (fig 11.10a). SDRAM-area DACK/single
       cycles are NOT implemented (ordinary/burst-ROM areas only). */
    output  wire    [1:0]   o_DACK_WIN,     //[0]=DACK0 window, [1]=DACK1 window

    /* REFRESH TIMER INTERRUPTS (table 6.4 REF entries) */
    output  wire            o_RCMI_REQ,     //compare match  (INTEVT 0x580)
    output  wire            o_ROVI_REQ      //count overflow (INTEVT 0x5A0)
);



///////////////////////////////////////////////////////////
//////  Register File (power-on reset only, p.228)
////

logic   [15:0]  bcr1;               //memory type select; init 0x0000 (p.233)
logic   [15:0]  bcr2;               //area bus width, bookkeeping; init 0x3FF0 (p.239)
logic   [15:0]  wcr1;               //WAITSEL + inter-access idles (enforced); init 0x3FF3 (p.240)
logic   [15:0]  wcr2;               //waits + SDRAM CAS latency; init 0xFFFF (p.241)
logic   [15:0]  mcr;                //SDRAM timing; init 0x0000 (p.245)
logic   [15:0]  pcr;                //PCMCIA, bookkeeping only; init 0x0000 (p.248)
logic   [7:0]   rtcsr;              //{CMF,CMIE,CKS[2:0],OVF,OVIE,LMTS} (p.253)
logic   [7:0]   rtcnt;              //refresh timer counter (p.255)
logic   [7:0]   rtcor;              //refresh time constant (p.256)
logic   [9:0]   rfcr;               //refresh count (p.256)
logic   [15:0]  mcscr [0:7];        //MCS0-7 pin control, bookkeeping (pp.258-259)

//decoded engine timing knobs (the natural-latency law, MCR pp.245-247)
wire            a2_sdram  = (bcr1[4:2] == 3'b011);      //DRAMTP=011: both areas SDRAM
wire            a3_sdram  = !bcr1[4] && bcr1[3];        //DRAMTP=010 or 011
//per-area SDRAM data bus width from BCR2 (p.276: 16 or 32 bits; both areas
//must match when both are SDRAM, so global commands may use either decode)
wire            sd16_a2   = (bcr2[5:4] == 2'b10);
wire            sd16_a3   = (bcr2[7:6] == 2'b10);
wire            sd16_gl   = a2_sdram ? sd16_a2 : sd16_a3;
wire    [2:0]   t_tpc     = {1'b0, mcr[15:14]} + 3'd1;              //precharge spacing 1-4
wire    [3:0]   t_tpc_slf = {2'b00, mcr[15:14]} * 3 + 4'd2;         //self-refresh exit 2/5/8/11
wire    [2:0]   t_rcd     = {1'b0, mcr[13:12]} + 3'd1;              //RAS-CAS spacing 1-4
wire    [2:0]   t_trwl    = {1'b0, mcr[11:10]} + 3'd1;              //write recovery 1-3
wire    [2:0]   t_tras    = {1'b0, mcr[9:8]}   + 3'd2;              //refresh lockout 2-5
wire            rasd      = mcr[7];                                 //bank active mode
wire            rfsh      = mcr[2];
wire            rmode     = mcr[1];
//CAS latency per area (WCR2 A3W/A2W, p.243: 00/01=1, 10=2, 11=3)
wire    [1:0]   cl_a3     = (wcr2[6:5] == 2'b00) ? 2'd1 : wcr2[6:5];
wire    [1:0]   cl_a2     = (wcr2[4:3] == 2'b00) ? 2'd1 : wcr2[4:3];

///////////////////////////////////////////////////////////
//////  Front-End Decode (core rate, zero added beats)
////

/*
    Route classes on the live request address:
      REG    P4 0xFFFFFF50-7F register window
      SDMR   P4 0xFFFFD000-DFFF (area 2) / 0xFFFFE000-EFFF (area 3)
      SDRAM  areas 2/3 when DRAMTP selects synchronous DRAM
      PBUS   P4 0xFFFFFE90-BF (TMU) / 0xFFFFFEC0-DF (RTC) /
             area 1 0x04000100-13F (PFC/ports)
      LOCAL  area 1 / area 7 / any other unclaimed P4 address (read 0, posted)
      GEN    everything else -> the ordinary/burst-ROM bus controller
*/

wire    [31:0]  fa       = I_BUS.req_addr;
wire            fe_p4    = (fa[31:29] == 3'b111);
wire            fe_reg   = (fa[31:8] == 24'hFFFF_FF) &&
                           (fa[7:4] == 4'h5 || fa[7:4] == 4'h6 || fa[7:4] == 4'h7);
wire            fe_sdmr  = (fa[31:12] == 20'hFFFFD) || (fa[31:12] == 20'hFFFFE);
wire    [2:0]   fe_area  = fa[28:26];
wire            fe_sdram = !fe_p4 && ((fe_area == 3'd2 && a2_sdram) ||
                                      (fe_area == 3'd3 && a3_sdram));
//P-bus register windows (Appendix B): TMU/RTC on P4, PFC/ports in area-1 space
wire            fe_tmu   = (fa[31:8] == 24'hFFFF_FE) &&
                           (fa[7:4] == 4'h9 || fa[7:4] == 4'hA || fa[7:4] == 4'hB);
wire            fe_rtc   = (fa[31:8] == 24'hFFFF_FE) &&
                           (fa[7:4] == 4'hC || fa[7:4] == 4'hD);
wire            fe_port  = !fe_p4 && (fe_area == 3'd1) && (fa[25:6] == 20'h0_0004);
//DMAC window (tables 11.2 + 11.7): quads+DMAOR 0x20-61, CMT 0x70-77; the
//0x62-6F hole stays undecoded (section 11.6 note 11) and 0x00-1F is the
//INTC-low page (bridge-owned P2 alias) - both fall to fe_dummy here
wire            fe_dma_pg = !fe_p4 && (fe_area == 3'd1) && (fa[25:7] == 19'h0_0000);
wire            fe_dmac  = fe_dma_pg && ((fa[6:5] == 2'b01) || (fa[6:5] == 2'b10) ||  //0x20-5F
                                         (fa[6:1] == 6'b11_0000) ||                   //0x60 DMAOR
                                         (fa[6:3] == 4'b1110));                       //0x70-77 CMT
wire            fe_pbus  = fe_tmu || fe_rtc || fe_port || fe_dmac;
wire            fe_dummy = (fe_p4 && !fe_reg && !fe_sdmr && !fe_tmu && !fe_rtc) ||
                           (!fe_p4 && ((fe_area == 3'd1 && !fe_port && !fe_dmac) ||
                                       fe_area == 3'd7));
wire            fe_local = fe_reg || fe_dummy;
wire            fe_gen   = !fe_p4 && !fe_sdram && !fe_dummy && !fe_port && !fe_dmac;
wire            fe_eng   = fe_sdram || fe_sdmr;         //engine-owned classes

wire            bus_held;           //BREQ requested or granted (defined below)

//owner of the outstanding response (splitter owner_brg pattern)
localparam [1:0] OWN_GEN = 2'd0, OWN_LOC = 2'd1, OWN_SDR = 2'd2, OWN_PBS = 2'd3;
logic   [1:0]   owner_q;

//SDRAM burst bookkeeping: a fill/drain arrives as 4 line-aligned beats; the
//head starts the engine, continuations ride the running op (line buffers)
logic           b_rd_act;           //burst read in flight (engine fetches the line)
logic   [1:0]   b_rd_cnt;           //fill beats consumed (fills WRAP: heads are not word 0)
logic           b_wr_act;           //burst write in flight (line buffer drains to pins)
//head = first beat of a line transfer (no matching burst open). Fill bursts
//wrap from the MISSED word (p.110); drains stay word-ordered 0..3.
wire            fe_b_head = I_BUS.req_write ? !b_wr_act : !b_rd_act;
wire            fe_b_cont = fe_sdram && ((b_rd_act && !I_BUS.req_write) ||
                                         (b_wr_act &&  I_BUS.req_write));

//engine request slot (single outstanding; consumed by the engine at a BCEN edge)
logic           eng_go;
wire            eng_busy;           //engine FSM outside E_IDLE
logic           self_active;        //self-refresh entered; requests stall until exit

//local (register/dummy) one-beat registered response
logic           loc_rsp_v;
logic   [31:0]  loc_rsp_d;

//SDRAM response trackers
logic           sd_rd_wait;         //a read beat awaits its line-buffer word
logic   [1:0]   sd_rd_beat;
logic           sd_wr_ack;          //posted-write/SDMR ack pending
logic   [3:0]   wv;                 //read line-buffer word valid (engine sets, head accept clears)
logic   [31:0]  rd_buf  [0:3];      //engine -> front-end read line buffer
logic   [31:0]  wr_buf  [0:3];      //front-end -> engine write line buffer
logic   [3:0]   wr_strb [0:3];
logic   [3:0]   wr_v;

//TAS atomicity (p.320): the bus is never released between a locked pair's
//read and write. fe_lock_hold tracks a pair on the ordinary/generic path
//(e_lock_hold is its SDRAM-engine twin). While a pair is open a pending
//BREQ must not stall accepts either (a fetch can sit ahead of the pair's
//write on the single-outstanding bus - blocking it deadlocks the release):
//the bus runs normally until the legal release point after the write.
logic           fe_lock_hold;
wire            bus_blk      = bus_held && !(fe_lock_hold || e_lock_hold);

//ready per class. A head/single engine op needs the whole engine idle; burst
//continuations only need the previous beat's response consumed.
wire            sd_ready  = fe_b_cont ? (!sd_rd_wait && !sd_wr_ack) :
                            (!eng_go && !eng_busy && !self_active && !bus_blk &&
                             !sd_rd_wait && !sd_wr_ack);
wire            loc_ready = !loc_rsp_v;
wire            pbs_ready;          //P-bus bridge idle (defined in its section)

//the SHARED external bus admits one cycle at a time: ordinary accepts wait
//for the engine (a posted SDRAM write/MRS/refresh still owns the pins) and
//the engine waits for an open ordinary cycle (dispatch gate in E_IDLE).
//Burst continuation calls ride the open envelope: they only need the
//previous call's response consumed (the sd_ready continuation twin)
assign  I_BUS.req_ready = fe_gen  ? (ord_bcont ? !(ob_wait || ord_wr_ack) :
                                     !(ord_busy || ordb_act || ordw_act ||
                                       eng_busy || eng_go || bus_blk)) :
                          fe_eng  ? sd_ready  :
                          fe_pbus ? pbs_ready : loc_ready;

wire            fe_acc     = I_BUS.req_valid && I_BUS.req_ready;
wire            fe_acc_eng = fe_acc && fe_eng;

//front-end locked pair: a locked read opens the hold, its write closes it
//at the accept edge (ord_busy then keeps the bus through the write cycle);
//a pair killed by a bus fault on the read unlatches at that response
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) fe_lock_hold <= 1'b0;
    else begin if(i_CEN) begin
        if(fe_acc && fe_gen && I_BUS.req_lock)
            fe_lock_hold <= !I_BUS.req_write;
        else if(fe_rsp_done && owner_q == OWN_GEN && I_BUS.rsp_fault)
            fe_lock_hold <= 1'b0;
    end end
end



///////////////////////////////////////////////////////////
//////  Generic Leg Pass-Through (the IPC-parity path)
////

/*
    ALL external areas are exposed on the port (user rule: the external
    controller masks by address + its own map). BSC-owned areas (SDRAM 2/3,
    dummy 1/7) appear as a ONE-CYCLE strobe at their internal accept edge -
    one pulse = one committed access, observation only. Ordinary/burst-ROM
    accesses are HELD on the port (latched fields, level o_MEM_REQ) for the
    whole register-timed bus cycle so a raw memory can decode them like a
    real external bus; a handshake controller instead pulses i_MEM_RSP_VALID
    to complete early (the IPC-parity fast path).
*/

//ordinary bus cycle in flight: the port carries the latched access
logic           ord_busy;
logic           ord_done;           //timed-path completion (registered, one rsp)
logic   [31:0]  ord_addr, ord_wdata, ord_data;
logic           ord_write, ord_burst;
logic   [1:0]   ord_size;
logic   [3:0]   ord_wstrb;
logic   [2:0]   ord_area;
logic           ord_dack;           //DMAC sideband: frame DACK on this cycle's CSn (p.363)
logic           ord_dack_ch;        //  which pin: 0 = DACK0, 1 = DACK1
logic           ord_saddr;          //  single-address cycle: a write leaves D undriven (fig 11.10a)

assign  o_MEM_REQ       = ord_busy ? 1'b1            :
                          (I_BUS.req_valid & ~fe_p4 & (fe_gen | I_BUS.req_ready));
assign  o_MEM_WRITE     = ord_busy ? ord_write       : I_BUS.req_write;
assign  o_MEM_BURST     = ord_busy ? ord_burst       : I_BUS.req_burst;
assign  o_MEM_SIZE      = ord_busy ? ord_size        : I_BUS.req_size;
assign  o_MEM_ADDR      = ord_busy ? ord_addr[28:0]  : fa[28:0];
assign  o_MEM_WSTRB     = ord_busy ? ord_wstrb       : I_BUS.req_wstrb;
//posted/prefetched envelope beats SELF-ACCEPT their external completion
//("accept pacing is always internal"): a pulse no call is waiting on must
//not wedge the external controller's response handshake
wire            ord_self_acc = (ordb_act && !(ob_wait && ordb_cnt == ordp_cnt)) ||
                               ((ordw_act || ord_wr_ack || ow_last_wait) &&
                                !(ow_last_wait && ordp_cnt == 2'd3));
assign  o_MEM_RSP_READY = (I_BUS.rsp_ready & (owner_q == OWN_GEN)) | ord_self_acc;

wire    [2:0]   cs_area = ord_busy ? ord_area : fe_area;
genvar gi;
generate for(gi = 0; gi < 7; gi = gi + 1) begin : g_cs
    assign o_MEM_CS_n[gi] = ~(o_MEM_REQ && cs_area == gi[2:0]);
end endgenerate

//response mux back to the cache: the generic owner completes on the external
//handshake (READY or RSP_VALID, one full 32-bit D-bus transfer - the parity
//fast path) OR the width-aware timed bus cycle (ord_data assembly).
//Envelope calls instead stream: a read call serves from the prefetch
//buffer (or passes a live hsk pulse straight through when it completes the
//very beat the call waits on); write calls 1-3 answer as posted acks and
//the 4th at the envelope end (ord_done) carrying any accumulated fault
wire            gen_ext_done = i_MEM_RSP_VALID | i_MEM_READY;
wire            pbs_rsp_v;          //P-bus bridge response (defined in its section)
logic   [31:0]  pbs_rdata_q;
wire            ob_rsp_v  = ob_wait && (obuf_v[ordb_cnt] ||
                                        (gen_ext_done && ordb_cnt == ordp_cnt));
wire            ow_rsp_v  = ord_wr_ack ||
                            (ow_last_wait && (ord_done ||
                                              (gen_ext_done && ordp_cnt == 2'd3)));
wire            gen_bctx  = ordb_act || ordw_act || ow_last_wait || ord_wr_ack;
assign  I_BUS.rsp_valid = (owner_q == OWN_GEN) ?
                              (gen_bctx ? (ordb_act ? ob_rsp_v : ow_rsp_v)
                                        : (gen_ext_done | ord_done)) :
                          (owner_q == OWN_LOC) ? loc_rsp_v       :
                          (owner_q == OWN_PBS) ? pbs_rsp_v       :
                          (sd_rd_wait ? wv[sd_rd_beat] : sd_wr_ack);
assign  I_BUS.rsp_rdata = (owner_q == OWN_GEN) ?
                              (ordb_act ? (obuf_v[ordb_cnt] ? obuf[ordb_cnt] : i_D_I)
                                        : (gen_ext_done ? i_D_I : ord_data)) :
                          (owner_q == OWN_LOC) ? loc_rsp_d   :
                          (owner_q == OWN_PBS) ? pbs_rdata_q : rd_buf[sd_rd_beat];
assign  I_BUS.rsp_fault = (owner_q == OWN_GEN) ?
                              (ordb_act ? (obuf_v[ordb_cnt] ? obuf_f[ordb_cnt]
                                                            : (gen_ext_done & i_MEM_FAULT)) :
                               gen_bctx ? (ordw_fault | (gen_ext_done & i_MEM_FAULT))
                                        : (gen_ext_done & i_MEM_FAULT)) : 1'b0;

wire            fe_rsp_done = I_BUS.rsp_valid && I_BUS.rsp_ready;



///////////////////////////////////////////////////////////
//////  Ordinary Memory / Burst ROM Controller (pp.268, 304)
////

/*
    Bus cycles tick on i_BCEN (the 50 MHz CKIO view). A first access runs
    T1 + n Tw + T2 = 2 + WCR2-first-wait states, read data sampled at the
    mid-T2 fall (figs 10.6/10.10, 23.16). The WAIT pin stretches the Tw
    region when enabled for the area (nonzero wait setting; a 0-wait area
    ignores the pin, pp.241-244) - the Tw->T2 decision uses the WAITSEL-
    selected sample (mid-state when WCR1[15]=1, fig 10.11). Every beat is
    T2-shaped (strobe mid-first-state -> mid-data-state, figs 23.19/23.20):
    burst-ROM continuation beats (beats 1-3 of an envelope on a BCR1 area,
    p.304) just total the burst-pitch states (ord_cnt = pitch - 1) and
    sample WAIT for every wait code (p.242); every burst WRITE run ignores
    the pin entirely (p.304/p.274). Read data samples i_D_I live at the
    sampling fall, like pins of a real asynchronous bus.
*/

//WCR2 3-bit encodings: first-access waits and burst pitch (states-1), p.241
function automatic logic [3:0] w3_first(input logic [2:0] c);
    case(c)
        3'd0: w3_first = 4'd0;  3'd1: w3_first = 4'd1;
        3'd2: w3_first = 4'd2;  3'd3: w3_first = 4'd3;
        3'd4: w3_first = 4'd4;  3'd5: w3_first = 4'd6;
        3'd6: w3_first = 4'd8;  default: w3_first = 4'd10;
    endcase
endfunction

function automatic logic [3:0] w3_pitch(input logic [2:0] c);
    case(c)
        3'd0: w3_pitch = 4'd1;  3'd1: w3_pitch = 4'd1;
        3'd2: w3_pitch = 4'd2;  3'd3: w3_pitch = 4'd3;
        3'd4: w3_pitch = 4'd3;  3'd5: w3_pitch = 4'd5;
        3'd6: w3_pitch = 4'd7;  default: w3_pitch = 4'd9;
    endcase
endfunction

//live per-area timing view of the incoming request (captured at accept)
logic   [2:0]   ord_w3;             //3-bit WCR2 code of the addressed area
logic   [3:0]   ord_first;          //first-access wait states
logic           ord_pin_en;         //WAIT pin sampled for this area
logic           ord_bst_en;         //burst ROM enabled for this area (BCR1)
always_comb begin
    case(fe_area)
        3'd0:    begin ord_w3 = wcr2[2:0];         ord_bst_en = (bcr1[10:9] != 2'd0); end
        3'd2:    begin ord_w3 = {1'b0, wcr2[4:3]}; ord_bst_en = 1'b0; end
        3'd3:    begin ord_w3 = {1'b0, wcr2[6:5]}; ord_bst_en = 1'b0; end
        3'd4:    begin ord_w3 = wcr2[9:7];         ord_bst_en = 1'b0; end
        3'd5:    begin ord_w3 = wcr2[12:10];       ord_bst_en = (bcr1[8:7] != 2'd0); end
        default: begin ord_w3 = wcr2[15:13];       ord_bst_en = (bcr1[6:5] != 2'd0); end
    endcase
    //areas 2/3 as ordinary memory use the plain 0-3 wait code (p.243)
    ord_first  = (fe_area == 3'd2 || fe_area == 3'd3) ? {2'd0, ord_w3[1:0]} : w3_first(ord_w3);
    ord_pin_en = (ord_w3 != 3'd0);
end

/*
    ORDINARY BURST ENVELOPE (figs 10.29/10.30, 23.19/23.20, 11.11): the HEAD
    call of a 4-beat line burst latches the pins; the controller then paces
    the remaining beats ITSELF, back-to-back on the grid - a one-outstanding
    call/return can never chain beats without idle states, so continuation
    calls only stream data. Reads PREFETCH into obuf (rd_buf twin) and the
    calls drain it; write calls queue into obuf ahead of the pins and post
    their acks (wr_buf twin), the 4th call completing with the envelope so
    a unit fault stays visible. Beat timing: head = first-access; beats 2-4
    = burst pitch on a BCR1 burst-ROM area (WAIT always sampled, p.304),
    first-access again on plain areas (fig 11.11). Burst-ROM READS hold
    CSn/DACK across the whole run ("CS0 is not negated, only the address
    is changed", p.304); writes re-frame per beat (basic cycles, figs
    10.29/10.30 notes) and ignore the WAIT pin (p.304). Fill bursts WRAP
    from the missed word (p.110): obuf is indexed by SERVICE ORDER (= call
    order); write bursts are position-ordered 0..3 (drains, DMAC units).
    A fault (hsk injection) aborts the run: the master abandons the rest.
*/
logic           ordb_act;           //read burst in flight (head opens, 4th rsp/fault closes)
logic   [1:0]   ordb_cnt;           //its consumed-call count = the waiting call's obuf slot
logic           ordw_act;           //write burst call tracker (head opens, 4th accept closes)
logic           ob_wait;            //a burst-read call outstanding (index rides ordb_cnt)
logic           ow_last_wait;       //the 4th write call outstanding (rsp at the envelope end)
logic           ord_wr_ack;         //posted burst-write ack pending (beats 0-2)
logic           ordw_fault;         //accumulated fault of posted write beats
logic   [31:0]  obuf    [0:3];      //beat buffer: read prefetch line / write play-out
logic   [3:0]   obuf_v;             //slot valid (read landed / write queued)
logic   [3:0]   obuf_f;             //read slot faulted (hsk injection, per beat)
logic   [3:0]   ostrb   [0:3];      //write beat strobes
logic   [1:0]   ordp_cnt;           //pin-side beat index within the envelope
logic           ordp_env;           //pin-side envelope open (beats remain)
logic           ordp_stall;         //write beat awaits its datum (masters stream fast
                                    //enough to never trigger it; safety valve)
logic           ordp_hold_cs;       //burst-ROM read: CSn spans the run (p.304)

//continuation calls of an open envelope (bookkeeping only, no pin touch)
wire            ord_bcont = I_BUS.req_burst && (I_BUS.req_write ? ordw_act : ordb_act);

///////////////////////////////////////////////////////////
//////  WCR1 Inter-Access Idles (p.240)
////

/*
    "For some memories, data bus drive may not be turned off quickly...
    conflicts when consecutive memory accesses are to different memories or
    when a write immediately follows a memory read" (10.2.3). AnIW idles
    (00/01=1, 10=2, 11=3) are inserted before a PIN bus cycle that (a)
    switches area, or (b) writes after a read in the same area. Only the
    shared physical bus is protected: the launch grid-align (ord_run) and
    the SDRAM engine dispatch are held - the accept and the generic-port
    handshake fast path never pay (they do not drive the D pins). One idle
    is structurally guaranteed by the single-outstanding front end, so only
    codes 10/11 ever add cycles.
*/

function automatic logic [1:0] iw_idles(input logic [1:0] c);
    iw_idles = c[1] ? (c[0] ? 2'd3 : 2'd2) : 2'd1;      //p.240 idle table
endfunction

//AnIW field of the addressed area (bits 2n+1:2n; areas 1/7 never reach the pins)
function automatic logic [1:0] iw_field(input logic [2:0] area);
    case(area)
        3'd0:    iw_field = wcr1[1:0];
        3'd2:    iw_field = wcr1[5:4];
        3'd3:    iw_field = wcr1[7:6];
        3'd4:    iw_field = wcr1[9:8];
        3'd5:    iw_field = wcr1[11:10];
        default: iw_field = wcr1[13:12];
    endcase
endfunction

//last PIN data cycle: area, direction, and bus cycles elapsed since its end
logic   [2:0]   turn_area;
logic           turn_read;
logic           turn_v;
logic   [1:0]   turn_gap;           //saturates at 3 (max programmable need - 1)

//launch allowed when the elapsed gap covers the idles (gap counts from the
//end edge, so a launch k bus cycles later reads gap = k-1 -> need-1 compare)
function automatic logic idle_ok(input logic [2:0] area, input logic wr);
    logic turn;
    turn = turn_v && ((area != turn_area) || (turn_read && wr));
    idle_ok = !turn || (turn_gap >= (iw_idles(iw_field(area)) - 2'd1));
endfunction

wire            ord_idle_ok_nx = idle_ok(fe_area, I_BUS.req_write);   //at the accept edge
wire            ord_idle_ok_q  = idle_ok(ord_area, ord_write);        //at a held launch

//pin data-cycle end events (BCEN edges): ord close of the LAST sub-cycle,
//engine last CL landing, engine last write beat. Refresh/MRS move no data.
//A handshake-completed (generic port) access never drove the D pins: skipped.
//every datum now closes through its T2 state; an envelope ends at the
//LAST beat's close only (mid-run chains insert no idles - same area/dir)
wire            ord_end_tk = ord_busy && ord_run && !ord_done && ord_subs == 2'd0 &&
                             ord_t2 && !(ordp_env && ordp_cnt != 2'd3);
wire            eng_rd_end;         //E_RD_DRAIN exit: the last CL landing edge

always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) begin
        turn_v    <= 1'b0;
        turn_area <= 3'd0;
        turn_read <= 1'b0;
        turn_gap  <= 2'd3;
    end
    else begin if(i_BCEN) begin
        if(ord_end_tk) begin
            turn_area <= ord_area;
            turn_read <= !ord_write;
            turn_v    <= 1'b1;
            turn_gap  <= 2'd0;
        end
        else if(eng_rd_end) begin
            turn_area <= e_cs3 ? 3'd3 : 3'd2;
            turn_read <= 1'b1;
            turn_v    <= 1'b1;
            turn_gap  <= 2'd0;
        end
        else if(eng_wr_done) begin
            turn_area <= e_cs3 ? 3'd3 : 3'd2;
            turn_read <= 1'b0;
            turn_v    <= 1'b1;
            turn_gap  <= 2'd0;
        end
        else if(bus_rel) turn_v <= 1'b0;        //a foreign master owned the pins
        else if(turn_gap != 2'd3) turn_gap <= turn_gap + 2'd1;
    end end
end

//per-area bus width: BCR2 AnSZ for areas 2-6, MD pins for area 0 (table
//10.4 / p.239): 11 = 32-bit, 10 = 16-bit on D15-D0, 01 = 8-bit on D7-D0.
//A datum wider than the port walks its byte addresses low-to-high, one
//full bus cycle each (tables 10.7-10.12); reserved 00 decodes as 32-bit
logic           ord_w16_c, ord_w8_c;
always_comb begin
    logic [1:0] a_sz;
    case(fe_area)
        3'd0:    a_sz = {i_MD4, i_MD3};
        3'd2:    a_sz = bcr2[5:4];
        3'd3:    a_sz = bcr2[7:6];
        3'd4:    a_sz = bcr2[9:8];
        3'd5:    a_sz = bcr2[11:10];
        default: a_sz = bcr2[13:12];
    endcase
    ord_w16_c = (a_sz == 2'b10);
    ord_w8_c  = (a_sz == 2'b01);
end

logic   [3:0]   ord_cnt;            //remaining wait states of this bus cycle
logic   [3:0]   ord_cnt2;           //wait count of the second 16-bit sub-cycle
logic           ord_pin_q;          //WAIT pin sampled for the current sub-cycle
logic           ord_pin2;           //WAIT pin enable of the second sub-cycle
logic           ord_bs_n;           //BS strobe: low for the first cycle of each sub
logic           ord_w16;            //this access runs on a 16-bit port (D15-D0)
logic           ord_w8;             //... or an 8-bit port (D7-D0, WE0 only)
logic   [1:0]   ord_ba;             //sub-cycle byte address (walks the A1:A0 pins)
logic   [1:0]   ord_subs;           //sub-cycles still owed after the current one
logic           ord_t2;             //in this sub's data state (sampled at its mid fall)
logic           ord_run;            //pins asserted: bus cycle is ON the bus-clock grid
logic           ord_stb;            //RD/WEn data strobe: mid-first-state -> mid-data-
                                    //state (tRSD/tWED at the CKIO falls, figs 23.16/
                                    //23.19) - re-pulses per beat, low through the waits
logic           ord_cs;             //CSn view: asserts with the launch, negates mid-T2
                                    //(tCSD2) - a split pair shows the half-CKIO gap;
                                    //a burst-ROM read run holds it low across beats

//width-split lane views (tables 10.7-10.12): the register lane of a byte
//address is endian-mirrored, and the core already laid the datum there
wire    [1:0]   ord_lane8 = BIG_ENDIAN ? ~ord_ba : ord_ba;      //D7-D0 sub-cycle lane
wire            ord_hi16  = ord_ba[1] ^ BIG_ENDIAN;             //1: word lanes 31:16

//WAITSEL (WCR1[15], p.241): 1 = WAIT sampled at the mid-state edge (the fall
//of CKIO), deciding the Tw->T2 transition; 0 = "operation not guaranteed" on
//silicon - HS3 keeps the legacy boundary-edge sample as its defined behavior
logic           wait_smp;
always_ff @(posedge i_CLK) begin
    if(i_CEN && !i_BCEN) wait_smp <= i_WAIT_n;
end
wire            wait_ok_now = wcr1[15] ? wait_smp : i_WAIT_n;

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        ord_busy <= 1'b0;
        ord_done <= 1'b0;
        ord_cnt  <= 4'd0;
        ord_cnt2 <= 4'd0;
        ord_pin_q<= 1'b0;
        ord_pin2 <= 1'b0;
        ord_bs_n <= 1'b1;
        ord_w16  <= 1'b0;
        ord_w8   <= 1'b0;
        ord_ba   <= 2'd0;
        ord_subs <= 2'd0;
        ord_t2   <= 1'b0;
        ord_run  <= 1'b0;
        ord_stb  <= 1'b0;
        ord_cs   <= 1'b0;
        ord_dack <= 1'b0;
        ord_dack_ch <= 1'b0;
        ord_saddr <= 1'b0;
        ordp_cnt <= 2'd0;
        ordp_env <= 1'b0;
        ordp_stall <= 1'b0;
        ordp_hold_cs <= 1'b0;
    end
    else begin
        if(i_CEN && fe_acc && fe_gen && !ord_bcont) begin   //HEAD/single latched at the
            ord_busy  <= 1'b1;                          //accept edge; pins assert on the
            ord_done  <= 1'b0;                          //bus-clock grid (an off-grid accept
            ord_run   <= i_BCEN && ord_idle_ok_nx;      //waits <=1 core cycle; WCR1 idles
            ord_cnt   <= ord_first;                     //hold the launch). Head beat =
            //continuation beats/subs: burst pitch on a BCR1 burst-ROM area  //first-access
            //(total states = pitch, so count = pitch - 1), first-access again on plain
            ord_cnt2  <= ord_bst_en ? (w3_pitch(ord_w3) - 4'd1) : ord_first;
            //WAIT pin: enabled per WCR2 for the first access, ALWAYS for pitch
            //beats (p.304/p.242) - but never inside a burst WRITE run (p.304)
            ord_pin_q <= (I_BUS.req_burst && I_BUS.req_write) ? 1'b0 : ord_pin_en;
            ord_pin2  <= (I_BUS.req_burst && I_BUS.req_write) ? 1'b0 :
                         ord_bst_en ? 1'b1 : ord_pin_en;
            ord_bs_n  <= 1'b0;
            ord_t2    <= 1'b0;
            ord_stb   <= 1'b0;                          //every beat strobes at its mid-launch
            ord_cs    <= 1'b1;                          //  (pins masked until ord_run)
            ord_w16   <= ord_w16_c;
            ord_w8    <= ord_w8_c;
            //sub-cycle plan (tables 10.7-10.12): a datum wider than the port
            //walks its byte addresses low-to-high, one full bus cycle each
            ord_ba    <= ((ord_w16_c || ord_w8_c) && I_BUS.req_size == 2'd2) ?
                         2'b00 : fa[1:0];
            ord_subs  <= ord_w8_c  ? ((I_BUS.req_size == 2'd2) ? 2'd3 :
                                      (I_BUS.req_size == 2'd1) ? 2'd1 : 2'd0) :
                         ord_w16_c ? ((I_BUS.req_size == 2'd2) ? 2'd1 : 2'd0) : 2'd0;
            ord_addr  <= fa;
            ord_write <= I_BUS.req_write;
            ord_burst <= I_BUS.req_burst;
            ord_size  <= I_BUS.req_size;
            ord_wstrb <= I_BUS.req_wstrb;
            ord_wdata <= I_BUS.req_wdata;
            ord_area  <= fe_area;
            ord_dack  <= I_BUS.req_dack;    //DMAC sideband rides the whole run (p.363)
            ord_dack_ch <= I_BUS.req_dack_ch;
            ord_saddr <= I_BUS.req_saddr;
            //line-burst envelope: the controller paces beats 2-4 itself
            ordp_env  <= I_BUS.req_burst;
            ordp_cnt  <= 2'd0;
            ordp_stall<= 1'b0;
            ordp_hold_cs <= I_BUS.req_burst && !I_BUS.req_write && ord_bst_en;
        end
        else if(i_CEN && fe_rsp_done && owner_q == OWN_GEN &&
                (!gen_bctx || I_BUS.rsp_fault ||
                 (ob_wait && ordb_cnt == 2'd3) ||
                 (ow_last_wait && !ord_wr_ack))) begin
            ord_busy <= 1'b0;               //completion closes it: a plain access on
            ord_done <= 1'b0;               //either path, an envelope's 4th call, or
            ord_run  <= 1'b0;               //a FAULT (the master abandons the rest)
            ord_stb  <= 1'b0;               //hsk fast path may end mid-cycle
            ord_cs   <= 1'b0;
            ordp_env <= 1'b0;
            ordp_stall <= 1'b0;
        end
        //hsk fast path completes the CURRENT envelope beat early (off-grid,
        //documented extension behavior - no physical device is on the pins)
        else if(i_CEN && gen_ext_done && ordp_env && ord_busy && !ordp_stall) begin
            if(i_MEM_FAULT || ordp_cnt == 2'd3) begin   //a fault abandons the run
                ord_done <= 1'b1;
                ordp_env <= 1'b0;
                ord_t2   <= 1'b0;
                ord_stb  <= 1'b0;
                ord_cs   <= 1'b0;
            end
            else if(!ord_write || obuf_v[ordp_cnt + 2'd1]) begin
                ordp_cnt <= ordp_cnt + 2'd1;
                ord_addr[3:2] <= ord_addr[3:2] + 2'd1;  //wraps round the line (p.110)
                ord_ba   <= 2'b00;                      //line beats are longwords
                ord_subs <= ord_w8 ? 2'd3 : ord_w16 ? 2'd1 : 2'd0;
                ord_cnt  <= ord_cnt2;
                ord_pin_q<= ord_pin2;
                ord_bs_n <= 1'b0;
                ord_t2   <= 1'b0;
                ord_stb  <= 1'b0;
                ord_cs   <= 1'b1;
                if(ord_write) begin
                    ord_wdata <= obuf [ordp_cnt + 2'd1];
                    ord_wstrb <= ostrb[ordp_cnt + 2'd1];
                end
            end
            else ordp_stall <= 1'b1;        //datum not queued yet (see below)
        end
        else if(i_BCEN && ord_busy && !ord_run) begin
            if(ord_idle_ok_q) ord_run <= 1'b1;          //grid-align + WCR1 idle gap
        end
        else if(i_BCEN && ord_busy && !ord_done) begin
            ord_bs_n <= 1'b1;                           //BS covers each sub's first cycle
            if(ordp_stall) begin
                //write envelope paused between beats: the next datum was not
                //queued in time. Masters stream calls faster than the pins
                //consume beats, so this is a safety valve only; the paused
                //shape is legal (write beats are stand-alone basic cycles)
                if(obuf_v[ordp_cnt + 2'd1]) begin
                    ordp_stall <= 1'b0;
                    ordp_cnt   <= ordp_cnt + 2'd1;
                    ord_addr[3:2] <= ord_addr[3:2] + 2'd1;
                    ord_ba     <= 2'b00;
                    ord_subs   <= ord_w8 ? 2'd3 : ord_w16 ? 2'd1 : 2'd0;
                    ord_cnt    <= ord_cnt2;
                    ord_pin_q  <= ord_pin2;
                    ord_bs_n   <= 1'b0;
                    ord_cs     <= 1'b1;
                    ord_wdata  <= obuf [ordp_cnt + 2'd1];
                    ord_wstrb  <= ostrb[ordp_cnt + 2'd1];
                end
            end
            else if(ord_t2) begin                       //data state closes (sampled at its mid)
                ord_t2 <= 1'b0;
                if(ord_subs != 2'd0) begin              //advance to the next byte address
                    ord_subs   <= ord_subs - 2'd1;
                    ord_ba     <= ord_ba + (ord_w8 ? 2'd1 : 2'd2);
                    ord_cnt    <= ord_cnt2;             //a fresh bus cycle, fresh waits
                    ord_pin_q  <= ord_pin2;
                    ord_bs_n   <= 1'b0;
                    ord_cs     <= 1'b1;                 //CSn re-asserts (held on a burst-
                end                                     //  ROM read run: mid-T2 kept it)
                else if(ordp_env && ordp_cnt != 2'd3) begin
                    //beat boundary: chain the next beat back-to-back (fig 11.11 /
                    //figs 10.29-10.30) - A3:A2 step at this launch posedge
                    if(!ord_write || obuf_v[ordp_cnt + 2'd1]) begin
                        ordp_cnt   <= ordp_cnt + 2'd1;
                        ord_addr[3:2] <= ord_addr[3:2] + 2'd1;  //wraps round the line
                        ord_ba     <= 2'b00;
                        ord_subs   <= ord_w8 ? 2'd3 : ord_w16 ? 2'd1 : 2'd0;
                        ord_cnt    <= ord_cnt2;
                        ord_pin_q  <= ord_pin2;
                        ord_bs_n   <= 1'b0;
                        ord_cs     <= 1'b1;
                        if(ord_write) begin
                            ord_wdata <= obuf [ordp_cnt + 2'd1];
                            ord_wstrb <= ostrb[ordp_cnt + 2'd1];
                        end
                    end
                    else ordp_stall <= 1'b1;
                end
                else begin
                    ord_done <= 1'b1;
                    ordp_env <= 1'b0;
                end
            end
            else if(ord_cnt != 4'd0)           ord_cnt <= ord_cnt - 4'd1;
            else if(!ord_pin_q || wait_ok_now) ord_t2  <= 1'b1;  //Tw -> data state
        end
        //mid-state edge (the CKIO fall): strobes assert at mid-launch (tRSD/
        //tWED) and negate at the data-state mid together with CSn (tCSD2);
        //the read sample is bound to the SAME fall - tRDH1 = 0 ns (p.671)
        //lets the device release data the moment RD rises, so sampling any
        //later is unbuildable. Inside a burst-ROM READ run only the strobe
        //re-pulses; CSn stays low until the run's last datum (p.304, fig 23.19)
        else if(i_CEN && !i_BCEN && ord_busy && ord_run && !ord_done) begin
            if(!ord_bs_n) ord_stb <= 1'b1;              //mid-launch (BS marks that state)
            if(ord_t2) begin                            //data mid: sample, then negate
                ord_stb <= 1'b0;
                if(!ordp_hold_cs || (ordp_cnt == 2'd3 && ord_subs == 2'd0))
                    ord_cs <= 1'b0;
                if(ord_w8)       ord_data[{ord_lane8, 3'd0} +: 8] <= i_D_I[7:0];  //8-bit: D7-D0
                else if(ord_w16) begin                  //16-bit port: D15-D0 per half
                    if(ord_hi16) ord_data[31:16] <= i_D_I[15:0];
                    else         ord_data[15:0]  <= i_D_I[15:0];
                end
                else ord_data <= i_D_I;                 //32-bit port: one live sample
            end
        end
    end
end

//beat buffer (single writer): write calls queue ahead of the pins (their
//own slot - bursts are position-ordered 0..3); a read beat lands at its
//close (timed path, ord_data holds the assembled datum) or straight off
//the live pulse (hsk path completes beats as full 32-bit transfers)
always_ff @(posedge i_CLK) begin if(i_CEN) begin
    if(fe_acc && fe_gen && I_BUS.req_burst && I_BUS.req_write) begin
        obuf [fa[3:2]] <= I_BUS.req_wdata;
        ostrb[fa[3:2]] <= I_BUS.req_wstrb;
    end
    else if(gen_ext_done && ordp_env && ord_busy && !ordp_stall && !ord_write)
        obuf[ordp_cnt] <= i_D_I;
    else if(i_BCEN && ord_busy && !ord_done && !ordp_stall && ord_t2 &&
            ord_subs == 2'd0 && ordp_env && !ord_write)
        obuf[ordp_cnt] <= ord_data;
end end

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        obuf_v <= 4'd0;
        obuf_f <= 4'd0;
    end
    else begin if(i_CEN) begin
        if(fe_acc && fe_gen && I_BUS.req_burst) begin
            if(!ord_bcont) begin                //head resets the line's marks
                obuf_v <= {3'b000, I_BUS.req_write};
                obuf_f <= 4'd0;
            end
            else if(I_BUS.req_write) obuf_v[fa[3:2]] <= 1'b1;
        end
        else if(gen_ext_done && ordp_env && ord_busy && !ordp_stall && !ord_write) begin
            obuf_v[ordp_cnt] <= 1'b1;
            obuf_f[ordp_cnt] <= i_MEM_FAULT;    //buffered fault: reported on ITS call
        end
        else if(i_BCEN && ord_busy && !ord_done && !ordp_stall && ord_t2 &&
                ord_subs == 2'd0 && ordp_env && !ord_write)
            obuf_v[ordp_cnt] <= 1'b1;
    end end
end



///////////////////////////////////////////////////////////
//////  Front-End Sequencing (core rate)
////

//register-file read word, muxed on the live address, captured at the accept edge
logic   [15:0]  reg_rd_w;
always_comb begin
    if(fa[7:4] == 4'h5) reg_rd_w = mcscr[fa[3:1]];
    else case(fa[7:1])
        7'h30:   reg_rd_w = bcr1;               //0xFFFFFF60
        7'h31:   reg_rd_w = bcr2;               //0xFFFFFF62
        7'h32:   reg_rd_w = wcr1;               //0xFFFFFF64
        7'h33:   reg_rd_w = wcr2;               //0xFFFFFF66
        7'h34:   reg_rd_w = mcr;                //0xFFFFFF68
        7'h36:   reg_rd_w = pcr;                //0xFFFFFF6C
        7'h37:   reg_rd_w = {8'd0, rtcsr};      //0xFFFFFF6E
        7'h38:   reg_rd_w = {8'd0, rtcnt};      //0xFFFFFF70
        7'h39:   reg_rd_w = {8'd0, rtcor};      //0xFFFFFF72
        7'h3A:   reg_rd_w = {6'd0, rfcr};       //0xFFFFFF74
        default: reg_rd_w = 16'd0;              //reserved offsets read 0
    endcase
end

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        owner_q    <= OWN_GEN;
        loc_rsp_v  <= 1'b0;
        loc_rsp_d  <= 32'd0;
        sd_rd_wait <= 1'b0;
        sd_rd_beat <= 2'd0;
        sd_wr_ack  <= 1'b0;
        b_rd_act   <= 1'b0;
        b_rd_cnt   <= 2'd0;
        b_wr_act   <= 1'b0;
        ordb_act   <= 1'b0;
        ordb_cnt   <= 2'd0;
        ordw_act   <= 1'b0;
        ob_wait    <= 1'b0;
        ow_last_wait <= 1'b0;
        ord_wr_ack <= 1'b0;
        ordw_fault <= 1'b0;
    end
    else begin if(i_CEN) begin
        //posted write beats: accumulate an hsk-injected fault for the 4th
        //call's response (the fire-and-forget beats already acknowledged)
        if(gen_ext_done && i_MEM_FAULT && ordp_env && ord_busy && ord_write)
            ordw_fault <= 1'b1;

        if(fe_acc) begin
            owner_q <= fe_gen ? OWN_GEN : fe_local ? OWN_LOC : fe_pbus ? OWN_PBS : OWN_SDR;

            if(fe_local) begin                  //register/dummy: one-beat response
                loc_rsp_v <= 1'b1;
                loc_rsp_d <= fe_reg ? {2{reg_rd_w}} : 32'd0;
            end

            //ordinary-bus read line burst: the head call opens the in-flight track
            if(fe_gen && I_BUS.req_burst && !I_BUS.req_write && !ordb_act) begin
                ordb_act <= 1'b1;
                ordb_cnt <= 2'd0;
            end

            //ordinary burst call bookkeeping (sd_rd_wait/sd_wr_ack twins)
            if(fe_gen && I_BUS.req_burst) begin
                if(!I_BUS.req_write) ob_wait <= 1'b1;   //call index rides ordb_cnt
                else if(fa[3:2] == 2'b11) begin         //4th call: rsp at the envelope
                    ordw_act     <= 1'b0;               //end so a unit fault is seen
                    ow_last_wait <= 1'b1;
                end
                else begin                              //beats 0-2 post their acks
                    ordw_act   <= 1'b1;
                    ord_wr_ack <= 1'b1;
                end
            end

            if(fe_eng) begin
                if(I_BUS.req_write || fe_sdmr) sd_wr_ack  <= 1'b1;  //posted
                else begin
                    sd_rd_wait <= 1'b1;
                    sd_rd_beat <= fa[3:2];
                end
                if(fe_sdram && !fe_sdmr && I_BUS.req_burst && fe_b_head) begin
                    b_rd_act <= !I_BUS.req_write;
                    b_wr_act <=  I_BUS.req_write;
                    b_rd_cnt <= 2'd0;
                end
                //last drain beat closes the write burst
                if(b_wr_act && fa[3:2] == 2'b11) b_wr_act <= 1'b0;
            end
        end

        if(fe_rsp_done) begin
            loc_rsp_v <= 1'b0;
            sd_wr_ack <= 1'b0;
            ob_wait    <= 1'b0;
            ord_wr_ack <= 1'b0;
            //the 4th write call's response retires the write-burst context;
            //a posted ack pending outranks it (one response per cycle)
            if(ow_last_wait && !ord_wr_ack) begin
                ow_last_wait <= 1'b0;
                ordw_fault   <= 1'b0;
            end
            if(sd_rd_wait) begin
                sd_rd_wait <= 1'b0;
                //the 4th consumed beat closes the read burst (count, not position)
                b_rd_cnt <= b_rd_cnt + 2'd1;
                if(b_rd_act && b_rd_cnt == 2'd3) b_rd_act <= 1'b0;
            end
            //ordinary burst: 4th call - or a fault (the master abandons the
            //run, no further calls will come) - closes the in-flight track
            if(owner_q == OWN_GEN && ordb_act) begin
                if(I_BUS.rsp_fault || ordb_cnt == 2'd3) ordb_act <= 1'b0;
                else ordb_cnt <= ordb_cnt + 2'd1;
            end
            if(owner_q == OWN_GEN && I_BUS.rsp_fault) ordw_act <= 1'b0;
        end
    end end
end

//engine op slot: set at the accept edge of a head/single op, cleared when the
//engine leaves E_IDLE with it. Continuation write beats only top up the buffer.
logic           eng_op_write, eng_op_burst, eng_op_mrs, eng_op_lock, eng_cs3;
logic   [1:0]   eng_op_size;
logic   [31:0]  eng_addr;

//the cache's drain is interruptible (it revisits S_IDLE between beats): a
//blocked ordinary request makes the engine YIELD mid-burst-write, so a
//resumed drain beat must START a fresh write op from its own beat index
wire            eng_wr_pend  = (eng_go && eng_op_write) ||
                               (e_write && (est == E_BA_DISP || est == E_PRE_WAIT ||
                                            est == E_ACTV    || est == E_RCD || est == E_WR));
wire            fe_eng_start = fe_acc_eng && (!fe_b_cont ||
                                              (I_BUS.req_write && !eng_wr_pend));
wire            eng_start_tk;                   //engine takes the op (BCEN edge)

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) eng_go <= 1'b0;
    else begin if(i_CEN) begin
        if(fe_eng_start)      eng_go <= 1'b1;
        else if(eng_start_tk) eng_go <= 1'b0;   //i_BCEN implies i_CEN
    end end
end

always_ff @(posedge i_CLK) begin if(i_CEN) begin
    if(fe_eng_start) begin
        eng_op_write <= I_BUS.req_write && !fe_sdmr;
        eng_op_burst <= I_BUS.req_burst && !fe_sdmr;
        eng_op_mrs   <= fe_sdmr;
        eng_op_lock  <= I_BUS.req_lock;
        eng_op_size  <= I_BUS.req_size;
        eng_cs3      <= fe_sdmr ? (fa[13:12] == 2'b10) : (fe_area == 3'd3);  //0xFFFFExxx = area 3
        eng_addr     <= fa;
    end
end end

//write line buffer: a beat lands in slot addr[3:2] (singles use their own slot);
//cleared when the engine finishes the write op
wire            eng_wr_done;
always_ff @(posedge i_CLK) begin
    if(i_CEN && fe_acc_eng && I_BUS.req_write && !fe_sdmr) begin
        wr_buf [fa[3:2]] <= I_BUS.req_wdata;
        wr_strb[fa[3:2]] <= I_BUS.req_wstrb;
    end
end
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) wr_v <= 4'd0;
    else begin if(i_CEN) begin
        if(fe_acc_eng && I_BUS.req_write && !fe_sdmr) wr_v[fa[3:2]] <= 1'b1;
        else if(eng_wr_done)                          wr_v <= 4'd0;
    end end
end



///////////////////////////////////////////////////////////
//////  P Bus Bridge (sessions 3-4: TMU / RTC / PFC-port register tier)
////

/*
    The P-bus peripherals live behind the BSC in the block diagram (Fig 1.1
    p.6; Appendix B marks their registers "P"). Same contract as the I-bus-2
    BRIDGE: IDLE -> ACCESS -> RESP, write payloads right-justified by the
    strobes, read values lane-replicated by size. Read latency 2 cycles
    accept-to-response - handshake latency is allowed on this tier only
    (user latency law). Front-end reset domain: a dying access must not
    wedge the bus; the slaves keep their own reset rules.
*/

localparam logic [1:0] P_IDLE = 2'd0, P_ACC = 2'd1, P_RSP = 2'd2;
localparam logic [1:0] PW_TMU = 2'd0, PW_RTC = 2'd1, PW_PRT = 2'd2, PW_DMA = 2'd3;

logic   [1:0]   pbs_state;
logic   [1:0]   pbs_win_q;          //captured window select (PW_*)
logic   [7:0]   pbs_addr_q;         //byte address within the window
logic           pbs_we_q;
logic   [1:0]   pbs_size_q;
logic   [31:0]  pbs_wdata_q;        //right-justified write payload

assign  pbs_ready = (pbs_state == P_IDLE);
assign  pbs_rsp_v = (pbs_state == P_RSP);

//right-justify off the strobes (ibus_bridge pattern: the strobed lane IS the
//payload lane in either endianness; misaligned accesses fault in MA)
logic   [31:0]  pbs_wdata_rj;
always_comb begin
    unique case(I_BUS.req_wstrb)
        4'b1000: pbs_wdata_rj = {24'd0, I_BUS.req_wdata[31:24]};
        4'b0100: pbs_wdata_rj = {24'd0, I_BUS.req_wdata[23:16]};
        4'b0010: pbs_wdata_rj = {24'd0, I_BUS.req_wdata[15:8]};
        4'b0001: pbs_wdata_rj = {24'd0, I_BUS.req_wdata[7:0]};
        4'b1100: pbs_wdata_rj = {16'd0, I_BUS.req_wdata[31:16]};
        4'b0011: pbs_wdata_rj = {16'd0, I_BUS.req_wdata[15:0]};
        default: pbs_wdata_rj = I_BUS.req_wdata;                //long (or read: don't-care)
    endcase
end

//selected slave read, replicated onto the lanes so the pipe's load aligner
//picks the correct byte/halfword from any naturally aligned offset
wire    [31:0]  pbs_sel_rdata = (pbs_win_q == PW_TMU) ? REG_TMU.rdata :
                                (pbs_win_q == PW_RTC) ? REG_RTC.rdata :
                                (pbs_win_q == PW_DMA) ? REG_DMAC.rdata : REG_PORT.rdata;
logic   [31:0]  pbs_rdata_rep;
always_comb begin
    unique case(pbs_size_q)
        2'd0:    pbs_rdata_rep = {4{pbs_sel_rdata[7:0]}};
        2'd1:    pbs_rdata_rep = {2{pbs_sel_rdata[15:0]}};
        default: pbs_rdata_rep = pbs_sel_rdata;
    endcase
end

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        pbs_state   <= P_IDLE;
        pbs_win_q   <= PW_TMU;
        pbs_addr_q  <= 8'd0;
        pbs_we_q    <= 1'b0;
        pbs_size_q  <= 2'd0;
        pbs_wdata_q <= 32'd0;
        pbs_rdata_q <= 32'd0;
    end
    else begin if(i_CEN) begin
        unique case(pbs_state)
            P_IDLE: begin
                if(fe_acc && fe_pbus) begin                     //accept edge: latch everything
                    pbs_win_q   <= fe_tmu  ? PW_TMU : fe_rtc ? PW_RTC :
                                   fe_dmac ? PW_DMA : PW_PRT;
                    pbs_addr_q  <= fa[7:0];
                    pbs_we_q    <= I_BUS.req_write;
                    pbs_size_q  <= I_BUS.req_size;
                    pbs_wdata_q <= pbs_wdata_rj;
                    pbs_state   <= P_ACC;
                end
            end
            P_ACC: begin                                        //slave strobe: sample the read
                pbs_rdata_q <= pbs_rdata_rep;
                pbs_state   <= P_RSP;
            end
            default: begin                                      //P_RSP
                if(I_BUS.rsp_ready) pbs_state <= P_IDLE;
            end
        endcase
    end end
end

//IBus_2 drive: shared payload fans to all slaves; only the captured window
//sees its strobe
assign  REG_TMU.stb    = (pbs_state == P_ACC) && (pbs_win_q == PW_TMU);
assign  REG_RTC.stb    = (pbs_state == P_ACC) && (pbs_win_q == PW_RTC);
assign  REG_PORT.stb   = (pbs_state == P_ACC) && (pbs_win_q == PW_PRT);
assign  REG_DMAC.stb   = (pbs_state == P_ACC) && (pbs_win_q == PW_DMA);

assign  REG_TMU.we     = pbs_we_q;
assign  REG_TMU.size   = pbs_size_q;
assign  REG_TMU.addr   = pbs_addr_q;
assign  REG_TMU.wdata  = pbs_wdata_q;
assign  REG_RTC.we     = pbs_we_q;
assign  REG_RTC.size   = pbs_size_q;
assign  REG_RTC.addr   = pbs_addr_q;
assign  REG_RTC.wdata  = pbs_wdata_q;
assign  REG_PORT.we    = pbs_we_q;
assign  REG_PORT.size  = pbs_size_q;
assign  REG_PORT.addr  = pbs_addr_q;
assign  REG_PORT.wdata = pbs_wdata_q;
assign  REG_DMAC.we    = pbs_we_q;
assign  REG_DMAC.size  = pbs_size_q;
assign  REG_DMAC.addr  = pbs_addr_q;
assign  REG_DMAC.wdata = pbs_wdata_q;



///////////////////////////////////////////////////////////
//////  SDRAM Engine (i_BCEN domain - the natural-latency machine)
////

/*
    Commands on {CS,RAS,CAS,WE} (p.276); one command per 20 ns bus cycle.
    Pin registers update at BCEN edges; the PCB clock rises mid-cycle (180
    degrees), giving 10 ns setup/hold either side. Spacing counts (TPC after
    a precharge, RCD after ACTV) are command-to-command distances: value 1
    means back-to-back commands, so wait states = value - 1. Duration counts
    (Tpc/Trwl tails, refresh lockout, self-refresh exit) hold the engine for
    exactly that many cycles.

    Auto-precharge mode (RASD=0): ACTV -> rcd -> READ.../WRIT... with
    auto-precharge on the last beat, then the Tpc tail (figs 10.14-10.18).
    Bank-active mode (RASD=1): commands without precharge; per-bank open-row
    table; PRE -> tpc -> ACTV on a row miss (figs 10.19-10.24); one Tnop
    before a row-hit READ when CL=1 (DQM two-cycle lead, p.290).
*/

typedef enum logic [4:0] {
    E_IDLE,
    E_MRS_PALL, E_MRS_WAIT, E_MRS_SET,  E_MRS_MRD,      //fig 10.28
    E_BA_DISP,  E_PRE_WAIT,                             //bank-active row lookup / precharge gap
    E_ACTV,     E_RCD,                                  //row open + RAS-CAS gap (also the Tnop)
    E_RD,       E_RD_DRAIN, E_RD_TPC,                   //READ beats + CL landing + Tpc tail
    E_WR,       E_WR_TRWL,  E_WR_TPC,                   //WRIT beats + Trwl + Tpc tails
    E_WR_YIELD,                                         //interrupted burst: Trwl then PRE
    E_BRQ_PALL, E_BRQ_WAIT,                             //close banks before a bus grant
    E_REF_PALL, E_REF_WAIT, E_REF_CMD,  E_REF_LOCK,     //fig 10.26
    E_SLF_PALL, E_SLF_WAIT, E_SLF_CMD,  E_SLF,  E_SLF_EXIT  //fig 10.27
} eng_state_t;

/*
    Address multiplexing (MCR.AMX + bus width, table 10.13 pp.279-280).
    ROW phase: pins A16-A1 output addr >> shift ("the row address begins
    with A(shift+1)", p.247); A25-A17 and A0 keep the original value.
    COLUMN phase: the original address, except (a) the auto-precharge flag
    replaces the device-A10 pin - chip A12 on a 32-bit bus, A11 on 16-bit -
    and (b) a per-mode set of the pins A16-A13 HOLDS its row-phase value,
    which keeps the bank bits standing on the device BA pins through the
    whole access (the *4 columns of table 10.13). Reserved and still-0000
    AMX codes decode as the shift-8 family; the manual forbids access
    before AMX is programmed.
*/

//row shift by AMX family (identical for both bus widths)
function automatic logic [3:0] amx_shift(input logic [3:0] amx);
    case(amx)
        4'b1110: amx_shift = 4'd10;         //8Mx16 (16-bit bus only)
        4'b1101: amx_shift = 4'd9;          //4Mx16
        4'b0101: amx_shift = 4'd9;          //2Mx16 / 2Mx8
        default: amx_shift = 4'd8;          //0100 1Mx16, 0111 512kx32, reserved
    endcase
endfunction

//column-phase hold mask over pins {A16,A15,A14,A13} (row values kept)
function automatic logic [3:0] amx_hold(input logic w16, input logic [3:0] amx);
    if(w16) amx_hold = (amx == 4'b0101 || amx == 4'b1101 || amx == 4'b1110) ?
                       4'b0111 : 4'b0011;
    else    amx_hold = (amx == 4'b1101) ? 4'b1110 :
                       (amx == 4'b0111) ? 4'b0011 : 4'b0110;
endfunction

//bank address LSB position within the physical address
function automatic logic [4:0] amx_blsb(input logic w16, input logic [3:0] amx);
    case(amx)
        4'b1110: amx_blsb = 5'd24;
        4'b1101: amx_blsb = w16 ? 5'd23 : 5'd24;
        4'b0101: amx_blsb = w16 ? 5'd22 : 5'd23;
        4'b0111: amx_blsb = 5'd21;
        default: amx_blsb = w16 ? 5'd21 : 5'd22;
    endcase
endfunction

//ROW-phase pin pattern of an address
function automatic logic [25:0] amx_row(input logic [3:0] amx, input logic [31:0] a);
    logic [4:0] sh;
    sh = {1'b0, amx_shift(amx)};
    amx_row = {a[25:17], a[(5'd16 + sh) -: 16], a[0]};
endfunction

//COLUMN-phase pin pattern (ap = auto-precharge flag on the device A10 pin)
function automatic logic [25:0] amx_col(input logic w16, input logic [3:0] amx,
                                        input logic [31:0] a, input logic ap);
    logic [4:0]  sh;
    logic [3:0]  hm;
    logic [25:0] c;
    sh = {1'b0, amx_shift(amx)};
    hm = amx_hold(w16, amx);
    c  = a[25:0];
    if(hm[0]) c[13] = a[5'd13 + sh];
    if(hm[1]) c[14] = a[5'd14 + sh];
    if(hm[2]) c[15] = a[5'd15 + sh];
    if(hm[3]) c[16] = a[5'd16 + sh];
    if(w16) c[11] = ap;
    else    c[12] = ap;
    amx_col = c;
endfunction

//bank bits of an address
function automatic logic [1:0] amx_bank(input logic w16, input logic [3:0] amx,
                                        input logic [31:0] a);
    amx_bank = a[amx_blsb(w16, amx) +: 2];
endfunction

//device row bits of an address, right-justified (11-13 bits; 0-padded so the
//open-row table compares only DEVICE-meaningful bits - a pin-pattern compare
//would fake row conflicts on the sub-row address bits)
function automatic logic [12:0] amx_rowv(input logic w16, input logic [3:0] amx,
                                         input logic [31:0] a);
    logic [4:0] rl;
    logic [4:0] w;
    rl = {1'b0, amx_shift(amx)} + (w16 ? 5'd1 : 5'd2);  //device A0 pin: A1 / A2
    w  = amx_blsb(w16, amx) - rl;                       //row width 11-13
    amx_rowv = 13'(a >> rl) & ~(13'h1FFF << w);
endfunction

//shared-bus pin registers of the SDRAM engine (merged with the ordinary
//controller's drive at the physical pin mux below)
logic           sd_cs2_n, sd_cs3_n;
logic           sd_rasl_n, sd_rasu_n, sd_casl_n, sd_casu_n;
logic           sd_cmdwe_n;         //the shared RD/WR pin, SDRAM command view
logic   [25:0]  sd_a;               //chip A25-A0 view (device A10:A0 sits on A12:A2)
logic   [3:0]   sd_dqm;
logic           sd_bs_n, sd_cke;
logic   [31:0]  sd_dq_o;
logic           sd_dq_oe;

eng_state_t     est;
logic   [3:0]   ecnt;               //shared wait counter
logic   [2:0]   ebeat;              //command beat within a burst: word index (32-bit bus,
                                    //wraps mod 4) or half-word index A3:A1 (16-bit, mod 8)
logic   [2:0]   ebcnt;              //command beats issued so far (fills wrap round the line)
logic   [2:0]   e_bm1;              //command beats of this op, minus 1 (0/1/3/7)
logic   [3:0]   rd_need;            //data landings still expected
logic   [2:0]   wr_recov;           //bank-active write recovery (tWR) before a precharge
logic           e_write, e_burst, e_cs3, e_lock_hold;
logic           e_sd16;             //op runs on a 16-bit SDRAM data bus (BCR2)
logic   [1:0]   e_size;             //single-access size (drives the read DQM byte lanes)
logic   [31:0]  e_addr;
logic           rst_z;              //front-end reset, sampled for the write-abort escape

//32-bit-lane byte enables of an access (strobe maps of tables 10.7/10.10):
//"a read/write is performed for the byte for which the corresponding DQM is
//low" (p.276) - single reads drive only their own lanes
function automatic logic [3:0] sz_lanes(input logic [1:0] a, input logic [1:0] sz);
    case(sz)
        2'd0:    sz_lanes = BIG_ENDIAN ? (4'b1000 >> a) : (4'b0001 << a);
        2'd1:    sz_lanes = (a[1] ^ BIG_ENDIAN) ? 4'b1100 : 4'b0011;
        default: sz_lanes = 4'b1111;
    endcase
endfunction

//read-latency pipeline: slot n = a READ issued n+1 bus cycles ago; the slot
//at CL-1 leaving the pipe means DQ carries that beat's data at this edge.
//rdp_f marks landings that COMPLETE their 32-bit word (16-bit bus: the odd
//half of a word pair, or a lone word/byte beat)
logic   [2:0]   rdp_v;
logic   [2:0]   rdp_b [0:2];
logic   [2:0]   rdp_f;
wire    [1:0]   e_cl    = e_cs3 ? cl_a3 : cl_a2;
wire            rd_lat  = rdp_v[e_cl - 2'd1];
wire    [2:0]   rd_latb = rdp_b[e_cl - 2'd1];
wire            rd_latf = rdp_f[e_cl - 2'd1];
wire    [1:0]   rd_latw = e_sd16 ? rd_latb[2:1] : rd_latb[1:0];     //word slot
wire            rd_lath = rd_latb[0] ^ BIG_ENDIAN;  //16-bit half: 1 = word lanes 31:16
assign  eng_rd_end = (est == E_RD_DRAIN) && (rd_need == 4'd1) && rd_lat;

//BS marks the Td DATA cycles of a read, not the commands ("asserted in each
//of cycles Td1-Td4 in a synchronous DRAM cycle", p.283): predict next-cycle
//landings one slot earlier in the CL pipe (a write's command IS its data
//cycle, so E_WR keeps its own BS)
logic           rd_td_nx;
always_comb begin
    case(e_cl)
        2'd1:    rd_td_nx = (est == E_RD);      //CL=1: data rides the cycle after issue
        2'd2:    rd_td_nx = rdp_v[0];
        default: rd_td_nx = rdp_v[1];
    endcase
end

//bank-active open-row table (device: 4 banks); bank/row per the AMX decode
logic   [3:0]   ba_v;
logic   [12:0]  ba_row [0:3];
wire    [1:0]   e_bank   = amx_bank(e_sd16, mcr[6:3], e_addr);
wire    [12:0]  e_row    = amx_rowv(e_sd16, mcr[6:3], e_addr);
wire            row_hit  = ba_v[e_bank] && (ba_row[e_bank] == e_row);
wire            row_conf = ba_v[e_bank] && (ba_row[e_bank] != e_row);

//live twins on the un-dispatched op (the E_IDLE fold decides from these)
wire            eng_sd16    = eng_cs3 ? sd16_a3 : sd16_a2;
wire    [1:0]   eng_bank    = amx_bank(eng_sd16, mcr[6:3], eng_addr);
wire    [12:0]  eng_rowv    = amx_rowv(eng_sd16, mcr[6:3], eng_addr);
wire            row_hit_nx  = ba_v[eng_bank] && (ba_row[eng_bank] == eng_rowv);
wire            row_conf_nx = ba_v[eng_bank] && (ba_row[eng_bank] != eng_rowv);
wire    [1:0]   e_cl_nx     = eng_cs3 ? cl_a3 : cl_a2;

//single-rail AMX modes (table 10.13 note 1: A25 is a bank bit, one device
//set spans the whole 64MB) never drive RAS3U/CASU; otherwise A25 picks the
//upper/lower 32MB rail (note 2)
wire            amx_nou    = (mcr[6:3] == (e_sd16   ? 4'b1110 : 4'b1101));
wire            amx_nou_nx = (mcr[6:3] == (eng_sd16 ? 4'b1110 : 4'b1101));
wire            amx_nou_gl = (mcr[6:3] == (sd16_gl  ? 4'b1110 : 4'b1101));
wire            e_up       = e_addr[25]   && !amx_nou;
wire            eng_up     = eng_addr[25] && !amx_nou_nx;

//bus arbitration (p.320): BREQ is granted only with the bus drained (engine
//idle or parked in self-refresh, no ordinary cycle, no queued op) and all
//bank-active rows closed (E_BRQ_PALL runs first so the next master - and
//the Micron model - meets precharged banks). While requested or granted, no
//new external cycle is accepted; refresh requests stay latched and run on
//regain. Register/local accesses keep working - only the pins are released.
logic           breq_z, breq_zz;    //2FF sync of the async pin
logic           bus_rel;
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) begin
        breq_z  <= 1'b0;
        breq_zz <= 1'b0;
    end
    else begin if(i_CEN) begin
        breq_z  <= ~i_BREQ_n;
        breq_zz <= breq_z;
    end end
end
wire            brq      = breq_zz;
assign          bus_held = brq | bus_rel;

always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) bus_rel <= 1'b0;
    else begin if(i_BCEN) begin
        if(!brq)                                       bus_rel <= 1'b0;
        else if((est == E_IDLE || est == E_SLF) &&
                !ord_busy && !eng_go && ba_v == 4'd0 &&
                !e_lock_hold && !fe_lock_hold)         bus_rel <= 1'b1;  //never split a TAS pair (p.320)
    end end
end

assign  o_BACK_n = ~bus_rel;
assign  o_BUS_OE = ~bus_rel;

//refresh arbitration: a pending request completes, then refresh wins over a
//NEW op; a locked RMW pair (TAS) is never split by a refresh
logic           ref_req;
wire            ref_ok = ref_req && rfsh && !rmode && !e_lock_hold;
wire            self_req = rfsh && rmode;               //MCR.RMODE level (p.300)

//dispatch qualifier: EXACTLY the E_IDLE case's arm order (ord bus ownership,
//BREQ row-close, refresh, self-refresh all outrank an op) + the WCR1 idle gap.
//eng_start_tk previously ignored the higher arms - an op could be consumed by
//a BRQ_PALL edge (lost op wedge) or double-dispatched under BREQ+refresh.
wire            eng_idle_ok    = idle_ok(eng_cs3 ? 3'd3 : 3'd2, eng_op_write);
wire            eng_dispatch_ok = !ord_busy && !(brq && ba_v != 4'd0) &&
                                  !(ref_ok && !bus_held) && !(self_req && !bus_held) &&
                                  eng_idle_ok;
assign  eng_start_tk = (est == E_IDLE) && i_BCEN && eng_go && eng_dispatch_ok;
//E_SLF is a PARKED state: the SDRAM sits in self-refresh on CKE alone and
//the shared bus is free for ordinary cycles (a new SDRAM op is still held
//off by self_active). Everything else counts as bus ownership.
assign  eng_busy     = (est != E_IDLE) && (est != E_SLF);

//per-beat views: the beat index rides A3:A2 of the column (A3:A1 on a 16-bit
//bus, p.283); drains are position-ordered so their last beat tests ebeat
wire    [31:0]  e_beat_addr = e_sd16 ? {e_addr[31:4], ebeat, e_addr[0]}
                                     : {e_addr[31:4], ebeat[1:0], e_addr[1:0]};
wire            rd_cmd_fin  = !e_sd16 ||
                              ((e_burst || e_size == 2'd2) ? ebeat[0] : 1'b1);
wire            wr_last     = e_sd16 ? (e_burst ? (ebeat == 3'd7) :
                                        (e_size == 2'd2 ? ebeat[0] : 1'b1))
                                     : (!e_burst || ebeat[1:0] == 2'd3);
wire            wr_hi       = ebeat[0] ^ BIG_ENDIAN;    //16-bit half: word lanes 31:16
wire    [1:0]   wr_slot     = e_burst ? (e_sd16 ? ebeat[2:1] : ebeat[1:0]) : e_addr[3:2];
assign  eng_wr_done = (est == E_WR) && i_BCEN && wr_v[wr_slot] && wr_last;

//16-bit-bus read DQM pair (DQMLU/DQMLL rails; tables 10.8/10.11 strobe map)
wire    [1:0]   rd16_dqml   = (e_burst || e_size != 2'd0) ? 2'b00 :
                              ((e_addr[0] ^ BIG_ENDIAN) ? 2'b01 : 2'b10);

always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) begin
        est   <= E_IDLE;
        ecnt  <= 4'd0;
        ebeat <= 3'd0;
        ebcnt <= 3'd0;
        e_bm1 <= 3'd0;
        rd_need <= 4'd0;
        wr_recov <= 3'd0;
        e_write <= 1'b0; e_burst <= 1'b0; e_cs3 <= 1'b0; e_lock_hold <= 1'b0;
        e_sd16  <= 1'b0;
        e_size  <= 2'd0;
        e_addr  <= 32'd0;
        rst_z   <= 1'b0;
        rdp_v <= 3'd0;
        rdp_f <= 3'd0;
        rdp_b[0] <= 3'd0; rdp_b[1] <= 3'd0; rdp_b[2] <= 3'd0;
        ba_v  <= 4'd0;
        ba_row[0] <= 13'd0; ba_row[1] <= 13'd0; ba_row[2] <= 13'd0; ba_row[3] <= 13'd0;
        sd_cs2_n  <= 1'b1; sd_cs3_n  <= 1'b1;
        sd_rasl_n <= 1'b1; sd_rasu_n <= 1'b1;
        sd_casl_n <= 1'b1; sd_casu_n <= 1'b1;
        sd_cmdwe_n <= 1'b1;
        sd_a      <= 26'd0;
        sd_dqm    <= 4'b1111;
        sd_bs_n   <= 1'b1;
        sd_cke    <= 1'b1;
        sd_dq_o   <= 32'd0; sd_dq_oe <= 1'b0;
    end
    else begin if(i_BCEN) begin
        rst_z <= i_RST_n;

        //every cycle defaults to NOP/deselect; op arms below override.
        //WE/DQM idles HIGH (shared with the ordinary write strobes); a read
        //op holds its LANES low from the row open (2-cycle DQM lead, p.290;
        //single reads enable only the addressed bytes - p.276, tables 10.7+)
        sd_cs2_n  <= 1'b1; sd_cs3_n  <= 1'b1;
        sd_rasl_n <= 1'b1; sd_rasu_n <= 1'b1;
        sd_casl_n <= 1'b1; sd_casu_n <= 1'b1;
        sd_cmdwe_n <= 1'b1;
        sd_dqm    <= (!e_write && (est == E_ACTV || est == E_RCD ||
                                   est == E_RD   || est == E_RD_DRAIN)) ?
                     (e_sd16  ? {2'b11, rd16_dqml} :
                      e_burst ? 4'b0000 : ~sz_lanes(e_addr[1:0], e_size)) : 4'b1111;
        sd_bs_n   <= 1'b1;
        sd_dq_oe  <= 1'b0;

        //read-latency pipeline always shifts; E_RD refills slot 0
        rdp_v    <= {rdp_v[1:0], 1'b0};
        rdp_f    <= {rdp_f[1:0], 1'b0};
        rdp_b[1] <= rdp_b[0];
        rdp_b[2] <= rdp_b[1];

        //bank-active write recovery drains every cycle (E_WR reloads it)
        if(wr_recov != 3'd0) wr_recov <= wr_recov - 3'd1;

        case(est)
        E_IDLE: begin
            if(ord_busy) begin end                      //an ordinary cycle owns the bus
            else if(brq && ba_v != 4'd0) est <= E_BRQ_PALL; //close rows, then grant
            else if(ref_ok && !bus_held)   est <= E_REF_PALL;
            else if(self_req && !bus_held) est <= E_SLF_PALL;
            else if(eng_go && eng_idle_ok) begin
                e_write <= eng_op_write;
                e_burst <= eng_op_burst;
                e_cs3   <= eng_cs3;
                e_sd16  <= eng_sd16;
                e_size  <= eng_op_size;
                e_addr  <= eng_addr;
                //a locked read opens the refresh-deferral window (TAS pair)
                if(eng_op_lock && !eng_op_write) e_lock_hold <= 1'b1;
                //bursts start at their own beat (fill reads wrap round the
                //line); a 16-bit bus runs half-word beats off A3:A1
                ebeat   <= eng_sd16 ? eng_addr[3:1] : {1'b0, eng_addr[3:2]};
                ebcnt   <= 3'd0;
                e_bm1   <= eng_sd16 ? (eng_op_burst ? 3'd7 :
                                       (eng_op_size == 2'd2 ? 3'd1 : 3'd0))
                                    : (eng_op_burst ? 3'd3 : 3'd0);
                rd_need <= eng_sd16 ? (eng_op_burst ? 4'd8 :
                                       (eng_op_size == 2'd2 ? 4'd2 : 4'd1))
                                    : (eng_op_burst ? 4'd4 : 4'd1);
                //the first command issues AT this dispatch edge from the live
                //op fields - the real chip overlaps dispatch with the previous
                //op's tail, no NOP between ops (figs 10.14-10.24)
                if(eng_op_mrs) begin
                    if(wr_recov == 3'd0) begin      //tWR guard (bank-active writes)
                        sd_cs2_n <= ~a2_sdram; sd_cs3_n <= ~a3_sdram;   //PALL, all devices
                        sd_rasl_n <= 1'b0; sd_rasu_n <= amx_nou_gl;
                        sd_cmdwe_n <= 1'b0;
                        sd_a[12] <= 1'b1; sd_a[11] <= 1'b1;     //device A10, either width
                        ba_v <= 4'd0;
                        if(t_tpc == 3'd1) est <= E_MRS_SET;
                        else begin ecnt <= {1'b0, t_tpc} - 4'd1; est <= E_MRS_WAIT; end
                    end
                    else est <= E_MRS_PALL;         //guard draining: park and retry
                end
                else if(rasd) begin                 //bank-active row decision, live fields
                    if(row_hit_nx) begin
                        if(!eng_op_write && e_cl_nx == 2'd1) begin  //Tnop: DQM 2-cycle lead
                            ecnt <= 4'd1;
                            est  <= E_RCD;
                        end
                        else est <= eng_op_write ? E_WR : E_RD;
                    end
                    else if(row_conf_nx) begin
                        if(wr_recov == 3'd0) begin  //tWR guard before the precharge
                            sd_cs2_n <= eng_cs3; sd_cs3_n <= ~eng_cs3;
                            if(eng_up) sd_rasu_n <= 1'b0;           //PRE this bank
                            else       sd_rasl_n <= 1'b0;
                            sd_cmdwe_n <= 1'b0;
                            sd_a <= amx_col(eng_sd16, mcr[6:3], eng_addr, 1'b0);
                            ba_v[eng_bank] <= 1'b0;
                            if(t_tpc == 3'd1) est <= E_ACTV;
                            else begin ecnt <= {1'b0, t_tpc} - 4'd1; est <= E_PRE_WAIT; end
                        end
                        else est <= E_BA_DISP;      //guard draining: park and retry
                    end
                    else begin                      //bank idle: ACTV at this edge
                        sd_cs2_n <= eng_cs3; sd_cs3_n <= ~eng_cs3;
                        if(eng_up) sd_rasu_n <= 1'b0;
                        else       sd_rasl_n <= 1'b0;
                        sd_a <= amx_row(mcr[6:3], eng_addr);
                        ba_v[eng_bank]   <= 1'b1;
                        ba_row[eng_bank] <= eng_rowv;
                        if(t_rcd == 3'd1) est <= eng_op_write ? E_WR : E_RD;
                        else begin ecnt <= {1'b0, t_rcd} - 4'd1; est <= E_RCD; end
                    end
                end
                else begin                          //auto-precharge: ACTV at this edge
                    sd_cs2_n <= eng_cs3; sd_cs3_n <= ~eng_cs3;
                    if(eng_up) sd_rasu_n <= 1'b0;
                    else       sd_rasl_n <= 1'b0;
                    sd_a <= amx_row(mcr[6:3], eng_addr);
                    if(t_rcd == 3'd1) est <= eng_op_write ? E_WR : E_RD;
                    else begin ecnt <= {1'b0, t_rcd} - 4'd1; est <= E_RCD; end
                end
            end
        end

        /* mode register set: PALL -> tpc gap -> MRS -> 4 cycles (fig 10.28) */
        E_MRS_PALL: begin
            if(wr_recov == 3'd0) begin                  //tWR guard (bank-active writes)
                sd_cs2_n <= ~a2_sdram; sd_cs3_n <= ~a3_sdram;   //PALL, all devices
                sd_rasl_n <= 1'b0; sd_rasu_n <= amx_nou_gl;     //U+L together (p.276)
                sd_cmdwe_n <= 1'b0;
                sd_a[12] <= 1'b1; sd_a[11] <= 1'b1;             //device A10, either width
                ba_v <= 4'd0;
                if(t_tpc == 3'd1) est <= E_MRS_SET;
                else begin ecnt <= {1'b0, t_tpc} - 4'd1; est <= E_MRS_WAIT; end
            end
        end
        E_MRS_WAIT: begin
            if(ecnt <= 4'd1) est <= E_MRS_SET;
            else             ecnt <= ecnt - 4'd1;
        end
        E_MRS_SET: begin
            sd_cs2_n <= e_cs3; sd_cs3_n <= ~e_cs3;
            sd_rasl_n <= 1'b0; sd_rasu_n <= amx_nou;    //MRS drives U+L (p.276)
            sd_casl_n <= 1'b0; sd_casu_n <= amx_nou;
            sd_cmdwe_n <= 1'b0;
            sd_a     <= e_addr[25:0];                   //mode value rides A12:A2 / A11:A1
            ecnt <= 4'd4;                               //TMw1-4 covers tMRD (fig 10.28)
            est  <= E_MRS_MRD;
        end
        E_MRS_MRD: begin
            if(ecnt <= 4'd1) est <= E_IDLE;
            else             ecnt <= ecnt - 4'd1;
        end

        /* bank-active dispatch (RASD=1, pp.289-290): row lookup, then either the
           command beats directly (hit), or PRE -> tpc gap -> ACTV (conflict) */
        E_BA_DISP: begin
            if(row_hit) begin
                if(!e_write && e_cl == 2'd1) begin      //Tnop: DQM two-cycle lead
                    ecnt <= 4'd1;
                    est  <= E_RCD;
                end
                else est <= e_write ? E_WR : E_RD;
            end
            else if(row_conf) begin
                if(wr_recov == 3'd0) begin              //tWR guard before the precharge
                    sd_cs2_n <= e_cs3; sd_cs3_n <= ~e_cs3;
                    if(e_up) sd_rasu_n <= 1'b0;             //PRE this bank
                    else     sd_rasl_n <= 1'b0;
                    sd_cmdwe_n <= 1'b0;
                    sd_a <= amx_col(e_sd16, mcr[6:3], e_addr, 1'b0);
                    ba_v[e_bank] <= 1'b0;
                    if(t_tpc == 3'd1) est <= E_ACTV;
                    else begin ecnt <= {1'b0, t_tpc} - 4'd1; est <= E_PRE_WAIT; end
                end
            end
            else est <= E_ACTV;                         //bank idle: activate
        end
        E_PRE_WAIT: begin
            if(ecnt <= 4'd1) est <= E_ACTV;
            else             ecnt <= ecnt - 4'd1;
        end

        /* row activate + RAS-CAS gap (Tr, Trw; p.281) */
        E_ACTV: begin
            sd_cs2_n <= e_cs3; sd_cs3_n <= ~e_cs3;
            if(e_up) sd_rasu_n <= 1'b0;                 //ACTV (RAS3L/U by 32MB half)
            else     sd_rasl_n <= 1'b0;
            sd_a     <= amx_row(mcr[6:3], e_addr);      //row = addr >> shift on A16-A1
            if(rasd) begin
                ba_v[e_bank]   <= 1'b1;
                ba_row[e_bank] <= e_row;
            end
            if(t_rcd == 3'd1) est <= e_write ? E_WR : E_RD;
            else begin ecnt <= {1'b0, t_rcd} - 4'd1; est <= E_RCD; end
        end
        E_RCD: begin
            if(ecnt <= 4'd1) est <= e_write ? E_WR : E_RD;
            else             ecnt <= ecnt - 4'd1;
        end

        /* READ/READA beats, one per cycle, WRAPPING round the line from the
           missed word (fig 10.16 column order; READA on the 4th/last beat) */
        E_RD: begin
            sd_cs2_n <= e_cs3; sd_cs3_n <= ~e_cs3;
            if(e_up) sd_casu_n <= 1'b0;                 //READ / READA
            else     sd_casl_n <= 1'b0;
            //column phase: beat index replaces A3:A2 (A3:A1 on 16-bit),
            //READA flag on the last command
            sd_a     <= amx_col(e_sd16, mcr[6:3], e_beat_addr,
                                !rasd && (ebcnt == e_bm1));
            rdp_v[0]   <= 1'b1;
            rdp_b[0]   <= ebeat;
            rdp_f[0]   <= rd_cmd_fin;
            if(ebcnt == e_bm1) est <= E_RD_DRAIN;
            else begin                                  //wraps round the line
                ebeat <= e_sd16 ? (ebeat + 3'd1) : {1'b0, ebeat[1:0] + 2'd1};
                ebcnt <= ebcnt + 3'd1;
            end
        end
        E_RD_DRAIN: begin                               //exit at the last CL landing edge
            if(eng_rd_end) begin
                if(rasd) est <= E_IDLE;                 //no precharge tail in bank-active
                else begin ecnt <= {1'b0, t_tpc}; est <= E_RD_TPC; end
            end
        end
        E_RD_TPC: begin
            if(ecnt <= 4'd1) est <= E_IDLE;
            else             ecnt <= ecnt - 4'd1;
        end

        /* WRIT/WRITA beats: data rides the command cycle (p.285). A beat whose
           posted data has not landed yet is a NOP stall (stretches, never breaks) */
        E_WR: begin
            if(!rst_z || (!wr_v[wr_slot] && I_BUS.req_valid && fe_gen)) begin
                //beats will never arrive (manual reset) or a blocked ordinary
                //request needs the bus: close the interrupted burst. Bank-
                //active just releases (banks stay open); auto-precharge must
                //Trwl then PRE the bank the plain WRITs left active
                if(rasd) begin
                    wr_recov <= t_trwl;
                    ecnt <= 4'd1;
                    est  <= E_WR_TRWL;
                end
                else begin
                    ecnt <= {1'b0, t_trwl};
                    est  <= E_WR_YIELD;
                end
            end
            else if(wr_v[wr_slot]) begin
                sd_cs2_n <= e_cs3; sd_cs3_n <= ~e_cs3;
                if(e_up) sd_casu_n <= 1'b0;             //WRIT / WRITA
                else     sd_casl_n <= 1'b0;
                sd_cmdwe_n <= 1'b0;
                sd_a     <= amx_col(e_sd16, mcr[6:3], e_beat_addr, !rasd && wr_last);
                //a 16-bit bus drains half a word per beat on D15-D0
                sd_dq_o  <= !e_sd16 ? wr_buf[wr_slot] :
                            {2{wr_hi ? wr_buf[wr_slot][31:16] : wr_buf[wr_slot][15:0]}};
                sd_dqm   <= !e_sd16 ? ~wr_strb[wr_slot] :
                            {2'b11, ~(wr_hi ? wr_strb[wr_slot][3:2]
                                            : wr_strb[wr_slot][1:0])};
                sd_dq_oe <= 1'b1;
                sd_bs_n  <= 1'b0;
                if(wr_last) begin
                    e_lock_hold <= 1'b0;                //locked pair completed
                    //bank-active: no Trwl/Tpc tail (p.289), but the bus stays
                    //owned for the WRIT command's own cycle (one E_WR_TRWL
                    //beat), else the pin mux flips mid-command; tWR before a
                    //precharge is guarded by wr_recov instead
                    if(rasd) wr_recov <= t_trwl;
                    ecnt <= rasd ? 4'd1 : {1'b0, t_trwl};
                    est  <= E_WR_TRWL;
                end
                else ebeat <= e_sd16 ? (ebeat + 3'd1) : {1'b0, ebeat[1:0] + 2'd1};
            end
        end
        E_WR_TRWL: begin
            if(ecnt <= 4'd1) begin
                if(rasd) est <= E_IDLE;                 //ownership beat served
                else begin
                    ecnt <= {1'b0, t_tpc};
                    est  <= E_WR_TPC;
                end
            end
            else ecnt <= ecnt - 4'd1;
        end
        E_WR_TPC: begin
            if(ecnt <= 4'd1) est <= E_IDLE;
            else             ecnt <= ecnt - 4'd1;
        end
        E_WR_YIELD: begin                               //close an interrupted AP burst
            if(ecnt <= 4'd1) begin
                sd_cs2_n <= e_cs3; sd_cs3_n <= ~e_cs3;
                if(e_up) sd_rasu_n <= 1'b0;             //PRE the bank the WRITs opened
                else     sd_rasl_n <= 1'b0;
                sd_cmdwe_n <= 1'b0;
                sd_a <= amx_col(e_sd16, mcr[6:3], e_addr, 1'b0);
                ecnt <= {1'b0, t_tpc};
                est  <= E_WR_TPC;
            end
            else ecnt <= ecnt - 4'd1;
        end

        /* bus-grant precharge: bank-active rows must close before another
           master (or the model) touches the SDRAM */
        E_BRQ_PALL: begin
            if(wr_recov == 3'd0) begin                  //tWR guard as everywhere
                sd_cs2_n <= ~a2_sdram; sd_cs3_n <= ~a3_sdram;
                sd_rasl_n <= 1'b0; sd_rasu_n <= amx_nou_gl;     //PALL, all devices
                sd_cmdwe_n <= 1'b0;
                sd_a[12] <= 1'b1; sd_a[11] <= 1'b1;             //device A10, either width
                ba_v <= 4'd0;
                if(t_tpc == 3'd1) est <= E_IDLE;
                else begin ecnt <= {1'b0, t_tpc} - 4'd1; est <= E_BRQ_WAIT; end
            end
        end
        E_BRQ_WAIT: begin
            if(ecnt <= 4'd1) est <= E_IDLE;
            else             ecnt <= ecnt - 4'd1;
        end

        /* auto-refresh: PALL -> tpc gap -> REF -> tras+tpc lockout (fig 10.26) */
        E_REF_PALL: begin
            if(wr_recov == 3'd0) begin                  //tWR guard (bank-active writes)
                sd_cs2_n <= ~a2_sdram; sd_cs3_n <= ~a3_sdram;
                sd_rasl_n <= 1'b0; sd_rasu_n <= amx_nou_gl;
                sd_cmdwe_n <= 1'b0;
                sd_a[12] <= 1'b1; sd_a[11] <= 1'b1;     //device A10, either width
                ba_v <= 4'd0;                           //refresh closes all banks (p.290)
                if(t_tpc == 3'd1) est <= E_REF_CMD;
                else begin ecnt <= {1'b0, t_tpc} - 4'd1; est <= E_REF_WAIT; end
            end
        end
        E_REF_WAIT: begin
            if(ecnt <= 4'd1) est <= E_REF_CMD;
            else             ecnt <= ecnt - 4'd1;
        end
        E_REF_CMD: begin
            sd_cs2_n <= ~a2_sdram; sd_cs3_n <= ~a3_sdram;
            sd_rasl_n <= 1'b0; sd_rasu_n <= amx_nou_gl; //REF drives U+L (p.276)
            sd_casl_n <= 1'b0; sd_casu_n <= amx_nou_gl;
            ecnt <= {1'b0, t_tras} + {1'b0, t_tpc} - 4'd1;
            est  <= E_REF_LOCK;
        end
        E_REF_LOCK: begin                               //no command for TRAS+TPC (p.297)
            if(ecnt <= 4'd1) est <= E_IDLE;
            else             ecnt <= ecnt - 4'd1;
        end

        /* self-refresh: REF entry with CKE low, held until RMODE clears (p.300) */
        E_SLF_PALL: begin
            if(wr_recov == 3'd0) begin                  //tWR guard (bank-active writes)
                sd_cs2_n <= ~a2_sdram; sd_cs3_n <= ~a3_sdram;
                sd_rasl_n <= 1'b0; sd_rasu_n <= amx_nou_gl;
                sd_cmdwe_n <= 1'b0;
                sd_a[12] <= 1'b1; sd_a[11] <= 1'b1;     //device A10, either width
                ba_v <= 4'd0;
                if(t_tpc == 3'd1) est <= E_SLF_CMD;
                else begin ecnt <= {1'b0, t_tpc} - 4'd1; est <= E_SLF_WAIT; end
            end
        end
        E_SLF_WAIT: begin
            if(ecnt <= 4'd1) est <= E_SLF_CMD;
            else             ecnt <= ecnt - 4'd1;
        end
        E_SLF_CMD: begin
            sd_cs2_n <= ~a2_sdram; sd_cs3_n <= ~a3_sdram;
            sd_rasl_n <= 1'b0; sd_rasu_n <= amx_nou_gl; //SELF = REF with CKE low
            sd_casl_n <= 1'b0; sd_casu_n <= amx_nou_gl;
            sd_cke   <= 1'b0;
            est <= E_SLF;
        end
        E_SLF: begin
            if(!self_req) begin                         //software cleared RMODE
                sd_cke <= 1'b1;
                ecnt <= t_tpc_slf;                      //exit wait 2/5/8/11 (p.245)
                est  <= E_SLF_EXIT;
            end
            else sd_cke <= 1'b0;
        end
        E_SLF_EXIT: begin
            if(ecnt <= 4'd1) est <= E_IDLE;
            else             ecnt <= ecnt - 4'd1;
        end

        default: est <= E_IDLE;
        endcase

        //read Td cycles carry BS (p.283) - override the default/arm value
        if(rd_td_nx) sd_bs_n <= 1'b0;

        //read-data landing: the slot leaving the CL pipeline carries this edge's DQ
        if(rd_lat) rd_need <= rd_need - 3'd1;


    end end
end

//landing capture: whole words on a 32-bit bus; a 16-bit bus assembles each
//word from two D15-D0 halves (lane placement per tables 10.8/10.11)
always_ff @(posedge i_CLK) begin
    if(i_BCEN && rd_lat) begin
        if(!e_sd16)      rd_buf[rd_latw]        <= i_D_I;
        else if(rd_lath) rd_buf[rd_latw][31:16] <= i_D_I[15:0];
        else             rd_buf[rd_latw][15:0]  <= i_D_I[15:0];
    end
end

//word-valid handoff: engine sets per word-COMPLETING landing; the head accept
//of the next read clears (the engine cannot land words before its first command)
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) wv <= 4'd0;
    else begin
        if(i_CEN && fe_acc_eng && !I_BUS.req_write && !fe_sdmr && !fe_b_cont) wv <= 4'd0;
        else if(i_BCEN && rd_lat && rd_latf) wv[rd_latw] <= 1'b1;
    end
end



///////////////////////////////////////////////////////////
//////  Refresh Timer (i_BCEN domain, CKIO-based; pp.253-257)
////

//prescaler taps: CKS 001=/4 010=/16 011=/64 100=/256 101=/1024 110=/2048 111=/4096
logic   [11:0]  presc;
logic           presc_tick;
always_comb begin
    case(rtcsr[5:3])
        3'b001:  presc_tick = (presc[1:0]  == 2'b11);
        3'b010:  presc_tick = (presc[3:0]  == 4'hF);
        3'b011:  presc_tick = (presc[5:0]  == 6'h3F);
        3'b100:  presc_tick = (presc[7:0]  == 8'hFF);
        3'b101:  presc_tick = (presc[9:0]  == 10'h3FF);
        3'b110:  presc_tick = (presc[10:0] == 11'h7FF);
        3'b111:  presc_tick = (presc[11:0] == 12'hFFF);
        default: presc_tick = 1'b0;             //clock input disabled
    endcase
end

always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) presc <= 12'd0;
    else begin if(i_BCEN) begin
        presc <= presc + 12'd1;
    end end
end

wire            rt_match  = presc_tick && (rtcnt == rtcor);
wire            ref_done  = (est == E_REF_CMD);         //the cycle the REF issues
wire    [9:0]   rfcr_lim  = rtcsr[0] ? 10'd511 : 10'd1023;  //LMTS (p.255)



///////////////////////////////////////////////////////////
//////  Register Writes (core rate; POR-only reset, p.228)
////

/*
    All registers are 16-bit word-access; the strobed half of the 32-bit bus is
    extracted right-justified (bridge pattern). The refresh group demands the
    write keys of fig 10.5: RTCSR/RTCNT/RTCOR = 0xA5 upper byte, RFCR =
    6'b101001 upper bits; wrong key or size is silently ignored.
*/

wire    [15:0]  wr_w     = I_BUS.req_wstrb[3] ? I_BUS.req_wdata[31:16] : I_BUS.req_wdata[15:0];
wire            wr_reg   = fe_acc && fe_reg && I_BUS.req_write && (I_BUS.req_size == 2'd1);
wire            key_a5   = (wr_w[15:8] == 8'hA5);
wire            key_rfcr = (wr_w[15:10] == 6'b101001);

wire            wr_rtcsr = wr_reg && (fa[7:1] == 7'h37) && key_a5;
wire            wr_rtcnt = wr_reg && (fa[7:1] == 7'h38) && key_a5;
wire            wr_rtcor = wr_reg && (fa[7:1] == 7'h39) && key_a5;
wire            wr_rfcr  = wr_reg && (fa[7:1] == 7'h3A) && key_rfcr;

always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) begin
        bcr1  <= {4'd0, !BIG_ENDIAN, 11'd0};    //H'0000; ENDIAN=0 means BIG endian
                                                //(MD5 pin low, p.236)
        bcr2  <= 16'h3FF0;
        wcr1  <= 16'h3FF3;
        wcr2  <= 16'hFFFF;
        mcr   <= 16'h0000;
        pcr   <= 16'h0000;
        rtcor <= 8'd0;
        mcscr[0] <= 16'd0; mcscr[1] <= 16'd0; mcscr[2] <= 16'd0; mcscr[3] <= 16'd0;
        mcscr[4] <= 16'd0; mcscr[5] <= 16'd0; mcscr[6] <= 16'd0; mcscr[7] <= 16'd0;
    end
    else begin if(i_CEN) begin
        if(wr_reg) begin
            if(fa[7:4] == 4'h5) mcscr[fa[3:1]] <= wr_w & 16'h007F;      //bits 15-7 reserved (p.258)
            else case(fa[7:1])
                7'h30: bcr1 <= {wr_w[15:12], !BIG_ENDIAN, wr_w[10:0]};  //ENDIAN read-only
                7'h31: bcr2 <= wr_w & 16'h3FF0;         //bits 15,14,3-0 reserved (p.239)
                7'h32: wcr1 <= wr_w & 16'hBFF3;         //bits 14,3,2 reserved (p.240)
                7'h33: wcr2 <= wr_w;
                7'h34: mcr  <= wr_w & 16'hFFFE;         //bit 0 reserved (p.244)
                7'h36: pcr  <= wr_w & 16'hCFFF;         //bits 13,12 reserved (p.248)
                default: ;                              //keyed group handled below
            endcase
        end
        if(wr_rtcor) rtcor <= wr_w[7:0];
    end end
end

//RTCNT: keyed software write beats the timer tick (WDT precedent)
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) rtcnt <= 8'd0;
    else begin
        if(i_CEN && wr_rtcnt)   rtcnt <= wr_w[7:0];
        else if(i_BCEN) begin
            if(rt_match)        rtcnt <= 8'd0;
            else if(presc_tick) rtcnt <= rtcnt + 8'd1;
        end
    end
end

//RTCSR: CMF/OVF set by hardware. OVF clears on a keyed write-0; CMF instead
//ARMS on the write-0 and clears when the next CBR refresh is PERFORMED
//(p.253 clearing condition: "when a refresh is performed after 0 has been
//written to CMF and RFSH=1 and RMODE=0"). A match landing on the consuming
//refresh edge wins - it is a new compare event
logic           cmf_clr_arm;        //write-0 seen, waiting for the refresh
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) begin
        rtcsr       <= 8'd0;
        cmf_clr_arm <= 1'b0;
    end
    else begin
        if(i_CEN && wr_rtcsr) begin
            rtcsr[6:3]  <= wr_w[6:3];
            rtcsr[1:0]  <= wr_w[1:0];
            cmf_clr_arm <= !wr_w[7];                    //write-1 never changes CMF
            rtcsr[2]    <= rtcsr[2] & wr_w[2];          //OVF: write-0 clears now
        end
        else if(i_BCEN) begin
            if(ref_done && cmf_clr_arm) begin           //the armed clear is consumed
                rtcsr[7]    <= rt_match;
                cmf_clr_arm <= 1'b0;
            end
            else if(rt_match) rtcsr[7] <= 1'b1;         //CMF
            if(ref_done && rfcr == rfcr_lim) rtcsr[2] <= 1'b1;      //OVF
        end
    end
end

//RFCR counts refresh cycles; exceeding the LMTS limit sets OVF and CLEARS
//the counter (p.256); a keyed write loads it directly
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) rfcr <= 10'd0;
    else begin
        if(i_CEN && wr_rfcr)        rfcr <= wr_w[9:0];
        else if(i_BCEN && ref_done) rfcr <= (rfcr == rfcr_lim) ? 10'd0 : rfcr + 10'd1;
    end
end

//refresh request: latched at compare match, cleared when the REF cycle runs
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) ref_req <= 1'b0;
    else begin if(i_BCEN) begin
        if(rt_match && rfsh && !rmode) ref_req <= 1'b1;
        else if(ref_done)              ref_req <= 1'b0;
    end end
end

//self-refresh entry tracker for the front-end request stall
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) self_active <= 1'b0;
    else begin if(i_BCEN) begin
        if(est == E_SLF_CMD)                       self_active <= 1'b1;
        else if(est == E_SLF_EXIT && ecnt <= 4'd1) self_active <= 1'b0;
    end end
end

assign  o_RCMI_REQ = rtcsr[7] & rtcsr[6];       //CMF & CMIE
assign  o_ROVI_REQ = rtcsr[2] & rtcsr[1];       //OVF & OVIE



///////////////////////////////////////////////////////////
//////  Pad States: Pull-Ups, Release Drive, IRQOUT (pp.236, 320-322)
////

//PULA: A25-A0 pulled up for 4 CKIO after BACK asserts, then Hi-Z (fig 10.41)
logic   [2:0]   apu_cnt;
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) apu_cnt <= 3'd0;
    else begin if(i_BCEN) begin
        if(!bus_rel)             apu_cnt <= 3'd0;
        else if(apu_cnt != 3'd4) apu_cnt <= apu_cnt + 3'd1;
    end end
end
assign  o_A_PU = bcr1[15] && bus_rel && (apu_cnt != 3'd4);

//PULD: D31-D0 pulled up whenever the data bus is not in use - dropped for
//the BSC's own drive, an ordinary read strobe window, or an SDRAM read op
//(figs 10.42/10.43 show the pull-up around the data phases)
wire            d_ord_rd = ord_pins && !ord_write && ord_stb;
wire            d_sd_rd  = !e_write && (est == E_ACTV || est == E_RCD ||
                                        est == E_RD   || est == E_RD_DRAIN);
assign  o_D_PU = bcr1[14] && !o_D_OE && !d_ord_rd && !d_sd_rd;

//HIZCNT: RAS/CAS pads stay driven through a bus release when set (p.236;
//HIZMEM concerns standby mode only, which this SoC does not enter)
assign  o_RASCAS_OE = ~bus_rel | bcr1[12];

//IRQOUT contribution: a latched refresh request whose cycle has not run yet
//(p.321) - a foreign master sees it and returns the bus
assign  o_REF_PEND = ref_req;



///////////////////////////////////////////////////////////
//////  MCS0-7 Mask-ROM Selects (MCSCR0-7; table 10.15 p.324)
////

//MCS[x] asserts - with the CS shape of the ordinary bus cycle - when the
//cycle's area matches CS2/0 and A25:22 falls in the CAP-sized block
function automatic logic mcs_hit(input logic [15:0] r, input logic [2:0] area,
                                 input logic [3:0] blk);
    logic am;
    case(r[5:4])                                //CAP: connected memory size
        2'b11:   am = (blk[3]   == r[3]);       //256 Mbit: A25 only
        2'b10:   am = (blk[3:2] == r[3:2]);     //128 Mbit: A25-A24
        2'b01:   am = (blk[3:1] == r[3:1]);     //64 Mbit:  A25-A23
        default: am = (blk      == r[3:0]);     //32 Mbit:  A25-A22
    endcase
    mcs_hit = am && (area == (r[6] ? 3'd2 : 3'd0));
endfunction

genvar gm;
generate for(gm = 0; gm < 8; gm = gm + 1) begin : g_mcs
    assign o_MCS_n[gm] = ~(ord_pins && ord_cs &&
                           mcs_hit(mcscr[gm], ord_area, ord_addr[25:22]));
end endgenerate

assign  o_MCS0_CS0 = ~mcscr[0][6];      //area-0 decode: CS0 pad may switch (p.323)



///////////////////////////////////////////////////////////
//////  Physical Pin Merge (the shared external bus, table 10.1)
////

/*
    Ordinary bus cycles (ord_busy, latched in the front-end) and SDRAM engine
    cycles (sd_* regs, BCEN domain) never overlap - one outstanding
    transaction - so every shared pin is a plain 2:1 mux of registers.
    Idle levels match silicon: strobes high, address holds, D released.
*/

//pins flip to the ordinary fields only from the grid-aligned launch edge
//(ord_run) and release at the T2-close boundary edge (!ord_done) - never at
//a core handshake edge, so every pin change sits ON the bus-clock grid.
//(The generic-port hsk fast path can still end a cycle early: documented)
wire    ord_pins  = ord_busy && ord_run && !ord_done;

assign  o_A       = ord_pins ? {ord_addr[25:2], ord_ba} : sd_a;
//narrow ports live on the low D lanes (tables 10.8/10.9): the register lane
//of the current byte address is driven there, endian-mirrored
assign  o_D_O     = !ord_pins ? sd_dq_o :
                    ord_w8    ? {4{ord_wdata[{ord_lane8, 3'd0} +: 8]}} :
                    ord_w16   ? {2{ord_hi16 ? ord_wdata[31:16] : ord_wdata[15:0]}} :
                    ord_wdata;
//single-address write: WE runs but the EXTERNAL device drives D31-0 (fig 11.10a)
assign  o_D_OE    = ord_pins ? (ord_write && !ord_saddr) : sd_dq_oe;    //write data T1..T2 end (tWDH1)
assign  o_BS_n    = ord_pins ? ord_bs_n       : sd_bs_n;
assign  o_CS0_n   = ~(ord_pins && ord_cs && ord_area == 3'd0);
assign  o_CS2_n   = ord_pins ? !(ord_cs && ord_area == 3'd2) : sd_cs2_n;
assign  o_CS3_n   = ord_pins ? !(ord_cs && ord_area == 3'd3) : sd_cs3_n;
assign  o_CS4_n   = ~(ord_pins && ord_cs && ord_area == 3'd4);
assign  o_CS5_n   = ~(ord_pins && ord_cs && ord_area == 3'd5);
assign  o_CS6_n   = ~(ord_pins && ord_cs && ord_area == 3'd6);
assign  o_RD_WR   = ord_pins ? ~ord_write     : sd_cmdwe_n; //low = write cycle
assign  o_RAS3L_n = ord_pins ? 1'b1           : sd_rasl_n;
assign  o_RAS3U_n = ord_pins ? 1'b1           : sd_rasu_n;
assign  o_CASL_n  = ord_pins ? 1'b1           : sd_casl_n;
assign  o_CASU_n  = ord_pins ? 1'b1           : sd_casu_n;
assign  o_WE_n    = !ord_pins ? sd_dqm :
                    !(ord_write && ord_stb) ? 4'b1111 :     //strobed mid-T1 -> mid-T2
                    ord_w8    ? {3'b111, ~ord_wstrb[ord_lane8]} :           //WE0 only
                    ord_w16   ? {2'b11, ord_hi16 ? ~ord_wstrb[3:2]
                                                 : ~ord_wstrb[1:0]} :       //WE1/WE0 lanes
                    ~ord_wstrb;
assign  o_RD_n    = ~(ord_pins && !ord_write && ord_stb);   //ordinary read strobe

//DACK windows: exactly the tagged cycle's CSn assertion window ("DACK is
//output for the same duration as CSn", p.363); a width-split access
//re-frames per sub-cycle just as CSn does. Polarity applied in the DMAC.
assign  o_DACK_WIN[0] = ord_pins && ord_cs && ord_dack && !ord_dack_ch;
assign  o_DACK_WIN[1] = ord_pins && ord_cs && ord_dack &&  ord_dack_ch;
assign  o_CKE     = sd_cke;

endmodule

`default_nettype none
