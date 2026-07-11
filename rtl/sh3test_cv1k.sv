//
// sh3test_cv1k.sv — SH-3 Fmax stress suite, engine top (ikacore_CV1k).
//
// Three clock domains:
//   clk50   : sh3test_mgr (schedule, watchdog, tallies) + PLL reconfig
//   clk_cpu : sh3test_cpu (HS3 core + program BRAM + test MMIO), 75..112 MHz
//   clkvid  : test_videoout scoreboard (53.69 MHz, as upstream)
//
// The manager owns the CPU reset and requests PLL reconfiguration through
// o_PLL_STEP/o_PLL_GO; the write sequencer for the reconfig IP lives in the
// MiSTer top (SH3Test.sv), next to the pll_cfg instance.
//

module sh3test_cv1k #(
    parameter SIM_SPEEDUP = 0,
    parameter INIT_FILE   = "rtl/sh3test_prog.hex"
)(
    input             clk50,
    input             clk_cpu,
    input             clkvid,
    input             reset,
    input             pause,        // OSD open: freeze schedule + watchdog

    input       [3:0] HOLD_STEP,    // 0 = auto schedule, 1..10 = soak one step

    output      [3:0] pll_step,     // 0..9 -> frequency table in SH3Test.sv
    output            pll_go,       // 1-cycle pulse (clk50): start reconfig

    output            video_hsync,
    output            video_vsync,
    output            video_hblank,
    output            video_vblank,
    output            video_ce,
    output      [7:0] video_r,
    output      [7:0] video_g,
    output      [7:0] video_b
);

   // ------------------------------------------------------------ CPU side
   wire        cpu_rst_n;

   wire        kick_tgl, result_tgl, trap_tgl, seqerr_tgl;
   wire [31:0] kick_cnt, sigxor, trap_code;
   wire  [4:0] result_map;
   wire  [2:0] stream;

   sh3test_cpu #(.INIT_FILE(INIT_FILE)) u_cpu (
      .i_CLK        (clk_cpu),
      .i_ARST_n     (cpu_rst_n),
      .o_KICK_TGL   (kick_tgl),
      .o_KICK_CNT   (kick_cnt),
      .o_RESULT_TGL (result_tgl),
      .o_RESULT_MAP (result_map),
      .o_SIGXOR     (sigxor),
      .o_STREAM     (stream),
      .o_TRAP_TGL   (trap_tgl),
      .o_TRAP_CODE  (trap_code),
      .o_SEQERR_TGL (seqerr_tgl),
      .o_SEQ_CODE   (),
      .o_MMIO_WE    (),
      .o_MMIO_ADDR  (),
      .o_MMIO_DATA  ()
   );

   // free-running CPU-clock counter for the frequency meter; independent of
   // the CPU reset so the meter reads during CFG/DONE/HALT too. Bit 6
   // changes every 64 clocks (128-clock toggle period: comfortably slow for
   // the 50 MHz synchronizer at any PLL setting).
   reg [6:0] cpuctr = '0;
   always @(posedge clk_cpu) cpuctr <= cpuctr + 1'd1;

   // ------------------------------------------------------------- manager
   wire [31:0] freq_hz, loops, loops_sec, errors, mg_sigxor;
   wire  [3:0] step_no;
   wire  [7:0] time_min, time_sec;
   wire [15:0] resets, seqerrs;
   wire  [2:0] hang_stream;
   wire [39:0] err_stream;
   wire [19:0] history;
   wire        alarm, done;

   sh3test_mgr #(.SIM_SPEEDUP(SIM_SPEEDUP)) u_mgr (
      .i_CLK50       (clk50),
      .i_RST         (reset),
      .i_PAUSE       (pause),
      .i_HOLD_STEP   (HOLD_STEP),

      .i_KICK_TGL    (kick_tgl),
      .i_RESULT_TGL  (result_tgl),
      .i_RESULT_MAP  (result_map),
      .i_SIGXOR      (sigxor),
      .i_STREAM      (stream),
      .i_SEQERR_TGL  (seqerr_tgl),
      .i_CPUCTR_TGL  (cpuctr[6]),

      .o_CPU_RST_n   (cpu_rst_n),
      .o_PLL_STEP    (pll_step),
      .o_PLL_GO      (pll_go),

      .o_FREQ_HZ     (freq_hz),
      .o_STEP_NO     (step_no),
      .o_TIME_MIN    (time_min),
      .o_TIME_SEC    (time_sec),
      .o_LOOPS       (loops),
      .o_LOOPS_SEC   (loops_sec),
      .o_RESETS      (resets),
      .o_HANG_STREAM (hang_stream),
      .o_ERRORS      (errors),
      .o_SEQERRS     (seqerrs),
      .o_ERR_STREAM  (err_stream),
      .o_SIGXOR      (mg_sigxor),
      .o_HISTORY     (history),
      .o_ALARM       (alarm),
      .o_DONE        (done)
   );

   // ---------------------------------------------------------- scoreboard
   test_videoout ivideoout (
      .clkvid        (clkvid),
      .reset_1x      (reset),

      .i_FREQ_HZ     (freq_hz),
      .i_STEP_NO     (step_no),
      .i_TIME_MIN    (time_min),
      .i_TIME_SEC    (time_sec),
      .i_LOOPS       (loops),
      .i_LOOPS_SEC   (loops_sec),
      .i_RESETS      (resets),
      .i_HANG_STREAM (hang_stream),
      .i_ERRORS      (errors),
      .i_SEQERRS     (seqerrs),
      .i_ERR_STREAM  (err_stream),
      .i_SIGXOR      (mg_sigxor),
      .i_HISTORY     (history),
      .i_ALARM       (alarm),
      .i_DONE        (done),

      .video_hsync   (video_hsync),
      .video_vsync   (video_vsync),
      .video_hblank  (video_hblank),
      .video_vblank  (video_vblank),
      .video_ce      (video_ce),
      .video_r       (video_r),
      .video_g       (video_g),
      .video_b       (video_b)
   );

endmodule
