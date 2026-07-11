//
// tb_wd.sv — sh3test_mgr watchdog unit test. Drives synthetic kick toggles
// (healthy CPU), then stops them (hung CPU): expects a CPU reset pulse, a
// reset tally, and the hang-stream latch; then resumes kicks and expects no
// further resets.
//

`timescale 1ns/1ps

module tb_wd;

logic clk50 = 0;
always #10 clk50 = ~clk50;

logic rst = 1;
logic kick = 0;
logic cpuctr = 0;
always #320 cpuctr = ~cpuctr;    // ~1.5625 MHz toggle = fake 100 MHz /64

wire cpu_rst_n;

sh3test_mgr #(.SIM_SPEEDUP(1)) dut (
   .i_CLK50      (clk50),
   .i_RST        (rst),
   .i_PAUSE      (1'b0),
   .i_HOLD_STEP  (4'd7),          // hold at 102.8: no schedule end, no alarm
   .i_KICK_TGL   (kick),
   .i_RESULT_TGL (1'b0),
   .i_RESULT_MAP (5'd0),
   .i_SIGXOR     (32'd0),
   .i_STREAM     (3'd2),          // "hung in stream B"
   .i_SEQERR_TGL (1'b0),
   .i_CPUCTR_TGL (cpuctr),
   .o_CPU_RST_n  (cpu_rst_n),
   .o_PLL_STEP   (),
   .o_PLL_GO     (),
   .o_FREQ_HZ    (),
   .o_STEP_NO    (),
   .o_TIME_MIN   (),
   .o_TIME_SEC   (),
   .o_LOOPS      (),
   .o_LOOPS_SEC  (),
   .o_RESETS     (),
   .o_HANG_STREAM(),
   .o_ERRORS     (),
   .o_SEQERRS    (),
   .o_ERR_STREAM (),
   .o_SIGXOR     (),
   .o_HISTORY    (),
   .o_ALARM      (),
   .o_DONE       ()
);

int resets_seen = 0;
logic rstn_q = 1;
always @(posedge clk50) begin
   rstn_q <= cpu_rst_n;
   if (rstn_q && !cpu_rst_n && !rst && dut.state == 3'd1)   // falling edge in S_RUN
      resets_seen++;
end

task kick_for(input int cycles);
   repeat (cycles / 200) begin
      repeat (100) @(posedge clk50);
      kick = ~kick;                 // toggle every 2 us: healthy
      repeat (100) @(posedge clk50);
   end
endtask

initial begin
   repeat (10) @(posedge clk50);
   rst = 0;

   wait (cpu_rst_n);                // S_RUN reached
   $display("RUN entered t=%0t", $time);

   kick_for(60_000);                // healthy for ~1.2 ms
   if (dut.o_RESETS != 0)
      $fatal(1, "reset while kicking (o_RESETS=%0d)", dut.o_RESETS);

   // stop kicking: watchdog (SIM timeout 30000 cycles = 600 us) must fire
   repeat (100_000) @(posedge clk50);
   if (dut.o_RESETS == 0 || resets_seen == 0)
      $fatal(1, "watchdog never fired (o_RESETS=%0d seen=%0d)",
             dut.o_RESETS, resets_seen);
   if (dut.o_HANG_STREAM != 3'd2)
      $fatal(1, "hang stream not latched (got %0d)", dut.o_HANG_STREAM);
   $display("watchdog fired: resets=%0d hang=%0d t=%0t",
            dut.o_RESETS, dut.o_HANG_STREAM, $time);

   // recover: resume kicking FIRST (the wd counter is mid-flight from the
   // hang), let it re-arm, then require the count to stop growing
   kick = ~kick;
   kick_for(2_000);
   begin
      automatic logic [15:0] r0 = dut.o_RESETS;
      kick_for(60_000);
      if (dut.o_RESETS != r0)
         $fatal(1, "resets kept counting after recovery (%0d -> %0d)",
                r0, dut.o_RESETS);
   end

   $display("TB_PASS (watchdog)");
   $finish;
end

endmodule
