//
// tb_engine.sv — full-engine simulation of the SH-3 stress suite
// (sh3test_cv1k with SIM_SPEEDUP=1: 1 "second" = 1 ms, steps 2..4 s).
//
//   +hex=PATH   program image (default ../rtl/sh3test_prog.hex)
//   +alarm      expect the 102.8 MHz halt rule to fire (bad-golden image)
//               instead of a clean full run
//
// Clocks: 50 MHz manager, 100 MHz CPU (fixed — the real PLL is not
// simulated; the cfg sequencer lives in the MiSTer top, outside this DUT),
// 53.69 MHz video.
//

`timescale 1ns/1ps

module tb_engine #(
   parameter TB_HEX = "../rtl/sh3test_prog.hex"
);

logic clk50 = 0, clkcpu = 0, clkvid = 0;
always #10.0 clk50  = ~clk50;
always #5.0  clkcpu = ~clkcpu;
always #9.3  clkvid = ~clkvid;

logic reset = 1;
bit expect_alarm = 0;

wire [3:0] pll_step;
wire       pll_go;

sh3test_cv1k #(.SIM_SPEEDUP(1), .INIT_FILE(TB_HEX)) dut (
   .clk50        (clk50),
   .clk_cpu      (clkcpu),
   .clkvid       (clkvid),
   .reset        (reset),
   .pause        (1'b0),
   .HOLD_STEP    (4'd0),
   .pll_step     (pll_step),
   .pll_go       (pll_go),
   .video_hsync  (),
   .video_vsync  (),
   .video_hblank (),
   .video_vblank (),
   .video_ce     (),
   .video_r      (),
   .video_g      (),
   .video_b      ()
);

int gos = 0;
logic [3:0] last_step = 4'hF;

always @(posedge clk50) begin
   if (pll_go) begin
      gos <= gos + 1;
      $display("PLL_GO step=%0d t=%0t", pll_step, $time);
   end
   if (dut.u_mgr.o_STEP_NO != last_step) begin
      last_step <= dut.u_mgr.o_STEP_NO;
      $display("STEP %0d/10 t=%0t", dut.u_mgr.o_STEP_NO, $time);
   end
end

initial begin
   expect_alarm = $test$plusargs("alarm") != 0;

   repeat (20) @(posedge clk50);
   reset = 0;

   fork
      begin : watch_end
         if (expect_alarm) begin
            wait (dut.u_mgr.o_ALARM);
            $display("ALARM raised at step %0d, errors=%0d seqerrs=%0d t=%0t",
                     dut.u_mgr.o_STEP_NO, dut.u_mgr.o_ERRORS,
                     dut.u_mgr.o_SEQERRS, $time);
            if (dut.u_mgr.o_STEP_NO != 4'd7)   // step index 6 -> STEP_NO 7
               $display("FATAL: alarm fired at the wrong step");
            else if (dut.u_mgr.o_ERR_STREAM == 40'd0)
               $display("FATAL: alarm without per-stream error counts");
            else
               $display("TB_PASS (alarm path)");
            $finish;
         end
         else begin
            wait (dut.u_mgr.o_DONE);
            $display("DONE t=%0t loops(last step)=%0d freq=%0d history=%05x seqerrs=%0d",
                     $time, dut.u_mgr.o_LOOPS, dut.u_mgr.o_FREQ_HZ,
                     dut.u_mgr.o_HISTORY, dut.u_mgr.o_SEQERRS);
            if (dut.u_mgr.o_HISTORY != 20'h55555)
               $display("FATAL: history not all-pass: %05x", dut.u_mgr.o_HISTORY);
            else if (gos != 10)
               $display("FATAL: expected 10 reconfig requests, got %0d", gos);
            else if (dut.u_mgr.o_ALARM)
               $display("FATAL: unexpected alarm");
            else if (dut.u_mgr.o_SEQERRS != 16'd0)
               $display("FATAL: sequence errors on a clean run: %0d",
                        dut.u_mgr.o_SEQERRS);
            else
               $display("TB_PASS (clean full run)");
            $finish;
         end
      end
      begin : timeout
         #80ms;
         $display("FATAL: engine tb timeout (step %0d, alarm=%0d, done=%0d)",
                  dut.u_mgr.o_STEP_NO, dut.u_mgr.o_ALARM, dut.u_mgr.o_DONE);
         $finish;
      end
   join_any
end

// sanity: measured frequency should read ~100 MHz once running
always @(posedge clk50) begin
   if (!reset && dut.u_mgr.o_FREQ_HZ != 0) begin
      if (dut.u_mgr.o_FREQ_HZ < 32'd90_000_000 ||
          dut.u_mgr.o_FREQ_HZ > 32'd110_000_000) begin
         $display("FATAL: frequency meter reads %0d, expected ~100 MHz",
                  dut.u_mgr.o_FREQ_HZ);
         $finish;
      end
   end
end

endmodule
