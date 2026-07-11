//
// test_videoout.sv — video timing generator + SH-3 stress-test scoreboard.
// Timing core is the SystemVerilog port of gpu_videoout.vhd (MiSTerDDR3Test,
// (c) FPGAzumSpass / Robert Peip — GPLv3 derivative work); the overlay rows
// are rewritten for the SH3Test scoreboard:
//
//   Frequency:  102.8 MHz  07/10     measured CPU clock + schedule step
//   Time:       09:42                remaining (auto) / elapsed (hold)
//   Loops:      0001234567           watchdog kicks this step
//   Loops / s:  0000020560
//   Resets:     0000000003 (B)       watchdog resets + hung stream
//   Errors:     0000000012           bad iterations this step
//   Err ABCDE:  00 0C 00 00 05       per-stream error counts (hex, saturating)
//   Sig XOR:    00420000             last bad-signature XOR vs golden
//   History:    PPPPPF>...           one char per step
//
// i_ALARM paints the background blue (B=255): the 102.8 MHz halt rule.
//
// All scoreboard inputs are quasi-static values from the 50 MHz manager
// (updated at most once per second / per event), consumed unsynchronized in
// the video domain — the same CDC convention as upstream.
//

module test_videoout
(
   input             clkvid,
   input             reset_1x,

   input      [31:0] i_FREQ_HZ,
   input       [3:0] i_STEP_NO,      // 1..10
   input       [7:0] i_TIME_MIN,
   input       [7:0] i_TIME_SEC,
   input      [31:0] i_LOOPS,
   input      [31:0] i_LOOPS_SEC,
   input      [15:0] i_RESETS,
   input       [2:0] i_HANG_STREAM,  // 0=none, 1..5 = A..E
   input      [31:0] i_ERRORS,
   input      [15:0] i_SEQERRS,      // execution-proof violations (canary/
                                     // protocol/kick-monotonicity)
   input      [39:0] i_ERR_STREAM,   // [39:32]=A ... [7:0]=E
   input      [31:0] i_SIGXOR,
   input      [19:0] i_HISTORY,      // 2b/step: 0 notrun/1 pass/2 fail/3 running
   input             i_ALARM,
   input             i_DONE,

   output logic      video_hsync  = 1'b0,
   output logic      video_vsync  = 1'b0,
   output logic      video_hblank = 1'b0,
   output            video_vblank,
   output            video_ce,
   output logic [7:0] video_r,
   output logic [7:0] video_g,
   output logic [7:0] video_b
);

   // ------------------------------------------------------------ helpers
   // BCD/hex nibbles -> ASCII characters ('0'-'9', 'A'-'F')
   function automatic [79:0] conv40(input [39:0] a);
      for (int i = 0; i < 10; i++) begin
         if (a[i*4 +: 4] < 4'd10) conv40[i*8 +: 8] = {4'h0, a[i*4 +: 4]} + 8'h30;
         else                     conv40[i*8 +: 8] = {4'h0, a[i*4 +: 4]} + 8'h37;
      end
   endfunction

   function automatic [63:0] conv32(input [31:0] a);
      for (int i = 0; i < 8; i++) begin
         if (a[i*4 +: 4] < 4'd10) conv32[i*8 +: 8] = {4'h0, a[i*4 +: 4]} + 8'h30;
         else                     conv32[i*8 +: 8] = {4'h0, a[i*4 +: 4]} + 8'h37;
      end
   endfunction

   // two-digit hex of a byte (Quartus cannot part-select a function call)
   function automatic [15:0] hex2(input [7:0] v);
      reg [63:0] c;
      begin
         c = conv32({24'd0, v});
         hex2 = c[15:0];
      end
   endfunction

   // 0..99 binary -> two ASCII digits {tens, ones}
   function automatic [15:0] two_dig(input [7:0] v);
      logic [3:0] t;
      logic [7:0] r;
      t = 4'd0;
      r = v;
      for (int i = 0; i < 9; i++) begin
         if (r >= 8'd10) begin
            r = r - 8'd10;
            t = t + 4'd1;
         end
      end
      two_dig = {8'h30 + {4'd0, t}, 8'h30 + r};
   endfunction

   logic reset_1 = 1'b0, reset_2 = 1'b0, reset = 1'b0;

   // overlay
   logic [23:0] overlay_data;
   logic        overlay_on;

   localparam int OVERLAY_COUNT = 20;
   logic [23:0] overlay_array [0:OVERLAY_COUNT-1];
   logic [OVERLAY_COUNT-1:0] overlay_ena;

   // timing
   int   nextHCount = 0;         // 0..4095
   int   vpos       = 0;         // 0..511
   int   vdisp      = 0;
   int   lineIn     = 0;
   logic inVsync    = 1'b0;

   int   htotal        = 3413;   // 3406..3413
   int   vtotal        = 263;    // 262..314
   int   vDisplayStart = 0;
   int   vDisplayEnd   = 239;
   int   vDisplayCnt   = 0;
   int   vDisplayMax   = 239;

   logic newLineTrigger = 1'b0;

   // output
   typedef enum int {
      WAITNEWLINE,
      WAITHBLANKEND,
      WAITHBLANKENDVSYNC,
      WAITINVSYNC,
      DRAW
   } tState;
   tState state = WAITNEWLINE;

   logic vid_ce = 1'b0;

   int   clkDiv = 4;             // 4..10
   int   clkCnt = 0;
   int   xCount = 256;           // 0..1023

   int   xpos = 0;
   int   ypos = 0;

   int   hsync_start = 0;
   int   hsync_end   = 0;

   logic [11:0] hCropCount  = '0;
   logic [1:0]  hCropPixels = '0;

   // --------------------------------------------------- value formatting
   logic [39:0] freqBCD, loopsBCD, loopsSecBCD, resetsBCD, errorsBCD, seqerrsBCD;

   // +50 kHz rounding so the displayed 100 kHz digit is stable against the
   // 64-cycle meter granularity
   to_bcd ibcd_freq     (clkvid, i_FREQ_HZ + 32'd50_000,  freqBCD);
   to_bcd ibcd_loops    (clkvid, i_LOOPS,                 loopsBCD);
   to_bcd ibcd_loopssec (clkvid, i_LOOPS_SEC,             loopsSecBCD);
   to_bcd ibcd_resets   (clkvid, {16'd0, i_RESETS},       resetsBCD);
   to_bcd ibcd_errors   (clkvid, i_ERRORS,                errorsBCD);
   to_bcd ibcd_seqerrs  (clkvid, {16'd0, i_SEQERRS},      seqerrsBCD);

   // "102.8 MHz  07/10" — BCD digits 8..6 = integer MHz, digit 5 = 100 kHz
   logic [127:0] freq_txt;
   always_comb begin
      freq_txt[127:120] = (freqBCD[35:32] == 4'd0) ? 8'h20
                                                   : (8'h30 + {4'd0, freqBCD[35:32]});
      freq_txt[119:112] = 8'h30 + {4'd0, freqBCD[31:28]};
      freq_txt[111:104] = 8'h30 + {4'd0, freqBCD[27:24]};
      freq_txt[103: 96] = ".";
      freq_txt[ 95: 88] = 8'h30 + {4'd0, freqBCD[23:20]};
      freq_txt[ 87: 80] = " ";
      freq_txt[ 79: 72] = "M";
      freq_txt[ 71: 64] = "H";
      freq_txt[ 63: 56] = "z";
      freq_txt[ 55: 48] = " ";
      freq_txt[ 47: 40] = " ";
      freq_txt[ 39: 24] = two_dig({4'd0, i_STEP_NO});
      freq_txt[ 23: 16] = "/";
      freq_txt[ 15:  8] = "1";
      freq_txt[  7:  0] = "0";
   end

   // "09:42"
   logic [39:0] time_txt;
   always_comb begin
      time_txt[39:24] = two_dig(i_TIME_MIN);
      time_txt[23:16] = ":";
      time_txt[15: 0] = two_dig(i_TIME_SEC);
   end

   // "0000000003 (B)"
   logic [111:0] resets_txt;
   always_comb begin
      resets_txt[111:32] = conv40(resetsBCD);
      resets_txt[ 31:24] = " ";
      resets_txt[ 23:16] = "(";
      resets_txt[ 15: 8] = (i_HANG_STREAM == 3'd0 || i_HANG_STREAM > 3'd5)
                           ? 8'h2D : (8'h40 + {5'd0, i_HANG_STREAM});
      resets_txt[  7: 0] = ")";
   end

   // "00 0C 00 00 05" — per-stream error counts A B C D E
   logic [111:0] errstr_txt;
   always_comb begin
      errstr_txt = '0;
      for (int i = 0; i < 5; i++) begin
         // stream i takes char slots 3i (hi nibble) and 3i+1; slot 3i+2 is the
         // separator. Char slot j starts at bit (13-j)*8, so the 16-bit pair
         // starts at the LOW char's slot, 3i+1.
         errstr_txt[(12 - 3*i)*8 +: 16] = hex2(i_ERR_STREAM[(4-i)*8 +: 8]);
         if (i != 4) errstr_txt[(11 - 3*i)*8 +: 8] = " ";
      end
   end

   // "PPPPPF>..." — schedule history
   logic [79:0] hist_txt;
   always_comb begin
      for (int i = 0; i < 10; i++) begin
         case (i_HISTORY[i*2 +: 2])
            2'd1:    hist_txt[(9-i)*8 +: 8] = "P";
            2'd2:    hist_txt[(9-i)*8 +: 8] = "F";
            2'd3:    hist_txt[(9-i)*8 +: 8] = ">";
            default: hist_txt[(9-i)*8 +: 8] = ".";
         endcase
      end
   end

   // ------------------------------------------------------------- labels
   test_overlay #(10,  10,  30, 24'h000000) ilabFreq
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[0],  overlay_ena[0],  "Frequency:");
   test_overlay #(10,  10,  50, 24'h000000) ilabTime
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[1],  overlay_ena[1],  "Time:     ");
   test_overlay #(10,  10,  70, 24'h000000) ilabLoops
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[2],  overlay_ena[2],  "Loops:    ");
   test_overlay #(10,  10,  90, 24'h000000) ilabLoopsSec
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[3],  overlay_ena[3],  "Loops / s:");
   test_overlay #(10,  10, 110, 24'h000000) ilabResets
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[4],  overlay_ena[4],  "Resets:   ");
   test_overlay #(10,  10, 130, 24'h000000) ilabErrors
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[5],  overlay_ena[5],  "Errors:   ");
   test_overlay #(10,  10, 150, 24'h000000) ilabErrStr
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[6],  overlay_ena[6],  "Err ABCDE:");
   test_overlay #(10,  10, 170, 24'h000000) ilabSigXor
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[7],  overlay_ena[7],  "Sig XOR:  ");
   test_overlay #(10,  10, 190, 24'h000000) ilabHistory
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[8],  overlay_ena[8],  "History:  ");
   test_overlay #(10,  10, 210, 24'h000000) ilabSeqErr
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[18], overlay_ena[18], "Seq err:  ");

   // ------------------------------------------------------------- values
   test_overlay #(16, 120,  30, 24'h0000FF) ivalFreq
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[9],  overlay_ena[9],  freq_txt);
   test_overlay #( 5, 120,  50, 24'h0000FF) ivalTime
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[10], overlay_ena[10], time_txt);
   test_overlay #(10, 120,  70, 24'h0000FF) ivalLoops
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[11], overlay_ena[11], conv40(loopsBCD));
   test_overlay #(10, 120,  90, 24'h0000FF) ivalLoopsSec
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[12], overlay_ena[12], conv40(loopsSecBCD));
   test_overlay #(14, 120, 110, 24'h0000FF) ivalResets
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[13], overlay_ena[13], resets_txt);
   test_overlay #(10, 120, 130, 24'h0000FF) ivalErrors
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[14], overlay_ena[14], conv40(errorsBCD));
   test_overlay #(14, 120, 150, 24'h0000FF) ivalErrStr
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[15], overlay_ena[15], errstr_txt);
   test_overlay #( 8, 120, 170, 24'h0000FF) ivalSigXor
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[16], overlay_ena[16], conv32(i_SIGXOR));
   test_overlay #(10, 120, 190, 24'h0000FF) ivalHistory
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[17], overlay_ena[17], hist_txt);
   test_overlay #(10, 120, 210, 24'h0000FF) ivalSeqErr
      (clkvid, vid_ce, 1'b1, xpos, ypos, overlay_array[19], overlay_ena[19], conv40(seqerrsBCD));


   assign video_ce = vid_ce;

   always_comb begin
      overlay_data = '0;
      overlay_on   = |overlay_ena;
      for (int i = 0; i < OVERLAY_COUNT; i++) begin
         if (overlay_ena[i]) overlay_data = overlay_array[i];
      end
   end

   // background: black, or blue (B=255) when the halt rule fired
   wire [23:0] bg_color = i_ALARM ? 24'hFF0000 : 24'h000000;

   // --------------------------------------------------- line/frame timing
   always @(posedge clkvid) begin : p_timing
      logic isVsync;
      int   vdispNew;

      reset_1 <= reset_1x;
      reset_2 <= reset_1;
      reset   <= reset_2;

      vDisplayMax   <= 240;
      vDisplayStart <= 0;
      vDisplayEnd   <= 239;

      newLineTrigger <= 1'b0;

      if (reset) begin

         nextHCount <= htotal;
         vpos       <= 0;
         inVsync    <= 1'b0;
         vdisp      <= 0;

      end
      else begin

         htotal <= 3413;
         vtotal <= 263;

         vdispNew = vdisp + 1;

         // gpu timing count
         if (nextHCount > 1) begin
            nextHCount <= nextHCount - 1;
         end
         else begin

            nextHCount <= htotal;

            vpos <= vpos + 1;
            if (vpos + 1 == vtotal) vpos <= 0;

            if (video_vsync) vdispNew = 0;

            // synthesis translate_off
            if (vdispNew >= vtotal) vdispNew = 0; // fix simulation issues with rollover
            // synthesis translate_on

            vdisp <= vdispNew;

            if (vDisplayCnt < vDisplayMax) vDisplayCnt <= vDisplayCnt + 1;

            isVsync = inVsync;
            if (vdispNew == vDisplayStart) begin
               isVsync     = 1'b0;
               vDisplayCnt <= 0;
            end
            else if (vdispNew == vDisplayEnd || vdispNew == 0) begin
               isVsync = 1'b1;
            end

            if (!isVsync) lineIn <= vdispNew - vDisplayStart;

            if (isVsync != inVsync) inVsync <= isVsync;

            newLineTrigger <= 1'b1;
            vdispNew = vdispNew + 1;

         end

      end
   end

   assign video_vblank = (vDisplayCnt < 240) ? inVsync : 1'b1;

   // ------------------------------------------------------ pixel pipeline
   always @(posedge clkvid) begin : p_draw
      int vsync_hstart;
      int vsync_vstart;

      vid_ce <= 1'b0;

      clkDiv <= 8;

      if (reset) begin

         state        <= WAITNEWLINE;
         clkCnt       <= 0;
         video_hblank <= 1'b1;
         video_vsync  <= 1'b0;
         ypos         <= 0;

      end
      else begin

         if (clkCnt < (clkDiv - 1)) begin
            clkCnt <= clkCnt + 1;
         end
         else begin
            clkCnt <= 0;
            vid_ce <= 1'b1;
         end

         if (newLineTrigger) clkCnt <= 0;   // clock divider reset at end of line

         hCropCount <= hCropCount + 1'd1;

         case (state)

            WAITNEWLINE: begin
               video_hblank <= 1'b1;

               if (lineIn != ypos) begin
                  state <= WAITHBLANKEND;
                  xpos  <= 0;
                  ypos  <= lineIn;

                  xCount      <= 0;
                  hCropCount  <= '0;
                  hCropPixels <= '0;
               end
               else if (newLineTrigger) begin
                  state       <= WAITHBLANKENDVSYNC;
                  hCropCount  <= '0;
                  hCropPixels <= '0;
               end
            end

            WAITHBLANKEND, WAITHBLANKENDVSYNC: begin
               if (clkCnt >= (clkDiv - 1)) begin
                  if (hCropCount >= 12'h260) begin
                     if (state == WAITHBLANKENDVSYNC) state <= WAITINVSYNC;
                     else                             state <= DRAW;
                  end
               end
            end

            WAITINVSYNC: begin
               if (clkCnt >= (clkDiv - 1)) begin
                  hCropPixels <= hCropPixels + 1'd1;
                  if ((hCropCount + 1) >= 12'hC70) begin
                     if ((hCropPixels + 1'd1) == 2'd0) state <= WAITNEWLINE;
                  end
               end
               if ((nextHCount == 32 + 3413/2) && (vpos == 242) && (vtotal == 262)) begin
                  state <= WAITHBLANKENDVSYNC;   // (interlace leftover, vtotal is 263)
               end
            end

            DRAW: begin
               if (clkCnt >= (clkDiv - 1)) begin
                  video_hblank <= 1'b0;
                  video_r      <= overlay_on ? overlay_data[ 7: 0] : bg_color[ 7: 0];
                  video_g      <= overlay_on ? overlay_data[15: 8] : bg_color[15: 8];
                  video_b      <= overlay_on ? overlay_data[23:16] : bg_color[23:16];

                  if (xCount < 1023) xCount <= xCount + 1;

                  xpos <= xpos + 1;

                  hCropPixels <= hCropPixels + 1'd1;
                  if ((hCropCount + 1) >= 12'hC70) begin
                     if ((hCropPixels + 1'd1) == 2'd0) state <= WAITNEWLINE;
                  end
               end
            end

            default: state <= WAITNEWLINE;

         endcase

         hsync_start <= 32;

         if (nextHCount == hsync_start) begin
            hsync_end   <= 252;
            video_hsync <= 1'b1;
         end

         if (hsync_end > 0) begin
            hsync_end <= hsync_end - 1;
            if (hsync_end == 1) video_hsync <= 1'b0;
         end

         vsync_hstart = hsync_start;
         vsync_vstart = 242;

         if (nextHCount == vsync_hstart) begin
            if (vpos == vsync_vstart)     video_vsync <= 1'b1;
            if (vpos == vsync_vstart + 3) video_vsync <= 1'b0;
         end

      end
   end

endmodule
