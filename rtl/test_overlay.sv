//
// test_overlay.sv — 8x16-font text overlay, one text row per instance.
// SystemVerilog port of gpu_overlay.vhd (MiSTerDDR3Test, (c) FPGAzumSpass /
// Robert Peip — GPLv3 derivative work). Behavior is 1:1: 10 pixel-enable
// ticks per character cell (8 font columns + 2 spacing), white background
// band, colored glyphs.
//

module test_overlay #(
   parameter int         COLS      = 10,
   parameter int         OFFSETX   = 10,
   parameter int         OFFSETY   = 60,
   parameter logic [23:0] RGB_FRONT = 24'h000000
)(
   input                     clk,
   input                     ce,

   input                     ena,             // overlay on/off

   input  int                i_pixel_out_x,
   input  int                i_pixel_out_y,

   output logic [23:0]       o_pixel_out_data = '0,
   output logic              o_pixel_out_ena  = 1'b0,

   input        [0:COLS*8-1] textstring       // char 0 in the topmost byte
);

   localparam logic        BACKGROUNDON = 1'b1;
   localparam logic [23:0] RGB_BACK     = 24'hFFFFFF;

   `include "font8x16.svh"

   logic [0:7] col = '0;          // ascending: col[0] = leftmost pixel
   logic       drawchar = 1'b0;
   logic       drawbg   = 1'b0;

   int         xchar  = 0;        // 0..COLS
   int         xpos   = 0;        // 0..7 font column
   int         xpos_1 = 0;
   int         xwait  = 0;        // 0..2 inter-character spacing

   always @(posedge clk) begin : p_overlay
      int char_v;

      if (ce) begin

         xpos_1 <= xpos;

         //----------------------------------
         // pick characters
         drawchar <= 1'b0;
         drawbg   <= 1'b0;

         if (i_pixel_out_x <= OFFSETX) begin
            xchar <= 0;
            xpos  <= 0;
            xwait <= 0;
         end
         else begin
            if (xpos < 7)          xpos  <= xpos + 1;
            else if (xwait < 2)    xwait <= xwait + 1;
            else if (xchar < COLS) begin
               xchar <= xchar + 1;
               xpos  <= 0;
               xwait <= 0;
            end
         end

         if (xchar < COLS && xwait == 0 && i_pixel_out_x > OFFSETX &&
             i_pixel_out_y >= OFFSETY && i_pixel_out_y < OFFSETY + 16) begin
            char_v   = int'(textstring[xchar*8 +: 8]) - 32;
            col      <= FONT[char_v*16 + 15 - ((i_pixel_out_y - (OFFSETY % 16)) % 16)];
            drawchar <= 1'b1;
         end

         if (xchar < COLS && i_pixel_out_x >= OFFSETX &&
             i_pixel_out_y >= OFFSETY - 1 && i_pixel_out_y < OFFSETY + 14) begin
            drawbg <= 1'b1;
         end

         //----------------------------------
         // insert overlay
         o_pixel_out_data <= RGB_BACK;
         o_pixel_out_ena  <= 1'b0;

         if (ena) begin
            if (drawchar && col[xpos_1]) begin
               o_pixel_out_data <= RGB_FRONT;
               o_pixel_out_ena  <= 1'b1;
            end
            else if (drawbg && BACKGROUNDON) begin
               o_pixel_out_ena  <= 1'b1;
            end
         end

      end
   end

endmodule
