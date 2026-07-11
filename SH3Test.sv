//============================================================================
//  SH3Test — SH-3 (HS3 core) Fmax stress suite for ikacore_CV1k
//  Derived from DDR3Test:
//  Copyright (C) 2019 Robert Peip
//
//  Port to MiSTer
//  Copyright (C) 2019 Sorgelig
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [48:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	//if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER, // Force VGA scaler
	output        VGA_DISABLE, // analog out is off

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
   output        HDMI_FREEZE,

`ifdef MISTER_FB
	// Use framebuffer in DDRAM
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	input         CLK_AUDIO, // 24.576 MHz
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	//Secondary SDRAM
	//Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

assign HDMI_FREEZE = 1'b0;

assign AUDIO_S = 0;
assign AUDIO_L = 0;
assign AUDIO_R = 0;
assign AUDIO_MIX = 0;

assign LED_USER  = 0;
assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;
assign VGA_SCALER= 0;

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;

//DDR3 unused by this test
assign DDRAM_CLK      = CLK_50M;
assign DDRAM_BURSTCNT = '0;
assign DDRAM_ADDR     = '0;
assign DDRAM_RD       = 0;
assign DDRAM_DIN      = '0;
assign DDRAM_BE       = '0;
assign DDRAM_WE       = 0;

assign VIDEO_ARX = 12'd4;
assign VIDEO_ARY = 12'd3;

///////////////////////  CLOCK/RESET  ///////////////////////////////////

wire clk_cpu;      // runtime-reconfigured 75..112 MHz CPU clock
wire clk_vid;

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_cpu),
   .reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll)
);

pll2 pll2
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_vid)
);

wire [63:0] reconfig_to_pll;
wire [63:0] reconfig_from_pll;
wire        cfg_waitrequest;
reg         cfg_write = 0;
reg   [5:0] cfg_address;
reg  [31:0] cfg_data;

wire  [3:0] pll_step;      // from sh3test_cv1k (manager)
wire        pll_go;

pll_cfg pll_cfg
(
	.mgmt_clk(CLK_50M),
	.mgmt_reset(0),
	.mgmt_waitrequest(cfg_waitrequest),
	.mgmt_read(0),
	.mgmt_readdata(),
	.mgmt_write(cfg_write),
	.mgmt_address(cfg_address),
	.mgmt_writedata(cfg_data),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll)
);

// Frequency table (refclk 50 MHz, VCO 600..896 MHz):
//   step :   0     1     2     3     4     5     6      7     8     9
//   MHz  :  75    80    85    90    95   100   102.8  105   108   112
//   M    :  12.0  12.8  13.6  14.4  15.2  16.0 16.448 16.8  17.28 17.92
// Steps 0..9 (the auto schedule) all use C0 = /8. Steps 10..13 are
// OSD-Hold-only overdrive points; /8 would need a 960..1330 MHz VCO, so
// they drop C0 instead (the manager never schedules them):
//   step :   10        11        12        13
//   MHz  :  120       133       150       166
//   VCO  :  720 (/6)  665 (/5)  750 (/5)  830 (/5)
//   M    :  14.4      13.3      15.0      16.6
// M/C counter format: bit17 = odd division, [15:8] hi count, [7:0] lo count.
// K = round(frac(M) * 2^32); the IP requires K >= 1 for integer M.
function automatic [17:0] pll_m(input [3:0] s);
   case (s)
      4'd0, 4'd1:        pll_m = 18'h00606;   // M int 12
      4'd2, 4'd11:       pll_m = 18'h20706;   // M int 13 (odd)
      4'd3, 4'd10:       pll_m = 18'h00707;   // M int 14
      4'd4, 4'd12:       pll_m = 18'h20807;   // M int 15 (odd)
      4'd5, 4'd6, 4'd7,
      4'd13:             pll_m = 18'h00808;   // M int 16
      default:           pll_m = 18'h20908;   // M int 17 (odd)
   endcase
endfunction

function automatic [31:0] pll_k(input [3:0] s);
   case (s)
      4'd0:    pll_k = 32'h00000001;   //  75.0 : .0
      4'd1:    pll_k = 32'hCCCCCCCD;   //  80.0 : .8
      4'd2:    pll_k = 32'h9999999A;   //  85.0 : .6
      4'd3:    pll_k = 32'h66666666;   //  90.0 : .4
      4'd4:    pll_k = 32'h33333333;   //  95.0 : .2
      4'd5:    pll_k = 32'h00000001;   // 100.0 : .0
      4'd6:    pll_k = 32'h72B020C5;   // 102.8 : .448
      4'd7:    pll_k = 32'hCCCCCCCD;   // 105.0 : .8
      4'd8:    pll_k = 32'h47AE147B;   // 108.0 : .28
      4'd9:    pll_k = 32'hEB851EB8;   // 112.0 : .92
      4'd10:   pll_k = 32'h66666666;   // 120.0 : .4
      4'd11:   pll_k = 32'h4CCCCCCD;   // 133.0 : .3
      4'd12:   pll_k = 32'h00000001;   // 150.0 : .0
      default: pll_k = 32'h9999999A;   // 166.0 : .6
   endcase
endfunction

function automatic [17:0] pll_c(input [3:0] s);
   case (s)
      4'd10:                pll_c = 18'h00303;   // /6
      4'd11, 4'd12, 4'd13:  pll_c = 18'h20302;   // /5 (odd)
      default:              pll_c = 18'h00404;   // /8 (whole auto schedule)
   endcase
endfunction

// Reconfig write sequencer (same shape as upstream); triggered by the
// manager's pll_go pulse, which is in this same 50 MHz domain.
always @(posedge CLK_50M) begin : cfg_block
	reg  [4:0] state = 0;
   reg  [3:0] step_q = 0;

   if (pll_go) begin
      step_q <= pll_step;
      state  <= 1;
   end

	cfg_write <= 0;

	if(!cfg_waitrequest) begin
		if(state && !pll_go) state<=state+1'd1;
		case(state)
			1: begin
					cfg_address <= 0;          // waitrequest mode
					cfg_data <= 0;
					cfg_write <= 1;
				end
         3: begin
					cfg_address <= 4;          // M counter
               cfg_data <= {14'd0, pll_m(step_q)};
					cfg_write <= 1;
				end
         5: begin
					cfg_address <= 5;          // C0 counter (per-step, see pll_c)
               cfg_data <= {14'd0, pll_c(step_q)};
					cfg_write <= 1;
				end
			7: begin
					cfg_address <= 7;          // K fractional
               cfg_data <= pll_k(step_q);
					cfg_write <= 1;
				end
			9: begin
					cfg_address <= 2;          // start reconfig
					cfg_data <= 0;
					cfg_write <= 1;
				end
		endcase
	end
end

wire reset_or = RESET | buttons[1] | status[0];

////////////////////////////  HPS I/O  //////////////////////////////////

// Status Bit Map: (0..31 => "O", 32..63 => "o")
// 0         1         2         3          4         5         6          7         8         9
// 01234567890123456789012345678901 23456789012345678901234567890123 45678901234567890123456789012345
// 0123456789ABCDEFGHIJKLMNOPQRSTUV 0123456789ABCDEFGHIJKLMNOPQRSTUV
//

`include "build_id.v"
parameter CONF_STR = {
	"SH3TEST;;",
   "O[13:10],Hold Step,Auto,75,80,85,90,95,100,102.8,105,108,112,120,133,150,166;",
	"R0,Reset;",
	"V,v",`BUILD_DATE
};

wire  [1:0] buttons;
wire [127:0] status;
wire        forced_scandoubler;

wire [19:0] joy;

wire [10:0] ps2_key;

wire [127:0] status_in = status;

wire bk_pending;
wire DIRECT_VIDEO;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(CLK_50M),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),

	.buttons(buttons),
	.forced_scandoubler(forced_scandoubler),

	.joystick_0(joy),
	.ps2_key(ps2_key),

	.status(status),
	.status_in(status_in),
	.status_set(0),
	.status_menumask(0),
	.info_req(0),
	.info(0),

   .direct_video(DIRECT_VIDEO)
);

////////////////////////////  SYSTEM  ///////////////////////////////////

wire HBlank;
wire VBlank;

sh3test_cv1k #(.INIT_FILE("rtl/sh3test_prog.hex"))
sh3test
(
   .clk50          (CLK_50M),
   .clk_cpu        (clk_cpu),
   .clkvid         (clk_vid),
   .reset          (reset_or),
   .pause          (OSD_STATUS),

   .HOLD_STEP      (status[13:10]),

   .pll_step       (pll_step),
   .pll_go         (pll_go),

   // Video
   .video_hsync    (VGA_HS),
   .video_vsync    (VGA_VS),
   .video_hblank   (HBlank),
   .video_vblank   (VBlank),
   .video_ce       (CE_PIXEL),
   .video_r        (VGA_R),
   .video_g        (VGA_G),
   .video_b        (VGA_B)
);

assign CLK_VIDEO = clk_vid;
assign VGA_DE = ~(HBlank | VBlank);
assign VGA_F1 = 0;
assign VGA_SL = 0;
assign VGA_DISABLE = 0;

endmodule
