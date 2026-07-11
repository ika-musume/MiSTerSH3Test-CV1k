//
// sh3test_cpu.sv — full HS3 chip top (pipeline + cache + BSC + fabric) with
// a pin-level external-bus slave, for the SH-3 Fmax stress suite. The whole
// HS3 is instantiated (from the local HS3/ copy of the IP) so the fitted
// footprint — and therefore the slack — matches the real ikacore_CV1k
// integration; INTC/DMAC/TMU/RTC/ports are present but idle.
//
// External bus (the BSC's physical pins, like the board):
//   CS0 (area 0, 32-bit via MD4=MD3=1) : 16 KB program/data BRAM
//       boot fetch uncached via P2 0xA0000000, main loop cached via P1
//   CS4 (area 4, phys 0x1000_00xx)     : test MMIO, uncached via P2
//       0xB000_00xx (see sw/make_prog.py header for the register map)
//
// The generic memory port is tied off exactly like the board top
// (i_MEM_READY=0, i_MEM_RSP_VALID=0): every access completes as a
// WAIT_n-timed physical bus cycle. WCR2 resets to maximum waits, so the
// registered BRAM read is settled long before the BSC samples i_D_I.
//
// Everything here lives in the variable CPU clock domain; the manager
// consumes the o_*_TGL toggles and quasi-static payloads through 2FF
// synchronizers on its side.
//

module sh3test_cpu #(
    parameter INIT_FILE = "sh3test_prog.hex"
)(
    input  wire         i_CLK,          // variable CPU clock
    input  wire         i_ARST_n,       // async assert; deassert synced here

    /* test events, CPU clock domain */
    output reg          o_KICK_TGL,     // flips on every KICK write
    output reg  [31:0]  o_KICK_CNT,     // last KICK payload (iteration count)
    output reg          o_RESULT_TGL,   // flips on every RESULT write
    output reg   [4:0]  o_RESULT_MAP,   // fail bitmap: bit4=A .. bit0=E
    output reg  [31:0]  o_SIGXOR,       // last nonzero SIGXOR payload
    output reg   [2:0]  o_STREAM,       // stream now executing (1..5, 0=none)
    output reg          o_TRAP_TGL,     // flips on unexpected exception
    output reg  [31:0]  o_TRAP_CODE,    // EXPEVT reported by the trap catcher

    /* iteration sequence checker: flags kicks whose window did not contain
       the full expected MMIO protocol (anti-false-negative: a control-flow
       fault that skips streams/checks can no longer look like a clean loop) */
    output reg          o_SEQERR_TGL,   // flips on a bad kick window
    output reg   [4:0]  o_SEQ_CODE,     // {kick!=+1, canary bad, RESULT missing,
                                        //  SIG[n] missing, STREAM marker missing}

    /* raw MMIO write tap (simulation/debug: SIG captures ride this) */
    output reg          o_MMIO_WE,
    output reg   [7:0]  o_MMIO_ADDR,
    output reg  [31:0]  o_MMIO_DATA
);

///////////////////////////////////////////////////////////
//////  Reset synchronizer (async assert, sync deassert)
////

reg     [1:0]   rst_sync = 2'b00;
always @(posedge i_CLK or negedge i_ARST_n) begin
    if(!i_ARST_n) rst_sync <= 2'b00;
    else          rst_sync <= {rst_sync[0], 1'b1};
end
wire    rst_n = rst_sync[1];


///////////////////////////////////////////////////////////
//////  HS3 chip top
////

wire    [25:0]  bus_a;
wire    [31:0]  bus_do;
wire            cs0_n, cs4_n;
wire            rd_n;
wire    [3:0]   we_n;
reg     [31:0]  bus_di;

HS3 #(
    .RESET_PC               (32'hA000_0000              ),
    .BIG_ENDIAN             (1'b1                       ),
    .DISABLE_CEN            (1'b1                       )
) u_hs3 (
    .i_POR_n                (rst_n                      ),
    .i_RST_n                (rst_n                      ),
    .i_CLK                  (i_CLK                      ),
    .i_CEN                  (1'b1                       ),
    .o_CKIO                 (                           ),
    .o_CKIO_PCEN            (                           ),
    .o_CKIO_NCEN            (                           ),
    .i_EXTAL2               (1'b0                       ),  //RTC crystal: unused

    /* BSC physical pins */
    .o_A                    (bus_a                      ),
    .o_D_O                  (bus_do                     ),
    .o_D_OE                 (                           ),
    .i_D_I                  (bus_di                     ),
    .o_BS_n                 (                           ),
    .o_CS0_n                (cs0_n                      ),
    .o_CS2_n                (                           ),
    .o_CS3_n                (                           ),
    .o_CS4_n                (cs4_n                      ),
    .o_CS5_n                (                           ),
    .o_CS6_n                (                           ),
    .o_RD_WR                (                           ),
    .o_RAS3L_n              (                           ),
    .o_RAS3U_n              (                           ),
    .o_CASL_n               (                           ),
    .o_CASU_n               (                           ),
    .o_WE_n                 (we_n                       ),
    .o_RD_n                 (rd_n                       ),
    .i_WAIT_n               (1'b1                       ),  //no external waits
    .i_MD4                  (1'b1                       ),  //area 0 = 32-bit
    .i_MD3                  (1'b1                       ),  //(table 10.4: 11)
    .o_CKE                  (                           ),
    .i_BREQ_n               (1'b1                       ),  //no foreign bus master
    .o_BACK_n               (                           ),
    .o_BUS_OE               (                           ),
    .o_RASCAS_OE            (                           ),
    .o_A_PU                 (                           ),
    .o_D_PU                 (                           ),
    .o_IRQOUT_n             (                           ),

    /* generic memory port: tied off like the board top — every access
       completes on the WAIT_n-timed physical bus cycle */
    .o_MEM_REQ              (                           ),
    .o_MEM_WRITE            (                           ),
    .o_MEM_BURST            (                           ),
    .o_MEM_SIZE             (                           ),
    .o_MEM_ADDR             (                           ),
    .o_MEM_CS_n             (                           ),
    .o_MEM_WSTRB            (                           ),
    .i_MEM_READY            (1'b0                       ),
    .i_MEM_RSP_VALID        (1'b0                       ),
    .i_MEM_FAULT            (1'b0                       ),
    .o_MEM_RSP_READY        (                           ),

    /* interrupts idle: NMI high; IRL3-0 = PTH 1111 = level 15 = no request */
    .i_NMI                  (1'b1                       ),

    /* port pads: inputs high (all shared IRQ/DREQ/PINT functions deasserted) */
    .i_PTA_I                (8'hFF                      ),
    .o_PTA_O                (), .o_PTA_OE               (), .o_PTA_PU (),
    .i_PTB_I                (8'hFF                      ),
    .o_PTB_O                (), .o_PTB_OE               (), .o_PTB_PU (),
    .i_PTC_I                (8'hFF                      ),
    .o_PTC_O                (), .o_PTC_OE               (), .o_PTC_PU (),
    .i_PTD_I                (8'hFF                      ),
    .o_PTD_O                (), .o_PTD_OE               (), .o_PTD_PU (),
    .i_PTE_I                (8'hFF                      ),
    .o_PTE_O                (), .o_PTE_OE               (), .o_PTE_PU (),
    .i_PTF_I                (8'hFF                      ),
    .o_PTF_PU               (),
    .i_PTG_I                (8'hFF                      ),
    .o_PTG_PU               (),
    .i_PTH_I                (8'hFF                      ),
    .o_PTH_O                (), .o_PTH_OE               (), .o_PTH_PU (),
    .i_PTJ_I                (8'hFF                      ),
    .o_PTJ_O                (), .o_PTJ_OE               (), .o_PTJ_PU (),
    .i_PTK_I                (8'hFF                      ),
    .o_PTK_O                (), .o_PTK_OE               (), .o_PTK_PU (),
    .i_PTL_I                (8'hFF                      ),
    .i_SCPT_I               (8'hFF                      ),
    .o_SCPT_O               (), .o_SCPT_OE              (), .o_SCPT_PU()
);


///////////////////////////////////////////////////////////
//////  CS0: 16 KB program/data BRAM (SRAM-style, 32-bit)
////

/*
    Ordinary-memory read cycle: address/CS stable over T1 + nTw + T2, data
    sampled at the end (WCR2 resets to maximum waits) — the 1-cycle
    registered read below settles with a wide margin. Write cycle: WE_n
    lanes strobed low with address/data held, so the per-cycle lane write
    repeats identically while the strobe is low (idempotent). WE_n[i]
    strobes D[8i+7:8i] (sz_lanes, big-endian byte 0 = D31:24 — matches the
    hex image packing).
*/

/*
    Byte lanes are a PACKED dimension, not a part-select of an unpacked word:
    Quartus 17 refuses to infer a byte-enabled RAM from `mem[a][31:24] <= ..`
    and builds 131072 flops instead. `mem[a][3] <= ..` on a [3:0][7:0] array
    is the inferrable form; the $readmemh below then becomes the altsyncram
    INIT_FILE. Byte 3 is D[31:24] (big-endian byte 0), matching WE_n[3].

    The hex image has exactly one 8-digit line per word (4096 = the array
    depth), unused words padded with zeros — sw/make_prog.py guarantees this.
*/

reg     [3:0][7:0]  mem [0:4095];
initial $readmemh(INIT_FILE, mem);

wire            ram_wr  = !cs0_n && (we_n != 4'hF);
wire    [11:0]  ram_a   = bus_a[13:2];
reg     [31:0]  ram_q;

always @(posedge i_CLK) begin
    if(ram_wr) begin
        if(!we_n[3]) mem[ram_a][3] <= bus_do[31:24];
        if(!we_n[2]) mem[ram_a][2] <= bus_do[23:16];
        if(!we_n[1]) mem[ram_a][1] <= bus_do[15: 8];
        if(!we_n[0]) mem[ram_a][0] <= bus_do[ 7: 0];
    end
    ram_q <= mem[ram_a];
end

always @(*) bus_di = ram_q;     //CS4 reads as ram_q: the program never reads MMIO


///////////////////////////////////////////////////////////
//////  CS4: test MMIO (write side effects on the strobe's leading edge)
////

wire    mmio_wr = !cs4_n && (we_n != 4'hF);
reg     mmio_wr_q;
wire    mmio_we = mmio_wr && !mmio_wr_q;    //one event per bus write cycle

//Iteration-window bookkeeping for the sequence checker. Cleared on every
//KICK; a watchdog CPU reset clears it too (rst_n), including the kick
//continuity flag, so the first kick after a reset is exempt from the +1 rule.
reg     [4:0]   seq_stream;     //STREAM markers 1..5 seen this window
reg     [4:0]   seq_sig;        //SIG[0..4] writes seen this window
reg             seq_result;     //RESULT seen this window
reg             seq_canary_ok;  //that RESULT carried canary [9:5] == 11111
reg     [31:0]  kick_prev;
reg             kick_prev_v;

//evaluated at the KICK write (bus_do = the new kick payload)
wire    [4:0]   seq_bad = { kick_prev_v && (bus_do != kick_prev + 32'd1),
                            !seq_canary_ok,
                            !seq_result,
                            seq_sig    != 5'h1F,
                            seq_stream != 5'h1F };

always @(posedge i_CLK or negedge rst_n) begin
    if(!rst_n) begin
        mmio_wr_q    <= 1'b0;
        o_KICK_TGL   <= 1'b0;
        o_KICK_CNT   <= 32'd0;
        o_RESULT_TGL <= 1'b0;
        o_RESULT_MAP <= 5'd0;
        o_SIGXOR     <= 32'd0;
        o_STREAM     <= 3'd0;
        o_TRAP_TGL   <= 1'b0;
        o_TRAP_CODE  <= 32'd0;
        o_SEQERR_TGL <= 1'b0;
        o_SEQ_CODE   <= 5'd0;
        o_MMIO_WE    <= 1'b0;
        o_MMIO_ADDR  <= 8'd0;
        o_MMIO_DATA  <= 32'd0;
        seq_stream   <= 5'd0;
        seq_sig      <= 5'd0;
        seq_result   <= 1'b0;
        seq_canary_ok<= 1'b0;
        kick_prev    <= 32'd0;
        kick_prev_v  <= 1'b0;
    end
    else begin
        mmio_wr_q <= mmio_wr;
        o_MMIO_WE <= mmio_we;
        if(mmio_we) begin
            o_MMIO_ADDR <= bus_a[7:0];
            o_MMIO_DATA <= bus_do;
            case(bus_a[7:2])
                6'h00: begin                                //KICK
                    o_KICK_TGL <= ~o_KICK_TGL;
                    o_KICK_CNT <= bus_do;
                    if(seq_bad != 5'd0) begin
                        o_SEQERR_TGL <= ~o_SEQERR_TGL;
                        o_SEQ_CODE   <= seq_bad;
                    end
                    seq_stream    <= 5'd0;
                    seq_sig       <= 5'd0;
                    seq_result    <= 1'b0;
                    seq_canary_ok <= 1'b0;
                    kick_prev     <= bus_do;
                    kick_prev_v   <= 1'b1;
                end
                6'h01: begin                                //RESULT
                    o_RESULT_TGL <= ~o_RESULT_TGL;
                    o_RESULT_MAP <= bus_do[4:0];
                    seq_result    <= 1'b1;
                    seq_canary_ok <= (bus_do[9:5] == 5'h1F);
                end
                6'h02: begin                                //SIGXOR
                    if(bus_do != 32'd0) o_SIGXOR <= bus_do;
                end
                6'h03: begin                                //STREAM
                    o_STREAM <= bus_do[2:0];
                    if(bus_do[2:0] >= 3'd1 && bus_do[2:0] <= 3'd5)
                        seq_stream[bus_do[2:0] - 3'd1] <= 1'b1;
                end
                6'h04: begin                                //TRAP
                    o_TRAP_TGL  <= ~o_TRAP_TGL;
                    o_TRAP_CODE <= bus_do;
                end
                default: begin                              //SIG[0..4]
                    if(bus_a[7:2] >= 6'h08 && bus_a[7:2] <= 6'h0C)
                        seq_sig[bus_a[7:2] - 6'h08] <= 1'b1;
                end
            endcase
        end
    end
end

endmodule
