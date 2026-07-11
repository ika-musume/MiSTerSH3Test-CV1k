//
// tb_cpu.sv — runs the SH-3 stress program on the real HS3 core (Verilator,
// --timing). Prints every test-MMIO write; the runner parses the SIG[n]
// lines for golden capture and checks the DONE summary afterwards.
//
// Two independent detection layers are observed (anti-false-negative):
//   * SEQERR: the DUT's hardware sequence checker (sh3test_cpu)
//   * TBSEQ : this bench's own mirror of the same rules — 5 STREAM markers,
//     5 SIG writes, RESULT with intact canary, KICK payload = previous + 1.
//     A DUT-checker bug shows up as TBSEQ-without-SEQERR (and vice versa).
//
//   +maxcycles=N   simulation budget in CPU clocks (default 200000)
//   +iters=N       stop after N completed loop iterations (default 3)
//   +rtrace        print the retire stream (debug)
//

module tb_cpu #(
    parameter TB_HEX = "../rtl/sh3test_prog.hex"
);

logic clk = 1'b0;
logic rst_n = 1'b0;

always #5 clk = ~clk;       //100 MHz nominal; frequency is irrelevant here

sh3test_cpu #(
    .INIT_FILE(TB_HEX)
) dut (
    .i_CLK          (clk),
    .i_ARST_n       (rst_n),
    .o_KICK_TGL     (),
    .o_KICK_CNT     (),
    .o_RESULT_TGL   (),
    .o_RESULT_MAP   (),
    .o_SIGXOR       (),
    .o_STREAM       (),
    .o_TRAP_TGL     (),
    .o_TRAP_CODE    (),
    .o_SEQERR_TGL   (),
    .o_SEQ_CODE     (),
    .o_MMIO_WE      (),
    .o_MMIO_ADDR    (),
    .o_MMIO_DATA    ()
);

int unsigned maxcycles = 200000;
int unsigned want_iters = 3;
bit          rtrace = 0;

int unsigned cyc = 0;
int unsigned kicks = 0;
int unsigned traps = 0;
int unsigned results_bad = 0;
int unsigned seqerrs = 0;       //DUT hardware checker events
int unsigned tbseq = 0;         //bench-side mirror violations

//bench-side iteration-window mirror
logic [4:0]  w_stream = '0;
logic [4:0]  w_sig = '0;
logic        w_result = 1'b0;
logic        w_canary_ok = 1'b0;
logic [31:0] last_kick = '0;
bit          last_kick_v = 0;

logic seqerr_tgl_q = 1'b0;

initial begin
    if(!$value$plusargs("maxcycles=%d", maxcycles)) maxcycles = 200000;
    if(!$value$plusargs("iters=%d", want_iters))    want_iters = 3;
    rtrace = $test$plusargs("rtrace") != 0;

    repeat(8) @(posedge clk);
    rst_n = 1'b1;
end

always @(posedge clk) begin
    cyc <= cyc + 1;

    if(rtrace && dut.u_hs3.u_cpu.dbg_o_RETIRE_VALID)
        $display("R %0d pc=%08x inst=%04x", cyc,
                 dut.u_hs3.u_cpu.dbg_o_RETIRE_PC, dut.u_hs3.u_cpu.dbg_o_RETIRE_INST);

    //DUT hardware checker tap
    if(dut.o_SEQERR_TGL !== seqerr_tgl_q) begin
        seqerrs++;
        $display("SEQERR code=%02x cyc=%0d", dut.o_SEQ_CODE, cyc);
    end
    seqerr_tgl_q <= dut.o_SEQERR_TGL;

    if(dut.o_MMIO_WE) begin
        case(dut.o_MMIO_ADDR[7:2])
            6'h00: begin        //KICK: close + judge the window
                if(w_stream != 5'h1F || w_sig != 5'h1F || !w_result || !w_canary_ok) begin
                    tbseq++;
                    $display("TBSEQ window stream=%02x sig=%02x result=%0d canary=%0d cyc=%0d",
                             w_stream, w_sig, w_result, w_canary_ok, cyc);
                end
                if(last_kick_v && dut.o_MMIO_DATA != last_kick + 1) begin
                    tbseq++;
                    $display("TBSEQ kick=%0d after %0d cyc=%0d",
                             dut.o_MMIO_DATA, last_kick, cyc);
                end
                last_kick   <= dut.o_MMIO_DATA;
                last_kick_v <= 1;
                w_stream <= '0;
                w_sig    <= '0;
                w_result <= 1'b0;
                w_canary_ok <= 1'b0;

                kicks <= kicks + 1;
                $display("KICK   iter=%0d cyc=%0d", dut.o_MMIO_DATA, cyc);
                if(kicks + 1 >= want_iters) begin
                    //window/tbseq of THIS kick were judged with blocking ++
                    //above, so the summary includes the final iteration
                    $display("DONE kicks=%0d bad_results=%0d traps=%0d seqerrs=%0d tbseq=%0d",
                             kicks + 1, results_bad, traps, seqerrs, tbseq);
                    $finish;
                end
            end
            6'h01: begin        //RESULT: [9:5] canary, [4:0] map
                $display("RESULT map=%02x canary=%02x cyc=%0d",
                         dut.o_MMIO_DATA[4:0], dut.o_MMIO_DATA[9:5], cyc);
                if(dut.o_MMIO_DATA[4:0] != 5'd0) results_bad <= results_bad + 1;
                w_result    <= 1'b1;
                w_canary_ok <= (dut.o_MMIO_DATA[9:5] == 5'h1F);
            end
            6'h02: $display("SIGXOR %08x", dut.o_MMIO_DATA);
            6'h03: begin        //STREAM marker
                if(dut.o_MMIO_DATA[2:0] >= 1 && dut.o_MMIO_DATA[2:0] <= 5)
                    w_stream[dut.o_MMIO_DATA[2:0] - 1] <= 1'b1;
            end
            6'h04: begin
                traps <= traps + 1;
                $display("TRAP   expevt=%08x cyc=%0d", dut.o_MMIO_DATA, cyc);
                $display("FATAL: unexpected exception");
                $finish;
            end
            default: begin
                if(dut.o_MMIO_ADDR[7:2] >= 6'h08 && dut.o_MMIO_ADDR[7:2] <= 6'h0C) begin
                    $display("SIG[%0d]=%08x", dut.o_MMIO_ADDR[7:2] - 6'h08,
                             dut.o_MMIO_DATA);
                    w_sig[dut.o_MMIO_ADDR[7:2] - 6'h08] <= 1'b1;
                end
            end
        endcase
    end

    if(cyc >= maxcycles) begin
        $display("FATAL: timeout after %0d cycles (kicks=%0d)", cyc, kicks);
        $finish;
    end
end

endmodule
