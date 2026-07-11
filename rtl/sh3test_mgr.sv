//
// sh3test_mgr.sv — test sequencer / external manager for the SH-3 Fmax
// stress suite. Runs entirely on the fixed 50 MHz reference clock.
//
//   * steps the CPU PLL through the frequency schedule
//       75/80/85/90 MHz          1 min each
//       95/100 MHz               5 min each
//       102.8/105/108/112 MHz   10 min each
//   * watchdog: no KICK for 100 ms -> reset the CPU, count it, latch the
//     stream that was running (the hang fingerprint)
//   * tallies kicks (loop completions), per-stream signature errors, and
//     the last bad-signature XOR
//   * halt rule: >= 5 errors at the 102.8 MHz step (auto run only) ->
//     stop the test, raise o_ALARM (videoout paints the background blue)
//   * hold mode (i_HOLD_STEP != 0): jump straight to one step and soak
//     there forever, time counts up, halt rule disabled
//
// CDC: all CPU-domain inputs are either toggles (2FF sync + edge detect)
// or quasi-static payloads that settle microseconds before their toggle
// arrives. The CPU clock is measured by counting a /64 toggle exported by
// the CPU-domain free counter.
//

module sh3test_mgr #(
    parameter SIM_SPEEDUP = 0       //1: 1000x faster ticks for simulation
)(
    input  wire         i_CLK50,
    input  wire         i_RST,          //core reset (MiSTer reset_or)
    input  wire         i_PAUSE,        //OSD open: freeze schedule + watchdog
    input  wire  [3:0]  i_HOLD_STEP,    //0 = auto schedule, 1..14 = soak step-1
                                        //(11..14 = 120/133/150/166 MHz, OSD-only
                                        // overdrive points outside the schedule)

    /* CPU domain (async) */
    input  wire         i_KICK_TGL,
    input  wire         i_RESULT_TGL,
    input  wire  [4:0]  i_RESULT_MAP,   //bit4=A .. bit0=E
    input  wire [31:0]  i_SIGXOR,
    input  wire  [2:0]  i_STREAM,
    input  wire         i_SEQERR_TGL,   //iteration-sequence checker (sh3test_cpu)
    input  wire         i_CPUCTR_TGL,   //CPU free-counter bit 5: toggles /64

    /* CPU + PLL control */
    output wire         o_CPU_RST_n,    //async-assert reset to sh3test_cpu
    output reg   [3:0]  o_PLL_STEP,     //0..13 -> cfg table in the top level
    output reg          o_PLL_GO,       //1-cycle pulse: start reconfig

    /* scoreboard (quasi-static; consumed in the video domain) */
    output reg  [31:0]  o_FREQ_HZ,      //measured CPU clock
    output reg   [3:0]  o_STEP_NO,      //1..10 (current), sticks at 10 when done
    output reg   [7:0]  o_TIME_MIN,     //remaining (auto) / elapsed (hold)
    output reg   [7:0]  o_TIME_SEC,
    output reg  [31:0]  o_LOOPS,        //kicks this step
    output reg  [31:0]  o_LOOPS_SEC,    //kicks in the last full second
    output reg  [15:0]  o_RESETS,       //watchdog resets this step
    output reg   [2:0]  o_HANG_STREAM,  //stream running at the last watchdog reset
    output reg  [31:0]  o_ERRORS,       //bad iterations this step
    output reg  [15:0]  o_SEQERRS,      //bad kick windows this step (protocol/
                                        //canary/monotonicity: execution proof)
    output reg  [39:0]  o_ERR_STREAM,   //5 x 8-bit per-stream counts {A,B,C,D,E}
    output reg  [31:0]  o_SIGXOR,       //last nonzero signature XOR
    output reg  [19:0]  o_HISTORY,      //10 x 2 bit: 0 notrun / 1 pass / 2 fail / 3 running
    output reg          o_ALARM,        //halt rule fired: paint background blue
    output reg          o_DONE
);

localparam integer NSTEP = 10;
localparam integer SEC_CYCLES    = SIM_SPEEDUP ? 50_000 : 50_000_000;
//real: 100 ms. SIM: 600 us — must cover the ~190 us cold boot (BSC comes out
//of reset at WCR2 maximum wait states, so the uncached boot + first fills
//are slow) with margin, while still firing well inside a 2 ms SIM step.
localparam integer WD_TIMEOUT    = SIM_SPEEDUP ? 30_000 :  5_000_000;
localparam integer CFG_SETTLE    = SIM_SPEEDUP ?    500 :    250_000;  //5 ms: cfg writes + PLL relock
localparam [3:0]  ALARM_STEP   = 4'd6;      //102.8 MHz
localparam [31:0] ALARM_ERRORS = 32'd5;

//step duration in seconds (auto mode) + its mm:ss split for the display
function automatic [15:0] step_secs(input [3:0] s);
    case(s)
        4'd0, 4'd1, 4'd2, 4'd3: step_secs = SIM_SPEEDUP ? 16'd2 : 16'd60;
        4'd4, 4'd5:             step_secs = SIM_SPEEDUP ? 16'd3 : 16'd300;
        default:                step_secs = SIM_SPEEDUP ? 16'd4 : 16'd600;
    endcase
endfunction

function automatic [7:0] step_mins(input [3:0] s);
    case(s)
        4'd0, 4'd1, 4'd2, 4'd3: step_mins = SIM_SPEEDUP ? 8'd0 : 8'd1;
        4'd4, 4'd5:             step_mins = SIM_SPEEDUP ? 8'd0 : 8'd5;
        default:                step_mins = SIM_SPEEDUP ? 8'd0 : 8'd10;
    endcase
endfunction

function automatic [7:0] step_modsecs(input [3:0] s);
    reg [15:0] t;                   //Quartus cannot part-select a call result
    begin
        t = step_secs(s);
        step_modsecs = SIM_SPEEDUP ? t[7:0] : 8'd0;
    end
endfunction


///////////////////////////////////////////////////////////
//////  CPU-domain input synchronizers
////

reg [2:0] kick_s, result_s, seqerr_s, cpuctr_s;
always @(posedge i_CLK50) begin
    kick_s   <= {kick_s[1:0],   i_KICK_TGL};
    result_s <= {result_s[1:0], i_RESULT_TGL};
    seqerr_s <= {seqerr_s[1:0], i_SEQERR_TGL};
    cpuctr_s <= {cpuctr_s[1:0], i_CPUCTR_TGL};
end
wire kick_ev   = kick_s[2]   ^ kick_s[1];
wire result_ev = result_s[2] ^ result_s[1];
wire seqerr_ev = seqerr_s[2] ^ seqerr_s[1];
wire cpuctr_ev = cpuctr_s[2] ^ cpuctr_s[1];
//payloads: stable long before their toggle is observed (see header)
wire  [4:0] result_map = i_RESULT_MAP;
wire [31:0] sigxor_in  = i_SIGXOR;
wire  [2:0] stream_in  = i_STREAM;


///////////////////////////////////////////////////////////
//////  Second tick + CPU frequency meter
////

reg [25:0] sec_ctr;
wire       sec_tick = (sec_ctr == SEC_CYCLES[25:0] - 1);
reg [25:0] fmeter_acc;      //cpuctr toggles this second (= f/64)
reg [31:0] loops_acc;

always @(posedge i_CLK50) begin
    if(i_RST) begin
        sec_ctr    <= 26'd0;
        fmeter_acc <= 26'd0;
        loops_acc  <= 32'd0;
        o_FREQ_HZ  <= 32'd0;
        o_LOOPS_SEC<= 32'd0;
    end
    else begin
        sec_ctr <= sec_tick ? 26'd0 : sec_ctr + 1'd1;
        if(sec_tick) begin
            //x64 (toggle = 64 CPU clocks); SIM_SPEEDUP window is 1 ms -> x1000
            o_FREQ_HZ  <= SIM_SPEEDUP ? ({fmeter_acc, 6'd0} * 32'd1000)
                                      : {fmeter_acc, 6'd0};
            fmeter_acc <= 26'd0;
            o_LOOPS_SEC<= loops_acc;
            loops_acc  <= 32'd0;
        end
        else begin
            if(cpuctr_ev) fmeter_acc <= fmeter_acc + 1'd1;
            if(kick_ev)   loops_acc  <= loops_acc + 1'd1;
        end
    end
end


///////////////////////////////////////////////////////////
//////  Sequencer
////

localparam [2:0] S_CFG  = 3'd0,     //program PLL, CPU in reset, settle
                 S_RUN  = 3'd1,     //test running
                 S_DONE = 3'd2,     //schedule finished
                 S_HALT = 3'd3;     //halt rule fired (alarm)

reg  [2:0]  state;
reg [17:0]  settle_ctr;
reg [22:0]  wd_ctr;
reg  [6:0]  wd_pulse;       //CPU reset stretch on watchdog timeout
reg [15:0]  secs_left;
reg  [3:0]  hold_q;
wire        hold_mode = (hold_q != 4'd0);
wire        step_fail = (o_ERRORS != 32'd0) || (o_RESETS != 16'd0) ||
                        (o_SEQERRS != 16'd0);

assign o_CPU_RST_n = (state == S_RUN) && (wd_pulse == 7'd0) && !i_RST;

//mm:ss maintenance shares the per-second tick
wire run_tick = sec_tick && (state == S_RUN) && !i_PAUSE;

always @(posedge i_CLK50) begin
    if(i_RST) begin
        state        <= S_CFG;
        hold_q       <= i_HOLD_STEP;
        o_PLL_STEP   <= (i_HOLD_STEP != 4'd0) ? (i_HOLD_STEP - 4'd1) : 4'd0;
        o_PLL_GO     <= 1'b0;   //pulsed by the first S_CFG cycle
        settle_ctr   <= 18'd0;
        wd_ctr       <= 23'd0;
        wd_pulse     <= 7'd0;
        secs_left    <= 16'd0;
        o_STEP_NO    <= 4'd1;
        o_TIME_MIN   <= 8'd0;
        o_TIME_SEC   <= 8'd0;
        o_LOOPS      <= 32'd0;
        o_RESETS     <= 16'd0;
        o_HANG_STREAM<= 3'd0;
        o_ERRORS     <= 32'd0;
        o_SEQERRS    <= 16'd0;
        o_ERR_STREAM <= 40'd0;
        o_SIGXOR     <= 32'd0;
        o_HISTORY    <= 20'd0;
        o_ALARM      <= 1'b0;
        o_DONE       <= 1'b0;
    end
    else begin
        o_PLL_GO <= 1'b0;

        //restart on a Hold-Step change (quasi-static OSD value)
        if(hold_q != i_HOLD_STEP) begin
            hold_q     <= i_HOLD_STEP;
            o_PLL_STEP <= (i_HOLD_STEP != 4'd0) ? (i_HOLD_STEP - 4'd1) : 4'd0;
            state      <= S_CFG;
            settle_ctr <= 18'd0;
            o_HISTORY  <= 20'd0;
            o_ALARM    <= 1'b0;
            o_DONE     <= 1'b0;
        end
        else case(state)

        S_CFG: begin
            o_PLL_GO   <= (settle_ctr == 18'd0);    //single reconfig request
            settle_ctr <= settle_ctr + 1'd1;
            o_STEP_NO  <= o_PLL_STEP + 4'd1;
            //clear per-step statistics
            o_LOOPS      <= 32'd0;
            o_RESETS     <= 16'd0;
            o_ERRORS     <= 32'd0;
            o_SEQERRS    <= 16'd0;
            o_ERR_STREAM <= 40'd0;
            o_SIGXOR     <= 32'd0;
            wd_ctr       <= 23'd0;
            wd_pulse     <= 7'd0;
            if(hold_mode) begin
                o_TIME_MIN <= 8'd0;
                o_TIME_SEC <= 8'd0;
            end
            else begin
                o_TIME_MIN <= step_mins(o_PLL_STEP);
                o_TIME_SEC <= step_modsecs(o_PLL_STEP);
                secs_left  <= step_secs(o_PLL_STEP);
            end
            if(settle_ctr == CFG_SETTLE[17:0] - 1) begin
                state <= S_RUN;
                //steps >= 10 (hold-only overdrive points) have no history slot
                if(o_PLL_STEP < NSTEP[3:0])
                    o_HISTORY[o_PLL_STEP*2 +: 2] <= 2'd3;   //running marker
            end
        end

        S_RUN: begin
            //---- watchdog (frozen while the OSD is open)
            if(wd_pulse != 7'd0) begin
                wd_pulse <= wd_pulse - 1'd1;
                wd_ctr   <= 23'd0;
            end
            else if(kick_ev) wd_ctr <= 23'd0;
            else if(!i_PAUSE) begin
                if(wd_ctr == WD_TIMEOUT[22:0] - 1) begin
                    wd_pulse      <= 7'd64;
                    wd_ctr        <= 23'd0;
                    o_HANG_STREAM <= stream_in;
                    if(o_RESETS != 16'hFFFF) o_RESETS <= o_RESETS + 1'd1;
                end
                else wd_ctr <= wd_ctr + 1'd1;
            end

            //---- tallies
            if(kick_ev && o_LOOPS != 32'hFFFF_FFFF) o_LOOPS <= o_LOOPS + 1'd1;
            if(result_ev && result_map != 5'd0) begin
                if(o_ERRORS != 32'hFFFF_FFFF) o_ERRORS <= o_ERRORS + 1'd1;
                o_SIGXOR <= sigxor_in;
                //o_ERR_STREAM byte order: [39:32]=A ... [7:0]=E; map bit4=A
                for(int i = 0; i < 5; i = i + 1) begin
                    if(result_map[4 - i] && o_ERR_STREAM[(4-i)*8 +: 8] != 8'hFF)
                        o_ERR_STREAM[(4-i)*8 +: 8] <= o_ERR_STREAM[(4-i)*8 +: 8] + 1'd1;
                end
            end
            if(seqerr_ev && o_SEQERRS != 16'hFFFF) o_SEQERRS <= o_SEQERRS + 1'd1;

            //---- time + step advance
            if(run_tick) begin
                if(hold_mode) begin
                    if(o_TIME_SEC == 8'd59) begin
                        o_TIME_SEC <= 8'd0;
                        if(o_TIME_MIN != 8'd99) o_TIME_MIN <= o_TIME_MIN + 1'd1;
                    end
                    else o_TIME_SEC <= o_TIME_SEC + 1'd1;
                end
                else begin
                    secs_left <= secs_left - 1'd1;
                    //display shows time REMAINING after this tick
                    if(o_TIME_SEC == 8'd0) begin
                        if(o_TIME_MIN != 8'd0) begin
                            o_TIME_MIN <= o_TIME_MIN - 1'd1;
                            o_TIME_SEC <= 8'd59;
                        end
                    end
                    else o_TIME_SEC <= o_TIME_SEC - 1'd1;

                    if(secs_left == 16'd1) begin
                        o_HISTORY[o_PLL_STEP*2 +: 2] <= step_fail ? 2'd2 : 2'd1;
                        if(o_PLL_STEP == NSTEP[3:0] - 1) begin
                            state  <= S_DONE;
                            o_DONE <= 1'b1;
                        end
                        else begin
                            o_PLL_STEP <= o_PLL_STEP + 1'd1;
                            settle_ctr <= 18'd0;
                            state      <= S_CFG;
                        end
                    end
                end
            end

            //---- halt rule: bad iterations (signature or sequence) at the
            //102.8 MHz step, auto run only. LAST state assignment in S_RUN,
            //so a 5th error landing on the same cycle as a step advance
            //still halts instead of racing the advance.
            if(!hold_mode && o_PLL_STEP == ALARM_STEP &&
               (o_ERRORS + {16'd0, o_SEQERRS}) >= ALARM_ERRORS) begin
                state   <= S_HALT;
                o_ALARM <= 1'b1;
                o_HISTORY[o_PLL_STEP*2 +: 2] <= 2'd2;
            end
        end

        S_DONE: begin
            o_TIME_MIN <= 8'd0;
            o_TIME_SEC <= 8'd0;
        end

        S_HALT: ;   //frozen; alarm stays up until core reset

        default: state <= S_CFG;
        endcase
    end
end

endmodule
