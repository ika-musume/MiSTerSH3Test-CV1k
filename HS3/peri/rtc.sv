`default_nettype wire

/*
    Realtime clock RTC (SH7709S section 13, pp.407-426).

    Clock/calendar in BCD (second/minute/hour/day-of-week/date/month/2-digit
    year) with automatic leap-year correction, the 64 Hz counter, the
    ENB-gated frame-compare alarm, periodic interrupts (1/256 s .. 2 s) and
    the carry-flag double-read protocol of fig 13.3. Requests are LEVELS:
    o_RTC_REQ = {CUI, PRI, ATI} -> INTC IPRA[3:0], codes 0x4C0/0x4A0/0x480.

    CLOCK DOMAINS (FPGA adaptation of fig 13.1): the crystal front end - the
    /2 prescaler (RTCCLK 16.384 kHz) and the divider head down to 256 Hz -
    clocks on the EXTAL2 pad in its OWN asynchronous domain, so timekeeping
    needs no relation to the core clock. The 256 Hz tap crosses into the bus
    domain through 2FF + edge detect; R64CNT, the calendar chain, alarms and
    all flags tick THERE (bus-side registers: reads never tear, writes never
    race a crossing). Bus->RTC commands cross the other way as a toggle
    (RCR2 RESET/ADJ divider clear) and a level (RTCEN oscillator gate) - the
    manual's own ~91.6 us command latency (p.421) absorbs the crossing delay.

    Reset domains (table 13.2): counters and alarm values are NEVER pin-reset
    (only RCR2 RESET/ADJ clear the divider + R64CNT, 13.2.1); the alarm ENB
    bits and RCR2.RTCEN/START initialize on power-on reset only (p.410); RCR1
    and RCR2's PEF/PES clear on every reset flavor (a manual reset leaves
    RTCEN/START, p.420).

    Deviations: no module standby / VCC-RTC power pin; XTAL2 is not modeled
    (clock input only); counter writes while START=1 are accepted (13.4.1
    forbids them - a write wins over a same-cycle carry); wrong-size register
    writes take the low byte.
*/

module rtc (
    /* CLOCK AND RESET - bus side (see reset-domain note above) */
    input   wire            i_POR_n,        //power-on only: alarm ENB + RCR2.RTCEN/START
    input   wire            i_RST_n,        //all flavors: RCR1 + RCR2 PEF/PES
    input   wire            i_CLK,
    input   wire            i_CEN,

    /* INTERFACES */
    IBus_2.slave            REG_BUS,        //P bus window 0xFFFFFEC0-DE (behind the BSC)

    /* RTC OSCILLATOR - own asynchronous clock domain */
    input   wire            i_EXTAL2,       //32.768 kHz crystal pad (table 13.1)

    /* MODULE CONNECTIONS */
    output  wire            o_RTCCLK,       //16.384 kHz divider output (EXTAL2-domain level)
    output  wire            o_RTC_TICK,     //RTCCLK rise tick, i_CEN-qualified (TMU TPSC=100)

    /* INTERRUPT REQUESTS - levels on IPRA[3:0] */
    output  wire    [2:0]   o_RTC_REQ       //{CUI, PRI, ATI}
);

///////////////////////////////////////////////////////////
//////  Register Storage
////

//counters and alarm values are NEVER pin-reset (table 13.2) - like the
//TMU's TCPR2 they live in no-reset flops (sim starts at 0)
logic   [6:0]   r64cnt;                     //0xC0: 128 Hz counter, bits = 64Hz..1Hz taps
logic   [6:0]   rseccnt;                    //0xC2: BCD 00-59
logic   [6:0]   rmincnt;                    //0xC4: BCD 00-59
logic   [5:0]   rhrcnt;                     //0xC6: BCD 00-23
logic   [2:0]   rwkcnt;                     //0xC8: 0-6 (0 = Sunday, table 13.3)
logic   [5:0]   rdaycnt;                    //0xCA: BCD 01-31
logic   [4:0]   rmoncnt;                    //0xCC: BCD 01-12
logic   [7:0]   ryrcnt;                     //0xCE: BCD 00-99
logic   [6:0]   aval [0:5];                 //0xD0-DA alarm values: sec,min,hr,wk,day,mon
logic   [5:0]   aenb;                       //alarm ENB bits (bit 7 of each; POR-cleared)
logic           cf, cie, aie, af;           //RCR1 0xDC (p.419)
logic           pef;                        //RCR2 0xDE (p.420)
logic   [2:0]   pes;
logic           rtcen, start;



///////////////////////////////////////////////////////////
//////  Crystal Divider Front End (EXTAL2 clock domain)
////

/*
    rdiv[0] is the /2 prescaler output = RTCCLK 16.384 kHz (fig 13.1: the
    TMU count source and the TOCR.TCOE TCLK pad output); rdiv[6] is the
    256 Hz tap the bus domain divides down from. RTCEN=0 models the halted
    crystal oscillator (p.421) by gating the count. RCR2 RESET/ADJ arrive
    as a toggle and clear the prescaler (divider-reset scope, p.421). No
    pin reset: on silicon this divider only ever clears by RESET/ADJ.
*/

logic   [6:0]   rdiv;
logic   [1:0]   rtcen_rs;                   //RCR2.RTCEN level, 2FF into this domain
logic   [2:0]   divrst_rs;                  //divider-reset toggle, 2FF + edge stage
logic           divrst_tgl;                 //bus-domain toggle source (control section)

always_ff @(posedge i_EXTAL2) begin
    rtcen_rs  <= {rtcen_rs[0],  rtcen};
    divrst_rs <= {divrst_rs[1:0], divrst_tgl};
    if(divrst_rs[2] ^ divrst_rs[1]) rdiv <= 7'd0;
    else if(rtcen_rs[1])            rdiv <= rdiv + 7'd1;
end

assign  o_RTCCLK = rdiv[0];



///////////////////////////////////////////////////////////
//////  Tick Synchronizers (into the bus domain)
////

//2FF + edge detect at core rate (the TMU TCLK sampler pattern). No reset:
//the flops follow an asynchronous level anyway, and a reset-forced 0 would
//fabricate an edge (= a phantom divider tick) on every warm reset.
logic   [1:0]   rck_s;                      //rdiv[0] = RTCCLK sync chain
logic           rck_z;
logic   [1:0]   q256_s;                     //rdiv[6] = 256 Hz sync chain
logic           q256_z;

always_ff @(posedge i_CLK) begin if(i_CEN) begin
    rck_s  <= {rck_s[0],  rdiv[0]};   rck_z  <= rck_s[1];
    q256_s <= {q256_s[0], rdiv[6]};   q256_z <= q256_s[1];
end end

assign  o_RTC_TICK = i_CEN & rck_s[1] & ~rck_z;         //16384 ticks/s
wire            tick256 = i_CEN & q256_s[1] & ~q256_z;  //256 ticks/s (1/256 s grid)



///////////////////////////////////////////////////////////
//////  Register Access Decode
////

wire            reg_wr  = REG_BUS.stb && REG_BUS.we;
wire    [7:0]   wd      = REG_BUS.wdata[7:0];           //byte registers: low byte of any size
wire            wr_rcr1 = reg_wr && (REG_BUS.addr == 8'hDC);
wire            wr_rcr2 = reg_wr && (REG_BUS.addr == 8'hDE);
wire            wr_sec  = reg_wr && (REG_BUS.addr == 8'hC2);
wire            wr_min  = reg_wr && (REG_BUS.addr == 8'hC4);
wire            wr_hr   = reg_wr && (REG_BUS.addr == 8'hC6);
wire            wr_wk   = reg_wr && (REG_BUS.addr == 8'hC8);
wire            wr_day  = reg_wr && (REG_BUS.addr == 8'hCA);
wire            wr_mon  = reg_wr && (REG_BUS.addr == 8'hCC);
wire            wr_yr   = reg_wr && (REG_BUS.addr == 8'hCE);

//alarm write strobes: 0xD0 + 2n, n = {sec,min,hr,wk,day,mon}
logic   [5:0]   wr_alm;
always_comb begin
    for(int a = 0; a < 6; a++)
        wr_alm[a] = reg_wr && (REG_BUS.addr == 8'hD0 + 8'(2*a));
end

//right-justified read mux (the P bridge replicates lanes); reserved bits 0,
//RCR2's ADJ/RESET action bits always read 0 (p.421)
always_comb begin
    unique case(REG_BUS.addr)
        8'hC0:   REG_BUS.rdata = {25'd0, r64cnt};
        8'hC2:   REG_BUS.rdata = {25'd0, rseccnt};
        8'hC4:   REG_BUS.rdata = {25'd0, rmincnt};
        8'hC6:   REG_BUS.rdata = {26'd0, rhrcnt};
        8'hC8:   REG_BUS.rdata = {29'd0, rwkcnt};
        8'hCA:   REG_BUS.rdata = {26'd0, rdaycnt};
        8'hCC:   REG_BUS.rdata = {27'd0, rmoncnt};
        8'hCE:   REG_BUS.rdata = {24'd0, ryrcnt};
        8'hD0:   REG_BUS.rdata = {24'd0, aenb[0], aval[0]};
        8'hD2:   REG_BUS.rdata = {24'd0, aenb[1], aval[1]};
        8'hD4:   REG_BUS.rdata = {24'd0, aenb[2], 1'b0, aval[2][5:0]};
        8'hD6:   REG_BUS.rdata = {24'd0, aenb[3], 4'd0, aval[3][2:0]};
        8'hD8:   REG_BUS.rdata = {24'd0, aenb[4], 1'b0, aval[4][5:0]};
        8'hDA:   REG_BUS.rdata = {24'd0, aenb[5], 2'd0, aval[5][4:0]};
        8'hDC:   REG_BUS.rdata = {24'd0, cf, 2'd0, cie, aie, 2'd0, af};
        8'hDE:   REG_BUS.rdata = {24'd0, pef, pes, rtcen, 2'd0, start};
        default: REG_BUS.rdata = 32'd0;                 //reserved offsets read 0
    endcase
end



///////////////////////////////////////////////////////////
//////  Divider Tail: R64CNT and the Periodic Grid
////

/*
    R64CNT increments at 128 Hz - bit 0 IS the 64 Hz output of p.411. Every
    R64CNT count-up is a CF setting event (13.2.15 "count up of R64CNT or
    RSECCNT": the seconds carry is the wrap of this same event). RCR2
    RESET/ADJ clear the tail immediately here; the EXTAL2-side prescaler
    follows a few RTC clocks later (the p.421 command latency).
*/

logic           t128_ph;                    //halves the 256 Hz tick to the R64CNT pace
logic           sec_par;                    //0.5 Hz divider bit (seconds parity): the 2 s
                                            //periodic tap. R64CNT is 7-bit and bottoms out at
                                            //1 Hz, so 2 s needs one more divider stage; it must
                                            //ride the free-running divider (not RSECCNT) so the
                                            //PES=2s interrupt is START-independent (p.421).
wire            ev128    = tick256 & t128_ph;           //R64CNT count-up (128/s)
wire            sec_ev   = ev128 & (r64cnt == 7'h7F);   //1 Hz wrap = seconds carry
wire            divrst_cmd;                             //RESET/ADJ write (control section)

always_ff @(posedge i_CLK) begin if(i_CEN) begin
    if(divrst_cmd) begin
        t128_ph <= 1'b0;
        r64cnt  <= 7'd0;
        sec_par <= 1'b0;
    end
    else begin
        if(tick256) t128_ph <= ~t128_ph;
        if(ev128)   r64cnt  <= r64cnt + 7'd1;
        if(sec_ev)  sec_par <= ~sec_par;                //toggle on the 1 Hz carry = 0.5 Hz
    end
end end

//periodic interrupt event per PES (p.421): all-ones low bits of the 128 Hz
//grid select the 2^n subdivisions; 2 s uses the 0.5 Hz divider bit (sec_par)
//so it fires off the free-running divider regardless of RCR2.START
logic           pev;
always_comb begin
    unique case(pes)
        3'd1:    pev = tick256;                         //1/256 s
        3'd2:    pev = ev128 &  r64cnt[0];              //1/64 s
        3'd3:    pev = ev128 & (r64cnt[2:0] == 3'h7);   //1/16 s
        3'd4:    pev = ev128 & (r64cnt[4:0] == 5'h1F);  //1/4 s
        3'd5:    pev = ev128 & (r64cnt[5:0] == 6'h3F);  //1/2 s
        3'd6:    pev = sec_ev;                          //1 s
        3'd7:    pev = sec_ev & sec_par;                //2 s: 1 Hz carry gated by the 0.5 Hz bit
        default: pev = 1'b0;                            //000: no periodic interrupts
    endcase
end



///////////////////////////////////////////////////////////
//////  Clock/Calendar Counters
////

/*
    BCD carry chain advanced by the 1 Hz wrap while START=1; a bus write
    wins over a same-cycle carry (the TMU pattern - 13.4.1 wants the count
    halted around writes anyway). ADJ (p.421) rounds: 00-29 s -> 00,
    30-59 s -> the next minute, and resets the divider alongside RESET.
*/

function automatic logic [7:0] bcd_inc(input logic [7:0] v);
    bcd_inc = (v[3:0] == 4'd9) ? {v[7:4] + 4'd1, 4'd0} : v + 8'd1;
endfunction

//leap year (p.415, year 00 included): BCD year % 4 == 0 <=> ones digit even
//and ones bit1 == tens bit0, since (10*hi + lo) % 4 = (2*hi + lo) % 4
wire            leap = !ryrcnt[0] && (ryrcnt[1] == ryrcnt[4]);

//BCD last day of the month (13.2.6); unassigned month codes get 31
logic   [5:0]   mon_last;
always_comb begin
    unique case(rmoncnt)
        5'h02:          mon_last = leap ? 6'h29 : 6'h28;
        5'h04, 5'h06,
        5'h09, 5'h11:   mon_last = 6'h30;
        default:        mon_last = 6'h31;
    endcase
end

wire            adj_cmd   = wr_rcr2 && wd[2];
wire            cal_tick  = sec_ev & start;
wire            sec_carry = (cal_tick && rseccnt == 7'h59) ||
                            (adj_cmd  && rseccnt >= 7'h30);     //30-59 s round up
wire            min_carry = sec_carry && (rmincnt == 7'h59);
wire            hr_carry  = min_carry && (rhrcnt  == 6'h23);    //= one day passed
wire            day_carry = hr_carry  && (rdaycnt == mon_last);
wire            mon_carry = day_carry && (rmoncnt == 5'h12);

wire    [7:0]   sec_nx = bcd_inc({1'b0, rseccnt});
wire    [7:0]   min_nx = bcd_inc({1'b0, rmincnt});
wire    [7:0]   hr_nx  = bcd_inc({2'd0, rhrcnt});
wire    [7:0]   day_nx = bcd_inc({2'd0, rdaycnt});
wire    [7:0]   mon_nx = bcd_inc({3'd0, rmoncnt});
wire    [7:0]   yr_nx  = bcd_inc(ryrcnt);

always_ff @(posedge i_CLK) begin if(i_CEN) begin        //no reset: table 13.2
    if(wr_sec)         rseccnt <= wd[6:0];
    else if(adj_cmd)   rseccnt <= 7'd0;                 //30-second adjust
    else if(cal_tick)  rseccnt <= (rseccnt == 7'h59) ? 7'd0 : sec_nx[6:0];

    if(wr_min)         rmincnt <= wd[6:0];
    else if(sec_carry) rmincnt <= (rmincnt == 7'h59) ? 7'd0 : min_nx[6:0];

    if(wr_hr)          rhrcnt  <= wd[5:0];
    else if(min_carry) rhrcnt  <= (rhrcnt == 6'h23) ? 6'd0 : hr_nx[5:0];

    if(wr_wk)          rwkcnt  <= wd[2:0];
    else if(hr_carry)  rwkcnt  <= (rwkcnt == 3'd6) ? 3'd0 : rwkcnt + 3'd1;

    if(wr_day)         rdaycnt <= wd[5:0];
    else if(hr_carry)  rdaycnt <= (rdaycnt == mon_last) ? 6'h01 : day_nx[5:0];

    if(wr_mon)         rmoncnt <= wd[4:0];
    else if(day_carry) rmoncnt <= (rmoncnt == 5'h12) ? 5'h01 : mon_nx[4:0];

    if(wr_yr)          ryrcnt  <= wd[7:0];
    else if(mon_carry) ryrcnt  <= (ryrcnt == 8'h99) ? 8'd0 : yr_nx;
end end



///////////////////////////////////////////////////////////
//////  Alarm Comparator
////

//frame compare across the ENB-selected registers only (13.2.9). AF sets on
//the match EDGE: clearing AF inside a still-matching second cannot re-fire
wire    [6:0]   acur [0:5];                 //counters aligned to the alarm value width
assign  acur[0] = rseccnt;
assign  acur[1] = rmincnt;
assign  acur[2] = {1'b0, rhrcnt};
assign  acur[3] = {4'd0, rwkcnt};
assign  acur[4] = {1'b0, rdaycnt};
assign  acur[5] = {2'd0, rmoncnt};

logic           alarm_hit;
always_comb begin
    alarm_hit = |aenb;                      //at least one register takes part
    for(int a = 0; a < 6; a++)
        if(aenb[a] && aval[a] != acur[a]) alarm_hit = 1'b0;
end

logic           alarm_hit_z;
wire            af_set = alarm_hit & ~alarm_hit_z;



///////////////////////////////////////////////////////////
//////  Control Registers and Flags
////

/*
    Flag discipline: a hardware set outranks a same-cycle software write
    (the INTC pending pattern). CF sets on every R64CNT count-up and is
    software-settable by write-1 (13.2.15), like PEF (p.420); AF write-1
    holds the old value (p.420 note). A manual reset leaves RTCEN/START and
    the CF "undefined" - cleared here, a legal pick.
*/

//divider-reset command: RESET or ADJ (both clear prescaler + R64CNT, p.421).
//The toggle has NO reset - a reset-forced flip would reach the EXTAL2
//domain as a phantom divider clear.
assign  divrst_cmd = wr_rcr2 && (wd[1] || wd[2]);

always_ff @(posedge i_CLK) begin if(i_CEN) begin
    if(divrst_cmd) divrst_tgl <= ~divrst_tgl;
end end

//RCR1 flags + RCR2 PEF/PES: cleared by every reset flavor (p.419-420)
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        cf  <= 1'b0; cie <= 1'b0; aie <= 1'b0; af <= 1'b0;
        pef <= 1'b0; pes <= 3'd0;
        alarm_hit_z <= 1'b0;
    end
    else begin if(i_CEN) begin
        alarm_hit_z <= alarm_hit;

        if(ev128)        cf <= 1'b1;                    //count-up set outranks
        else if(wr_rcr1) cf <= wd[7];                   //write-1 sets, write-0 clears

        if(wr_rcr1) begin
            cie <= wd[4];
            aie <= wd[3];
        end

        if(af_set)       af <= 1'b1;
        else if(wr_rcr1) af <= af & wd[0];              //write-0 clears, write-1 holds

        if(pev)          pef <= 1'b1;                   //period elapse set outranks
        else if(wr_rcr2) pef <= wd[7];                  //write-1 sets (p.420)

        if(wr_rcr2)      pes <= wd[6:4];
    end end
end

//RCR2.RTCEN/START + alarm ENB bits: power-on reset only (p.410, table 13.2)
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) begin
        rtcen <= 1'b1;                                  //RCR2 initial 0x09 (p.410)
        start <= 1'b1;
        aenb  <= 6'd0;
    end
    else begin if(i_CEN) begin
        if(wr_rcr2) begin
            rtcen <= wd[3];
            start <= wd[0];
        end
        for(int a = 0; a < 6; a++)
            if(wr_alm[a]) aenb[a] <= wd[7];
    end end
end

//alarm values: never reset; per-register masks zero the unimplemented bits
localparam logic [6:0] AMSK [0:5] = '{7'h7F, 7'h7F, 7'h3F, 7'h07, 7'h3F, 7'h1F};
always_ff @(posedge i_CLK) begin if(i_CEN) begin
    for(int a = 0; a < 6; a++)
        if(wr_alm[a]) aval[a] <= wd[6:0] & AMSK[a];
end end



///////////////////////////////////////////////////////////
//////  Interrupt Requests
////

//levels; PES=000 generates no periodic interrupts even with PEF set (p.421)
assign  o_RTC_REQ = {cf & cie, pef & (pes != 3'd0), af & aie};

endmodule

`default_nettype none
