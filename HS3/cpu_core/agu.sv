`default_nettype wire

/*
    SH7709S EX-stage AGU - the single time-shared address adder.

    One 32-bit adder generates EVERY pipeline address in one logic stage:
      - data effective address (MA side),
      - sequential fetch PC+2 (IF side),
      - branch target, incl. conditional BT/BF resolved by the running T flag.

    All source/select decisions are made in ID and arrive here registered, so the
    cen_p->cen_n half-clock (5 ns @ 100 MHz arch) carries only: one base mux, one
    en-gated addend, and the carry chain. The addend gate folds T (i_R_T) and the
    mode bits into the adder's own second-addend LUT (Cyclone V ALM LUT4 / Xilinx
    7-series LUT6 + carry), so no discrete mux sits before or after the adder.

      o_ADDR = X + (en . Y) + ci
        X  = i_USE_BASE ? i_AGU_A : i_FETCH_PC         ;MA/branch base vs live fetch PC
        en = {NULL:0, FORCE:1, USE_T:T, USE_NT:~T}     ;offset / conditional gate
        ci = +2@bit1 when i_PC_INC                     ;sequential PC+2 only

    PC+2 rides a carry, not an addend: SH-3 has no byte-aligned instructions so
    PC[0]=0, and +2 is a carry into bit 1 (bit 0 is don't-care). PREDEC's -step is
    formed in ID as ~step+1 via the ID-adder carry-in, so EX needs no +1 here - the
    addend i_AGU_B already carries -step for that mode. EN/PC_INC/USE_BASE are
    decoded in ID (see int_pipe); i_R_T is the latched running SR.T (no forward).
*/

module agu (
    /* OPERANDS (selected and registered in ID) */
    input   wire    [31:0]  i_AGU_A,        //base: EA base / EA2 - pre-selected in ID
    input   wire    [31:0]  i_AGU_B,        //addend: disp / R0 / Rn / ~step
    input   wire    [31:0]  i_FETCH_PC,     //live fetch PC (IF side of the time-share)

    /* CONTROL (ID-decoded, except i_R_T which is the running flag) */
    input   wire            i_USE_BASE,     //1 = EX/2nd base (MA); 0 = fetch PC (IF)
    input   wire    [1:0]   i_EN_MODE,      //addend gate: 0 NULL, 1 FORCE, 2 USE_T, 3 USE_NT
    input   wire            i_R_T,          //running SR.T (latched leaving EX, no forward)
    input   wire            i_PC_INC,       //1: inject +2 @ bit1 for the sequential fetch PC+2

    /* RESULT */
    output  wire    [31:0]  o_ADDR
);

//Folding the fetch PC INTO i_AGU_A (to delete the per-bit FETCH_PC input and pack each adder bit
//into 1 Cyclone V ALM) was attempted and REVERTED: the shared base register clobbered the held
//second-access address on MAC/RMW/TAS.B cycles the pipeline did not preload. So the fetch mux stays;
//each arith-mode bit sees {A[i], FETCH_PC[i], B[i], agu_en} (CV Handbook Vol1, ALM Arith Mode, p.1-8).

//en folds T into the second-addend gate; one of {0, 1, T, ~T} per the ID mode - one shared wire
wire            agu_en = (i_EN_MODE == 2'd0) ? 1'b0  :   //NULL : REG/POSTINC/MAC, JMP/JSR/RTS/RTE, fetch
                         (i_EN_MODE == 2'd1) ? 1'b1  :   //FORCE: disp/index/PREDEC, uncond branch
                         (i_EN_MODE == 2'd2) ? i_R_T :   //USE_T : BT (taken when T=1)
                                              ~i_R_T;    //USE_NT: BF (taken when T=0)

//X-addend: EX/2nd base vs live fetch PC (the IF/MA time-share select)
wire    [31:0]  agu_x  = i_USE_BASE ? i_AGU_A : i_FETCH_PC;
//Y-addend: the offset, nulled unless en (the gate folds into the per-bit adder LUT)
wire    [31:0]  agu_y  = agu_en ? i_AGU_B : 32'd0;
//Carry inject: PC+2 only (carry into bit1; PC[0]=0 aligned). PREDEC -step lives in i_AGU_B.
wire    [31:0]  agu_ci = i_PC_INC ? 32'd2 : 32'd0;

//One carry chain; the A pass-through and en-gated B fold into the per-bit arith-mode pre-adders
assign  o_ADDR = agu_x + agu_y + agu_ci;

endmodule

`default_nettype none
