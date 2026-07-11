`default_nettype wire

/*
    DMAC channel register quad (SH7709S section 11, pp.331-342).

    One instance per channel: Fig 11.1's block repeated four times is
    SARn/DARn/DMATCRn/CHCRn plus its iteration datapath (address/count
    update, ch2's reload image + 4-counter). The channel feature
    asymmetry of pp.336-342 is parameterized:
      HAS_EXT      ch0/1: external request bits RL/AM/AL (18:16), DS (6)
      HAS_RELOAD   ch2:   source address reload bit RO (19)
      HAS_INDIRECT ch3:   indirect addressing bit DI (20)
    Absent-feature bits: write invalid, read 0 (p.336) - which also
    starves the reload/indirect datapaths where the bit can't set.

    Write rules (table 11.2 notes, p.332): TE (CHCR[1]) is write-0-only -
    a hardware set outranks a same-edge clear, write-1 holds; DMATCR
    bits 31:24 read 0 / write-ignored; 16-bit partial access keeps the
    untouched half (the parent delivers lane-aligned data + a byte mask).
    CHCR clears on power-on AND manual reset; SAR/DAR/DMATCR are
    "undefined" at reset on silicon - implemented as reset-to-0, tests
    never rely on it.
*/

module dmac_channel #(
    parameter               CH_ID        = 0,       //channel number (debug/comments only)
    parameter               HAS_EXT      = 1'b0,    //ch0/1: DREQ/DACK control bits exist
    parameter               HAS_RELOAD   = 1'b0,    //ch2: RO bit exists
    parameter               HAS_INDIRECT = 1'b0     //ch3: DI bit exists
) (
    /* CLOCK AND RESET - clears on any reset flavor (p.332) */
    input   wire            i_RST_n,
    input   wire            i_CLK,
    input   wire            i_CEN,

    /* REGISTER ACCESS - one-cycle strobes from the parent's P-bus decode */
    input   wire            i_WR_SAR,
    input   wire            i_WR_DAR,
    input   wire            i_WR_TCR,       //TCR = DMATCR, the 24-bit transfer count register
    input   wire            i_WR_CHCR,
    input   wire    [31:0]  i_WDATA,        //pre-aligned to register bit lanes (big-endian)
    input   wire    [3:0]   i_WMASK,        //byte-lane mask, [3] = bits 31:24

    /* TRANSFER ENGINE HOOKS - the iteration datapath of fig 11.1 */
    input   wire            i_UPD,          //sequencer: one transfer unit completed
    input   wire    [1:0]   i_UPD_MASK,     //{DAR, SAR} step enables - single-address units
                                            //only step the memory-side register (fig 11.10)
    input   wire            i_EN,           //this channel's live enable (ch2 reload counter:
                                            //any enable drop resets the 4-count, p.373)

    /* REGISTER READ-BACK */
    output  wire    [31:0]  o_SAR,
    output  wire    [31:0]  o_DAR,
    output  wire    [23:0]  o_TCR,
    output  wire    [31:0]  o_CHCR
);

///////////////////////////////////////////////////////////
//////  Register Storage
////

logic   [31:0]  sar;                        //next source address during transfer (p.333)
logic   [31:0]  dar;                        //next destination address during transfer (p.334)
logic   [23:0]  tcr;                        //remaining transfer count; 0 = 16M max (p.335)
logic   [31:0]  sar_init;                   //SAR as last written: the ch2 reload image
                                            //(fig 11.22; pruned where RO can never set)
logic   [1:0]   ro_cnt;                     //ch2 reload 4-transfer counter (fig 11.22)
//CHCR fields (pp.336-342); absent-feature bits stay 0 forever
logic           di;                         //CHCR[20]: ch3 indirect address mode
logic           ro;                         //CHCR[19]: ch2 source address reload
logic           rl;                         //CHCR[18]: DRAK polarity (1 = active-high)
logic           am;                         //CHCR[17]: DACK in read(0)/write(1) dual cycle
logic           al;                         //CHCR[16]: DACK polarity (1 = active-high)
logic   [1:0]   dm, sm;                     //dest/source address mode: fixed/inc/dec
logic   [3:0]   rs;                         //resource select (request source, p.339)
logic           ds;                         //CHCR[6]: DREQ low-level(0)/falling-edge(1)
logic           tm;                         //CHCR[5]: cycle-steal(0)/burst(1)
logic   [1:0]   ts;                         //transmit size: byte/word/long/16-byte
logic           ie, te, de;                 //interrupt enable / transfer end / enable



///////////////////////////////////////////////////////////
//////  Register Writes + Iteration Datapath
////

/*
    i_UPD marks one completed transfer unit (the dual R+W pair): SAR/DAR
    step by the TS size per their SM/DM modes (fixed/inc/dec - the 11
    code is prohibited, treated as fixed), DMATCR decrements, and TE
    sets on the unit that brings the count to 0 (DMATCR=1 -> last;
    DMATCR=0 programs 16M, the natural wrap gives that for free, p.335).
    A same-edge CPU register write wins over the update (the manual
    forbids writing a running channel's registers anyway, section 11.6);
    the TE set outranks everything.
*/

//address step: byte/word/long/16-byte -> 1/2/4/16 (p.337); ch3 indirect
//steps SAR (the pointer table) by 4 regardless of TS (p.339)
wire    [31:0]  step = (ts == 2'b11) ? 32'd16 : ts[1] ? 32'd4 : ts[0] ? 32'd2 : 32'd1;
wire    [31:0]  sar_step = di ? 32'd4 : step;
wire    [31:0]  sar_nx = sm[1] ? sar - sar_step : sm[0] ? sar + sar_step : sar;
wire    [31:0]  dar_nx = dm[1] ? dar - step : dm[0] ? dar + step : dar;

//ch2 source reload (section 11.3.6): every 4th completed transfer returns
//SAR to its written image instead of stepping; 8/16/32-bit sizes only
wire            sar_reload = ro && (ro_cnt == 2'd3);

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        sar <= 32'd0;
        dar <= 32'd0;
        tcr <= 24'd0;
        sar_init <= 32'd0;
        ro_cnt   <= 2'd0;
        {di, ro, rl, am, al} <= 5'd0;
        dm  <= 2'd0;
        sm  <= 2'd0;
        rs  <= 4'd0;
        {ds, tm} <= 2'd0;
        ts  <= 2'd0;
        {ie, te, de} <= 3'd0;
    end
    else begin if(i_CEN) begin
        //unit completion first; a same-edge bus write below overrides per lane
        if(i_UPD) begin
            if(i_UPD_MASK[0]) sar <= sar_reload ? sar_init : sar_nx;
            if(i_UPD_MASK[1]) dar <= dar_nx;
            tcr <= tcr - 24'd1;
        end

        //reload 4-counter: any enable drop (DE/DME clear, TE set, NMI, AE,
        //reset) clears the count but NOT SAR/DAR/DMATCR - the p.373 restriction
        //(software must re-program all three before restarting a reload run)
        if(!i_EN)            ro_cnt <= 2'd0;
        else if(i_UPD && ro) ro_cnt <= ro_cnt + 2'd1;

        //byte-lane writes: a 16-bit access keeps the untouched half (p.332 note 2);
        //a SAR write refreshes the reload image alongside
        for(int b = 0; b < 4; b++) begin
            if(i_WR_SAR && i_WMASK[b]) sar[b*8 +: 8]      <= i_WDATA[b*8 +: 8];
            if(i_WR_SAR && i_WMASK[b]) sar_init[b*8 +: 8] <= i_WDATA[b*8 +: 8];
            if(i_WR_DAR && i_WMASK[b]) dar[b*8 +: 8]      <= i_WDATA[b*8 +: 8];
        end
        for(int b = 0; b < 3; b++) begin    //DMATCR[31:24] write-ignored (p.332 note 3)
            if(i_WR_TCR && i_WMASK[b]) tcr[b*8 +: 8] <= i_WDATA[b*8 +: 8];
        end

        //absent-feature bits load constant 0 (write invalid, read 0, p.336):
        //gating the VALUE, not the assignment, keeps a real flop D input -
        //an if(PARAM)-guarded assign makes Quartus infer a reset-only latch
        if(i_WR_CHCR) begin
            if(i_WMASK[2]) begin            //bits 23:16 - the channel-exclusive controls
                di <= HAS_INDIRECT ? i_WDATA[20] : 1'b0;
                ro <= HAS_RELOAD   ? i_WDATA[19] : 1'b0;
                rl <= HAS_EXT      ? i_WDATA[18] : 1'b0;
                am <= HAS_EXT      ? i_WDATA[17] : 1'b0;
                al <= HAS_EXT      ? i_WDATA[16] : 1'b0;
            end
            if(i_WMASK[1]) begin            //bits 15:8
                dm <= i_WDATA[15:14];
                sm <= i_WDATA[13:12];
                rs <= i_WDATA[11:8];
            end
            if(i_WMASK[0]) begin            //bits 7:0 (TE handled below)
                ds <= HAS_EXT ? i_WDATA[6] : 1'b0;
                tm <= i_WDATA[5];
                ts <= i_WDATA[4:3];
                ie <= i_WDATA[2];
                de <= i_WDATA[0];
            end
        end

        //TE: count exhaustion sets (outranks a same-edge write); write-1
        //never sets, write-0 clears after reading 1 (p.341). NMI/AE/DE-clear
        //endings do NOT set TE (p.374)
        if(i_UPD && tcr == 24'd1)        te <= 1'b1;
        else if(i_WR_CHCR && i_WMASK[0]) te <= te & i_WDATA[1];
    end end
end



///////////////////////////////////////////////////////////
//////  Read-Back
////

assign  o_SAR  = sar;
assign  o_DAR  = dar;
assign  o_TCR  = tcr;
assign  o_CHCR = {11'd0, di, ro, rl, am, al, dm, sm, rs, 1'b0, ds, tm, ts, ie, te, de};

endmodule

`default_nettype none
