`default_nettype none

/*
    Shared integer-pipeline types and small combinational helpers.

    The package mirrors cache_pkg.sv: pipeline packet typedefs stay near the
    datapath, while leaf functions are kept outside int_pipe.sv. This keeps the
    main file focused on stage logic and leaves memory primitives in
    int_pipe_mem.sv.
*/

package int_pipe_pkg;

//Memory-operation type carried from decode through memory access.
typedef enum logic [2:0] {
    MEM_NONE,  //instruction performs no data access
    MEM_LOAD,  //memory value returns to a register
    MEM_STORE, //register value is written to memory
    MEM_MAC,   //MAC.W/MAC.L read two ordered memory operands
    MEM_RMW,   //atomic byte read-modify-write sequence
    MEM_PREF   //PREF: allocate a cache line; no data returned, no fault, no GPR write
} mem_op_t;

//Byte operation performed between MEM_RMW read and optional write requests.
typedef enum logic [2:0] {
    BYTE_NONE, //ordinary memory operation
    BYTE_TST,  //test memory byte without modifying it
    BYTE_AND,  //AND immediate into memory byte
    BYTE_OR,   //OR immediate into memory byte
    BYTE_XOR,  //XOR immediate into memory byte
    BYTE_TAS   //test zero and set bit seven
} byte_op_t;

//Encoded transfer size; values also provide log2(bytes) for address updates.
typedef enum logic [1:0] {
    SIZE_BYTE, //8-bit transfer
    SIZE_WORD, //16-bit transfer
    SIZE_LONG  //32-bit transfer
} mem_size_t;

//Effective-address circuit selection; see manual table 2.2, pp.28-31.
typedef enum logic [3:0] {
    ADDR_NONE,      //no memory address
    ADDR_REG,       //direct register-indirect address
    ADDR_PREDEC,    //decrement base before access
    ADDR_POSTINC,   //increment base after access
    ADDR_DISP,      //register plus scaled displacement
    ADDR_INDEX,     //register plus active-bank R0
    ADDR_GBR_DISP,  //GBR plus scaled displacement
    ADDR_GBR_INDEX, //GBR plus active-bank R0
    ADDR_PC_WORD,   //PC-relative word address
    ADDR_PC_LONG,   //aligned PC-relative longword address
    ADDR_MAC        //two post-increment addresses for MAC.W/MAC.L
} addr_op_t;

//Data-execution operation selected by instruction decode.
typedef enum logic [5:0] {
    ALU_PASS_A,
    ALU_PASS_B,
    ALU_ADD,
    ALU_ADDC,
    ALU_ADDV,
    ALU_SUB,
    ALU_SUBC,
    ALU_SUBV,
    ALU_AND,
    ALU_OR,
    ALU_XOR,
    ALU_NOT,
    ALU_TST,
    ALU_CMP_EQ,
    ALU_CMP_HS,
    ALU_CMP_GE,
    ALU_CMP_HI,
    ALU_CMP_GT,
    ALU_CMP_PZ,
    ALU_CMP_PL,
    ALU_CMP_STR,
    ALU_NEG,
    ALU_NEGC,
    ALU_EXTU_B,
    ALU_EXTU_W,
    ALU_EXTS_B,
    ALU_EXTS_W,
    ALU_SWAP_B,
    ALU_SWAP_W,
    ALU_XTRCT,
    ALU_MOVT,
    ALU_DT,
    ALU_DIV1,
    ALU_SHLL,
    ALU_SHLR,
    ALU_SHAR,
    ALU_ROTL,
    ALU_ROTR,
    ALU_ROTCL,
    ALU_ROTCR,
    ALU_SHLL2,
    ALU_SHLL8,
    ALU_SHLL16,
    ALU_SHLR2,
    ALU_SHLR8,
    ALU_SHLR16,
    ALU_SHAD,
    ALU_SHLD
} alu_op_t;

//EX result-mux selector; ID classifies each alu_op into one of four DEU units.
typedef enum logic [1:0] {
    ALU_CLASS_ARITH, //adder/subtractor group: ADD/SUB family, NEG, DT
    ALU_CLASS_LOGIC, //bitwise group: AND, OR, XOR, NOT
    ALU_CLASS_SHIFT, //constant shifts plus the SHAD/SHLD barrel shifter
    ALU_CLASS_MISC   //bypass, MOVT, sign/zero extension, byte/word swap, extract
} alu_class_t;

//DIV means divide; this compact command controls SR.M/Q/T initialization or one step.
typedef enum logic [1:0] {
    DIV_NONE,
    DIV_INIT_UNSIGNED,
    DIV_INIT_SIGNED,
    DIV_STEP
} div_op_t;

//Encoded control-register destination; CTRL_NONE means no architectural write.
typedef enum logic [2:0] {
    CTRL_NONE,
    CTRL_SR,
    CTRL_GBR,
    CTRL_VBR,
    CTRL_SSR,
    CTRL_SPC
} ctrl_dst_t;

//MACH/MACL command sent to the DSP unit; multiplies no longer cross the EX result path.
typedef enum logic [3:0] {
    MAC_NONE,      //instruction does not touch MACH/MACL
    MAC_MULL,      //MUL.L  : 32x32 low longword to MACL
    MAC_MULS_W,    //MULS.W : signed 16x16 to MACL
    MAC_MULU_W,    //MULU.W : unsigned 16x16 to MACL
    MAC_DMULS_L,   //DMULS.L: signed 32x32 product to MACH:MACL
    MAC_DMULU_L,   //DMULU.L: unsigned 32x32 product to MACH:MACL
    MAC_ACCUM_L,   //MAC.L  : signed 32x32 plus 64-bit accumulator
    MAC_ACCUM_W,   //MAC.W  : signed 16x16 plus 64-bit accumulator
    MAC_LOAD_MACH, //LDS Rm,MACH
    MAC_LOAD_MACL, //LDS Rm,MACL
    MAC_CLEAR      //CLRMAC clears both registers
} mac_cmd_t;

//Control-flow operation; delayed forms preserve one sequential instruction.
typedef enum logic [3:0] {
    BR_NONE,
    BR_BT,
    BR_BF,
    BR_BRA,
    BR_BSR,
    BR_BRAF,
    BR_BSRF,
    BR_JMP,
    BR_JSR,
    BR_RTS,
    BR_RTE
} branch_op_t;

//Unified D-side bus request descriptor. One builder drives the single D bus port;
//the MA sequencer phase-selects the EX first access or its own second access.
typedef struct packed {
    logic           valid; //request present this cycle
    logic           write; //memory write request
    logic   [1:0]   size;  //byte/word/long transfer size
    logic   [31:0]  addr;  //byte address sent to the cache
    logic   [31:0]  wdata; //store data aligned to lanes
    logic   [3:0]   wstrb; //byte write strobes
    logic           lock;  //atomic read-modify-write qualifier
    logic           pref;  //PREF line-allocate qualifier
} dbus_req_pkt_t;

//Predecode source routing: which GPR feeds each read/store port. Every SH source is
//one of four forms (n-field, m-field, R0, inactive-bank), so a 2-bit selector + used
//bit fully captures it - bank-free, so it can be computed in the cen_n->cen_p fetch
//window and latched into IF/ID ahead of decode. The physical id is reconstructed in ID
//by qualifying the selector with the CURRENT bank (see pd_qualify), keeping bank state
//out of predecode where it could go stale behind an unflushed LDC ...,SR / RTE stall.
typedef enum logic [1:0] {
    PD_SRC_N    = 2'd0,  //n-field GPR (dec_n_id)
    PD_SRC_M    = 2'd1,  //m-field GPR (dec_m_id)
    PD_SRC_R0   = 2'd2,  //R0 in the active bank (dec_r0_id)
    PD_SRC_BANK = 2'd3   //inactive-bank register (dec_bank_id), LDC/STC ...,Rn_BANK
} pd_src_sel_t;

typedef struct packed {
    logic           a_used;   //port A (src_a) reads a GPR
    pd_src_sel_t    a_sel;
    logic           b_used;   //port B (src_b) reads a GPR
    pd_src_sel_t    b_sel;
    logic           st_used;  //store-data port reads a GPR
    pd_src_sel_t    st_sel;
    //Predecoded GPR read-port assignment (BANK-FREE, see pd_need_n): the deep ID-time
    //hz-vs-dec compare cone reduces to instruction fields, so the cen_n read-address
    //captures (gpr_read_ctx / RAM address) see one registered mux select, not the cone.
    logic           need_a;   //some source needs the n-field line (read port 0 = port A)
    logic           rib;      //reads inactive bank: LDC/STC Rm_BANK picks dec_bank_id on port B
    //Fetch-time HAZARD classification (pure opcode functions, sim-asserted against the
    //ID-time originals in int_pipe.sv): these were the last live ifid.inst decodes on
    //the id_hazard -> id_issue -> IF/ID capture-enable cone.
    logic           reads_sr;    //STC SR,Rn / STC.L SR,@-Rn (id_reads_sr)
    logic           uses_mac;    //reads or writes MACH/MACL (id_uses_mac_state)
    logic           reads_cmisc; //reads GBR/VBR/SSR/SPC/PR (id_reads_ctrl_misc)
    //Fetch-time ADDRESSING-MODE classification (pure opcode functions, sim-asserted
    //against id_decode.addr_op in int_pipe.sv): the last live ifid.inst decodes on the
    //ID operand-select cone (fit4 ifid.inst -> sel_b -> idex.src_b legs).
    logic           agbr;        //a-side GBR override: ADDR_GBR_DISP or ADDR_GBR_INDEX
    logic           apc;         //a-side PC override: ADDR_PC_WORD or ADDR_PC_LONG
    logic           pdec;        //ADDR_PREDEC (@-Rn store forms; b rides the -step)
    logic           gbrx;        //ADDR_GBR_INDEX (byte RMW @(R0,GBR); b rides a's pick)
} pd_route_t;

//IF/ID means the register between instruction-fetch and instruction-decode stages.
typedef struct packed {
    logic           valid;       //packet contains a live instruction
    logic   [31:0]  pc;          //address of this instruction
    logic   [15:0]  inst;        //fixed-width SH instruction
    logic           fetch_fault; //instruction-port fault follows its PC
    logic           delay_slot;  //instruction belongs to a delayed branch
    pd_route_t      pd;          //predecoded (bank-free) source routing, latched at fetch
} ifid_t;

/*
    ID/EX means the register between decode and execute stages.
    It carries physical identities, resolved operands, controls, and fault metadata.
    E2 captures BRAM or forwarded values so EX receives registered operands.
*/
typedef struct packed {
    //Instruction identity
    logic           valid;
    logic   [31:0]  pc;
    logic   [15:0]  inst;
    logic           delay_slot;

    //Bank-qualified identities support hazards; values follow the E1 BRAM read
    logic           src_a_used;
    logic   [4:0]   src_a_id;
    logic   [31:0]  src_a_value;
    logic           src_b_used;
    logic   [4:0]   src_b_id;
    logic   [31:0]  src_b_value;
    logic           store_used;
    logic   [4:0]   store_id;
    logic   [31:0]  store_value;
    logic   [31:0]  immediate;

    //Two GPR write lanes support a result plus address update
    logic           gpr0_we;
    logic   [4:0]   gpr0_dst;
    logic           gpr1_we;
    logic   [4:0]   gpr1_dst;
    alu_op_t        alu_op;
    alu_class_t     alu_class;      //selects which EX unit drives the result mux

    //Memory controls consumed by address generation and MA
    mem_op_t        mem_op;
    mem_size_t      mem_size;
    addr_op_t       addr_op;
    logic           load_signed;
    byte_op_t       byte_op;        //memory byte test or modification
    logic           is_data;        //= valid && mem_op!=NONE, PRE-DECODED in ID; AGU time-share select (off the mem_op decode)
    logic   [1:0]   agu_en_mode;    //AGU addend gate, PRE-DECODED in ID (0 NULL: REG/POSTINC/MAC; 1 FORCE: disp/index/predec); off the EX addr_op decode

    //Control-flow controls consumed by the branch circuit
    branch_op_t     branch_op;
    logic           branch_delayed;
    logic           pr_link;

    //Dedicated architectural-register and single-bit SR writes
    logic           pr_we;
    ctrl_dst_t      ctrl_dst;       //LDC/LDC.L destination control register
    mac_cmd_t       mac_cmd;        //MACH/MACL multiply, load, or clear command for the DSP
    div_op_t        div_op;         //DIV0S, DIV0U, or DIV1 operation
    logic           t_write_decode;
    logic           t_decode_value;
    logic           s_write_decode;
    logic           s_decode_value;

    //Serialized events handled by the future CPU-state controller
    logic           event_trapa;
    logic   [7:0]   trapa_imm;
    logic           event_rte;
    logic           event_sleep;
    logic           event_ldtlb;

    //Decode and fetch exceptions carried to precise WB reporting
    logic           illegal;
    logic           privileged;
    logic           fetch_fault;
} idex_t;

/*
    EX/MA means the register between execute and memory-access stages.
    EX finishes arithmetic and address generation before creating this packet.
    Loads remain unavailable until the memory response creates MA/WB data.
*/
typedef struct packed {
    //Instruction identity
    logic           valid;
    logic   [31:0]  pc;
    logic   [15:0]  inst;
    logic           delay_slot;
    logic           dbr;         //instruction IS a delayed branch (its slot must follow
                                 //before an interrupt is accepted; see 4.5.3 pp.98-100)
    logic           nd_taken;    //non-delayed TAKEN branch (BT/BF): its commit-time
                                 //successor is the redirect target, not pc+2

    //Forwardable GPR results
    logic           gpr0_we;
    logic   [4:0]   gpr0_dst;
    logic   [31:0]  gpr0_data;
    logic           gpr1_we;
    logic   [4:0]   gpr1_dst;
    logic   [31:0]  gpr1_data;

    //Memory context for the response and rare second access.
    mem_op_t        mem_op;
    mem_size_t      mem_size;
    logic           load_signed;
    byte_op_t       byte_op;        //registered byte read-modify-write operation
    logic   [31:0]  mem_addr;
    logic   [31:0]  mem_addr_second;
    logic   [31:0]  store_wdata;
    logic   [3:0]   store_wstrb;

    //Dedicated register and SR updates
    mac_cmd_t       mac_cmd;        //MACH/MACL command forwarded to the DSP unit
    logic   [31:0]  mac_a;          //multiplicand, or load data for LDS to MACH/MACL
    logic   [31:0]  mac_b;          //multiplier operand
    logic           mac_saturate;   //captured SR.S for MAC.W/MAC.L saturation
    logic           pr_we;
    logic   [31:0]  pr_data;
    ctrl_dst_t      ctrl_dst;
    logic   [31:0]  ctrl_data;
    logic           t_we;
    logic           t_data;
    logic           s_we;
    logic           s_data;
    logic   [1:0]   mq_we;          //write enables for SR.M and SR.Q
    logic   [1:0]   mq_data;        //new SR.M and SR.Q values

    //CPU-state events
    logic           event_trapa;
    logic   [7:0]   trapa_imm;
    logic           event_rte;
    logic           event_sleep;
    logic           event_ldtlb;

    //Precise fault metadata
    logic           fault;
    logic   [2:0]   fault_cause;
    // Sideband address information for TEA and EXPEVT selection.
    logic           fault_write;
    logic   [31:0]  fault_addr;
} exma_t;

/*
    MA/WB means the register between memory-access and writeback stages.
    It contains final values only; WB qualifies architectural writes and events.
*/
typedef struct packed {
    //Instruction identity
    logic           valid;
    logic   [31:0]  pc;
    logic   [15:0]  inst;
    logic           delay_slot;
    logic           dbr;         //delayed branch (interrupt-defer marker, see exma_t)
    logic           nd_taken;    //non-delayed taken branch (see exma_t)

    //Final GPR commit values
    logic           gpr0_we;
    logic   [4:0]   gpr0_dst;
    logic   [31:0]  gpr0_data;
    logic           gpr1_we;
    logic   [4:0]   gpr1_dst;
    logic   [31:0]  gpr1_data;

    //Final dedicated-register and SR values
    mac_cmd_t       mac_cmd;        //MACH/MACL command applied at writeback
    logic   [31:0]  mac_a;          //multiplicand, or load data for LDS to MACH/MACL
    logic   [31:0]  mac_b;          //multiplier operand
    logic           mac_saturate;   //captured SR.S for MAC.W/MAC.L saturation
    logic           pr_we;
    logic   [31:0]  pr_data;
    ctrl_dst_t      ctrl_dst;
    logic   [31:0]  ctrl_data;
    logic           t_we;
    logic           t_data;
    logic           s_we;
    logic           s_data;
    logic   [1:0]   mq_we;
    logic   [1:0]   mq_data;

    //Retirement events for external state control
    logic           event_trapa;
    logic   [7:0]   trapa_imm;
    logic           event_rte;
    logic           event_sleep;
    logic           event_ldtlb;

    //Precise fault metadata
    logic           fault;
    logic   [2:0]   fault_cause;
    logic           fault_write;
    logic   [31:0]  fault_addr;
} mawb_t;

function automatic logic [4:0] active_gpr_id(
    input logic [3:0] logical_id,
    input logic       active_bank1
);
    //Physical IDs 0-7 are BANK0, 8-15 BANK1, and 16-23 shared R8-R15; see pp.19-22.
    if(logical_id < 4'd8) active_gpr_id = active_bank1 ? {2'b01, logical_id[2:0]} :
                                                        {2'b00, logical_id[2:0]};
    else                  active_gpr_id = {2'b10, logical_id[2:0]};
endfunction

function automatic logic [4:0] gpr_bram_address(
    input logic [3:0] logical_id,
    input logic       active_bank1
);
    logic bank_select;
    begin
        //Upper registers force the common area; lower registers select one bank.
        bank_select = ~logical_id[3] & active_bank1;
        gpr_bram_address = {logical_id[3], bank_select, logical_id[2:0]};
    end
endfunction

function automatic logic [4:0] inactive_bank_id(
    input logic [2:0] bank_index,
    input logic       active_bank1
);
    //Inactive R0-R7 bank occupies the BRAM half opposite the current SR.RB bank.
    inactive_bank_id = {1'b0, ~active_bank1, bank_index};
endfunction

function automatic logic [4:0] pd_qualify(
    input pd_src_sel_t sel,
    input logic [4:0]  n_id,     //dec_n_id  (active_gpr_id of the n-field)
    input logic [4:0]  m_id,     //dec_m_id  (active_gpr_id of the m-field)
    input logic [4:0]  r0_id,    //dec_r0_id (active_gpr_id of R0)
    input logic [4:0]  bank_id   //dec_bank_id (inactive-bank register)
);
    //Reconstruct the physical id in ID from the predecoded selector and the CURRENT-bank
    //qualified building blocks - a 4:1 mux, one LUT level, replacing the ~3-level decode.
    case(sel)
        PD_SRC_N:    pd_qualify = n_id;
        PD_SRC_M:    pd_qualify = m_id;
        PD_SRC_R0:   pd_qualify = r0_id;
        default:     pd_qualify = bank_id;  //PD_SRC_BANK
    endcase
endfunction

function automatic logic pd_need_n(
    input pd_src_sel_t     sel,
    input logic    [15:0]  inst
);
    //Does a source with this selector demand the n-field BRAM line? Bank-free reduction
    //of (hz_id == dec_n_id) && hz_id not an R0 mirror (ids 0/8), valid because
    //active_gpr_id is injective per bank, maps x to {0,8} iff x==0, and inactive-bank
    //ids never alias active ones (int_pipe_pkg encodings, pp.19-22):
    //  N-sel: match always, mirror iff n==0.  M-sel: match iff m==n, mirror iff m==0.
    //  R0-sel: always mirror-served.  BANK-sel: can never equal the n-line id.
    case(sel)
        PD_SRC_N: pd_need_n = inst[11:8] != 4'd0;
        PD_SRC_M: pd_need_n = (inst[7:4] == inst[11:8]) && (inst[7:4] != 4'd0);
        default:  pd_need_n = 1'b0;
    endcase
endfunction

function automatic pd_route_t pd_route(input logic [15:0] inst);
    //Bank-free mirror of the decode case's SOURCE routing only (int_pipe.sv). Which GPR
    //feeds src_a/src_b/store per opcode; used bits match the case's src_*_used exactly.
    //A sim assertion checks pd_qualify(pd_route) === id_decode.src_*_id every issue, so any
    //transcription slip surfaces on the 64-test suite before this is trusted (int_pipe.sv).
    logic [3:0] hi, lo, nn;
    logic       fam_n, fam_m;
    begin
        pd_route = '0;
        hi = inst[15:12];
        lo = inst[3:0];
        nn = inst[11:8];
        case(hi)
            4'h0: begin
                case(lo)
                    4'h3: if(inst[7:4] == 4'h0 || inst[7:4] == 4'h2 || inst[7:4] == 4'h8)
                              begin pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_N; end //BSRF/BRAF/PREF
                    4'h2: if(inst[7:4] <= 4'h4) ; //STC ctrl,Rn - no GPR source
                          else if(inst[7]) begin pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_BANK; end //STC Rm_BANK
                    4'h4, 4'h5, 4'h6: begin //MOV.x Rm,@(R0,Rn)
                        pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_N;
                        pd_route.b_used = 1'b1; pd_route.b_sel = PD_SRC_R0;
                        pd_route.st_used = 1'b1; pd_route.st_sel = PD_SRC_M;
                    end
                    4'h7: begin pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_N; //MUL.L
                                pd_route.b_used = 1'b1; pd_route.b_sel = PD_SRC_M; end
                    4'hC, 4'hD, 4'hE: begin //MOV.x @(R0,Rm),Rn
                        pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_M;
                        pd_route.b_used = 1'b1; pd_route.b_sel = PD_SRC_R0;
                    end
                    4'hF: begin pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_N; //MAC.L
                                pd_route.b_used = 1'b1; pd_route.b_sel = PD_SRC_M; end
                    default: ;
                endcase
            end
            4'h1: begin pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_N; //MOV.L Rm,@(disp,Rn)
                        pd_route.st_used = 1'b1; pd_route.st_sel = PD_SRC_M; end
            4'h2: begin
                case(lo)
                    4'h0, 4'h1, 4'h2, 4'h4, 4'h5, 4'h6: begin //MOV.x Rm,@Rn / @-Rn
                        pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_N;
                        pd_route.st_used = 1'b1; pd_route.st_sel = PD_SRC_M;
                    end
                    4'h7, 4'h8, 4'h9, 4'hA, 4'hB, 4'hC, 4'hD, 4'hE, 4'hF: begin //DIV0S/logic/CMP_STR/XTRCT/MUL.W
                        pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_N;
                        pd_route.b_used = 1'b1; pd_route.b_sel = PD_SRC_M;
                    end
                    default: ;
                endcase
            end
            4'h3: begin pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_N; //two-register arith/compare
                        pd_route.b_used = 1'b1; pd_route.b_sel = PD_SRC_M; end
            4'h4: begin //shifts/jumps/system: family always reads the n-field (base or operand)
                pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_N;
                if(lo == 4'hC || lo == 4'hD || lo == 4'hF) begin //SHAD/SHLD/MAC.W add the m-field
                    pd_route.b_used = 1'b1; pd_route.b_sel = PD_SRC_M;
                end
                else if(lo == 4'h3 && inst[7]) begin //STC.L Rm_BANK,@-Rn stores the bank register
                    pd_route.st_used = 1'b1; pd_route.st_sel = PD_SRC_BANK;
                end
            end
            4'h5: begin pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_M; end //MOV.L @(disp,Rm),Rn
            4'h6: begin pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_M; end //MOV/load/unary from Rm
            4'h7: begin pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_N; end //ADD #imm,Rn
            4'h8: begin
                case(nn)
                    4'h0, 4'h1: begin //MOV.x R0,@(disp,Rn)
                        pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_M;
                        pd_route.st_used = 1'b1; pd_route.st_sel = PD_SRC_R0;
                    end
                    4'h4, 4'h5: begin pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_M; end //MOV.x @(disp,Rm),R0
                    4'h8: begin pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_R0; end //CMP/EQ #imm,R0
                    default: ;
                endcase
            end
            4'hC: begin
                case(nn)
                    4'h0, 4'h1, 4'h2: begin pd_route.st_used = 1'b1; pd_route.st_sel = PD_SRC_R0; end //MOV.x R0,@(disp,GBR)
                    4'h8, 4'h9, 4'hA, 4'hB, 4'hC, 4'hD, 4'hE, 4'hF: begin //imm logic / byte GBR RMW on R0
                        pd_route.a_used = 1'b1; pd_route.a_sel = PD_SRC_R0;
                    end
                    default: ;
                endcase
            end
            default: ; //0x9/0xA/0xB/0xD/0xE: no GPR read/store port on the hazard path
        endcase

        //Port-assignment predecode (bank-free): folded here so the ID-time read-address
        //selects are registered pd bits, not the 3-source compare cone (see pd_need_n).
        //FLAT decode: composing pd_need_n over the case selectors synthesized ~9 levels
        //behind the fetch-window rsp_inst mux. The same function over instruction fields
        //directly is two family predicates (which encodings source the n-/m-field on any
        //port) AND the two field tests. Exhaustively checked against the selector form at
        //sim start (int_pipe.sv), which is in turn checked against the ID-time cone live.
        fam_n = (hi == 4'h0 && ((lo == 4'h3 && (inst[7:4] == 4'h0 ||
                                                inst[7:4] == 4'h2 || inst[7:4] == 4'h8)) ||
                                lo == 4'h4 || lo == 4'h5 || lo == 4'h6 ||
                                lo == 4'h7 || lo == 4'hF)) ||
                hi == 4'h1 || (hi == 4'h2 && lo != 4'h3) || hi == 4'h3 ||
                hi == 4'h4 || hi == 4'h7;
        fam_m = (hi == 4'h0 && (lo == 4'h4 || lo == 4'h5 || lo == 4'h6 || lo == 4'h7 ||
                                lo == 4'hC || lo == 4'hD || lo == 4'hE || lo == 4'hF)) ||
                hi == 4'h1 || (hi == 4'h2 && lo != 4'h3) || hi == 4'h3 ||
                (hi == 4'h4 && (lo == 4'hC || lo == 4'hD || lo == 4'hF)) ||
                hi == 4'h5 || hi == 4'h6 ||
                (hi == 4'h8 && (nn == 4'h0 || nn == 4'h1 || nn == 4'h4 || nn == 4'h5));
        pd_route.need_a = ((inst[11:8] != 4'd0) && fam_n) ||
                          ((inst[7:4] == inst[11:8]) && (inst[7:4] != 4'd0) && fam_m);
        pd_route.rib    = ((hi == 4'h0 && lo == 4'h2) ||
                           (hi == 4'h4 && lo == 4'h3)) && inst[7];   //LDC/STC Rm_BANK forms

        //Hazard classification (see the struct note; asserted vs the originals in sim).
        //reads_sr: STC SR,Rn (0n02) / STC.L SR,@-Rn (4n03).
        pd_route.reads_sr = (hi == 4'h0 && inst[7:0] == 8'h02) ||
                            (hi == 4'h4 && inst[7:0] == 8'h03);
        //uses_mac: every mac_cmd-setting opcode plus the STS/STS.L MACH/MACL readers -
        //MUL.L (0..7), CLRMAC (0028), MAC.L (0..F), STS MACH/MACL (0.{0,1}A),
        //MULS/MULU.W (2..E/F), DMULU/DMULS.L (3..5/D), MAC.W (4..F),
        //LDS/STS.L/LDS.L MACH/MACL (4.{0,1}{A,2,6}).
        pd_route.uses_mac = (hi == 4'h0 && lo == 4'h7) ||
                            (inst == 16'h0028) ||
                            (hi == 4'h0 && lo == 4'hF) ||
                            (hi == 4'h0 && lo == 4'hA && inst[7:4] <= 4'h1) ||
                            (hi == 4'h2 && (lo == 4'hE || lo == 4'hF)) ||
                            (hi == 4'h3 && (lo == 4'h5 || lo == 4'hD)) ||
                            (hi == 4'h4 && lo == 4'hF) ||
                            (hi == 4'h4 && (lo == 4'hA || lo == 4'h6 || lo == 4'h2) &&
                             inst[7:4] <= 4'h1);
        //reads_cmisc: GBR-relative addressing (C.{0,1,2,4,5,6} disp / C.{C,D,E,F} index),
        //the STC (0..2) / STC.L (4..3) supersets, STS PR (0n2A), STS.L PR (4n22), RTS,
        //and RTE (002B): its EX branch target is i_SPC and its restore reads SSR, so it
        //must stall behind an uncommitted LDC Rm,SPC/SSR (the handler-epilogue idiom).
        pd_route.reads_cmisc = (hi == 4'hC && (nn == 4'h0 || nn == 4'h1 || nn == 4'h2 ||
                                               nn == 4'h4 || nn == 4'h5 || nn == 4'h6 ||
                                               nn == 4'hC || nn == 4'hD || nn == 4'hE ||
                                               nn == 4'hF)) ||
                               (hi == 4'h0 && lo == 4'h2) ||
                               (hi == 4'h4 && lo == 4'h3) ||
                               (hi == 4'h0 && inst[7:0] == 8'h2A) ||
                               (hi == 4'h4 && inst[7:0] == 8'h22) ||
                               (inst == 16'h00_0B) ||
                               (inst == 16'h00_2B);

        //Addressing-mode class (see the struct note; asserted vs id_decode.addr_op).
        //Mirrors the decode case's addr_op assignments only - manual table 2.12, pp.50-52.
        //gbrx: TST.B/AND.B/XOR.B/OR.B #imm,@(R0,GBR) (C.{C-F}). agbr adds the GBR-disp
        //moves (C.{0,1,2} store / C.{4,5,6} load). apc: MOV.W/L @(disp,PC) (9./D.) -
        //MOVA (C7) is ALU-side, addr_op stays NONE. pdec: MOV.x Rm,@-Rn (2.{4,5,6}),
        //STS.L MACH/MACL/PR,@-Rn (4.{0,1,2}2), STC.L ctrl/bank,@-Rn (4n03/4n13..4n43/4nx3-bank).
        pd_route.gbrx = (hi == 4'hC) && nn[3:2] == 2'b11;
        pd_route.agbr = ((hi == 4'hC) && (nn == 4'h0 || nn == 4'h1 || nn == 4'h2 ||
                                          nn == 4'h4 || nn == 4'h5 || nn == 4'h6)) ||
                        pd_route.gbrx;
        pd_route.apc  = (hi == 4'h9) || (hi == 4'hD);
        pd_route.pdec = (hi == 4'h2 && (lo == 4'h4 || lo == 4'h5 || lo == 4'h6)) ||
                        (hi == 4'h4 && lo == 4'h2 && inst[7:4] <= 4'h2) ||
                        (hi == 4'h4 && lo == 4'h3 && (inst[7] || inst[7:4] <= 4'h4));
    end
endfunction

function automatic logic [31:0] decoded_gpr_value(
    input logic [4:0]  physical_id,
    input logic [4:0]  read_address_a,
    input logic [4:0]  read_address_b,
    input logic [31:0] read_data_a,
    input logic [31:0] read_data_b,
    input logic [31:0] r0_bank0,
    input logic [31:0] r0_bank1
);
    //The two encoded fields use BRAM ports; banked R0 mirrors supply source three.
    decoded_gpr_value = 32'd0;
    if(physical_id == read_address_a) decoded_gpr_value = read_data_a;
    else if(physical_id == read_address_b) decoded_gpr_value = read_data_b;
    else if(physical_id == 5'd0) decoded_gpr_value = r0_bank0;
    else if(physical_id == 5'd8) decoded_gpr_value = r0_bank1;
endfunction

function automatic logic [31:0] control_read_value(
    input logic [2:0]  selector,
    input logic [31:0] sr,
    input logic [31:0] gbr,
    input logic [31:0] vbr,
    input logic [31:0] ssr,
    input logic [31:0] spc
);
    //Selectors zero through four encode SR, GBR, VBR, SSR, and SPC respectively.
    case(selector)
        3'd0: control_read_value = sr;
        3'd1: control_read_value = gbr;
        3'd2: control_read_value = vbr;
        3'd3: control_read_value = ssr;
        default: control_read_value = spc;
    endcase
endfunction

function automatic ctrl_dst_t control_destination(input logic [2:0] selector);
    case(selector)
        3'd0: control_destination = CTRL_SR;
        3'd1: control_destination = CTRL_GBR;
        3'd2: control_destination = CTRL_VBR;
        3'd3: control_destination = CTRL_SSR;
        3'd4: control_destination = CTRL_SPC;
        default: control_destination = CTRL_NONE;
    endcase
endfunction

function automatic alu_class_t alu_class_of(input alu_op_t op);
    //Each op routes to exactly one EX unit; compares write no result, so use MISC.
    case(op)
        ALU_ADD, ALU_ADDC, ALU_ADDV, ALU_SUB, ALU_SUBC, ALU_SUBV,
        ALU_NEG, ALU_NEGC, ALU_DT, ALU_DIV1:
            alu_class_of = ALU_CLASS_ARITH;
        ALU_AND, ALU_OR, ALU_XOR, ALU_NOT:
            alu_class_of = ALU_CLASS_LOGIC;
        ALU_SHLL, ALU_SHLR, ALU_SHAR, ALU_ROTL, ALU_ROTR, ALU_ROTCL, ALU_ROTCR,
        ALU_SHLL2, ALU_SHLL8, ALU_SHLL16, ALU_SHLR2, ALU_SHLR8, ALU_SHLR16,
        ALU_SHAD, ALU_SHLD:
            alu_class_of = ALU_CLASS_SHIFT;
        default:
            alu_class_of = ALU_CLASS_MISC;
    endcase
endfunction

function automatic logic source_matches_load(
    input logic        source_used,
    input logic [4:0]  source_id,
    input logic        load_valid,
    input logic        load_write,
    input logic [4:0]  load_dst
);
    //A match is hazardous while the producer's load value is unavailable.
    source_matches_load = source_used && load_valid && load_write && source_id == load_dst;
endfunction

//EX-head forward lane: which REGISTERED word patches this operand at the head of EX.
//Replaces the ID-mux LIVE legs (the ex_result ALU tail and the ld_word aligner):
//the EX producer's result is read from EX/MA one cycle later (G0/G1), and the WB
//view (FWD_WB, or a drained G0/G1 producer) reads the registered MA/WB packet on
//its one live cycle, then a shadow register DEPOSITED from it (reg->reg; the r_t
//running-deposit model on operands - no live cache/completion term anywhere).
typedef enum logic [1:0] {
    FWD_NONE    = 2'd0,     //no patch: idex.src_* / agu_base_q as captured in ID
    FWD_EXMA_G0 = 2'd1,     //producer in EX at my ID -> exma.gpr0_data at my EX
    FWD_EXMA_G1 = 2'd2,     //producer address-update lane -> exma.gpr1_data
    FWD_WB      = 2'd3      //WB view: mawb word (live cycle) / deposited shadow
} fwd_lane_t;

//Lane pick, ID time, REGISTERED-field compares ONLY (no live cache/ALU term).
//Priority matches the old select chain: EX result (youngest) shadows the MA load;
//gpr1/gpr0 of one producer never both match (double writes suppressed in ID).
//The EX gpr0 leg excludes loads (word not ready until MA; the load-use interlock
//holds the consumer in ID instead, and the FWD_WB view catches the release: the
//consumer issues at the load's completion edge, so its first EX cycle IS the
//producer's one live mawb cycle).
//Fault/flush qualifiers omitted on the discard argument (see load_forward_active
//history): a faulting/killed producer flushes this consumer's packet with it.
function automatic fwd_lane_t fwd_lane_pick(
    input logic        source_used,
    input logic [4:0]  source_id,
    input idex_t       idex,           //producer candidate in EX
    input exma_t       exma            //older load candidate in MA
);
    logic ex_g1, ex_g0, ld_ma;
    begin
        ex_g1 = source_used && idex.valid &&
                idex.gpr1_we && idex.gpr1_dst == source_id;
        ex_g0 = source_used && idex.valid && idex.mem_op != MEM_LOAD &&
                idex.gpr0_we && idex.gpr0_dst == source_id;
        ld_ma = source_used && exma.valid && exma.mem_op == MEM_LOAD &&
                exma.gpr0_we && exma.gpr0_dst == source_id;
        fwd_lane_pick = ex_g1 ? FWD_EXMA_G1 :
                        ex_g0 ? FWD_EXMA_G0 :
                        ld_ma ? FWD_WB      : FWD_NONE;
    end
endfunction

//MA-only ID residue forward: the EX live leg and the MA load word moved to the
//EX-head lanes (fwd_lane_pick above), so ID folds ONLY the registered MA (exma)
//producer over the WB-lane / R0-mirror base residue. Registered fields throughout;
//exma.fault is a registered bit (no live AGU cone rides this select).
function automatic logic [31:0] ma_forward(
    input logic [31:0] base_value,     //WB-lane / R0-mirror residue for this source
    input logic        source_used,
    input logic [4:0]  source_id,
    input exma_t       exma
);
    logic        ma_g1, ma_take;
    logic [31:0] ma_val;
    begin
        //gpr1 is the address-update lane (ready in EX, loads included); gpr0
        //excludes loads (their word rides the EX-head SHADOW deposit instead).
        ma_g1   = source_used && exma.valid && !exma.fault &&
                  exma.gpr1_we && exma.gpr1_dst == source_id;
        ma_take = ma_g1 || (source_used && exma.valid && !exma.fault &&
                            exma.mem_op != MEM_LOAD &&
                            exma.gpr0_we && exma.gpr0_dst == source_id);
        ma_val  = ma_g1 ? exma.gpr1_data : exma.gpr0_data;
        ma_forward = ma_take ? ma_val : base_value;
    end
endfunction

//Select-only twin of ma_forward: does the MA producer shadow this source (steers
//the GPR BRAM words off the last-level operand mux onto the early residue).
function automatic logic ma_take_only(
    input logic        source_used,
    input logic [4:0]  source_id,
    input exma_t       exma
);
    begin
        ma_take_only = (source_used && exma.valid && !exma.fault &&
                        exma.gpr1_we && exma.gpr1_dst == source_id) ||
                       (source_used && exma.valid && !exma.fault &&
                        exma.mem_op != MEM_LOAD &&
                        exma.gpr0_we && exma.gpr0_dst == source_id);
    end
endfunction

function automatic logic [31:0] shift_reverse32(input logic [31:0] x);
    //Bit-reverse is pure wiring; it lets one right barrel also perform left shifts.
    for(int i = 0; i < 32; i = i + 1) shift_reverse32[i] = x[31 - i];
endfunction

function automatic logic [63:0] accumulated_result(
    input logic [63:0]        accumulator,
    input logic signed [63:0] product,
    input logic               word_operation,
    input logic               saturate
);
    //The three accumulate variants are formed in parallel and selected by
    //{word_operation, saturate}; the common non-saturating MAC just takes the plain
    //64-bit sum. Saturation overflow is read from the sum's sign/guard bits instead of
    //a wide magnitude compare, keeping the 64-bit add off the critical path (int_pipe.md
    //11.2). Results stay bit-identical to the manual definition; see pp.308-314.
    logic        [63:0] sum_full;       //plain modular MAC accumulate (SR.S = 0)
    logic signed [32:0] sum_w;          //MAC.W: signed 32+32 -> 33 bits, guard bit at [32]
    logic               w_pos, w_neg;   //MAC.W positive / negative int32 overflow
    logic signed [64:0] sum_l;          //MAC.L: 48-bit acc + 64-bit product, 65 bits
    logic               l_ovf;          //MAC.L 48-bit signed overflow (guard bits not sign-clean)
    logic        [63:0] res_w, res_l;   //saturated MAC.W and MAC.L results
    begin
        //Non-saturating accumulate (MAC.W and MAC.L when SR.S = 0): low 64 bits of acc+product.
        sum_full = accumulator + product;

        //MAC.W saturate: signed 32-bit accumulate of MACL with the 32-bit product. int32
        //overflow shows up as the 33-bit guard bit differing from the sign bit; MACH is
        //preserved except bit 32 (the MACH[0] overflow flag). See pp.308-314.
        sum_w       = $signed({accumulator[31], accumulator[31:0]}) +
                      $signed({product[31],     product[31:0]});
        w_pos       = ~sum_w[32] &  sum_w[31];          //operands positive, sum wrapped negative
        w_neg       =  sum_w[32] & ~sum_w[31];          //operands negative, sum wrapped positive
        res_w       = accumulator;
        res_w[32]   = (w_pos | w_neg) ? 1'b1 : accumulator[32];
        res_w[31:0] = w_pos ? 32'h7FFF_FFFF :
                      w_neg ? 32'h8000_0000 : sum_w[31:0];

        //MAC.L saturate: signed 48-bit accumulate; MACH[31:16] (result[63:48]) preserved.
        //Overflow = guard bits [64:47] are not a clean sign extension (neither all-0 nor
        //all-1); the 65-bit sign bit then picks the clamp direction. See pp.308-314.
        sum_l       = $signed({{17{accumulator[47]}}, accumulator[47:0]}) +
                      $signed({product[63], product});
        l_ovf       = |sum_l[64:47] & ~(&sum_l[64:47]);
        res_l       = accumulator;
        res_l[47:0] = !l_ovf      ? sum_l[47:0] :
                      sum_l[64]   ? 48'h8000_0000_0000 : 48'h7FFF_FFFF_FFFF;

        accumulated_result = !saturate      ? sum_full :
                             word_operation ? res_w : res_l;
    end
endfunction

endpackage

`default_nettype none
