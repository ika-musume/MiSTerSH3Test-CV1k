`default_nettype none

/*
    Shared geometry constants and small combinational helpers for the unified
    cache variants (cache_dpump, cache_tdp).

    The SH7709S unified cache is 16 kbytes, 4-way set associative, 16-byte
    lines (see SH7709S.PDF pp.103-104). That yields 256 sets per way:
        16384 / 4 ways / 16 bytes = 256 entries.
    Address slicing used throughout:
        addr[28:10] -> physical tag body (top 3 region bits are shadow, p.104)
        addr[11:4]  -> 8-bit set index
        addr[3:2]   -> 2-bit longword select inside a line
        addr[1]     -> halfword select inside a longword
*/

package cache_pkg;

localparam int unsigned WAYS        = 4;
localparam int unsigned SETS        = 256;
localparam int unsigned IDX_BITS    = 8;     //log2(SETS)
localparam int unsigned TAG_BITS    = 19;    //PA[28:10]
localparam int unsigned WAY_BITS    = 2;     //log2(WAYS)

//tag_of: the stored/compared tag is exactly PA[28:10]. The SH3 physical
//address space is 29 bits, so PA[31:29] are region shadow bits and are not
//part of the tag (p.104). Keeping it 19 bits trims the way comparators.
function automatic logic [TAG_BITS-1:0] tag_of(input logic [31:0] addr);
    tag_of = addr[28:10];
endfunction

//cacheable: P2 (101) and P4 (111) are non-cacheable control spaces.
function automatic logic cacheable(input logic [31:0] addr);
    cacheable = (addr[31:29] != 3'b101) && (addr[31:29] != 3'b111);
endfunction

/*
    6-bit pseudo-LRU, per SH7709S.PDF p.104 (Table 5.2). The six bits are one
    "which way is newer" flag per way-pair:
        lru[5]=(0,1) lru[4]=(0,2) lru[3]=(0,3) lru[2]=(1,2) lru[1]=(1,3) lru[0]=(2,3)
    For each pair, 0 means the lower-numbered way is the newer one.
*/

//lru_victim: the way to replace = the way that is older in all its pairs.
//Patterns are exactly Table 5.2 (matches the hardware's WayFromLRU).
function automatic logic [1:0] lru_victim(input logic [5:0] lru);
    casez(lru)
        6'b111???: lru_victim = 2'd0;   //0 older than 1,2,3
        6'b0??11?: lru_victim = 2'd1;   //1 older than 0,2,3
        6'b?0?0?1: lru_victim = 2'd2;   //2 older than 0,1,3
        6'b??0?00: lru_victim = 2'd3;   //3 older than 0,1,2
        default:   lru_victim = 2'd3;   //reset state 000000 -> way 3
    endcase
endfunction

//lru_victim_oh: ONE-HOT form of lru_victim. The four casez rows above are pairwise
//disjoint (each pair of rows conflicts on at least one bit), so every output bit is an
//independent 6-input function - one LUT each, no priority chain. Bit 3 also absorbs the
//all-zero default (no row matches -> way 3). Used where the victim way gates or selects
//per-way values in PARALLEL (dirty check, victim_tag), replacing the serial
//lru->encode->tag_rdata[victim] index chain on the 5 ns dispatch cone.
function automatic logic [3:0] lru_victim_oh(input logic [5:0] lru);
    lru_victim_oh[0] =   lru[5] &&  lru[4] &&  lru[3];
    lru_victim_oh[1] =  !lru[5] &&  lru[2] &&  lru[1];
    lru_victim_oh[2] =  !lru[4] && !lru[2] &&  lru[0];
    lru_victim_oh[3] = !(lru[5] &&  lru[4] &&  lru[3]) &&
                       !(!lru[5] && lru[2] &&  lru[1]) &&
                       !(!lru[4] && !lru[2] && lru[0]);
endfunction

//lru_update: mark 'way' most-recently-used by forcing every pair that
//contains it to point at it; the other pairs keep their bits.
function automatic logic [5:0] lru_update(input logic [1:0] way, input logic [5:0] lru);
    unique case(way)
        2'd0: lru_update = {3'b000,    lru[2:0]};                       //(0,1)(0,2)(0,3)=0
        2'd1: lru_update = {1'b1, lru[4:3], 2'b00, lru[0]};             //(0,1)=1 (1,2)(1,3)=0
        2'd2: lru_update = {lru[5], 1'b1, lru[3], 1'b1, lru[1], 1'b0};  //(0,2)(1,2)=1 (2,3)=0
        2'd3: lru_update = {lru[5:4], 1'b1, lru[2], 1'b1, 1'b1};        //(0,3)(1,3)(2,3)=1
    endcase
endfunction

//pick_inst: extract the addressed 16-bit opcode from a 32-bit longword.
//'upper_half' is addr[1]; 'be' selects big-endian halfword ordering.
function automatic logic [15:0] pick_inst(
    input logic [31:0] word_data,
    input logic        upper_half,
    input logic        be
);
    if(be) pick_inst = upper_half ? word_data[15:0]  : word_data[31:16];
    else   pick_inst = upper_half ? word_data[31:16] : word_data[15:0];
endfunction

//merge_word: apply byte strobes of a store onto the existing cached word.
function automatic logic [31:0] merge_word(
    input logic [31:0] old_word,
    input logic [31:0] new_word,
    input logic [3:0]  strobe
);
    merge_word = old_word;
    if(strobe[0]) merge_word[7:0]   = new_word[7:0];
    if(strobe[1]) merge_word[15:8]  = new_word[15:8];
    if(strobe[2]) merge_word[23:16] = new_word[23:16];
    if(strobe[3]) merge_word[31:24] = new_word[31:24];
endfunction

endpackage

`default_nettype none
