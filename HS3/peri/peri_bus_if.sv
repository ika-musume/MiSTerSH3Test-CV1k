`default_nettype wire

/*
    Peripheral-tier bus interfaces.

    IBus_2 is the SH7709S "I bus 2" of the block diagram (Fig 1.1, hw manual
    p.6): the register bus the BRIDGE drives into the I-bus-2 register owners
    (INTC, CPG/WDT, and later UDI). The BRIDGE performs ALL byte-lane work
    once - write payloads arrive right-justified, read values return
    right-justified and the bridge re-replicates them onto the 32-bit lanes -
    so slaves are plain register files with no wstrb logic.

    The P bus reuses this same shape: the BSC's in-built P bridge drives the
    REG_TMU/REG_RTC/REG_PORT legs for the P-bus peripherals (TMU, RTC,
    PFC/ports; SCI arrives later).
*/

interface IBus_2;
logic           stb;        //one-cycle access strobe (the bridge ACCESS cycle)
logic           we;         //1: write commits at the closing edge of the strobe cycle
logic   [1:0]   size;       //access size 0:byte 1:word 2:long (mirrors IBus_1 req_size)
logic   [7:0]   addr;       //byte address within the slave's decode window
logic   [31:0]  wdata;      //right-justified write payload
logic   [31:0]  rdata;      //right-justified read value, valid during the strobe cycle

modport master (
    output stb, we, size, addr, wdata,
    input  rdata
);

modport slave (
    input  stb, we, size, addr, wdata,
    output rdata
);
endinterface

`default_nettype none
