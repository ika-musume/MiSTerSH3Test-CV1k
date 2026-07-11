//
// to_bcd.sv — free-running 32-bit binary -> 10-digit BCD converter.
// SystemVerilog port of ToBCD.vhd (MiSTerDDR3Test, (c) FPGAzumSpass /
// Robert Peip — GPLv3 derivative work). One conversion per 12 clocks;
// dataOut updates atomically at the end of each pass.
//

module to_bcd
(
   input              clk,

   input       [31:0] dataIn,
   output logic [39:0] dataOut = '0
);

   localparam [1:0] S_IDLE = 2'd0, S_CALC = 2'd1, S_OUT = 2'd2;

   logic [1:0]  state    = S_IDLE;
   logic [35:0] data     = '0;
   logic [39:0] result   = '0;
   logic [3:0]  position = 4'd9;

   always @(posedge clk) begin : p_bcd
      logic [35:0] newData;
      logic [39:0] prod;

      case (state)

         S_IDLE: begin
            state    <= S_CALC;
            data     <= {4'h0, dataIn};
            result   <= '0;
            position <= 4'd9;
         end

         S_CALC: begin
            newData = data;
            if      (data >= 36'h218711A00) begin newData = data - 36'h218711A00; result[position*4 +: 4] <= 4'd9; end
            else if (data >= 36'h1DCD65000) begin newData = data - 36'h1DCD65000; result[position*4 +: 4] <= 4'd8; end
            else if (data >= 36'h1A13B8600) begin newData = data - 36'h1A13B8600; result[position*4 +: 4] <= 4'd7; end
            else if (data >= 36'h165A0BC00) begin newData = data - 36'h165A0BC00; result[position*4 +: 4] <= 4'd6; end
            else if (data >= 36'h12A05F200) begin newData = data - 36'h12A05F200; result[position*4 +: 4] <= 4'd5; end
            else if (data >= 36'h0EE6B2800) begin newData = data - 36'h0EE6B2800; result[position*4 +: 4] <= 4'd4; end
            else if (data >= 36'h0B2D05E00) begin newData = data - 36'h0B2D05E00; result[position*4 +: 4] <= 4'd3; end
            else if (data >= 36'h077359400) begin newData = data - 36'h077359400; result[position*4 +: 4] <= 4'd2; end
            else if (data >= 36'h03B9ACA00) begin newData = data - 36'h03B9ACA00; result[position*4 +: 4] <= 4'd1; end
            else                                                                  result[position*4 +: 4] <= 4'd0;

            prod = newData * 8'd10;
            data <= prod[35:0];

            if (position == 4'd0) state <= S_OUT;
            else                  position <= position - 1'd1;
         end

         S_OUT: begin
            state   <= S_IDLE;
            dataOut <= result;
         end

         default: state <= S_IDLE;

      endcase
   end

endmodule
