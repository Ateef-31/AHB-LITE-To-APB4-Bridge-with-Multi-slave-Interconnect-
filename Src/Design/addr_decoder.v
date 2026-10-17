`default_nettype none

module addr_decoder #(
    parameter NUM_SLAVES = 8,
    parameter ADDR_WIDTH = 32,
    parameter SLAVE_ADDR_BITS = 10,
    parameter [ADDR_WIDTH-1:0] BASE_ADDR = 32'h0000_0000 
)(
    input  wire [ADDR_WIDTH-1:0] addr,
    output reg [(NUM_SLAVES>1?$clog2(NUM_SLAVES):1)-1:0] slave_sel,
    output reg  unmapped
);

    localparam SEL_BITS = (NUM_SLAVES > 1) ? $clog2(NUM_SLAVES) : 1;
    
    // Create a mask to isolate the upper bits (e.g., masks out the bottom 13 bits)
    localparam [ADDR_WIDTH-1:0] BASE_MASK = ~((1 << (SLAVE_ADDR_BITS + SEL_BITS)) - 1);

    wire [SEL_BITS-1:0] eq_sel = addr[SLAVE_ADDR_BITS +: SEL_BITS];
    
    // Check if the upper bits of the incoming address match the upper bits of the BASE_ADDR
    wire base_mismatch = ((addr & BASE_MASK) != (BASE_ADDR & BASE_MASK));
    
    // Check if the target slave index is higher than the number of valid slaves
    wire eq_oob = (eq_sel >= NUM_SLAVES);

    always @(*) begin
        // Address is unmapped if either the base address is wrong OR the slave index doesn't exist
        unmapped  = base_mismatch || eq_oob;
        slave_sel = unmapped ? {SEL_BITS{1'b0}} : eq_sel;
    end

endmodule
