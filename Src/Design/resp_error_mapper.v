`default_nettype none

module resp_error_mapper #(
    parameter DATA_WIDTH = 32
)(
    input wire pclk,
    input wire presetn,
    
    // Inputs from the APB Master FSM
    input wire apb_done,
    input wire [DATA_WIDTH-1:0] apb_rdata,
    input wire apb_error,

    output reg resp_valid,
    output reg [DATA_WIDTH-1:0] resp_rdata,
    output reg resp_code
);

    localparam RESP_OKAY = 1'b0;
    localparam RESP_ERROR = 1'b1;

    // This block acts as a pipeline register (isolation stage) between the APB bus 
    // and the upstream interconnect. By registering the outputs here, we cut the 
    // combinational timing path from the APB slave's prdata/pslverr signals 
    // back to the master, which is crucial for meeting timing closure in SoC designs.
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            resp_valid <= 1'b0;
            resp_rdata <= {DATA_WIDTH{1'b0}};
            resp_code <= RESP_OKAY;
        end else begin
            // resp_valid acts as a 1-cycle delayed pulse of apb_done
            resp_valid <= apb_done;
            
            if (apb_done) begin
                // Latch the read data and map the APB protocol's error signal 
                // This holds the data stable for the CDC handshake or upstream FSM.
                resp_rdata <= apb_rdata;
                resp_code  <= apb_error ? RESP_ERROR : RESP_OKAY;
            end
        end
    end

endmodule
