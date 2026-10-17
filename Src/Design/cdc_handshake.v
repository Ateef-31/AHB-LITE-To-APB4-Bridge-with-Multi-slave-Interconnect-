`default_nettype none

module cdc_handshake #(
    parameter WIDTH = 32
)(
    input wire src_clk,
    input wire src_rstn,
    input wire src_valid,
    input wire [WIDTH-1:0] src_payload,

    input wire dst_clk,
    input wire dst_rstn,
    output reg dst_valid,
    output reg [WIDTH-1:0] dst_payload
);

    // Toggle registers used for the 2-phase handshake.
    // Instead of a 4-phase (req high -> ack high -> req low -> ack low),
    // a 2-phase handshake just flips the bit (0->1 or 1->0) to indicate a new event.
    reg req_toggle;
    reg ack_toggle;
    reg [WIDTH-1:0] payload_hold;

    reg ack_sync0, ack_sync1;
    reg req_sync0, req_sync1, req_sync2;
    reg src_busy_reg;

    // Source Clock Domain
    always @(posedge src_clk or negedge src_rstn) begin
        if (!src_rstn) begin
            req_toggle <= 1'b0;
            payload_hold <= {WIDTH{1'b0}};
            ack_sync0 <= 1'b0;
            ack_sync1 <= 1'b0;
            src_busy_reg <= 1'b0;
        end else begin
            // Standard 2-flop synchronizer to safely bring the destination's 
            // acknowledge toggle into the source clock domain.
            ack_sync0 <= ack_toggle;
            ack_sync1 <= ack_sync0;
            
            // The CDC is "busy" (in-flight) if our current request toggle state 
            // does not match the synchronized acknowledge toggle state.
            src_busy_reg <= (req_toggle != ack_sync1);

            // Only accept a new transaction if the previous one has been fully acknowledged.
            if (src_valid && !src_busy_reg) begin
                // Latch the multi-bit payload. It will be held perfectly stable 
                // while the req_toggle propagates, preventing CDC data tearing.
                payload_hold <= src_payload;
                req_toggle  <= ~req_toggle;
            end
        end
    end

    // Destination Clock Domain
    always @(posedge dst_clk or negedge dst_rstn) begin
        if (!dst_rstn) begin
            req_sync0 <= 1'b0;
            req_sync1 <= 1'b0;
            req_sync2 <= 1'b0;
            ack_toggle <= 1'b0;
            dst_valid <= 1'b0;
            dst_payload <= {WIDTH{1'b0}};
        end else begin
            // 3-flop shift register:
            // req_sync0 & req_sync1 act as the standard 2-flop synchronizer for metastability.
            // req_sync2 is used to delay the signal by one clock cycle for edge detection.
            req_sync0 <= req_toggle;
            req_sync1 <= req_sync0;
            req_sync2 <= req_sync1;
            // XOR edge detection: If sync1 and sync2 differ, a toggle occurred (either 0->1 or 1->0).
            if (req_sync1 != req_sync2) begin
                dst_valid <= 1'b1;
                
                // Because of the delay through the synchronizer flops, enough time 
                // has passed to guarantee 'payload_hold' is stable and safe to sample.
                dst_payload <= payload_hold;
                
                // Acknowledge the receipt by matching the ack_toggle to the received req state.
                ack_toggle <= req_sync1;
            end else begin
                dst_valid <= 1'b0;
            end
        end
    end
endmodule
