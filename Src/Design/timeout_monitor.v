`default_nettype none

module timeout_monitor #(
    parameter TIMEOUT_CYCLES = 10
)(
    input wire pclk,
    input wire presetn,
    input wire xfer_active,
    input wire pready,
    output reg  timeout_hit
);

    localparam COUNTER_WIDTH = (TIMEOUT_CYCLES > 1) ? $clog2(TIMEOUT_CYCLES) : 1;
    reg [COUNTER_WIDTH-1:0] COUNT;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            COUNT       <= {COUNTER_WIDTH{1'b0}};
            timeout_hit <= 1'b0;
        end else begin
            // Default to 0 so timeout_hit generates a clean, single-cycle pulse
            timeout_hit <= 1'b0;
            
            if (!xfer_active) begin
                // No active transfer on the bus; keep the watchdog counter reset.
                COUNT <= {COUNTER_WIDTH{1'b0}};
            end else if (!pready) begin
                // A transfer is active, but the slave is inserting wait states (!pready).
                if (COUNT == TIMEOUT_CYCLES - 1) begin
                    // The slave has stalled the bus for the maximum allowed cycles.
                    // Fire the timeout abort pulse to force the master to drop the transfer,
                    // preventing a complete system deadlock.
                    timeout_hit <= 1'b1;
                    COUNT <= {COUNTER_WIDTH{1'b0}};
                end else begin
                    COUNT <= COUNT + 1'b1;
                end
            end else begin
                // The slave successfully responded (pready == 1) before the timeout.
                // Reset the counter for the next transaction.
                COUNT <= {COUNTER_WIDTH{1'b0}};
            end
        end
    end

endmodule
