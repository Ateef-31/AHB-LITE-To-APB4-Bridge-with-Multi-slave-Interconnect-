`default_nettype none

module apb_master_fsm #(
    parameter NUM_SLAVES = 8,
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter [DATA_WIDTH-1:0] DEF_RDATA = {DATA_WIDTH{1'b0}}
)(
    input wire pclk,
    input wire presetn,

    // Backend Request Interface
    input wire req_valid,
    input wire [ADDR_WIDTH-1:0] req_addr,
    input wire req_write,
    input wire [2:0] req_size,
    input wire [DATA_WIDTH-1:0] req_wdata,
    input wire [(NUM_SLAVES>1?$clog2(NUM_SLAVES):1)-1:0] req_slave_sel,
    input wire req_unmapped,
    input wire timeout_hit,

    // APB Bus Interface
    output reg [NUM_SLAVES-1:0] psel,
    output reg penable,
    output reg [ADDR_WIDTH-1:0] paddr,
    output reg pwrite,
    output reg [DATA_WIDTH-1:0] pwdata,
    output reg [DATA_WIDTH/8-1:0] pstrb, // APB4 write strobes
    input  wire [DATA_WIDTH-1:0] prdata,
    input  wire pready,
    input  wire pslverr,

    // Backend Response Interface
    output reg apb_done,
    output reg [DATA_WIDTH-1:0] apb_rdata,
    output reg apb_error,
    output reg xfer_active
);

    // Standard APB protocol states
    localparam IDLE = 2'd0;
    localparam SETUP = 2'd1;
    localparam ACCESS = 2'd2;

    reg [1:0] ct_st, nt_st;

    // Generates APB4 byte strobes (pstrb) based on transfer size and address alignment.
    // For example, a byte write (size 3'b000) at address offset 2 yields a strobe of 4'b0100.
    function automatic [DATA_WIDTH/8-1:0] gen_strb;
        input [2:0] size;
        input [$clog2(DATA_WIDTH/8)-1:0] byte_off;
        reg [DATA_WIDTH/8-1:0] mask;
        begin
            case (size)
                3'b000:  mask = 1;  // 8-bit (1 byte)
                3'b001:  mask = 3;  // 16-bit (2 bytes)
                3'b010:  mask = 15; // 32-bit (4 bytes)
                default: mask = {(DATA_WIDTH/8){1'b1}};
            endcase
            gen_strb = mask << byte_off;
        end
    endfunction

    // State flip-flops
    always @(posedge pclk or negedge presetn) begin
        if (!presetn)
            ct_st <= IDLE;
        else
            ct_st <= nt_st;
    end

    // Next State Logic
    always @(*) begin
        nt_st = ct_st;
        case (ct_st)
            IDLE: begin
                // Only start a bus transaction if the address is legally mapped in the system
                if (req_valid && !req_unmapped)
                    nt_st = SETUP;
            end
            SETUP: begin
                // APB protocol strictly requires transitioning from SETUP to ACCESS in one clock cycle
                nt_st = ACCESS;
            end
            ACCESS: begin
                // Stay in ACCESS phase (inserting wait states) until the slave asserts pready,
                // or an external watchdog timeout forces an abort to prevent a bus hang.
                if (timeout_hit || pready)
                    nt_st = IDLE;
            end
            default: nt_st = IDLE;
        endcase
    end

    // Output Logic
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            psel <= {NUM_SLAVES{1'b0}};
            penable <= 1'b0;
            paddr <= {ADDR_WIDTH{1'b0}};
            pwrite <= 1'b0;
            pwdata <= {DATA_WIDTH{1'b0}};
            pstrb <= {(DATA_WIDTH/8){1'b0}};
            apb_done <= 1'b0;
            apb_rdata <= {DATA_WIDTH{1'b0}};
            apb_error <= 1'b0;
            xfer_active <= 1'b0;
        end else begin
            // Default apb_done to 0 so it acts as a single-cycle pulse back to the backend
            apb_done <= 1'b0;
            apb_error <= 1'b0;

            case (ct_st)
                IDLE: begin
                    penable <= 1'b0;

                    if (req_valid && !req_unmapped) begin
                        // Latch request info. Decode the binary slave ID into a one-hot
                        // psel vector to select the specific peripheral on the APB bus.
                        // These are registered on the IDLE->SETUP edge, so the bus shows
                        // psel=1 and penable=0 while the FSM is in SETUP.
                        paddr <= req_addr;
                        pwrite <= req_write;
                        pwdata <= req_wdata;
                        pstrb <= req_write ? gen_strb(req_size, req_addr[$clog2(DATA_WIDTH/8)-1:0]) : {(DATA_WIDTH/8){1'b0}};
                        psel <= (1'b1 << req_slave_sel);
                        xfer_active <= 1'b1;
                    end else begin
                        psel <= {NUM_SLAVES{1'b0}};
                        xfer_active <= 1'b0;

                        if (req_valid && req_unmapped) begin
                            // Immediately terminate the transaction with a fault if the address is invalid.
                            // Do NOT broadcast this to the APB bus to prevent unpredictable slave behavior.
                            apb_done <= 1'b1;
                            apb_error <= 1'b1;
                            apb_rdata <= DEF_RDATA;
                        end
                    end
                end

                SETUP: begin
                    // APB Setup Phase: psel, paddr, pwrite, etc. were set in IDLE and are stable.
                    // Assert penable FSM moves to the Access phase.
                    penable <= 1'b1;
                end

                ACCESS: begin
                    if (timeout_hit) begin
                        // Forcefully abort if the slave is unresponsive
                        psel <= {NUM_SLAVES{1'b0}};
                        penable <= 1'b0;
                        xfer_active <= 1'b0;
                        apb_done <= 1'b1;
                        apb_error <= 1'b1;
                        apb_rdata <= {DATA_WIDTH{1'b0}};
                    end else if (pready) begin
                        // Slave has completed the transfer. Safely deassert bus control signals.
                        psel <= {NUM_SLAVES{1'b0}};
                        penable <= 1'b0;
                        xfer_active <= 1'b0;
                        apb_done <= 1'b1;

                        // Capture the slave's error response and read data.
                        // As a security/safety measure, force read data to 0 on writes.
                        apb_error <= pslverr;
                        apb_rdata <= pwrite ? {DATA_WIDTH{1'b0}} : prdata;
                    end
                end

                default: begin
                    // Recover safely from any illegal state encoding
                    psel <= {NUM_SLAVES{1'b0}};
                    penable <= 1'b0;
                    xfer_active <= 1'b0;
                end
            endcase
        end
    end
endmodule
