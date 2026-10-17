`default_nettype none
module ahb_capture_fsm #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input  wire hclk,
    input  wire hresetn,
    input  wire hsel,
    input  wire hready,
    input  wire [ADDR_WIDTH-1:0] haddr,
    input  wire [1:0] htrans,
    input  wire hwrite,
    input  wire [2:0] hsize,
    input  wire [DATA_WIDTH-1:0] hwdata,

    output reg  [DATA_WIDTH-1:0] hrdata,
    output reg  hreadyout,
    output reg  hresp,

    output reg  xfer_valid,
    output wire [ADDR_WIDTH-1:0] xfer_addr,
    output wire xfer_write,
    output wire [2:0] xfer_size,
    output reg  [DATA_WIDTH-1:0] xfer_wdata,

    input  wire resp_valid,
    input  wire [DATA_WIDTH-1:0] resp_rdata,
    input  wire resp_code
);

    localparam TRANS_NONSEQ = 2'b10;
    localparam TRANS_SEQ = 2'b11;
    localparam RESP_OKAY = 1'b0;
    localparam RESP_ERROR = 1'b1;

    localparam IDLE  = 2'b00;
    localparam DATA  = 2'b01;
    localparam WAIT  = 2'b10;
    localparam ERROR = 2'b11;

    reg [1:0] ct_st, nt_st;

    reg [ADDR_WIDTH-1:0] pending_addr;
    reg pending_write;
    reg [2:0] pending_size;

    wire xfer_req = hsel && hready && (htrans == TRANS_NONSEQ || htrans == TRANS_SEQ);
    
    localparam BYTES_PER_BEAT = DATA_WIDTH/8;

    // Transfer wider than the data bus
    wire size_illegal = ((32'd1 << hsize) > BYTES_PER_BEAT);
    
    // Low address bits below the transfer size must be zero
    wire [ADDR_WIDTH-1:0] align_mask = ({{(ADDR_WIDTH-1){1'b0}}, 1'b1} << hsize) - 1'b1;
    wire misaligned = |(haddr & align_mask);
    
    wire bad_req = size_illegal || misaligned;

    assign xfer_addr = pending_addr;
    assign xfer_write = pending_write;
    assign xfer_size = pending_size;

    always @(posedge hclk or negedge hresetn) begin
        if (!hresetn) 
            ct_st <= IDLE;
        else 
           ct_st <= nt_st;
    end

    // Next State Logic
    always @(*) begin
        nt_st = ct_st;
        case (ct_st)
            IDLE: begin
                if (xfer_req) begin
                      nt_st = bad_req ? ERROR : DATA; // Trap error immediately, skip backend transfer
                end
            end
            DATA: begin
                nt_st = WAIT;
            end
            WAIT: begin
                if (resp_valid) begin
                    if (resp_code == RESP_ERROR) 
                        nt_st = ERROR;
                    else 
                        nt_st = IDLE;
                end
            end
            ERROR: begin
                nt_st = IDLE;
            end
            default: nt_st = IDLE;
        endcase
    end
      always @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            hreadyout <= 1'b1;
            hresp <= RESP_OKAY;
            hrdata <= {DATA_WIDTH{1'b0}};
            xfer_valid <= 1'b0;
            xfer_wdata <= {DATA_WIDTH{1'b0}};
            pending_addr <= {ADDR_WIDTH{1'b0}};
            pending_write <= 1'b0;
            pending_size <= 3'b000;
        end else begin
            xfer_valid <= 1'b0;

            if (ct_st == IDLE && xfer_req && !bad_req) begin
                pending_addr  <= haddr;
                pending_write <= hwrite;
                pending_size  <= hsize;
            end

            case (nt_st)
                IDLE: begin
                    hreadyout <= 1'b1;
                    if (ct_st == ERROR)
                        hresp <= RESP_ERROR;
                    else
                        hresp <= RESP_OKAY;
                end
                DATA: begin
                    hreadyout <= 1'b0;
                    hresp <= RESP_OKAY;
                end
                WAIT: begin
                    hreadyout <= 1'b0;
                    hresp <= RESP_OKAY;
                    if (ct_st == DATA) begin
                        xfer_valid <= 1'b1;
                        xfer_wdata <= pending_write ? hwdata : {DATA_WIDTH{1'b0}};
                    end
                end
                ERROR: begin
                    hreadyout <= 1'b0;
                    hresp <= RESP_ERROR;
                  //  hrdata <= {DATA_WIDTH{1'b0}};
                end
            endcase

            if (nt_st == ERROR)
                hrdata <= {DATA_WIDTH{1'b0}};
            else if (ct_st == WAIT && resp_valid && (resp_code == RESP_OKAY))
                hrdata <= resp_rdata;
        end
    end
endmodule
