`default_nettype none 

module ahb2apb_bridge #( 
    parameter NUM_SLAVES = 8,           // number of APB slaves behind this bridge
    parameter ADDR_WIDTH = 32,          // width of address bus on both sides
    parameter DATA_WIDTH = 32,          // width of data bus on both sides
    parameter SLAVE_ADDR_BITS = 10,          // address bits allotted to each slave's local window
    parameter TIMEOUT_CYCLES = 10,          // cycles of !pready before a stalled APB access is aborted
    parameter [ADDR_WIDTH-1:0] BASE_ADDR = 32'h0000_0000 // base address of the mapped slave region
)(
    // AHB-Lite Interface
    input wire hclk,      
    input wire hresetn,
    input wire hsel,     
    input wire hready,    
    input wire [ADDR_WIDTH-1:0] haddr,     
    input wire [1:0] htrans,    
    input wire hwrite,    
    input wire [2:0] hsize,    
    input wire [DATA_WIDTH-1:0] hwdata,    
    output wire [DATA_WIDTH-1:0] hrdata,   
    output wire hreadyout, 
    output wire hresp,    //0 response okay , 1 = error

    // APB4 Interface
    input  wire pclk,     
    input  wire presetn,   
    output wire [NUM_SLAVES-1:0] psel, // one-hot select, one bit per slave
    output wire  penable,   
    output wire [ADDR_WIDTH-1:0] paddr,
    output wire  pwrite,
    output wire [DATA_WIDTH-1:0] pwdata,
    output wire [DATA_WIDTH/8-1:0] pstrb, 
    input wire [(DATA_WIDTH*NUM_SLAVES)-1:0] prdata_bus,  // concatenated read-data bus from all slaves
    input wire [NUM_SLAVES-1:0] pready_bus,  // per-slave ready bits
    input wire [NUM_SLAVES-1:0] pslverr_bus // per-slave error bits
);

    localparam SEL_BITS = (NUM_SLAVES > 1) ? $clog2(NUM_SLAVES) : 1; // slave-select bus width, min 1 bit

    // AHB write operation signals
    wire xfer_valid;   // pulse: captured AHB transfer ready to cross into pclk domain
    wire [ADDR_WIDTH-1:0] xfer_addr;
    wire xfer_write; 
    wire [2:0] xfer_size; // captured AHB transfer size
    wire [DATA_WIDTH-1:0] xfer_wdata;   // captured AHB write data

    // AHB read operation signals
    wire resp_valid_h; // pulse: APB response has arrived back in hclk domain
    wire [DATA_WIDTH-1:0] resp_rdata_h; // read data returned to AHB master side
    wire resp_code_h;  // response code (OKAY/ERROR) returned to AHB master side

    ahb_capture_fsm #( 
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH) 
    ) ahb_cap (
        .hclk(hclk),         
        .hresetn(hresetn),   
        .hsel(hsel),          
        .hready(hready),       
        .haddr(haddr),        
        .htrans(htrans),      
        .hwrite(hwrite),       
        .hsize(hsize),         
        .hwdata(hwdata),      
        .hrdata(hrdata),       
        .hreadyout(hreadyout), 
        .hresp(hresp),       

        .xfer_valid(xfer_valid), 
        .xfer_addr(xfer_addr),   
        .xfer_write(xfer_write), 
        .xfer_size(xfer_size),   
        .xfer_wdata(xfer_wdata), 

        .resp_valid(resp_valid_h), 
        .resp_rdata(resp_rdata_h), 
        .resp_code(resp_code_h)   
    );

    //  REQUEST CDC
    localparam REQ_PAYLOAD_W = 1 + 3 + ADDR_WIDTH + DATA_WIDTH; // hwrite + hsize + addr_width + data_width

    wire [REQ_PAYLOAD_W-1:0] req_payload_p; 
    wire req_valid_p;   // pulse: request payload valid in pclk domain

// instantiate hclk->pclk request crossing
    cdc_handshake #( 
        .WIDTH(REQ_PAYLOAD_W) 
    ) cdc_req (
        .src_clk(hclk),       
        .src_rstn(hresetn),   
        .src_valid(xfer_valid), // source valid pulse from AHB capture
        .src_payload({xfer_write, xfer_size, xfer_addr, xfer_wdata}), // pack write/size/addr/wdata into one bus

        .dst_clk(pclk),         
        .dst_rstn(presetn),   
        .dst_valid(req_valid_p),   // destination valid pulse out
        .dst_payload(req_payload_p) // destination payload out
    );

    wire req_write_p = req_payload_p[REQ_PAYLOAD_W-1]; // unpack write flag MSB Bit
    wire [2:0] req_size_p  = req_payload_p[REQ_PAYLOAD_W-2 -: 3];  // unpack transfer size part select
    wire [ADDR_WIDTH-1:0] req_addr_p  = req_payload_p[DATA_WIDTH+ADDR_WIDTH-1 : DATA_WIDTH]; // unpack address
    wire [DATA_WIDTH-1:0] req_wdata_p = req_payload_p[DATA_WIDTH-1:0];  // unpack write data

    //  Address Decoder
    wire [SEL_BITS-1:0] slave_sel; 
    wire unmapped;  // flag: address falls outside any slave's window

    addr_decoder #( 
        .NUM_SLAVES(NUM_SLAVES),           
        .ADDR_WIDTH(ADDR_WIDTH),          
        .SLAVE_ADDR_BITS(SLAVE_ADDR_BITS),
        .BASE_ADDR(BASE_ADDR)      
    ) decoder (
        .addr(req_addr_p),     
        .slave_sel(slave_sel),  // decoded slave select out
        .unmapped(unmapped)     // decode-error flag out
    );

    // Muxing APB inputs
    wire [DATA_WIDTH-1:0] prdata_mux  = prdata_bus[slave_sel * DATA_WIDTH +: DATA_WIDTH]; // select read data from chosen slave
    wire pready_mux  = pready_bus[slave_sel];   // select ready bit from chosen slave
    wire pslverr_mux = pslverr_bus[slave_sel];  // select error bit from chosen slave

    //  APB Master FSM 
    wire apb_done;    // pulse: APB transaction completed
    wire [DATA_WIDTH-1:0] apb_rdata;   
    wire apb_error;   // error flag from completed APB transaction
    wire xfer_active; // flag: APB transaction currently in progress
    wire timeout_hit; // pulse: timeout monitor forced an abort

    apb_master_fsm #( 
        .NUM_SLAVES(NUM_SLAVES), 
        .ADDR_WIDTH(ADDR_WIDTH), 
        .DATA_WIDTH(DATA_WIDTH)  
    ) apb_interface (
        .pclk(pclk),               
        .presetn(presetn),         
        .req_valid(req_valid_p),  
        .req_addr(req_addr_p),     
        .req_write(req_write_p),   
        .req_size(req_size_p),     
        .req_wdata(req_wdata_p),  
        .req_slave_sel(slave_sel), 
        .req_unmapped(unmapped),   
        .timeout_hit(timeout_hit), 

        .psel(psel),         
        .penable(penable),  
        .paddr(paddr),       
        .pwrite(pwrite),     
        .pwdata(pwdata),     
        .pstrb(pstrb),       
        .prdata(prdata_mux),   
        .pready(pready_mux),   
        .pslverr(pslverr_mux), 

        .apb_done(apb_done),      
        .apb_rdata(apb_rdata),     
        .apb_error(apb_error),   
        .xfer_active(xfer_active)  
    );

    timeout_monitor #( 
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES) 
    ) timeout (
        .pclk(pclk),            
        .presetn(presetn),       
        .xfer_active(xfer_active), 
        .pready(pready_mux),     
        .timeout_hit(timeout_hit) 
    );

    // Response Error Mapper
    wire resp_valid_p;
    wire [DATA_WIDTH-1:0] resp_rdata_p; 
    wire resp_code_p;

    resp_error_mapper #( 
        .DATA_WIDTH(DATA_WIDTH) 
    ) resp_mapper (
        .pclk(pclk),           
        .presetn(presetn),    
        .apb_done(apb_done),   
        .apb_rdata(apb_rdata), 
        .apb_error(apb_error), 

        .resp_valid(resp_valid_p), 
        .resp_rdata(resp_rdata_p), 
        .resp_code(resp_code_p)  
    );

    // RESPONSE CDC 
    localparam RSP_PAYLOAD_W = 1 + DATA_WIDTH; // bit width of packed response payload

    wire [RSP_PAYLOAD_W-1:0] resp_payload_h; // packed response payload synchronized into hclk domain

// instantiate pclk->hclk response crossing
    cdc_handshake #( 
        .WIDTH(RSP_PAYLOAD_W) 
    ) cdc_ack (
        .src_clk(pclk),         
        .src_rstn(presetn),     
        .src_valid(resp_valid_p), 
        .src_payload({resp_code_p, resp_rdata_p}), 

        .dst_clk(hclk),             
        .dst_rstn(hresetn),         
        .dst_valid(resp_valid_h),  
        .dst_payload(resp_payload_h) 
    );

    assign resp_code_h  = resp_payload_h[RSP_PAYLOAD_W-1]; // unpack response code for AHB capture FSM
    assign resp_rdata_h = resp_payload_h[DATA_WIDTH-1:0];  // unpack response read data for AHB capture FSM

endmodule // end of ahb2apb_bridge
