`timescale 1ns/1ps
`default_nettype none

module tb_ahb2apb_bridge();

    parameter NUM_SLAVES      = 8;
    parameter ADDR_WIDTH      = 32;
    parameter DATA_WIDTH      = 32;
    parameter SLAVE_ADDR_BITS = 10;
    parameter TIMEOUT_CYCLES  = 10;

    localparam [31:0] BASE_ADDR     = 32'h0000_0000;
    localparam        OFFSET_BITS   = SLAVE_ADDR_BITS - 2;         // word offset bits within a slave
    localparam         WORDS_PER_SLV = (1 << OFFSET_BITS);
    localparam [31:0] UNMAPPED_ADDR = 32'h0000_0000 + (NUM_SLAVES << SLAVE_ADDR_BITS) + 32'h100;

    // ============================================================
    // AHB-LITE SIGNALS
    // ============================================================

    reg                         hclk;
    reg                         hresetn;
    reg                         hsel;
    reg                         hready;

    reg  [ADDR_WIDTH-1:0]       haddr;
    reg  [1:0]                  htrans;
    reg                         hwrite;
    reg  [2:0]                  hsize;
    reg  [DATA_WIDTH-1:0]       hwdata;

    wire [DATA_WIDTH-1:0]       hrdata;
    wire                        hreadyout;
    wire                        hresp;

    // ============================================================
    // APB SIGNALS
    // ============================================================

    reg                         pclk;
    reg                         presetn;

    wire [NUM_SLAVES-1:0]       psel;
    wire                        penable;
    wire [ADDR_WIDTH-1:0]       paddr;
    wire                        pwrite;
    wire [DATA_WIDTH-1:0]       pwdata;
    wire [DATA_WIDTH/8-1:0]     pstrb;

    wire [(DATA_WIDTH*NUM_SLAVES)-1:0] prdata_bus;
    wire [NUM_SLAVES-1:0]              pready_bus;
    wire [NUM_SLAVES-1:0]              pslverr_bus;
    
    // ============================================================
    // TEST CONTROL
    // ============================================================

    integer pass_count = 0;
    integer fail_count = 0;
    integer i, s,w;

    integer ahb_wait_states = 0;
    reg [31:0] ahb_wait_cnt;

    // ============================================================
    // PER-SLAVE APB MEMORY MODELS 
    // ============================================================
    reg [DATA_WIDTH-1:0] slave_mem [0:NUM_SLAVES-1][0:WORDS_PER_SLV-1];

    initial begin
        for (i = 0; i < NUM_SLAVES; i = i + 1)
            for (w = 0; w < WORDS_PER_SLV; w = w + 1)
                slave_mem[i][w] = {DATA_WIDTH{1'b0}};
    end

    // ============================================================
    // DUT
    // ============================================================

    ahb2apb_bridge #(
        .NUM_SLAVES      (NUM_SLAVES),
        .ADDR_WIDTH      (ADDR_WIDTH),
        .DATA_WIDTH      (DATA_WIDTH),
        .SLAVE_ADDR_BITS (SLAVE_ADDR_BITS),
        .TIMEOUT_CYCLES  (TIMEOUT_CYCLES)
    ) dut (
        .hclk        (hclk),
        .hresetn     (hresetn),
        .hsel        (hsel),
        .hready      (hready),
        .haddr       (haddr),
        .htrans      (htrans),
        .hwrite      (hwrite),
        .hsize       (hsize),
        .hwdata      (hwdata),
        .hrdata      (hrdata),
        .hreadyout   (hreadyout),
        .hresp       (hresp),

        .pclk        (pclk),
        .presetn     (presetn),
        .psel        (psel),
        .penable     (penable),
        .paddr       (paddr),
        .pwrite      (pwrite),
        .pwdata      (pwdata),
        .pstrb       (pstrb),
        .prdata_bus  (prdata_bus),
        .pready_bus  (pready_bus),
        .pslverr_bus (pslverr_bus)
    );

    // ============================================================
    // CLOCKS
    // ============================================================

    initial hclk = 1'b0;
    always #3.7 hclk = ~hclk;

    initial pclk = 1'b0;
    always #5.0 pclk = ~pclk;

    // ============================================================
    // RESET
    // ============================================================

    initial begin
        hresetn = 1'b0;
        presetn = 1'b0;

        repeat (5) @(posedge hclk);
        hresetn = 1'b1;

        repeat (5) @(posedge pclk);
        presetn = 1'b1;
    end
    // Counts APB setup phases: must NOT change for rejected requests
    integer apb_access_cnt = 0;
    always @(posedge pclk) begin
        if (presetn && (|psel) && !penable)
            apb_access_cnt <= apb_access_cnt + 1;
    
        // APB4: PSTRB must be 0 on reads
        if (presetn && (|psel) && !pwrite && (|pstrb)) begin
            $display("[%0t] FAIL: PSTRB non-zero on APB read (0x%h)", $time, pstrb);
            fail_count = fail_count + 1;
        end
    end 
    // ============================================================
    // AHB HREADY WAIT-STATE GENERATOR
    // ============================================================

    always @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            hready       <= 1'b1;
            ahb_wait_cnt <= 32'd0;
        end
        else if (hsel && (ahb_wait_cnt < ahb_wait_states)) begin
            hready       <= 1'b0;
            ahb_wait_cnt <= ahb_wait_cnt + 1'b1;
        end
        else begin
            hready       <= 1'b1;
            ahb_wait_cnt <= 32'd0;
        end
    end

    // ============================================================
    // APB SLAVE LOGIC (Memory + Error generation)
    // ============================================================

    integer apb_wait_states   [0:NUM_SLAVES-1];
    reg     force_never_ready [0:NUM_SLAVES-1];
    reg     force_slverr      [0:NUM_SLAVES-1];

    wire [OFFSET_BITS-1:0] word_idx = paddr[SLAVE_ADDR_BITS-1:2];

    genvar gi;
    generate
        for (gi = 0; gi < NUM_SLAVES; gi = gi + 1) begin : g_slave

            reg [31:0] wait_cnt;
            wire       access_s = psel[gi] && penable;
            // PREADY is asserted as soon as the configured wait count is met.
            // With apb_wait_states[gi] = 0, it is already high when the
            // selected APB transfer enters ACCESS, avoiding extra PCLK waits.
            wire       ready_s  = !force_never_ready[gi] &&
                                   (wait_cnt >= apb_wait_states[gi]);
            wire       access_complete = access_s && ready_s;

            always @(posedge pclk or negedge presetn) begin
                if (!presetn)
                    wait_cnt <= 32'd0;
                else if (access_s && !ready_s)
                    wait_cnt <= wait_cnt + 1'b1;
                else
                    wait_cnt <= 32'd0;
            end

            assign pready_bus[gi]  = force_never_ready[gi] ? 1'b0 :
                                      psel[gi]              ? ready_s : 1'b1;

            assign pslverr_bus[gi] = ready_s && force_slverr[gi];

            assign prdata_bus[gi*DATA_WIDTH +: DATA_WIDTH] = slave_mem[gi][word_idx];

            always @(posedge pclk) begin
                if (access_complete && pwrite && !force_slverr[gi]) begin
                    if (pstrb[0]) slave_mem[gi][word_idx][7:0]   <= pwdata[7:0];
                    if (pstrb[1]) slave_mem[gi][word_idx][15:8]  <= pwdata[15:8];
                    if (pstrb[2]) slave_mem[gi][word_idx][23:16] <= pwdata[23:16];
                    if (pstrb[3]) slave_mem[gi][word_idx][31:24] <= pwdata[31:24];
                end
            end

        end
    endgenerate

    // Burst waveform visibility: PRDATA is combinational from the selected
    // slave memory and should follow PADDR through 0x200, 0x204, 0x208, 0x20C.
    always @(posedge pclk) begin
        if (presetn && psel[0] && (paddr >= 32'h0000_0200) &&
            (paddr <= 32'h0000_020C)) begin
            #1;
            $display("[%0t] APB TRACE: PSEL0=%b PENABLE=%b PADDR=%08h PRDATA=%08h PREADY=%b",
                     $time, psel[0], penable, paddr,
                     prdata_bus[0 +: DATA_WIDTH], pready_bus[0]);
            case (paddr)
                32'h0000_0200: if (prdata_bus[0 +: DATA_WIDTH] !== 32'h2000_0000) begin
                    $display("[%0t] FAIL: PRDATA mismatch at PADDR=0x200", $time);
                    fail_count = fail_count + 1;
                end
                32'h0000_0204: if (prdata_bus[0 +: DATA_WIDTH] !== 32'h2000_0001) begin
                    $display("[%0t] FAIL: PRDATA mismatch at PADDR=0x204", $time);
                    fail_count = fail_count + 1;
                end
                32'h0000_0208: if (prdata_bus[0 +: DATA_WIDTH] !== 32'h2000_0002) begin
                    $display("[%0t] FAIL: PRDATA mismatch at PADDR=0x208", $time);
                    fail_count = fail_count + 1;
                end
                32'h0000_020C: if (prdata_bus[0 +: DATA_WIDTH] !== 32'h2000_0003) begin
                    $display("[%0t] FAIL: PRDATA mismatch at PADDR=0x20C", $time);
                    fail_count = fail_count + 1;
                end
            endcase
        end
    end

    // ============================================================
    // ADDRESS HELPER
    // ============================================================

    function [ADDR_WIDTH-1:0] slv_addr;
        input integer slave_idx;
        input integer word_off;
        begin
            slv_addr = BASE_ADDR + (slave_idx << SLAVE_ADDR_BITS) + (word_off << 2);
        end
    endfunction

    // ============================================================
    // AHB IDLE
    // ============================================================

    task automatic ahb_idle;
        begin
            hsel   = 1'b0;
            htrans = 2'b00;
            haddr  = {ADDR_WIDTH{1'b0}};
            hwrite = 1'b0;
            hwdata = {DATA_WIDTH{1'b0}};
            hsize  = 3'b010;
        end
    endtask

    // ============================================================
    // SINGLE AHB TRANSFER (32-bit - Default)
    // ============================================================

    task automatic ahb_transfer(
        input  [ADDR_WIDTH-1:0] addr,
        input                   write,
        input  [DATA_WIDTH-1:0] wdata,
        output [DATA_WIDTH-1:0] rdata,
        output                  resp_err
    );
        reg addr_done;
        reg data_done;
        begin
            @(posedge hclk);
            #1;

            hsel   = 1'b1;
            htrans = 2'b10;        // NONSEQ
            haddr  = addr;
            hwrite = write;
            hsize  = 3'b010;       // Fixed 32-bit size
            hwdata = {DATA_WIDTH{1'b0}};

            addr_done = 1'b0;
            while (!addr_done) begin
                @(posedge hclk);
                if ((hreadyout === 1'b1) && (hready === 1'b1))
                    addr_done = 1'b1;
            end

            #1;
            htrans = 2'b00;
            hsel   = 1'b0;
            hwdata = write ? wdata : {DATA_WIDTH{1'b0}};

            data_done = 1'b0;
            while (!data_done) begin
                @(posedge hclk);
                if ((hreadyout === 1'b1) && (hready === 1'b1)) begin
                    data_done = 1'b1;
                    rdata     = hrdata;
                    resp_err  = hresp;
                end
            end

            #1;
            ahb_idle;
        end
    endtask

    // ============================================================
    // SINGLE AHB TRANSFER (Custom HSIZE for Error Injection)
    // ============================================================

    task automatic ahb_transfer_size(
        input  [ADDR_WIDTH-1:0] addr,
        input                   write,
        input  [2:0]            custom_size,
        input  [DATA_WIDTH-1:0] wdata,
        output [DATA_WIDTH-1:0] rdata,
        output                  resp_err
    );
        reg addr_done;
        reg data_done;
        begin
            @(posedge hclk);
            #1;

            hsel   = 1'b1;
            htrans = 2'b10;        // NONSEQ
            haddr  = addr;
            hwrite = write;
            hsize  = custom_size;  // Injected custom size
            hwdata = {DATA_WIDTH{1'b0}};

            addr_done = 1'b0;
            while (!addr_done) begin
                @(posedge hclk);
                if ((hreadyout === 1'b1) && (hready === 1'b1))
                    addr_done = 1'b1;
            end

            #1;
            htrans = 2'b00;
            hsel   = 1'b0;
            hwdata = write ? wdata : {DATA_WIDTH{1'b0}};

            data_done = 1'b0;
            while (!data_done) begin
                @(posedge hclk);
                if ((hreadyout === 1'b1) && (hready === 1'b1)) begin
                    data_done = 1'b1;
                    rdata     = hrdata;
                    resp_err  = hresp;
                end
            end

            #1;
            ahb_idle;
        end
    endtask

    // ============================================================
    // CHECKERS
    // ============================================================

    task automatic check_write(
        input [ADDR_WIDTH-1:0] addr,
        input [DATA_WIDTH-1:0] data,
        input integer          slave_idx,
        input integer          word_off
    );
        reg [DATA_WIDTH-1:0] rdata_dummy;
        reg                  resp_dummy;
        begin
            ahb_transfer(addr, 1'b1, data, rdata_dummy, resp_dummy);

            if (resp_dummy !== 1'b0) begin
                $display("[%0t] FAIL: write to 0x%08h (slv %0d) got HRESP=ERROR unexpectedly",
                          $time, addr, slave_idx);
                fail_count = fail_count + 1;
            end
            else if (slave_mem[slave_idx][word_off] !== data) begin
                $display("[%0t] FAIL: write to 0x%08h (slv %0d) - mem = 0x%08h, expected 0x%08h",
                          $time, addr, slave_idx, slave_mem[slave_idx][word_off], data);
                fail_count = fail_count + 1;
            end
            else begin
                $display("[%0t] PASS: write 0x%08h -> addr 0x%08h (slave %0d)",
                          $time, data, addr, slave_idx);
                pass_count = pass_count + 1;
            end
        end
    endtask

    task automatic check_read(
        input [ADDR_WIDTH-1:0] addr,
        input [DATA_WIDTH-1:0] expected,
        input integer          slave_idx
    );
        reg [DATA_WIDTH-1:0] rdata;
        reg                  resp_err;
        begin
            ahb_transfer(addr, 1'b0, {DATA_WIDTH{1'b0}}, rdata, resp_err);

            if (resp_err !== 1'b0) begin
                $display("[%0t] FAIL: read from 0x%08h (slv %0d) got HRESP=ERROR unexpectedly",
                          $time, addr, slave_idx);
                fail_count = fail_count + 1;
            end
            else if (rdata !== expected) begin
                $display("[%0t] FAIL: read from 0x%08h (slv %0d) = 0x%08h, expected 0x%08h",
                          $time, addr, slave_idx, rdata, expected);
                fail_count = fail_count + 1;
            end
            else begin
                $display("[%0t] PASS: read 0x%08h == 0x%08h (slave %0d)",
                          $time, addr, rdata, slave_idx);
                pass_count = pass_count + 1;
            end
        end
    endtask

    task automatic check_timeout(
        input [ADDR_WIDTH-1:0] addr,
        input integer          slave_idx
    );
        reg [DATA_WIDTH-1:0] rdata;
        reg                  resp_err;
        begin
            force_never_ready[slave_idx] = 1'b1;

            ahb_transfer(addr, 1'b0, {DATA_WIDTH{1'b0}}, rdata, resp_err);

            force_never_ready[slave_idx] = 1'b0;

            if (resp_err !== 1'b1) begin
                $display("[%0t] FAIL: expected HRESP=ERROR on slave %0d timeout", $time, slave_idx);
                fail_count = fail_count + 1;
            end
            else begin
                $display("[%0t] PASS: slave %0d timeout correctly produced HRESP=ERROR", $time, slave_idx);
                pass_count = pass_count + 1;
            end
        end
    endtask

    task automatic check_slverr(
        input [ADDR_WIDTH-1:0] addr,
        input integer          slave_idx
    );
        reg [DATA_WIDTH-1:0] rdata;
        reg                  resp_err;
        begin
            force_slverr[slave_idx] = 1'b1;

            ahb_transfer(addr, 1'b0, {DATA_WIDTH{1'b0}}, rdata, resp_err);

            force_slverr[slave_idx] = 1'b0;

            if (resp_err !== 1'b1) begin
                $display("[%0t] FAIL: expected HRESP=ERROR on slave %0d PSLVERR", $time, slave_idx);
                fail_count = fail_count + 1;
            end
            else begin
                $display("[%0t] PASS: slave %0d PSLVERR correctly propagated to HRESP", $time, slave_idx);
                pass_count = pass_count + 1;
            end
        end
    endtask

    task automatic check_decode_error(
        input [ADDR_WIDTH-1:0] addr
    );
        reg [DATA_WIDTH-1:0] rdata;
        reg                  resp_err;
        begin
            ahb_transfer(addr, 1'b0, {DATA_WIDTH{1'b0}}, rdata, resp_err);

            if (resp_err !== 1'b1) begin
                $display("[%0t] FAIL: expected decode HRESP=ERROR for unmapped 0x%08h", $time, addr);
                fail_count = fail_count + 1;
            end
            else begin
                $display("[%0t] PASS: unmapped addr 0x%08h correctly returned HRESP=ERROR", $time, addr);
                pass_count = pass_count + 1;
            end
        end
    endtask

    // Added checker for hsize > 2 error scenario
    task automatic check_hsize_error(
        input [ADDR_WIDTH-1:0] addr,
        input integer          slave_idx,
        input integer          word_off
    );
        reg [DATA_WIDTH-1:0] rdata;
        reg                  resp_err;
        begin
            // 1. Write a known good value with a VALID size first
            ahb_transfer(addr, 1'b1, 32'hBEEF_CAFE, rdata, resp_err);
            
            // 2. Attempt invalid WRITE with hsize = 011 (64-bit)
            ahb_transfer_size(addr, 1'b1, 3'b011, 32'hDEAD_DEAD, rdata, resp_err);
            if (resp_err !== 1'b1) begin
                $display("[%0t] FAIL: invalid 64-bit write did NOT trigger HRESP=ERROR", $time);
                fail_count = fail_count + 1;
            end 
            else begin
                // Check memory via backdoor to ensure it wasn't overwritten
                if (slave_mem[slave_idx][word_off] === 32'hDEAD_DEAD) begin
                    $display("[%0t] FAIL: invalid write corrupted APB memory!", $time);
                    fail_count = fail_count + 1;
                end else begin
                    $display("[%0t] PASS: invalid write correctly blocked and returned HRESP=ERROR", $time);
                    pass_count = pass_count + 1;
                end
            end

            // 3. Attempt invalid READ with hsize = 100 (128-bit)
            ahb_transfer_size(addr, 1'b0, 3'b100, 32'h0, rdata, resp_err);
            if (resp_err !== 1'b1) begin
                $display("[%0t] FAIL: invalid 128-bit read did NOT trigger HRESP=ERROR", $time);
                fail_count = fail_count + 1;
            end 
            else if (rdata !== {DATA_WIDTH{1'b0}}) begin
                $display("[%0t] FAIL: invalid read rdata was not zeroed, got 0x%08h", $time, rdata);
                fail_count = fail_count + 1;
            end 
            else begin
                $display("[%0t] PASS: invalid read correctly returned HRESP=ERROR and zeroed rdata", $time);
                pass_count = pass_count + 1;
            end
        end
    endtask
    
    // Sub-word write: lane_data must already be placed on the correct byte lanes
task automatic check_sub_write(
    input [ADDR_WIDTH-1:0] byte_addr,
    input [2:0]            size,
    input [DATA_WIDTH-1:0] lane_data,
    input integer          slave_idx,
    input integer          word_off,
    input [DATA_WIDTH-1:0] expected_mem
);
    reg [DATA_WIDTH-1:0] rd;
    reg                  err;
    begin
        ahb_transfer_size(byte_addr, 1'b1, size, lane_data, rd, err);
        if (err !== 1'b0 || slave_mem[slave_idx][word_off] !== expected_mem) begin
            $display("[%0t] FAIL: size=%0d addr=0x%08h err=%b mem=0x%08h expected=0x%08h",
                     $time, size, byte_addr, err, slave_mem[slave_idx][word_off], expected_mem);
            fail_count = fail_count + 1;
        end else begin
            $display("[%0t] PASS: size=%0d @0x%08h -> mem 0x%08h", $time, size, byte_addr, expected_mem);
            pass_count = pass_count + 1;
        end
    end
endtask

    // Illegal request: must give ERROR, no APB access, memory untouched
    task automatic check_reject(
        input [ADDR_WIDTH-1:0] addr,
        input                  write,
        input [2:0]            size,
        input integer          slave_idx,
        input integer          word_off
    );
        reg [DATA_WIDTH-1:0] rd, mem_before;
        reg                  err;
        integer              cnt_before;
        begin
            mem_before = slave_mem[slave_idx][word_off];
            cnt_before = apb_access_cnt;
    
            ahb_transfer_size(addr, write, size, 32'hFFFF_FFFF, rd, err);
    
            if (err !== 1'b1) begin
                $display("[%0t] FAIL: %s size=%0d addr=0x%08h expected HRESP=ERROR",
                         $time, write ? "WR" : "RD", size, addr);
                fail_count = fail_count + 1;
            end else if (apb_access_cnt !== cnt_before) begin
                $display("[%0t] FAIL: rejected request (size=%0d addr=0x%08h) reached APB",
                         $time, size, addr);
                fail_count = fail_count + 1;
            end else if (slave_mem[slave_idx][word_off] !== mem_before) begin
                $display("[%0t] FAIL: rejected request modified memory", $time);
                fail_count = fail_count + 1;
            end else if (!write && rd !== {DATA_WIDTH{1'b0}}) begin
                $display("[%0t] FAIL: rejected read returned non-zero data 0x%08h", $time, rd);
                fail_count = fail_count + 1;
            end else begin
                $display("[%0t] PASS: %s size=%0d addr=0x%08h rejected cleanly",
                         $time, write ? "WR" : "RD", size, addr);
                pass_count = pass_count + 1;
            end
        end
    endtask


    // ============================================================
    // FOUR-BEAT INCR4-STYLE AHB SEQUENCE WITH WAIT STATES
    // ============================================================
    // Drives NONSEQ for the first beat and SEQ for the remaining beats.
    // Each address/control phase is held stable while stalled. HSEL remains
    // asserted across the burst; HTRANS is NONSEQ, SEQ, SEQ, SEQ (no IDLE
    // between beats). APB slaves are configured ready immediately so each
    // APB access phase lasts exactly one PCLK cycle (PENABLE high once).
    task automatic ahb_incr4_stalled_test;
        reg [ADDR_WIDTH-1:0] beat_addr;
        reg [DATA_WIDTH-1:0] rd;
        reg [DATA_WIDTH-1:0] expected_data;
        reg                  err;
        reg                  accepted;
        integer              beat;
        integer              wait_cycles;
        integer              stalls_for_previous;
        integer              timeout_count;
        integer              stalls [0:3];
        begin
            $display("");
            $display("=== Test 11: INCR4 read with one-cycle APB access phases ===");
            $display("HTRANS sequence: IDLE -> NONSEQ -> SEQ -> SEQ -> SEQ -> IDLE");
            $display("HADDR sequence : 0x00000200 -> 204 -> 208 -> 20C");

            // Unique values make PRDATA visibly change with PADDR.
            slave_mem[0][(32'h200 >> 2)] = 32'h2000_0000;
            slave_mem[0][(32'h204 >> 2)] = 32'h2000_0001;
            slave_mem[0][(32'h208 >> 2)] = 32'h2000_0002;
            slave_mem[0][(32'h20C >> 2)] = 32'h2000_0003;
            // No APB wait states: PREADY is high on the first access cycle,
            // so PENABLE must be high for only one PCLK cycle per transfer.
            apb_wait_states[0] = 0;

            for (beat = 0; beat < 4; beat = beat + 1)
                stalls[beat] = 0;

            // Present the first address phase as NONSEQ.
            @(negedge hclk);
            hsel   = 1'b1;
            htrans = 2'b10;
            haddr  = 32'h0000_0200;
            hwrite = 1'b0;
            hsize  = 3'b010;
            hwdata = {DATA_WIDTH{1'b0}};

            // Each address is held stable until HREADYOUT/HREADY accept it.
            // The APB slave is immediately ready, so each APB access phase is
            // one PCLK cycle; the bridge still controls AHB response timing.
            // This gives NONSEQ, SEQ, SEQ, SEQ without IDLE bubbles.
            for (beat = 0; beat < 4; beat = beat + 1) begin
                accepted = 1'b0;
                timeout_count = 0;
                wait_cycles = 0;

                while (!accepted && timeout_count < 1000) begin
                    @(posedge hclk);
                    // Sample handshake before the DUT's NBA updates.
                    if ((hreadyout === 1'b1) && (hready === 1'b1)) begin
                        accepted = 1'b1;
                        if (beat > 0) begin
                            // This handshake completes the previous read
                            // data phase while accepting this SEQ address.
                            rd = hrdata;
                            err = hresp;
                            expected_data = 32'h2000_0000 + (beat - 1);
                            if (err !== 1'b0) begin
                                $display("[%0t] FAIL: beat %0d response HRESP=%b",
                                         $time, beat, err);
                                fail_count = fail_count + 1;
                            end else if (rd !== expected_data) begin
                                $display("[%0t] FAIL: read data for address %08h = %08h expected %08h",
                                         $time, 32'h200 + ((beat-1)*4), rd, expected_data);
                                fail_count = fail_count + 1;
                            end else begin
                                $display("[%0t] PASS: read address %08h returned %08h",
                                         $time, 32'h200 + ((beat-1)*4), rd);
                                pass_count = pass_count + 1;
                            end
                        end
                    end else if (hreadyout === 1'b0) begin
                        wait_cycles = wait_cycles + 1;
                        if (beat > 0)
                            stalls[beat-1] = stalls[beat-1] + 1;
                    end
                    timeout_count = timeout_count + 1;
                    #1;
                end

                if (!accepted) begin
                    $display("[%0t] FAIL: timeout accepting burst address beat %0d", $time, beat + 1);
                    fail_count = fail_count + 1;
                end

                // After beat 0/1/2 is accepted, present the next address
                // on the following falling edge. HTRANS never goes IDLE.
                if (beat < 3) begin
                    @(negedge hclk);
                    hsel   = 1'b1;
                    htrans = 2'b11;
                    haddr  = 32'h0000_0200 + ((beat + 1) * 4);
                    hwrite = 1'b0;
                    hsize  = 3'b010;
                    hwdata = {DATA_WIDTH{1'b0}};
                    $display("[%0t] DRIVE beat %0d: HADDR=%08h HTRANS=%b (next address held during stall)",
                             $time, beat + 2, haddr, htrans);
                end
            end

            // The fourth address has now been accepted. Remove the next
            // address phase by driving IDLE, then wait for final read data.
            @(negedge hclk);
            hsel   = 1'b0;
            htrans = 2'b00;
            haddr  = {ADDR_WIDTH{1'b0}};
            hwrite = 1'b0;

            accepted = 1'b0;
            timeout_count = 0;
            while (!accepted && timeout_count < 1000) begin
                @(posedge hclk);
                if ((hreadyout === 1'b1) && (hready === 1'b1)) begin
                    accepted = 1'b1;
                    rd = hrdata;
                    err = hresp;
                end else if (hreadyout === 1'b0) begin
                    stalls[3] = stalls[3] + 1;
                end
                timeout_count = timeout_count + 1;
                #1;
            end

            if (!accepted) begin
                $display("[%0t] FAIL: timeout waiting for final INCR4 read", $time);
                fail_count = fail_count + 1;
            end else if (err !== 1'b0) begin
                $display("[%0t] FAIL: final beat HRESP=%b", $time, err);
                fail_count = fail_count + 1;
            end else if (rd !== 32'h2000_0003) begin
                $display("[%0t] FAIL: final beat HRDATA=%08h expected=20000003", $time, rd);
                fail_count = fail_count + 1;
            end else begin
                $display("[%0t] PASS: final beat address 0000020C returned %08h", $time, rd);
                pass_count = pass_count + 1;
            end

            for (beat = 0; beat < 4; beat = beat + 1) begin
                if (stalls[beat] == 0) begin
                    $display("[%0t] FAIL: beat %0d did not observe HREADYOUT low", $time, beat + 1);
                    fail_count = fail_count + 1;
                end else begin
                    $display("[%0t] INFO: beat %0d observed HREADYOUT low for %0d sampled cycles",
                             $time, beat + 1, stalls[beat]);
                end
            end

            apb_wait_states[0] = 0;
            @(negedge hclk);
            ahb_idle;
        end
    endtask

    // ============================================================
    // MAIN TEST SEQUENCE
    // ============================================================

    initial begin

        ahb_idle;

        for (i = 0; i < NUM_SLAVES; i = i + 1) begin
            apb_wait_states[i]   = 0;
            force_never_ready[i] = 1'b0;
            force_slverr[i]      = 1'b0;
        end

        @(posedge hresetn);
        @(posedge presetn);
        repeat (5) @(posedge hclk);

        $display("");
        $display("==================================================");
        $display("   MULTI-SLAVE AHB2APB BRIDGE SELF-CHECK");
        $display("==================================================");
        $display("");

        // ========================================================
        // TEST 1: write/read every slave once - proves routing
        // ========================================================

        $display("=== Test 1: write+read every slave (routing check) ===");

        for (s = 0; s < NUM_SLAVES; s = s + 1) begin
            check_write(slv_addr(s, 0), 32'hA000_0000 + s, s, 0);
        end

        for (s = 0; s < NUM_SLAVES; s = s + 1) begin
            check_read(slv_addr(s, 0), 32'hA000_0000 + s, s);
        end

        // ========================================================
        // TEST 2: cross-slave isolation at the SAME offset
        // ========================================================

        $display("");
        $display("=== Test 2: cross-slave isolation ===");

        check_write(slv_addr(0, 5), 32'h1111_1111, 0, 5);
        check_write(slv_addr(1, 5), 32'h2222_2222, 1, 5);

        check_read(slv_addr(0, 5), 32'h1111_1111, 0);
        check_read(slv_addr(1, 5), 32'h2222_2222, 1);

        // ========================================================
        // TEST 3: independent per-slave wait states
        // ========================================================

        $display("");
        $display("=== Test 3: independent per-slave APB wait states ===");

        apb_wait_states[2] = 4;
        check_write(slv_addr(2, 10), 32'hDEAD_BEEF, 2, 10);
        check_read(slv_addr(2, 10), 32'hDEAD_BEEF, 2);
        apb_wait_states[2] = 0;

        // fast slave right after - proves slow slave 2 didn't wedge the bus
        check_write(slv_addr(3, 10), 32'hFEED_FACE, 3, 10);
        check_read(slv_addr(3, 10), 32'hFEED_FACE, 3);

        // ========================================================
        // TEST 4: slave timeout on one slave, recovery on another
        // ========================================================

        $display("");
        $display("=== Test 4: slave timeout + recovery on a different slave ===");

        check_timeout(slv_addr(4, 0), 4);

        check_write(slv_addr(5, 0), 32'h5A5A_5A5A, 5, 0);
        check_read(slv_addr(5, 0), 32'h5A5A_5A5A, 5);

        // ========================================================
        // TEST 5: PSLVERR on one slave, recovery on another
        // ========================================================

        $display("");
        $display("=== Test 5: PSLVERR + recovery on a different slave ===");

        check_slverr(slv_addr(6, 0), 6);

        check_write(slv_addr(7, 0), 32'h0BAD_F00D, 7, 0);
        check_read(slv_addr(7, 0), 32'h0BAD_F00D, 7);

        // ========================================================
        // TEST 6: unmapped address -> decode error
        // ========================================================

        $display("");
        $display("=== Test 6: unmapped address decode error ===");

        check_decode_error(UNMAPPED_ADDR);

        // recovery after decode error
        check_write(slv_addr(0, 20), 32'hCAFE_BABE, 0, 20);
        check_read(slv_addr(0, 20), 32'hCAFE_BABE, 0);

        // ========================================================
        // TEST 7: AHB-side wait states combined with slave access
        // ========================================================

        $display("");
        $display("=== Test 7: AHB HREADY wait states across multiple slaves ===");

        ahb_wait_states = 3;

        check_write(slv_addr(1, 30), 32'hAAAA_BBBB, 1, 30);
        check_read(slv_addr(1, 30), 32'hAAAA_BBBB, 1);

        check_write(slv_addr(3, 30), 32'hCCCC_DDDD, 3, 30);
        check_read(slv_addr(3, 30), 32'hCCCC_DDDD, 3);

        ahb_wait_states = 0;
        
        

        // ========================================================
        // TEST 8: Invalid HSIZE Error Handling
        // ========================================================
        
        $display("");
        $display("=== Test 8: Invalid HSIZE Error Trap and Zeroing ===");
        
        // Target Slave 0, offset 30
        check_hsize_error(slv_addr(0, 30), 0, 30);
        
                // ========================================================
        // TEST 9: legal sub-word writes (byte lanes / PSTRB)
        // ========================================================
        $display("");
        $display("=== Test 9: byte / halfword writes, strobe check ===");
        
        check_write(slv_addr(1, 40), 32'h1122_3344, 1, 40);
        //                      addr                 size    lane data      slv word expected
        check_sub_write(slv_addr(1,40) + 2, 3'b000, 32'h00AA_0000, 1, 40, 32'h11AA_3344); // byte, lane 2
        check_sub_write(slv_addr(1,40) + 0, 3'b000, 32'h0000_00BB, 1, 40, 32'h11AA_33BB); // byte, lane 0
        check_sub_write(slv_addr(1,40) + 3, 3'b000, 32'h5500_0000, 1, 40, 32'h55AA_33BB); // byte, lane 3
        check_sub_write(slv_addr(1,40) + 2, 3'b001, 32'hCCDD_0000, 1, 40, 32'hCCDD_33BB); // half, lanes 3:2
        check_sub_write(slv_addr(1,40) + 0, 3'b001, 32'h0000_EEFF, 1, 40, 32'hCCDD_EEFF); // half, lanes 1:0
        
        // ========================================================
        // TEST 10: misaligned requests rejected
        // ========================================================
        $display("");
        $display("=== Test 10: misaligned transfers rejected ===");
        
        // memory at slave 1, word 40 is 32'hCCDD_EEFF and must survive all of these
        check_reject(slv_addr(1,40) + 1, 1'b1, 3'b001, 1, 40); // half @ +1
        check_reject(slv_addr(1,40) + 3, 1'b1, 3'b001, 1, 40); // half @ +3
        check_reject(slv_addr(1,40) + 1, 1'b1, 3'b010, 1, 40); // word @ +1
        check_reject(slv_addr(1,40) + 2, 1'b1, 3'b010, 1, 40); // word @ +2
        check_reject(slv_addr(1,40) + 3, 1'b1, 3'b010, 1, 40); // word @ +3
        check_reject(slv_addr(1,40) + 1, 1'b0, 3'b001, 1, 40); // read, half @ +1
        check_reject(slv_addr(1,40) + 2, 1'b0, 3'b010, 1, 40); // read, word @ +2
        
        // back-to-back recovery: a legal transfer right after an error
        check_read(slv_addr(1, 40), 32'hCCDD_EEFF, 1);
        check_write(slv_addr(1, 41), 32'h0123_4567, 1, 41);

        // ========================================================
        // TEST 11: four-beat INCR4-style sequence with HREADYOUT stalls
        // ========================================================
        ahb_incr4_stalled_test;

        // ========================================================
        // FINAL RESULT
        // ========================================================

        $display("");
        $display("========================================");
        $display(" TESTS PASSED : %0d", pass_count);
        $display(" TESTS FAILED : %0d", fail_count);

        if (fail_count == 0)
            $display(" RESULT: ALL TESTS PASSED");
        else
            $display(" RESULT: FAILURES DETECTED");

        $display("========================================");
        $display("");

        $finish;
    end

    // ============================================================
    // GLOBAL WATCHDOG
    // ============================================================

    initial begin
        #200000;
        $display("[%0t] FAIL: global watchdog timeout - simulation hung", $time);$finish;
    end

endmodule
