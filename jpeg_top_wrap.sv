//
// AXI wrapper around JPEG encoder
//

module jpeg_top_wrap (
    input logic	clk,
    input logic	rst,

    // Output interrupts
    output logic fifo_interrupt,
    output logic end_interrupt,
    output logic error_interrupt,

    // External interface
    XBAR_PERIPH_BUS.Slave slv
);

    localparam logic [1:0] DATA_FIFO_ADDR  = 2'b00;
    localparam logic [1:0] FIFO_DEPTH_ADDR = 2'b01;
    localparam logic [1:0] END_BITS_ADDR   = 2'b10;

    localparam logic WRITE = 1'b0;
    localparam logic READ  = 1'b1;

    // SW visible register to hold the end of stream bit count
    logic [4:0] eof_bit_count;

    // Is this a valid cycle (or clock gated) cycle of jpeg activity
    logic jpeg_clk_en;
    logic jpeg_clk_en_d;

    always_ff @(posedge clk) jpeg_clk_en_d <= jpeg_clk_en;

    // Is this the last 8x8 of the input
    logic last_block, last_block_next;


    logic end_interrupt_next;

    //
    // control state machine
    //
    logic [7:0] pixel_counter, pixel_counter_next;
    logic [1:0] write_state, write_state_next;
    logic rd_full;
    logic jpeg_wr_gnt;
    logic eof_data_partial_ready;

    localparam IDLE = 0;
    localparam COUNTING = 1;
    localparam PROCESSING = 2;
    localparam END_PROCESSING = 3;

    always_ff @(posedge clk) write_state   <= rst ? IDLE : write_state_next;
    always_ff @(posedge clk) pixel_counter <= pixel_counter_next;

    always_comb begin
        write_state_next   = write_state;
        pixel_counter_next = pixel_counter;
        jpeg_wr_gnt        = 1'b0;
        jpeg_clk_en        = 1'b0;
        last_block_next    = 1'b0;
        end_interrupt_next = 1'b0;

        case ( write_state )

            IDLE: begin
                if ( slv.req && slv.wen == WRITE ) begin
                    last_block_next = slv.add[8];

                    // Transition to write accepting state
                    write_state_next   = COUNTING;

                    // Implicit is blocks of 8x8 RGB pixels
                    pixel_counter_next = 63;
                end
            end

            // Accepting one of 64 pixel writes of a 8x8
            COUNTING: begin
                if ( slv.req && slv.wen == WRITE ) begin
                    jpeg_wr_gnt = 1'b1;
                    jpeg_clk_en = 1'b1;
                    if ( pixel_counter == 0 ) begin
                        write_state_next   = PROCESSING;
                        pixel_counter_next = 32;
                    end else begin
                        pixel_counter_next = pixel_counter - 1;
                    end
                end
            end

            // While the block is processing, do not accept more writes
            PROCESSING: begin
                jpeg_clk_en = 1'b1;
                if ( pixel_counter == 0 ) begin
                    write_state_next = last_block ? END_PROCESSING : IDLE;
                end else begin
                    pixel_counter_next = pixel_counter - 1;
                end
            end

            // Continue processing until end of bitstream without accepting
            // new data
            END_PROCESSING: begin
                jpeg_clk_en = ~rd_full;
                if (eof_data_partial_ready) begin
                    write_state_next = IDLE;
                    end_interrupt_next = 1'b1;
                end
            end

            default: begin
                write_state_next = IDLE;
            end

        endcase
    end

    always_ff @(posedge clk) end_interrupt <= rst ? 1'b0 : end_interrupt_next;
    always_ff @(posedge clk) last_block    <= rst ? 1'b0 : last_block_next;

    //
    // JPEG engine clock gating control
    //
    // JPEG engine only runs when there's data being added or for the post
    // data encoding cycles. To absorb non-contiguous write bursts, the
    // clock gate disables the JPEG block on non write cycles until all
    // 64 pixels have been fed into the engine.
    //
    // The engine should also run after the last pixels have been fed in
    // until the end of the bitsream is generated.
    //
    logic jpeg_engine_en;
    assign jpeg_engine_en = ((write_state == COUNTING)    ||
                             (write_state == PROCESSING)  ||
                             (write_state == END_PROCESSING));

    // jpeg clock gate
    pd_icg i_jpeg_icg (
        .clk_en          ( jpeg_clk_en || rst ),
        .ungated_clk_in  ( clk                ),
        .gated_clk       ( jpeg_clk_gated     )
    );


    logic [31:0] fifo_data;
    logic [31:0] rdata;
    logic [31:0] jpg_data_out;
    logic [$bits(slv.id)-1:0] r_id;

    //
    // JPEG data stream output FIFO
    //
    // Pushed into on cycles when the JPEG block produces new 32 bit
    // words of compressed stream data. Huffman coding and DCT means
    // that the pattern of writes is unpredictable based on the input
    // data.
    //
    // The FIFO is 32 entries ( 128 bytes ) deep. The effective compression
    // ratio of JPEG is about 8:1 which means that 3 (RGB) * 8x8 input
    // pixels should on average result in 3*8*8/8 = 24 output bytes.
    //
    // A fixed threshold interrupt indicates a FIFO level
    logic [4:0] rd_depth;

    fifo_v3 #(
        .DATA_WIDTH(32),   // default data width if the fifo is of type logic
        .DEPTH(32)
    ) i_read_data_fifo (
        .clk_i        (  clk        ),           // Clock
        .rst_ni       (  ~rst       ),           // Asynchronous reset active low
        .flush_i      (  1'b0       ),           // flush the queue
        .testmode_i   (  1'b0       ),           // test_mode to bypass clock gating
        // status flags
        .full_o       ( rd_full     ),           // queue is full
        .empty_o      ( rd_empty    ),           // queue is empty
        .usage_o      ( rd_depth    ),           // fill pointer
        // as long as the queue is not full we can push new data
        .data_i       ( jpg_data_out                ),           // data to push into the queue
        .push_i       ( data_ready && jpeg_clk_en_d ),           // data is valid and can be pushed to the queue
        // as long as the queue is not empty we can pop new elements
        .data_o       ( fifo_data  ),           // output data
        .pop_i        ( ~rd_empty && decode_fifo && slv.req && (slv.wen == READ) && slv.r_valid )            // pop head from queue
    );

    jpeg_top i_jtag_top (
        .clk                 ( jpeg_clk_gated     ),
        .rst                 ( rst                ),
        .end_of_file_signal  ( last_block         ),
        .enable              ( jpeg_engine_en     ),
        .data_in             ( slv.wdata          ),
        .JPEG_bitstream      ( jpg_data_out       ),
        .data_ready          ( data_ready         ),

        .end_of_file_bitstream_count ( end_of_file_bitstream_count ),
        .eof_data_partial_ready      ( eof_data_partial_ready      )
    );

    // Store end of stream bit count when eof_data_partial_ready is asserted
    always_ff @(posedge clk) begin
        if ( jpeg_clk_en_d && eof_data_partial_ready ) begin
            eof_bit_count <= end_of_file_bitstream_count;
        end
    end

    assign decode_fifo   = (slv.add[9:8] == 2'b00) || (slv.add[9:8] == 2'b01 );
    assign decode_depth  = (slv.add[9:8] == 2'b10);
    assign decode_end    = (slv.add[9:8] == 2'b11);

    // Read data mux
    always_ff @(posedge clk) begin
        rdata <= decode_fifo  ? fifo_data :
                 decode_depth ? (32'h0 | rd_depth) :
                 decode_end   ? (32'h0 | eof_bit_count) :
                 '0;
    end

    logic r_valid;
    always_ff @(posedge clk) begin
        if ( slv.req ) begin
            r_id <= slv.id;
            r_valid <= 1'b1;
        end else begin
            r_id    <= '0;
            r_valid <= 1'b0;
        end
    end

    assign slv.r_rdata = rdata;
    assign slv.r_valid = r_valid;
    assign slv.r_id    = r_id;
    assign slv.r_opc   = '0;

    assign slv.gnt = (slv.wen == WRITE) ? jpeg_wr_gnt : slv.req;

    // Static value for FIFO interrupt
    assign fifo_interrupt = rd_depth > 8;

    // Read fifo should never ever fill to the top
    assign error_interrupt = rd_full;

endmodule
