module sd_dat_intf(
  input logic sdclk_i,
  input logic rst_dat_ni,

  input logic issue_read_i,
  input logic issue_write_i,
  input logic issue_busy_i,

  input logic dat_width_i,
  input logic [11:0] block_size_i,

  input  logic [7:0] tx_data_i,
  output logic tx_ready_o,

  output logic [7:0] rx_data_o,
  output logic rx_valid_o,

  output logic transfer_complete_o,
  output logic crc_error_o,
  output logic end_bit_error_o,
  output logic timeout_error_o,

  output logic timeout_start_o,
  output logic timeout_clear_o,
  input  logic timeout_trigger_i,

  // Connect to the DAT pin
  input  logic [3:0] dat_i,
  output logic [3:0] dat_o,
  output logic [3:0] dat_t
);

  function logic [15:0] crc16_1b(input logic [15:0] crc, input logic in);
      return {crc[14:0], 1'b0} ^ ({16{crc[15] ^ in}} & 16'h1021);
  endfunction

  logic dat_oe_n;
  assign dat_t = dat_width_i ? {4{dat_oe_n}} : {3'b111, dat_oe_n};

  // RX stream interfacing
  logic [7:0] rx_data_d;
  logic rx_valid_d;
  always_ff @(posedge sdclk_i or negedge rst_dat_ni)
    if (!rst_dat_ni) begin
      rx_valid_o <= 1'b0;
      rx_data_o <= 0;
    end
    else begin
      rx_valid_o <= rx_valid_d;
      rx_data_o <= rx_data_d;
    end

  // States
  enum logic [4:0] {
    ST_IDLE,
    ST_WRITE_START,
    ST_WRITE_DATA,
    ST_WRITE_CRC,
    ST_WRITE_GAP,
    ST_WRITE_WAIT,
    ST_WRITE_STATUS,
    ST_WRITE_END,
    ST_WRITE_BUSY,
    ST_READ_WAIT,
    ST_READ_DATA,
    ST_READ_CRC,
    ST_READ_END,
    ST_BUSY,
    ST_FINISH
  } state, state_d;

  logic [4:0]  count, count_d;
  logic [10:0] data_count, data_count_d;
  logic [3:0][15:0] crc, crc_d;

  logic [7:0] tx_buffer, tx_buffer_d;
  logic [2:0] write_status, write_status_d;

  logic crc_error_d;
  logic end_bit_error_d;

  assign transfer_complete_o = state != ST_IDLE && state_d == ST_IDLE;

  always_comb begin
    // Keep these constant
    state_d = state;
    data_count_d = data_count;
    crc_d = crc;
    rx_data_d = rx_data_o;
    tx_buffer_d = tx_buffer;
    write_status_d = write_status;
    crc_error_d = crc_error_o;
    end_bit_error_d = end_bit_error_o;
    timeout_error_o = 1'b0;

    timeout_start_o = 1'b0;
    timeout_clear_o = 1'b0;

    // By default increment count by 1.
    count_d = count + 1;

    // By default disable the tristate.
    dat_oe_n = 1'b1;
    dat_o = 1'bx;
    rx_valid_d = 1'b0;
    tx_ready_o = 1'b0;

    case (state)
      ST_IDLE: begin
        count_d = 0;
        crc_d = 0;
        tx_buffer_d = 0;
        data_count_d = 0;

        // Command send issued
        unique case (1'b1)
          issue_write_i: begin
            state_d = ST_WRITE_START;
          end
          issue_read_i: begin
            timeout_start_o = 1'b1;
            state_d = ST_READ_WAIT;
          end
          issue_busy_i: begin
            state_d = ST_BUSY;
          end
          default:;
        endcase

        // Only clear these registers upon the start of next transaction,
        // so we need CDC.
        if (issue_read_i || issue_write_i || issue_busy_i) begin
          crc_error_d = 1'b0;
          end_bit_error_d = 1'b0;
        end
      end

      ST_WRITE_START: begin
        // HACK: Wait a cycle so data is ready
        if (count != 0) begin
          count_d = 0;
          tx_buffer_d = tx_data_i;
          tx_ready_o = 1'b1;
          dat_oe_n = 1'b0;
          dat_o = 4'b0000;
          state_d = ST_WRITE_DATA;
        end
      end

      ST_WRITE_DATA: begin
        // Shift a bit out from the shift register
        dat_oe_n = 1'b0;
        if (dat_width_i) begin
          tx_buffer_d = {tx_buffer[3:0], 4'b0};
          dat_o = tx_buffer[7:4];
        end
        else begin
          tx_buffer_d = {tx_buffer[6:0], 1'b0};
          dat_o = {3'bx, tx_buffer[7]};
        end

        // Update CRC
        for (int i = 0; i < 4; i++) 
          crc_d[i] = crc16_1b(crc[i], dat_o[i]);

        // Last bit of the byte
        if (count == (dat_width_i ? 1 : 7)) begin
          count_d = 0;
          // Receive from TX stream
          tx_buffer_d = tx_data_i;
          tx_ready_o = 1'b1;
          // Reduce count by one
          data_count_d = data_count + 1;

          // Last byte of the block
          if (data_count_d == block_size_i) begin
            state_d = ST_WRITE_CRC;
            tx_ready_o = 1'b0;
          end
        end
      end
      ST_WRITE_CRC: begin
        // Shift a bit out from the CRC
        dat_oe_n = 1'b0;
        // Pad 1 here so that we don't need a special state for the end bit.
        for (int i = 0; i < 4; i++) begin
          dat_o[i] = crc[i][15];
          crc_d[i] = {crc[i][14:0], 1'b1};
        end

        // Last bit shifted out
        if (count == 16) begin
          count_d = 0;
          state_d = ST_WRITE_GAP;
        end
      end
      ST_WRITE_GAP: begin
        // Within these two cycles the dat_i we read is still the ones that we outputed,
        // so it is necessary to skip these.
        if (count == 1) begin
          count_d = 0;
          state_d = ST_WRITE_WAIT;
          timeout_start_o = 1'b1;
        end
      end
      ST_WRITE_WAIT: begin
        if (!dat_i[0]) begin
          count_d = 0;
          state_d = ST_WRITE_STATUS;
          timeout_clear_o = 1'b1;
        end
        else if (timeout_trigger_i) begin
          timeout_error_o = 1'b1;
          state_d = ST_IDLE;
        end
      end
      ST_WRITE_STATUS: begin
        write_status_d = {write_status[1:0], dat_i[0]};

        if (count == 2) begin
          crc_error_d = write_status_d != 3'b010;
          state_d = ST_WRITE_END;
        end
      end
      ST_WRITE_END: begin
        // Check the end bit is indeed 1.
        // Note that we don't need to consider bus width here because they'll be pulled high.
        end_bit_error_d = dat_i[0] != 1'b1;
        state_d = ST_WRITE_BUSY;
        timeout_start_o = 1'b1;
      end
      ST_WRITE_BUSY: begin
        count_d = 0;
        // Wait until the device no longer drive DAT[0] to zero.
        if (dat_i[0] == 1'b1) begin
          state_d = ST_FINISH;
          timeout_clear_o = 1'b1;
        end
        else if (timeout_trigger_i) begin
          timeout_error_o = 1'b1;
          state_d = ST_IDLE;
        end
      end

      ST_READ_WAIT: begin
        count_d = 0;
        if (!dat_i[0]) begin
          state_d = ST_READ_DATA;
          timeout_clear_o = 1'b1;
        end
        else if (timeout_trigger_i) begin
          timeout_error_o = 1'b1;
          state_d = ST_IDLE;
        end
      end
      ST_READ_DATA: begin
        // Shift the data into the receive buffer.
        if (dat_width_i)
          rx_data_d = {rx_data_o[3:0], dat_i};
        else
          rx_data_d = {rx_data_o[6:0], dat_i[0]};

        // Update CRC
        for (int i = 0; i < 4; i++)
          crc_d[i] = crc16_1b(crc[i], dat_i[i]);

        // Last bit of the byte
        if (count == (dat_width_i ? 1 : 7)) begin
          count_d = 0;
          // Send out to rx stream
          rx_valid_d = 1'b1;
          // Reduce count by one
          data_count_d = data_count + 1;

          // Last byte of the block
          if (data_count_d == block_size_i) begin
            state_d = ST_READ_CRC;
          end
        end
      end
      ST_READ_CRC: begin
        // Shift a bit out from the CRC and compare
        if (dat_i[0] != crc[0][15]) crc_error_d = 1'b1;
        if (dat_width_i) begin
          for (int i = 1; i < 4; i++)
            if (dat_i[i] != crc[i][15]) crc_error_d = 1'b1;
        end
        for (int i = 0; i < 4; i++)
          crc_d[i] = {crc[i][14:0], 1'b1};

        // Last bit consumed
        if (count == 15) begin
          state_d = ST_READ_END;
        end
      end
      ST_READ_END: begin
        // Check the end bit is indeed 1.
        // Note that we don't need to consider bus width here because they'll be pulled high.
        end_bit_error_d = dat_i != 4'b1111;
        count_d = 0;
        state_d = ST_FINISH;
      end

      ST_BUSY: begin
        if (dat_i[0]) begin
          state_d = ST_IDLE;
        end
      end

      ST_FINISH: begin
        // N_WR is 2 cycles
        state_d = ST_IDLE;
      end
    endcase
  end

  always_ff @(posedge sdclk_i or negedge rst_dat_ni)
    if (!rst_dat_ni) begin
      state <= ST_IDLE;
      crc <= 0;
      tx_buffer <= 0;
      write_status <= 0;
      crc_error_o <= 1'b0;
      end_bit_error_o <= 1'b0;
      count <= 0;
      data_count <= 0;
    end
    else begin
      state <= state_d;
      crc <= crc_d;
      tx_buffer <= tx_buffer_d;
      write_status <= write_status_d;
      crc_error_o <= crc_error_d;
      end_bit_error_o <= end_bit_error_d;
      count <= count_d;
      data_count <= data_count_d;
    end

endmodule
