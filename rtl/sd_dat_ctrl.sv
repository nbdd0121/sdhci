module sd_dat_ctrl (
  input logic sdclk_i,
  input logic rst_dat_ni,

  // Command config.
  input logic [1:0] response_type_i,
  input logic data_present_i,
  input logic [1:0] command_type_i,

  // Data transfer config.
  input logic dat_width_i,
  input logic transfer_direction_i,
  input logic auto_cmd12_enable_i,
  input logic [11:0] block_size_i,
  input logic [15:0] block_count_i,
  output logic [15:0] block_count_o,
  input logic multi_block_i,
  input logic block_count_enable_i,

  // Gap control
  input logic gap_stop_i,
  input logic gap_continue_i,

  // Read buffer interface (in SDCLK).
  // r_buf_full_i must be asserted when the buffer cannot receive any data,
  // When the buffer indicates that it is not full, it must be able to accept a whole block of data.
  output logic [7:0] r_data_o,
  output logic       r_valid_o,
  input  logic       r_buf_full_i,
  output logic       r_reset_o,

  // Write buffer interface (in SDCLK).
  // w_buf_empty_i must be asserted when the buffer does not have 1 block of data.
  // When the buffer indicates that it has data, it must be able to continously supply all data through
  // w_data_i for at least a block, change to the next word when w_ready_o is asserted high.
  input  logic [7:0] w_data_i,
  output logic       w_ready_o,
  input  logic       w_buf_empty_i,
  output logic       w_reset_o,

  // Timeout control
  output logic timeout_start_o,
  output logic timeout_clear_o,
  input  logic timeout_trigger_i,

  // SD pause request interface
  output logic sd_pause_req_full_o,
  output logic sd_pause_req_gap_o,
  input  logic sd_resumed_i,
  output logic block_gap_o,

  output logic read_xfer_active_o,
  output logic write_xfer_active_o,
  output logic dat_line_active_o,
  output logic auto_cmd12_issue_o,

  // Indicate the command has been sent.
  // For read transactions, DAT logic should prepare for incoming data and raise Read Transfer Active/DAT Line Active.
  // For write transactions, Write Transfer Active/DAT Line Active should be raised.
  // For busy transactions, DAT Line Active should be raised.
  input  logic command_end_i,

  // Indicate a response has been received.
  // For write transactions, write could start.
  // For busy transactions, busy signal should start to be detected.
  // Command can start to be issued upon receiving this pulse.
  input  logic command_complete_i,
  input  logic auto_cmd12_complete_i,

  output logic transfer_complete_o,

  output logic card_irq_o,
  output logic data_crc_error_o,
  output logic data_end_bit_error_o,
  output logic data_timeout_error_o,

  input  logic [3:0] dat_i,
  output logic [3:0] dat_o,
  output logic [3:0] dat_t
);

  typedef enum logic [3:0] {
    // Waiting for transfer to start
    ST_IDLE,
    // Waiting for transfer to complete
    ST_OP,
    // Transient state after OP is completed and before the issue of next block
    ST_COMP,
    // Waiting for command with busy
    ST_BUSY,
    // Waiting for SDCLK pause to be resumed
    ST_READ_PAUSED,
    // Waiting for write gap to be continued
    ST_WRITE_GAP,
    // Waiting for write buffer to be available
    ST_WRITE_BUF,
    // Auto CMD12 in progress
    ST_AUTO_CMD12,
    // Error or abort has happened
    ST_ERROR
  } state_e;

  logic issue_read;
  logic issue_write;
  logic issue_busy;
  logic transfer_complete;

  logic data_crc_error;
  logic data_end_bit_error;
  logic data_timeout_error;

  logic card_irq_q, card_irq_d;

  sd_dat_intf dat_intf(
    .sdclk_i,
    .rst_dat_ni,
    .issue_read_i (issue_read),
    .issue_write_i (issue_write),
    .issue_busy_i (issue_busy),
    .dat_width_i,
    .block_size_i,
    .tx_data_i  (w_data_i),
    .tx_ready_o (w_ready_o),
    .rx_data_o  (r_data_o),
    .rx_valid_o (r_valid_o),
    .transfer_complete_o (transfer_complete),
    .crc_error_o (data_crc_error),
    .end_bit_error_o (data_end_bit_error),
    .timeout_error_o (data_timeout_error),
    .timeout_start_o,
    .timeout_clear_o,
    .timeout_trigger_i,
    .dat_i (dat_i),
    .dat_o (dat_o),
    .dat_t (dat_t)
  );

  
  ///////////////////////
  // CMD Control Logic //
  ///////////////////////

  // Response from CMD logic.
  // For similar reasoning we don't need CDC the signal, but we do need to multiplex into
  // different location of response depending on whether it is from Auto CMD12.

  state_e state_q, state_d;

  logic [15:0] xfer_block_cnt_q, xfer_block_cnt_d;

  logic read_xfer_active_q, read_xfer_active_d;
  logic write_xfer_active_q, write_xfer_active_d;
  logic dat_line_active_q, dat_line_active_d;
  logic auto_cmd12_issue;

  // Connect CMD and DAT logic
  always_comb begin
    issue_read = 1'b0;
    issue_write = 1'b0;
    issue_busy = 1'b0;
    r_reset_o = 1'b0;
    w_reset_o = 1'b0;
    sd_pause_req_full_o = 1'b0;
    sd_pause_req_gap_o = 1'b0;
    block_gap_o = 1'b0;
    auto_cmd12_issue = 1'b0;
    state_d = state_q;
    xfer_block_cnt_d = xfer_block_cnt_q;
    card_irq_d = card_irq_q;

    data_crc_error_o = 1'b0;
    data_end_bit_error_o = 1'b0;
    data_timeout_error_o = 1'b0;

    read_xfer_active_d = read_xfer_active_q;
    write_xfer_active_d = write_xfer_active_q;
    dat_line_active_d = dat_line_active_q;

    unique case (state_q)
      ST_IDLE: begin
        if (command_end_i) begin
          if (data_present_i) begin
            // Calculate number of blocks to transfer. 0 indicates infinite.
            xfer_block_cnt_d = multi_block_i ? (block_count_enable_i ? block_count_i : 0) : 1;
            dat_line_active_d = 1'b1;

            // For read requests, issue now.
            if (transfer_direction_i) begin
              read_xfer_active_d = 1'b1;
              r_reset_o = 1'b1;
              issue_read = 1'b1;
              state_d = ST_OP;
            end else begin
              w_reset_o = 1'b1;
              write_xfer_active_d = 1'b1;
            end
          end
        end

        if (command_complete_i) begin
          // For write/busy requests, issue now.
          if (data_present_i && !transfer_direction_i) begin
            state_d = ST_WRITE_BUF;
          end
          if (response_type_i == 2'b11) begin
            dat_line_active_d = 1'b1;
            issue_busy = 1'b1;
            state_d = ST_BUSY;
          end
        end

        card_irq_d = !dat_i[1];
      end
      ST_BUSY: begin
        if (transfer_complete) begin
          read_xfer_active_d = 1'b0;
          write_xfer_active_d = 1'b0;
          dat_line_active_d = 1'b0;
          state_d = ST_IDLE;
        end
      end
      ST_OP: begin
        if (transfer_complete) begin
          if (data_crc_error) data_crc_error_o = 1'b1;
          if (data_end_bit_error) data_end_bit_error_o = 1'b1;
          if (data_timeout_error) data_timeout_error_o = 1'b1;

          if (data_crc_error || data_end_bit_error || data_timeout_error) begin
            state_d = ST_ERROR;
          end else begin
            xfer_block_cnt_d = xfer_block_cnt_q == 0 ? 0 : xfer_block_cnt_q - 1;
            if (xfer_block_cnt_q == 1) begin
              if (auto_cmd12_enable_i) begin
                auto_cmd12_issue = 1'b1;
                state_d = ST_AUTO_CMD12;
              end else begin
                // Transfer has completed
                read_xfer_active_d = 1'b0;
                write_xfer_active_d = 1'b0;
                dat_line_active_d = 1'b0;
                state_d = ST_IDLE;
              end
            end else begin
              // We need a transient state because we couldn't reissue anything within this cycle
              state_d = ST_COMP;
            end
          end
        end
      end
      ST_COMP: begin
        // Transfer is no longer active when we are paused at block gap.
        if (gap_stop_i) begin
          read_xfer_active_d = 1'b0;
          write_xfer_active_d = 1'b0;
          dat_line_active_d = 1'b0;
          block_gap_o = 1'b1;
        end

        // More block to transfer, continuing.
        if (transfer_direction_i) begin
          // If we're running out of blocks, try to pause the SD clock.
          // We could still issue read, because DAT logic is clocked by SD clock as well.
          if (r_buf_full_i) sd_pause_req_full_o = 1'b1;

          // Also stop SD clock if we're told to pause at block gap.
          if (gap_stop_i) begin
            sd_pause_req_gap_o = 1'b1;
          end

          if (r_buf_full_i || gap_stop_i) begin
            state_d = ST_READ_PAUSED;
          end else begin
            state_d = ST_OP;
            issue_read = 1'b1;
          end
        end else begin
          if (gap_stop_i) begin
            state_d = ST_WRITE_GAP;
          end else begin
            state_d = ST_WRITE_BUF;
          end
        end
      end
      ST_READ_PAUSED: begin
        if (sd_resumed_i) begin
          read_xfer_active_d = 1'b1;
          dat_line_active_d = 1'b1;
          state_d = ST_OP;
          issue_read = 1'b1;
        end
      end
      ST_WRITE_GAP: begin
        if (gap_continue_i) begin
          write_xfer_active_d = 1'b1;
          dat_line_active_d = 1'b1;
          state_d = ST_WRITE_BUF;
        end
      end
      ST_WRITE_BUF: begin
        if (!w_buf_empty_i) begin
          issue_write = 1'b1;
          state_d = ST_OP;
        end
      end
      ST_AUTO_CMD12: begin
        if (auto_cmd12_complete_i) begin
          issue_busy = 1'b1;
          state_d = ST_BUSY;
        end
      end
      ST_ERROR:;
      default:;
    endcase

    if (command_end_i && command_type_i == 2'b11) begin
      state_d = ST_ERROR;
    end
  end

  always_ff @(posedge sdclk_i or negedge rst_dat_ni) begin
    if (!rst_dat_ni) begin
      state_q <= ST_IDLE;
      card_irq_q <= 1'b0;
      xfer_block_cnt_q <= 0;

      read_xfer_active_q <= 1'b0;
      write_xfer_active_q <= 1'b0;
      dat_line_active_q <= 1'b0;
    end else begin
      state_q <= state_d;
      card_irq_q <= card_irq_d;
      xfer_block_cnt_q <= xfer_block_cnt_d;

      read_xfer_active_q <= read_xfer_active_d;
      write_xfer_active_q <= write_xfer_active_d;
      dat_line_active_q <= dat_line_active_d;
    end
  end

  assign card_irq_o = !card_irq_q && card_irq_d;
  assign block_count_o = xfer_block_cnt_q;
  assign transfer_complete_o = state_q != ST_IDLE && state_d == ST_IDLE;

  assign read_xfer_active_o = read_xfer_active_q;
  assign write_xfer_active_o = write_xfer_active_q;
  assign dat_line_active_o = dat_line_active_q;
  assign auto_cmd12_issue_o = auto_cmd12_issue;

endmodule
