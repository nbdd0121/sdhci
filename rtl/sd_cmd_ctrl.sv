module sd_cmd_ctrl #(
  parameter ConflictDetection = 0
) (
  input logic sdclk_i,
  input logic rst_cmd_ni,

  // Command config.
  input  logic [31:0] argument_i,
  input  logic [5:0] command_index_i,
  input  logic [1:0] response_type_i,
  input  logic command_index_check_i,
  input  logic command_crc_check_i,
  output logic [127:0] response_o,

  // Notify CMD ctrl that a command is to be issued.
  input logic cmd_issue_i,

  // Signal from DAT logic that an Auto CMD12 should be issued.
  input logic auto_cmd12_issue_i,

  // Whether this control logic is allowed to actually issue CMD.
  // We should not issue command if we are running low on buffer, because we couldn't stop SDCLK
  // when a transaction is in progress.
  input logic cmd_issue_allowed_i,

  // Indicate the command has been sent.
  // For read transactions, DAT logic should prepare for incoming data and raise Read Transfer Active/DAT Line Active.
  // For write transactions, Write Transfer Active/DAT Line Active should be raised.
  // For busy transactions, DAT Line Active should be raised.
  // For abort, transaction should stop.
  output logic command_end_o,

  // Indicate a response has been received.
  // For write transactions, write could start.
  // For busy transactions, busy signal should start to be detected.
  // Command can start to be issued upon receiving this pulse.
  output logic command_complete_o,

  // Indicate Auto CMD12 has been completed or an eror happens during the process.
  output logic auto_cmd12_complete_o,

  // Indicate the command inhibit (CMD) bit can now be cleared.
  output logic command_inhibit_cmd_clear_o,
  output logic cmd_index_error_o,
  output logic cmd_crc_error_o,
  output logic cmd_end_bit_error_o,
  output logic cmd_timeout_error_o,
  output logic auto_cmd12_error_cmd_no_ex_o,
  output logic auto_cmd12_error_index_o,
  output logic auto_cmd12_error_end_bit_o,
  output logic auto_cmd12_error_crc_o,
  output logic auto_cmd12_error_timeout_o,
  output logic auto_cmd12_error_no_ex_o,

  // Connect to the CMD pin.
  // These are async signals.
  input  logic cmd_i,
  output logic cmd_o,
  output logic cmd_t
);

  typedef enum logic [1:0] {
    ST_IDLE,
    ST_OP,
    ST_AUTO_CMD12,
    // An error has occured. This will prevent further Auto CMD12 or CMD from being issued.
    ST_ERROR
  } state_e;

  logic cmd_pending_q, cmd_pending_d;
  logic auto_cmd12_pending_q, auto_cmd12_pending_d;

  logic [119:0] response_raw;
  logic [127:0] response_d;
  logic do_issue_q, do_issue_d;
  state_e state_q, state_d;

  logic command_end, command_complete;
  assign command_end_o = command_end && state_q == ST_OP;
  assign auto_cmd12_complete_o = command_complete && state_q == ST_AUTO_CMD12;

  logic cmd_index_error;
  logic cmd_crc_error;
  logic cmd_end_bit_error;
  logic cmd_timeout_error;
  logic cmd_conflict_error;

  sd_cmd_intf #(.ConflictDetection(ConflictDetection)) cmd_intf (
    .sdclk_i,
    .rst_cmd_ni,
    .argument_i (argument_i),
    .command_index_i (state_q == ST_AUTO_CMD12 ? 6'd12 : command_index_i),
    .response_type_i (state_q == ST_AUTO_CMD12 ? 2'b11 : response_type_i),
    .command_index_check_i (state_q == ST_AUTO_CMD12 ? 1'b1 : command_index_check_i),
    .command_crc_check_i (state_q == ST_AUTO_CMD12 ? 1'b1 : command_crc_check_i),
    .issue_i (do_issue_q),
    .response_o (response_raw),
    .command_end_o (command_end),
    .command_complete_o (command_complete),
    .index_error_o (cmd_index_error),
    .crc_error_o (cmd_crc_error),
    .end_bit_error_o (cmd_end_bit_error),
    .timeout_error_o (cmd_timeout_error),
    .conflict_error_o (cmd_conflict_error),
    .cmd_i,
    .cmd_o,
    .cmd_t
  );

  always_comb begin
    state_d = state_q;
    cmd_pending_d = cmd_pending_q;
    auto_cmd12_pending_d = auto_cmd12_pending_q;
    do_issue_d = 1'b0;
    response_d = response_o;

    // Combinational pulses
    command_complete_o = 1'b0;
    command_inhibit_cmd_clear_o = 1'b0;
    cmd_index_error_o = 1'b0;
    cmd_end_bit_error_o = 1'b0;
    cmd_crc_error_o = 1'b0;
    cmd_timeout_error_o = 1'b0;
    auto_cmd12_error_cmd_no_ex_o = 1'b0;
    auto_cmd12_error_index_o = 1'b0;
    auto_cmd12_error_end_bit_o = 1'b0;
    auto_cmd12_error_crc_o = 1'b0;
    auto_cmd12_error_timeout_o = 1'b0;
    auto_cmd12_error_no_ex_o = 1'b0;

    if (cmd_issue_i) cmd_pending_d = 1'b1;
    if (auto_cmd12_issue_i) auto_cmd12_pending_d = 1'b1;

    unique case (state_q)
      ST_IDLE: begin
        priority case (1'b1)
          auto_cmd12_pending_q: begin
            do_issue_d = 1'b1;
            auto_cmd12_pending_d = 1'b0;
            state_d = ST_AUTO_CMD12;
          end
          cmd_pending_q: begin
            if (cmd_issue_allowed_i) begin
              do_issue_d = 1'b1;
              cmd_pending_d = 1'b0;
              state_d = ST_OP;
            end
          end
          default:;
        endcase
      end
      ST_OP: begin
        if (command_complete) begin
          response_d[119:0] = response_raw;

          if (cmd_index_error) cmd_index_error_o = 1'b1;
          if (cmd_end_bit_error) cmd_end_bit_error_o = 1'b1;
          if (cmd_crc_error) cmd_crc_error_o = 1'b1;
          if (cmd_timeout_error) cmd_timeout_error_o = 1'b1;
          if (cmd_conflict_error) begin
            cmd_crc_error_o = 1'b1;
            cmd_timeout_error_o = 1'b1;
          end

          if (cmd_index_error || cmd_crc_error ||
              cmd_end_bit_error || cmd_timeout_error || cmd_conflict_error) begin
            if (auto_cmd12_pending_q) auto_cmd12_error_no_ex_o = 1'b1;
          end else begin
            // Data xfer will only begin if we succeed
            command_complete_o = 1'b1;
          end

          if (cmd_conflict_error) begin
            state_d = ST_ERROR;
          end else begin
            command_inhibit_cmd_clear_o = 1'b1;
            state_d = ST_IDLE;
          end
        end
      end
      ST_AUTO_CMD12: begin
        if (command_complete) begin
          response_d[127:96] = response_raw[31:0];

          if (cmd_index_error) auto_cmd12_error_index_o = 1'b1;
          if (cmd_end_bit_error) auto_cmd12_error_end_bit_o = 1'b1;
          if (cmd_crc_error) auto_cmd12_error_crc_o = 1'b1;
          if (cmd_timeout_error) auto_cmd12_error_timeout_o = 1'b1;
        
          if (cmd_index_error || cmd_crc_error || cmd_end_bit_error || cmd_timeout_error) begin
            if (cmd_pending_q) auto_cmd12_error_cmd_no_ex_o = 1'b1;
            state_d = ST_ERROR;
          end else begin
            state_d = ST_IDLE;
          end
        end
      end
      ST_ERROR:;
      default:;
    endcase
  end

  always_ff @(posedge sdclk_i or negedge rst_cmd_ni) begin
    if (!rst_cmd_ni) begin
      state_q <= ST_IDLE;
      cmd_pending_q <= 1'b0;
      auto_cmd12_pending_q <= 1'b0;
      do_issue_q <= 1'b0;
      response_o <= 0;
    end else begin
      state_q <= state_d;
      cmd_pending_q <= cmd_pending_d;
      auto_cmd12_pending_q <= auto_cmd12_pending_d;
      do_issue_q <= do_issue_d;
      response_o <= response_d;
    end
  end

endmodule
