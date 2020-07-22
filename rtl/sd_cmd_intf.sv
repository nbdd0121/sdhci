module sd_cmd_intf #(
  // Number of cycles taken between driving cmd_o and sampling cmd_i.
  // If this value is less than 2, conflict detection will be disabled.
  parameter ConflictDetection = 0
) (
  input logic sdclk_i,
  input logic rst_cmd_ni,

  input logic [31:0] argument_i,
  input logic [5:0] command_index_i,
  input logic [1:0] response_type_i,
  input logic command_index_check_i,
  input logic command_crc_check_i,
  input logic issue_i,

  output logic [119:0] response_o,

  // Indicate the end bit of the command has been sent.
  output logic command_end_o,

  // Indicate a response has been received or an exception has happened.
  output logic command_complete_o,

  output logic index_error_o,
  output logic crc_error_o,
  output logic end_bit_error_o,
  output logic timeout_error_o,
  output logic conflict_error_o,

  input  logic cmd_i,
  output logic cmd_o,
  output logic cmd_t
);

  function logic [6:0] crc7_1b(input logic [6:0] crc, input logic in);
    return {crc[5:0], 1'b0} ^ ({7{crc[6] ^ in}} & 7'h09);
  endfunction

  if (ConflictDetection > 1) begin
    // Conflict detection.
    // CMD line conflict can be detected by checking if we does not read back the value we drive.
    logic [ConflictDetection-1:0] cmd_t_q;
    logic [ConflictDetection-1:0] cmd_o_q;
    always_ff @(posedge sdclk_i) begin
      cmd_t_q <= {cmd_t, cmd_t_q[ConflictDetection-1:1]};
      cmd_o_q <= {cmd_o, cmd_o_q[ConflictDetection-1:1]};
    end

    assign conflict_error_o = !cmd_t_q[0] && cmd_o_q[0] && !cmd_i;
  end else begin
    assign conflict_error_o = 1'b0;
  end

  // States
  typedef enum logic [3:0] {
    ST_IDLE,
    ST_SEND_COMMAND,
    ST_SEND_CRC,
    ST_GAP,
    ST_WAIT,
    ST_RECV_COMMAND,
    ST_RECV_RESPONSE,
    ST_RECV_CRC,
    ST_RECV_END,
    ST_FINISH
  } state_e;
  
  state_e state_q, state_d;

  logic [6:0]  count_q, count_d;
  logic [6:0]  crc_q, crc_d;
  logic [39:0] shift_reg_q, shift_reg_d;

  logic [119:0] response_d;
  logic index_error_d;
  logic crc_error_d;
  logic end_bit_error_d;

  assign command_end_o = state_q == ST_SEND_CRC && state_d != ST_SEND_CRC;
  assign command_complete_o = state_q != ST_IDLE && state_d == ST_IDLE;

  always_comb begin
    // Keep these constant
    state_d = state_q;
    crc_d = crc_q;
    shift_reg_d = shift_reg_q;
    response_d = response_o;
    index_error_d = index_error_o;
    crc_error_d = crc_error_o;
    end_bit_error_d = end_bit_error_o;
    timeout_error_o = 1'b0;

    // By default increment count by 1.
    count_d = count_q + 1;

    // By default disable the tristate.
    cmd_t = 1'b1;
    cmd_o = 1'bx;

    unique case (state_q)
      ST_IDLE: begin
        count_d = 0;
        crc_d = 7'd0;

        // Command send issued
        if (issue_i) begin
          state_d = ST_SEND_COMMAND;
          index_error_d = 1'b0;
          crc_error_d = 1'b0;
          end_bit_error_d = 1'b0;

          shift_reg_d = {2'b01, command_index_i, argument_i};
        end
      end
      ST_SEND_COMMAND: begin
        // Shift a bit out from the shift register
        cmd_t = 1'b0;
        cmd_o = shift_reg_q[39];
        shift_reg_d = {shift_reg_q[38:0], 1'b0};

        // Update CRC
        crc_d = crc7_1b(crc_q, shift_reg_q[39]);

        // Last bit shifted out
        if (count_q == 39) begin
          count_d = 0;
          state_d = ST_SEND_CRC;
        end
      end
      ST_SEND_CRC: begin
        // Shift a bit out from the CRC
        cmd_t = 1'b0;
        cmd_o = crc_q[6];
        // Pad 1 here so that we don't need a special state for the end bit.
        crc_d = {crc_q[5:0], 1'b1};

        // Last bit shifted out
        if (count_q == 7) begin
          count_d = 0;
          state_d = response_type_i == 2'b00 ? ST_FINISH : ST_GAP;
        end
      end
      ST_GAP: begin
        // Within these two cycles the cmd_i we read is still the ones that we outputed,
        // so it is necessary to skip these.
        if (count_q == 1) begin
          count_d = 0;
          state_d = ST_WAIT;
        end
      end
      ST_WAIT: begin
        // Note that when we transition away from ST_WAIT we would already consumed a zero.
        // But that's okay because crc7_1b(0, 0) = 0.
        crc_d = 0;

        if (!cmd_i) begin
          count_d = 0;
          state_d = ST_RECV_COMMAND;
        end
        else if (count_q == 63) begin
          timeout_error_o = 1'b1;
          state_d = ST_IDLE;
        end
      end
      ST_RECV_COMMAND: begin
        // Shift the data into the receive buffer.
        // This isn't response but we don't need command anyway after checking the index.
        response_d = {response_o[118:0], cmd_i};

        // Update CRC
        crc_d = crc7_1b(crc_q, cmd_i);

        // Last bit in of first byte shifted in
        if (count_q == 6) begin
          // Check if index matches
          index_error_d = command_index_check_i && response_d[5:0] != command_index_i;

          // For R2, CRC does not include command bits
          if (!response_type_i[1]) begin
            crc_d = 0;
          end

          count_d = 0;
          state_d = ST_RECV_RESPONSE;
        end
      end
      ST_RECV_RESPONSE: begin
        // Shift the data into the receive buffer.
        response_d = {response_o[118:0], cmd_i};

        // Update CRC
        crc_d = crc7_1b(crc_q, cmd_i);

        // Last bit in of response shifted in
        if (count_q == (response_type_i[1] ? 31 : 119)) begin
          count_d = 0;
          state_d = ST_RECV_CRC;
        end
      end
      ST_RECV_CRC: begin
        // Shift a bit out from the CRC and compare
        if (command_crc_check_i && cmd_i != crc_q[6]) crc_error_d = 1'b1;
        crc_d = {crc_q[5:0], 1'b1};

        // Last bit consumed
        if (count_q == 6) begin
          state_d = ST_RECV_END;
        end
      end
      ST_RECV_END: begin
        // Check the end bit is indeed 1.
        end_bit_error_d = !cmd_i;
        count_d = 0;
        state_d = ST_FINISH;
      end
      ST_FINISH: begin
        // Both N_RC and N_CC are 8 cycles.
        // So we need to spend 7 additional cyclces here before moving to ST_IDLE
        if (count_q == 6) begin
          count_d = 0;
          state_d = ST_IDLE;
        end
      end
      default:;
    endcase

    if (conflict_error_o) begin
      state_d = ST_IDLE;
    end
  end

  always_ff @(posedge sdclk_i or negedge rst_cmd_ni) begin
    if (!rst_cmd_ni) begin
      state_q <= ST_IDLE;
      count_q <= 0;
      crc_q <= 0;
      shift_reg_q <= 0;
      response_o <= 0;
      index_error_o <= 1'b0;
      crc_error_o <= 1'b0;
      end_bit_error_o <= 1'b0;
    end
    else begin
      state_q <= state_d;
      count_q <= count_d;
      crc_q <= crc_d;
      shift_reg_q <= shift_reg_d;
      response_o <= response_d;
      index_error_o <= index_error_d;
      crc_error_o <= crc_error_d;
      end_bit_error_o <= end_bit_error_d;
    end
  end

endmodule
