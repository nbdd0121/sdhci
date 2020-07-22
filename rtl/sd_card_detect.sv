module sd_card_detect #(
  // Maximum number of cycles in clk_i takes for SDCD to stablise
  // The default value is 100ms when clock is 50MHz
  parameter DebouncePeriod = 5000000
) (
  input  logic clk_i,
  input  logic rst_ni,

  // This should already be synchronised to clk_i
  input  logic card_detect_pin_level_i,

  output logic card_state_stable_o,
  output logic card_inserted_o,

  // Combinational ports for event notification
  output logic card_insertion_o,
  output logic card_removal_o
);

  localparam COUNTER_WIDTH = $clog2(DebouncePeriod);

  logic [COUNTER_WIDTH-1:0] counter_q, counter_d;
  logic card_detect_pin_level_q;
  logic card_state_stable_q, card_state_stable_d;
  logic card_inserted_q, card_inserted_d;

  assign card_state_stable_o = card_state_stable_q;
  assign card_inserted_o = card_inserted_q;

  always_comb begin
    counter_d = counter_q;
    card_state_stable_d = card_state_stable_q;
    card_inserted_d = card_inserted_q;

    card_insertion_o = 1'b0;
    card_removal_o = 1'b0;

    if (card_detect_pin_level_q != card_detect_pin_level_i) begin
      // Pin changed, reset to debouncing state
      counter_d = DebouncePeriod - 1;
      card_state_stable_d = 1'b0;
      card_inserted_d = 1'b0;
      if (card_inserted_q) card_removal_o = 1'b1;
    end else if (!card_state_stable_q) begin
      counter_d = counter_q - 1;
      if (counter_q == 0) begin
        // Debounce period reached, consider the current state stable.
        card_state_stable_d = 1'b1;
        card_inserted_d = card_detect_pin_level_i;
        if (card_detect_pin_level_i) card_insertion_o = 1'b1;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Start in debouncing state
      counter_q <= DebouncePeriod - 1;
      card_detect_pin_level_q <= 1'b0;
      card_state_stable_q <= 1'b0;
      card_inserted_q <= 1'b0;
    end
    else begin
      counter_q <= counter_d;
      card_detect_pin_level_q <= card_detect_pin_level_i;
      card_state_stable_q <= card_state_stable_d;
      card_inserted_q <= card_inserted_d;
    end
  end

endmodule
