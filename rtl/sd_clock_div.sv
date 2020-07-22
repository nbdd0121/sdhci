// Clock divider logic.
//
// This logic operates entirely in the "base clock" frequency,
// which can be different from the host interface clock.

module sd_clock_div (
  input  logic clk_i,
  input  logic rst_ni,

  // 2 * SD Base clock.
  //
  // Technically we could just take the SD base clock as the input, but it is somewhat more
  // difficult because we have to multiplex the original clock and the divided clock, and
  // it is harder for us to disable the clock only after the falling edge, as required by
  // the SD spec.
  input  logic clk_base_i,

  // Whether SD power should be enabled. When power is off, we reset internal logic and
  // drive SDCLK to low.
  input  logic power_i,

  // Whether SDCLK should be enabled. When disabled, we will finish current period of high
  // before driving SDCLK to low.
  input  logic enable_i,

  // Select frequency. This corresponds to the "SDCLK Frequency Select" register.
  // A value of `x` indicates SDCLK should be clk divided by `2x` if `x` > 0, if `x` = 0, then
  // the base clock should be used.
  input  logic [9:0] freq_i,

  // The SDCLK signal divided
  output logic sdclk_o
);

  logic power;
  logic enable;
  prim_flop_2sync #(.Width(1)) power_sync (
    .clk_i  (clk_base_i),
    .rst_ni (rst_ni),
    .d      (power_i),
    .q      (power)
  );
  prim_flop_2sync #(.Width(1)) enable_sync (
    .clk_i  (clk_base_i),
    .rst_ni (rst_ni),
    .d      (enable_i),
    .q      (enable)
  );

  logic [10:0] counter_q, counter_d;
  logic running_q, running_d;
  logic sdclk_d;

  always_comb begin
    counter_d = counter_q;
    running_d = running_q;
    sdclk_d = sdclk_o;

    // SD Bus power is turned off. Stop driving immediately.
    if (!power) begin
      sdclk_d = 1'b0;
      running_d = 1'b0;
    end
    else begin
      // Division is actively running
      if (running_q) begin
        counter_d = counter_q - 1;
        
        if (counter_q == 1) begin
          counter_d = freq_i == 0 ? 1 : {freq_i, 1'b0};
          sdclk_d   = !sdclk_o;
          // When clock enable is turned off, we still need for the period to lapse.
          if (!enable) begin
            sdclk_d = 1'b0;
            running_d = 1'b0;
          end
        end
      end
      // Clock is turned off
      else begin
        if (enable) begin
          // Rising edge at next cycle.
          counter_d = 1;
          running_d = 1'b1;
        end
      end
    end
  end

  always_ff @(posedge clk_base_i or negedge rst_ni) begin
    if (!rst_ni) begin
      counter_q <= 0;
      running_q <= 1'b0;
      sdclk_o <= 1'b0;
    end else begin
      counter_q <= counter_d;
      running_q <= running_d;
      sdclk_o <= sdclk_d;
    end
  end

endmodule


// Timeout clock divider logic
module sd_timeout_clk_div (
  input  logic clk_base_i,
  input  logic sdclk_i,
  input  logic rst_ni,
  
  input  logic [3:0] freq_i,

  input  logic start_i,
  input  logic clear_i,
  output logic trigger_o
);

  logic start;
  logic clear;
  logic trigger;

  prim_pulse_sync start_sync (
    .clk_src_i (sdclk_i),
    .rst_src_ni (rst_ni),
    .src_pulse_i (start_i),
    .clk_dst_i (clk_base_i),
    .rst_dst_ni (rst_ni),
    .dst_pulse_o (start)
  );

  prim_pulse_sync clear_sync (
    .clk_src_i (sdclk_i),
    .rst_src_ni (rst_ni),
    .src_pulse_i (clear_i),
    .clk_dst_i (clk_base_i),
    .rst_dst_ni (rst_ni),
    .dst_pulse_o (clear)
  );

  prim_pulse_sync trigger_sync (
    .clk_src_i (clk_base_i),
    .rst_src_ni (rst_ni),
    .src_pulse_i (trigger),
    .clk_dst_i (sdclk_i),
    .rst_dst_ni (rst_ni),
    .dst_pulse_o (trigger_o)
  );

  logic running_q, running_d;
  logic [28:0] counter_q, counter_d;

  always_comb begin
    running_d = running_q;
    counter_d = counter_q;
    trigger = 1'b0;

    if (running_q) begin
      counter_d = counter_q + 1;
      if (counter_q[freq_i + 14]) begin
        trigger = 1'b1;
        running_d = 1'b0;
      end
      if (clear) begin
        running_d = 1'b0;
      end
    end
    else begin
      if (start) begin
        counter_d = 0;
        running_d = 1'b1;
      end
    end
  end

  always_ff @(posedge clk_base_i or negedge rst_ni) begin
    if (!rst_ni) begin
      running_q <= 1'b0;
      counter_q <= 0;
    end else begin
      running_q <= running_d;
      counter_q <= counter_d;
    end
  end

endmodule

