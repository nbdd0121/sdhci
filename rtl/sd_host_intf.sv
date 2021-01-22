module sd_host_intf #(
  // Number of cycles taken between driving cmd_o and sampling cmd_i.
  // If this value is less than 0, conflict detection will be disabled.
  parameter ConflictDetection = -1,

  // Supported voltage of this slot. Currently we only support one voltage.
  // The value corresponds to the 3-bit value in power control register.
  parameter Voltage = 3'b111,

  // Maximum current capability for the Voltage.
  // This value corresponds to the 8-bit value in maximum current capability register.
  parameter Current = 8'd0,

  // Maximum number of cycles in clk_i takes for SDCD to stablise
  // The default value is 100ms when clock is 50MHz
  parameter DebouncePeriod = 5000000,

  // SD Base Clock Frequency in MHz. This must range from 1MHz to 63MHz.
  parameter SdBaseClockFreq = 25
) (
  input  logic clk_i,
  input  logic rst_ni,

  // 2 * SD Base Clock
  input  logic sd_base_clock,

  input  logic cmd_i,
  output logic cmd_o,
  output logic cmd_t,

  input  logic [3:0] dat_i,
  output logic [3:0] dat_o,
  output logic [3:0] dat_t,

  input  logic sdwp_ni,
  input  logic sdcd_ni,
  output logic sdpower_o,
  output logic sdclk_o,

  // Whether LED should be on
  output logic led_o,

  output logic irq_o,
  output logic wakeup_o,
  input  logic [7:0] slot_irq_i,

  // BRAM port, 256 Bytes in size
  input  logic        bram_en,
  input  logic [3:0]  bram_we,
  input  logic [7:0]  bram_addr,
  input  logic [31:0] bram_wrdata,
  output logic [31:0] bram_rddata
);

  // Ignore the LSB of bram_addr
  wire [7:0] bram_addr_actual = {bram_addr[7:2], 2'b00};

  /////////////////
  // Reset logic //
  /////////////////

  logic sw_rst_all_q, sw_rst_all_d;
  logic sw_rst_cmd_q, sw_rst_cmd_d;
  logic sw_rst_dat_q, sw_rst_dat_d;

  wire rst_all_n = rst_ni && !sw_rst_all_q;
  wire rst_cmd_n = rst_ni && !sw_rst_cmd_q;
  wire rst_dat_n = rst_ni && !sw_rst_dat_q;

  // BRAM interfacing
  wire [7:0] reg_sw_rst = {5'd0, sw_rst_dat_q, sw_rst_cmd_q, sw_rst_all_q};

  always_comb begin
    sw_rst_all_d = 1'b0;
    sw_rst_cmd_d = 1'b0;
    sw_rst_dat_d = 1'b0;

    if (bram_en) begin
      // Software Reset Register (02Fh)
      // [02]    Software Reset For DAT Line
      // [01]    Software Reset For CMD Line
      // [00]    Software Reset For All
      if (bram_addr_actual == 8'h2C && bram_we[3]) begin
        sw_rst_all_d = bram_wrdata[24];
        sw_rst_cmd_d = bram_wrdata[25] || bram_wrdata[24];
        sw_rst_dat_d = bram_wrdata[26] || bram_wrdata[24];
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sw_rst_all_q <= 1'b1;
      sw_rst_cmd_q <= 1'b1;
      sw_rst_dat_q <= 1'b1;
    end
    else begin
      sw_rst_all_q <= sw_rst_all_d;
      sw_rst_cmd_q <= sw_rst_cmd_d;
      sw_rst_dat_q <= sw_rst_dat_d;
    end
  end

  ////////////////////
  // Card detection //
  ////////////////////

  // Card detection logic is isolated from the rest part of the SD controller and is not affected
  // by software reset (except for the test registers)

  // Test registers managed by BRAM interface
  logic card_detect_signal_selection_q, card_detect_signal_selection_d;
  logic card_detect_test_level_q, card_detect_test_level_d;

  // Synchronise async IO signal to clk_i.
  logic sdcd;
  prim_flop_2sync #(.Width(1)) sdcd_sync_inst (.clk_i, .rst_ni, .d_i(!sdcd_ni), .q_o(sdcd));

  // Multiplex between test signals and IO input
  wire card_detect_pin_level = card_detect_signal_selection_q ? card_detect_test_level_q : sdcd;

  // Current card states
  logic card_state_stable;
  logic card_inserted;

  // Pulse signals for IRQ
  logic card_insertion;
  logic card_removal;

  sd_card_detect #(
    .DebouncePeriod (DebouncePeriod)
  ) card_detection (
    .clk_i,
    .rst_ni,
    .card_detect_pin_level_i (card_detect_pin_level),
    .card_state_stable_o (card_state_stable),
    .card_inserted_o     (card_inserted),
    .card_insertion_o (card_insertion),
    .card_removal_o   (card_removal)
  );

  always_comb begin
    card_detect_signal_selection_d = card_detect_signal_selection_q;
    card_detect_test_level_d = card_detect_test_level_q;

    if (bram_en) begin
      // Host Control Register (028h)
      // [07]    Card Detect Signal Selection
      // [06]    Card Detect Test Level
      if (bram_addr_actual == 8'h28 && bram_we[0]) begin
        card_detect_signal_selection_d = bram_wrdata[07];
        card_detect_test_level_d = bram_wrdata[06];
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_all_n) begin
    if (!rst_all_n) begin
      card_detect_signal_selection_q <= 1'b0;
      card_detect_test_level_q <= 1'b0;
    end else begin
      card_detect_signal_selection_q <= card_detect_signal_selection_d;
      card_detect_test_level_q <= card_detect_test_level_d;
    end
  end

  ///////////////////
  // Power control //
  ///////////////////

  // We only support one voltage, so this currently just holds whether the requested voltage matches.
  logic sd_bus_voltage_q, sd_bus_voltage_d;
  logic sd_bus_power_q, sd_bus_power_d;
  assign sdpower_o = sd_bus_power_q;

  // BRAM interfacing
  wire [7:0] reg_power_ctrl = {4'd0, sd_bus_voltage_q ? Voltage : 3'd0, sd_bus_power_q};

  always_comb begin
    sd_bus_voltage_d = sd_bus_voltage_q;
    sd_bus_power_d = sd_bus_power_q;

    if (bram_en) begin
      // Power Control Register (029h)
      // [03:01] SD Bus Voltage Select
      // [00]    SD Bus Power
      if (bram_addr_actual == 8'h28 && bram_we[1]) begin
        // Only turn on power when requested voltage matches our supported voltage.
        sd_bus_voltage_d = bram_wrdata[11:09] == Voltage;
        sd_bus_power_d = sd_bus_voltage_d && bram_wrdata[08];
      end
    end

    // When a card is removed we must immediately disconnect power.
    // TODO: CMD and DAT pins also need to stop being driven.
    if (card_removal) begin
      sd_bus_power_d = 1'b0;
    end
  end

  always_ff @(posedge clk_i or negedge rst_all_n) begin
    if (!rst_all_n) begin
      sd_bus_voltage_q <= 1'b0;
      sd_bus_power_q <= 1'b0;
    end else begin
      sd_bus_voltage_q <= sd_bus_voltage_d;
      sd_bus_power_q <= sd_bus_power_d;
    end
  end

  /////////////////////////
  // SDCLK Pause Control //
  /////////////////////////

  // Signals from DAT logic that determines whether clock should be paused due to
  // buffer full or stop at block gap request.
  logic sd_pause_req_full_sdclk;
  logic sd_pause_req_gap_sdclk;

  // Signals from host interface that determines whether clock should be resumed.
  logic sd_resume_req_full;
  logic sd_resume_req_gap;

  // CDC to clk domain.
  logic sd_pause_req_full;
  logic sd_pause_req_gap;
  prim_pulse_sync sd_pause_req_full_inst (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_ni),
    .src_pulse_i (sd_pause_req_full_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_ni),
    .dst_pulse_o (sd_pause_req_full)
  );
  prim_pulse_sync sd_pause_req_gap_inst (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_ni),
    .src_pulse_i (sd_pause_req_gap_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_ni),
    .dst_pulse_o (sd_pause_req_gap)
  );
  
  // Indication on whether we are currently paused.
  logic sd_paused_full_q, sd_paused_full_d;
  logic sd_paused_gap_q, sd_paused_gap_d;
  logic sd_paused_q, sd_paused_d;

  // We need a pulse back to the SDCLK domain so that it can know whe the clock is resumed.
  logic sd_resumed_sdclk;
  prim_pulse_sync sd_resumed_sdclk_sync (
    .clk_src_i (clk_i),
    .rst_src_ni (rst_ni),
    .src_pulse_i (sd_paused_q && !sd_paused_d),
    .clk_dst_i (sdclk_o),
    .rst_dst_ni (rst_ni),
    .dst_pulse_o (sd_resumed_sdclk)
  );

  always_comb begin
    sd_paused_full_d = sd_paused_full_q;
    sd_paused_gap_d = sd_paused_gap_q;
    unique case (1'b1)
      sd_pause_req_full: sd_paused_full_d = 1'b1;
      sd_resume_req_full: sd_paused_full_d = 1'b0;
      default:;
    endcase
    unique case (1'b1)
      sd_pause_req_gap: sd_paused_gap_d = 1'b1;
      sd_resume_req_gap: sd_paused_gap_d = 1'b0;
      default:;
    endcase
    sd_paused_d = sd_paused_full_d || sd_paused_gap_d;
  end

  always_ff @(posedge clk_i or negedge rst_dat_n) begin
    if (!rst_dat_n) begin
      sd_paused_full_q <= 1'b0;
      sd_paused_gap_q <= 1'b0;
      sd_paused_q <= 1'b0;
    end else begin
      sd_paused_full_q <= sd_paused_full_d;
      sd_paused_gap_q <= sd_paused_gap_d;
      sd_paused_q <= sd_paused_d;
    end
  end

  ///////////////////
  // Clock divider //
  ///////////////////

  logic sd_clock_enable_q, sd_clock_enable_d;
  logic [9:0] freq_select_q, freq_select_d;

  sd_clock_div clock_div (
    .clk_i,
    .rst_ni,
    .clk_base_i (sd_base_clock),
    .power_i    (sd_bus_power_q),
    .enable_i   (sd_clock_enable_q && !sd_paused_q),
    .freq_i     (freq_select_q),
    .sdclk_o
  );

  // BRAM interfacing
  wire [15:0] reg_clock_ctrl = {freq_select_q[7:0], freq_select_q[9:8], 3'b0, sd_clock_enable_q, 1'b1, 1'b1};

  always_comb begin
    freq_select_d = freq_select_q;
    sd_clock_enable_d = sd_clock_enable_q;

    if (bram_en) begin
      // Clock Control Register (02Ch)
      // [15:08] SDCLK Frequency Select
      // [07:06] Upper Bits of SDCLK Frequency Select
      //   - This is introduced in Host Controller 3.0 but it's easy to support anyway
      // [02]    SD Clock Enable
      // [01]    Internal Clock State
      // [00]    Internal Clock Enable
      //   - We hardwire this to 1 because we does not support stopping internal clock
      if (bram_addr_actual == 8'h2C) begin
        if (bram_we[0]) begin
          freq_select_d[9:8] = bram_wrdata[07:06];
          sd_clock_enable_d  = bram_wrdata[02];
        end
        if (bram_we[1]) begin
          freq_select_d[7:0] = bram_wrdata[15:08];
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_all_n) begin
    if (!rst_all_n) begin
      sd_clock_enable_q <= 1'b0;
      freq_select_q <= 0;
    end else begin
      sd_clock_enable_q <= sd_clock_enable_d;
      freq_select_q <= freq_select_d;
    end
  end

  ///////////////////////////
  // Timeout clock divider //
  ///////////////////////////

  logic timeout_start;
  logic timeout_clear;
  logic timeout_trigger;
  logic [3:0] timeout_freq_q, timeout_freq_d;
  sd_timeout_clk_div timeout_clock_div (
    .clk_base_i (sd_base_clock),
    .sdclk_i    (sdclk_o),
    .rst_ni     (rst_dat_n),
    .start_i    (timeout_start),
    .clear_i    (timeout_clear),
    .trigger_o  (timeout_trigger),
    .freq_i     (timeout_freq_q)
  );

  // BRAM interfacing
  wire [7:0] reg_timeout_ctrl = {4'd0, timeout_freq_q};

  always_comb begin
    timeout_freq_d = timeout_freq_q;

    if (bram_en) begin
      // Timeout Control Register (02Eh)
      // [03:00] Data Timeout Counter Value
      if (bram_addr_actual == 8'h2C && bram_we[2]) begin
        timeout_freq_d = bram_wrdata[16+3:16+0];
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_all_n) begin
    if (!rst_all_n) begin
      timeout_freq_q <= 0;
    end else begin
      timeout_freq_q <= timeout_freq_d;
    end
  end

  /////////////////////////////////
  // Interrupt, wakeup and error //
  /////////////////////////////////

  // Interrupt and error status.
  logic [8:0] normal_irq_status_q, normal_irq_status_d;
  logic [9:0] error_irq_status_q, error_irq_status_d;
  logic [5:0] auto_cmd12_error_q, auto_cmd12_error_d;
  logic auto_cmd12_no_issue_q, auto_cmd12_no_issue_d;

  // Interrupt-related RW enable registers.
  logic [8:0] normal_irq_enable_q, normal_irq_enable_d;
  logic [9:0] error_irq_enable_q, error_irq_enable_d;
  logic [8:0] normal_irq_signal_q, normal_irq_signal_d;
  logic [9:0] error_irq_signal_q, error_irq_signal_d;

  // Wakeup control registers
  logic wakeup_removal_q, wakeup_removal_d;
  logic wakeup_insertion_q, wakeup_insertion_d;
  logic wakeup_interrupt_q, wakeup_interrupt_d;

  // IRQ sources, independent from status enable.
  logic [8:0] normal_irq_trigger;
  logic [9:0] error_irq_trigger;
  logic [4:0] auto_cmd12_error_trigger;
  logic auto_cmd12_no_issue_trigger;

  // Connect sources defined above, internal, or not implemented
  assign normal_irq_trigger[7] = card_removal;
  assign normal_irq_trigger[6] = card_insertion;
  assign normal_irq_trigger[3] = 1'b0; // DMA

  assign error_irq_trigger[9] = 1'b0; // ADMA
  assign error_irq_trigger[8] = |auto_cmd12_error_trigger;
  assign error_irq_trigger[7] = 1'b0; // Current Limit

  // IRQ Aggregation
  logic error_irq;
  assign error_irq = |error_irq_status_q;
  assign irq_o = |normal_irq_status_q || error_irq;
  assign wakeup_o = (wakeup_interrupt_q & normal_irq_status_q[8]) ||
                    (wakeup_insertion_q & normal_irq_status_q[6]) ||
                    (wakeup_removal_q   & normal_irq_status_q[7]);

  wire [7:0] reg_wakeup_ctrl = {5'd0, wakeup_removal_q, wakeup_insertion_q, wakeup_interrupt_q};
  wire [31:0] reg_irq_status = {6'b0, error_irq_status_q, |error_irq, 6'b0, normal_irq_status_q};
  wire [31:0] reg_irq_enable = {6'b0, error_irq_enable_q, 7'b0, normal_irq_enable_q};
  wire [31:0] reg_irq_signal = {6'b0, error_irq_signal_q, 7'b0, normal_irq_signal_q};
  // Auto CMD12 Error Status Register (03Ch)
  // [07]    Command Not Issued by Auto CMD12 Error
  // [04]    Auto CMD12 Index Error
  // [03]    Auto CMD12 End Bit Error
  // [02]    Auto CMD12 CRC Error
  // [01]    Auto CMD12 Timeout Error
  // [00]    Auto CMD12 Not Executed
  wire [15:0] reg_auto_cmd12_error = {8'd0, auto_cmd12_no_issue_q, 2'b0, auto_cmd12_error_q};

  always_comb begin
    normal_irq_status_d = normal_irq_status_q;
    error_irq_status_d = error_irq_status_q;
    auto_cmd12_error_d = auto_cmd12_error_q;
    auto_cmd12_no_issue_d = auto_cmd12_no_issue_q;
    normal_irq_enable_d = normal_irq_enable_q;
    error_irq_enable_d = error_irq_enable_q;
    normal_irq_signal_d = normal_irq_signal_q;
    error_irq_signal_d = error_irq_signal_q;
    wakeup_removal_d = wakeup_removal_q;
    wakeup_insertion_d = wakeup_insertion_q;
    wakeup_interrupt_d = wakeup_interrupt_q;

    if (bram_en) begin
      unique case (bram_addr_actual)
        // Wakeup Control Register (02Bh)
        // [02]    Wakeup Event Enable on SD Card Removal
        // [01]    Wakeup Event Enable on SD Card Insertion
        // [00]    Wakeup Event Enable On Card Interrupt
        8'h28: begin
          if (bram_we[3]) begin
            wakeup_removal_d   = bram_wrdata[24+02];
            wakeup_insertion_d = bram_wrdata[24+01];
            wakeup_interrupt_d = bram_wrdata[24+00];
          end
        end

        // Normal Interrupt Status Register (030h)
        // [15]    Error Interrupt
        // [08:00] Normal Interrupts
        //
        // Error Interrupt Status Register (032h)
        // [09:00] Error Interrupts
        8'h30: begin
          if (bram_we[0]) normal_irq_status_d[7:0] &= ~bram_wrdata[7:0];
          // Card interrupt cannot be cleared this way
          // so we don't need to consider bram_we[1]
          if (bram_we[2]) error_irq_status_d[7:0] &= ~bram_wrdata[16+07:16+00];
          if (bram_we[3]) begin
            error_irq_status_d[9:8] &= ~bram_wrdata[16+09:16+08];
            // Auto CMD12 error clear will also clear the entire Auto CMD12 register
            if (bram_wrdata[16+08]) begin
              auto_cmd12_error_d = 0;
              auto_cmd12_no_issue_d = 1'b0;
            end
          end
        end

        // Normal Interrupt Status Enable Register (034h)
        // [08:00] Normal Interrupt Status Enables
        //
        // Error Interrupt Status Enable Register (036h)
        // [09:00] Error Interrupt Status Enables
        8'h34: begin
          if (bram_we[0]) begin
            normal_irq_enable_d[7:0] = bram_wrdata[7:0];
            normal_irq_status_d[7:0] &= bram_wrdata[7:0];
          end
          if (bram_we[1]) begin
            normal_irq_enable_d[8] = bram_wrdata[8];
            normal_irq_status_d[8] &= bram_wrdata[8];
          end
          if (bram_we[2]) begin
            error_irq_enable_d[7:0] = bram_wrdata[16+7:16+0];
            error_irq_status_d[7:0] &= bram_wrdata[16+7:16+0];
          end
          if (bram_we[3]) begin
            error_irq_enable_d[9:8] = bram_wrdata[16+9:16+8];
            error_irq_status_d[9:8] &= bram_wrdata[16+9:16+8];
            if (!bram_wrdata[16+08]) begin
              auto_cmd12_error_d = 0;
              auto_cmd12_no_issue_d = 1'b0;
            end
          end
        end

        // Normal Interrupt Signal Enable Register (038h)
        // [08:00] Normal Interrupt Signal Enables
        //
        // Error Interrupt Signal Enable Register (03Ah)
        // [09:00] Error Interrupt Signal Enables
        8'h38: begin
          if (bram_we[0]) normal_irq_signal_d[7:0] = bram_wrdata[7:0];
          if (bram_we[1]) normal_irq_signal_d[8] = bram_wrdata[8];
          if (bram_we[2]) error_irq_signal_d[7:0] = bram_wrdata[16+7:16+0];
          if (bram_we[3]) error_irq_signal_d[9:8] = bram_wrdata[16+9:16+8];
        end

        // Force Event Register for Auto CMD12 Error Status (050h)
        // [07]    Force Event for Command Not Issued by Auto CMD12 Error
        // [04:00] Force Event for Auto CMD12 Errors
        //
        // Force Event Register for Error Interrupt Status (052h)
        // [25:16] Force Event for Errors
        8'h50: begin
          if (bram_we[0]) begin
            auto_cmd12_error_d |= bram_wrdata[04:00];
            auto_cmd12_no_issue_d |= bram_wrdata[07];
            error_irq_status_d[8] |= |bram_wrdata[04:00] & error_irq_enable_d[8];
          end
          if (bram_we[2]) error_irq_status_d[7:0] |= bram_wrdata[16+7:16+0] & error_irq_enable_d[7:0];
          if (bram_we[3]) error_irq_status_d[9:8] |= bram_wrdata[16+9:16+8] & error_irq_enable_d[9:8];
        end

        default: ;
      endcase
    end

    normal_irq_status_d |= normal_irq_trigger & normal_irq_enable_q;
    error_irq_status_d |= error_irq_trigger & error_irq_enable_q;
    auto_cmd12_error_d |= auto_cmd12_error_trigger;
    auto_cmd12_no_issue_d |= auto_cmd12_no_issue_trigger;
  end

  always_ff @(posedge clk_i or negedge rst_dat_n) begin
    if (!rst_dat_n) begin
      normal_irq_status_q[5:1] <= '0;
    end else begin
      normal_irq_status_q[5:1] <= normal_irq_status_d[5:1];
    end
  end

  always_ff @(posedge clk_i or negedge rst_cmd_n) begin
    if (!rst_cmd_n) begin
      normal_irq_status_q[0] <= '0;
    end else begin
      normal_irq_status_q[0] <= normal_irq_status_d[0];
    end
  end

  always_ff @(posedge clk_i or negedge rst_all_n) begin
    if (!rst_all_n) begin
      normal_irq_status_q[8:6] <= '0;
      error_irq_status_q <= '0;
      auto_cmd12_error_q <= '0;
      auto_cmd12_no_issue_q <= 1'b0;
      normal_irq_enable_q <= '0;
      error_irq_enable_q <= '0;
      normal_irq_signal_q <= '0;
      error_irq_signal_q <= '0;
      wakeup_removal_q <= '0;
      wakeup_insertion_q <= '0;
      wakeup_interrupt_q <= '0;
    end else begin
      normal_irq_status_q[8:6] <= normal_irq_status_d[8:6];
      error_irq_status_q <= error_irq_status_d;
      auto_cmd12_error_q <= auto_cmd12_error_d;
      auto_cmd12_no_issue_q <= auto_cmd12_no_issue_d;
      normal_irq_enable_q <= normal_irq_enable_d;
      error_irq_enable_q <= error_irq_enable_d;
      normal_irq_signal_q <= normal_irq_signal_d;
      error_irq_signal_q <= error_irq_signal_d;
      wakeup_removal_q <= wakeup_removal_d;
      wakeup_insertion_q <= wakeup_insertion_d;
      wakeup_interrupt_q <= wakeup_interrupt_d;
    end
  end

  ////////////
  // Buffer //
  ////////////

  // Size of BRAM plus 1 to be able to track whether the buffer is full/empty.
  localparam BUF_PTR_WIDTH = 13;
  localparam BUF_SIZE = 2 ** (BUF_PTR_WIDTH - 1);

  function automatic [BUF_PTR_WIDTH-1:0] bin2gray(logic [BUF_PTR_WIDTH-1:0] binary);
    return (binary >> 1) ^ binary;
  endfunction

  function automatic [BUF_PTR_WIDTH-1:0] gray2bin(logic [BUF_PTR_WIDTH-1:0] gray);
    gray2bin = 0;
    for (int i = 0; i < BUF_PTR_WIDTH; i++) begin
      for (int j = i; j < BUF_PTR_WIDTH; j++) begin
        gray2bin[i] ^= gray[j];
      end
    end
  endfunction

  logic [BUF_PTR_WIDTH-1:0] buffer_host_ptr_q, buffer_host_ptr_d;
  logic [BUF_PTR_WIDTH-1:0] buffer_dat_ptr_sdclk_q, buffer_dat_ptr_sdclk_d;

  // This is meant to be declared in DAT logic but we need to use it early.
  logic [11:0] transfer_block_size_q, transfer_block_size_d;
  logic [15:0] block_count_q, block_count_d;
  logic multi_block_q, multi_block_d;
  logic block_count_enable_q, block_count_enable_d;

  // Cross clock domain with gray code.
  logic [BUF_PTR_WIDTH-1:0] buffer_host_ptr_gray;
  logic [BUF_PTR_WIDTH-1:0] buffer_host_ptr_gray_sdclk;
  logic [BUF_PTR_WIDTH-1:0] buffer_host_ptr_sdclk;
  logic [BUF_PTR_WIDTH-1:0] buffer_dat_ptr_gray_sdclk;
  logic [BUF_PTR_WIDTH-1:0] buffer_dat_ptr_gray;
  logic [BUF_PTR_WIDTH-1:0] buffer_dat_ptr;

  assign buffer_host_ptr_gray = bin2gray(buffer_host_ptr_q);
  assign buffer_host_ptr_sdclk = gray2bin(buffer_host_ptr_gray_sdclk);
  assign buffer_dat_ptr_gray_sdclk = bin2gray(buffer_dat_ptr_sdclk_q);
  assign buffer_dat_ptr = gray2bin(buffer_dat_ptr_gray);

  prim_flop_2sync #(.Width(BUF_PTR_WIDTH)) buffer_host_ptr_sync (
    .clk_i  (sdclk_o),
    .rst_ni (rst_dat_n),
    .d_i    (buffer_host_ptr_gray),
    .q_o    (buffer_host_ptr_gray_sdclk)
  );
  prim_flop_2sync #(.Width(BUF_PTR_WIDTH)) buffer_dat_ptr_sync (
    .clk_i  (clk_i),
    .rst_ni (rst_dat_n),
    .d_i    (buffer_dat_ptr_gray_sdclk),
    .q_o    (buffer_dat_ptr_gray)
  );

  wire [BUF_PTR_WIDTH-1:0] buffer_write_used_sdclk = buffer_host_ptr_sdclk - buffer_dat_ptr_sdclk_q;
  wire [BUF_PTR_WIDTH-1:0] buffer_read_left_sdclk = buffer_write_used_sdclk + BUF_SIZE;

  wire [BUF_PTR_WIDTH-1:0] buffer_read_used = buffer_dat_ptr - buffer_host_ptr_q;
  wire [BUF_PTR_WIDTH-1:0] buffer_write_left = buffer_read_used + BUF_SIZE;
  wire [BUF_PTR_WIDTH-1:0] buffer_read_left = buffer_host_ptr_q - buffer_dat_ptr + BUF_SIZE;

  // If the SD clock is paused due to the buffer being full, resume it when the buffer returns
  // to normal.
  //
  // Note that buffer_read_left is an overestimate, because dat_ptr could be moved to later slots already.
  // It's okay to use the overestimation here because sd_paused_full_q can only be set to high by SDCLK,
  // and when it is raised buffer_dat_ptr shouldn't be updated anymore, thus `buffer_read_left`
  // will indeed be accurate.
  //
  // Note that we do not resume the SD clock when the buffer is near full. If we resume the SD
  // clock when the buffer is near full, we could resume data transfer but not command issue, which means
  // we could starve CMD.
  assign sd_resume_req_full = sd_paused_full_q &&
                              buffer_read_left >= transfer_block_size_q &&
                              buffer_read_left >= 512;

  logic [31:0] buffer_host_rdata;


  logic buffer_read;
  logic buffer_write;
  logic buffer_read_q, buffer_read_d;
  logic [7:0] buffer_wdata;

  // Access the wide port as a 8-bit narrow port.
  // No need to register buffer_addr here because we only access this BRAM every other cycle.
  logic [3:0][7:0] buffer_rdata_full;
  wire [7:0] buffer_rdata = buffer_rdata_full[buffer_dat_ptr_sdclk_q[1:0]];

  prim_ram_2p #(
    .Width (32),
    // The size must be at least 2x largest block size supported (2048)
    .Depth (1024),
    .DataBitsPerMask (8)
  ) buffer (
    .clk_a_i (clk_i),
    .clk_b_i (sdclk_o),
    .a_req_i (bram_en && bram_addr_actual == 8'h20),
    .a_write_i (&bram_we),
    .a_addr_i (buffer_host_ptr_q[11:2]),
    .a_wdata_i (bram_wrdata),
    .a_wmask_i (32'hffffffff),
    .a_rdata_o (buffer_host_rdata),
    .b_req_i (buffer_read_q || buffer_write),
    .b_write_i (buffer_write),
    .b_addr_i (buffer_dat_ptr_sdclk_q[11:2]),
    .b_wdata_i ({4{buffer_wdata}}),
    .b_wmask_i (32'hff << (buffer_dat_ptr_sdclk_q[1:0] * 8)),
    .b_rdata_o (buffer_rdata_full)
  );

  logic cmd_issue, cmd_issue_q;
  logic data_present_q, data_present_d;
  logic data_transfer_direction_q, data_transfer_direction_d;

  logic w_mode_d, w_mode_q;
  logic [15:0] w_block_count_q, w_block_count_d;

  always_comb begin
    buffer_host_ptr_d = buffer_host_ptr_q;
    w_mode_d = w_mode_q;
    w_block_count_d = w_block_count_q;

    if (bram_en) begin
      if (bram_addr_actual == 8'h20) begin
        buffer_host_ptr_d = buffer_host_ptr_q + 4;
        if (w_mode_q && (buffer_host_ptr_d & (transfer_block_size_q - 1)) == 0) begin
          w_block_count_d = w_block_count_q - 1;
        end
      end
    end

    if (cmd_issue_q && data_present_q) begin
      // Clear buffer by updating host pointer rather than relying on both pointer
      // to be reset to zero. Because dat pointer shouldn't move anymore (the transfer must be
      // completed before the next transfer is issued), this is safe.
      // In the case where all data has been consumed, the pointers are equal so this is a no-op,
      // otherwise the buffer is cleared.
      //
      // Note that we shouldn't reset both host and dat pointers to zero to clear the buffer.
      // Otherwise, because these pointers are resetted in different
      // clock we may see some transients states and these can cause glitches in our
      // buffer size computation logic.
      buffer_host_ptr_d = buffer_dat_ptr;

      w_mode_d = !data_transfer_direction_q;
      if (!data_transfer_direction_q) begin
        w_block_count_d = multi_block_q ? (block_count_enable_q ? block_count_q : 0) : 1;
      end
    end
  end

  wire buffer_read_enable = !w_mode_q && buffer_read_used >= transfer_block_size_q;
  wire buffer_write_enable = w_mode_q && w_block_count_q != 0 && buffer_write_left >= transfer_block_size_q;
  logic buffer_read_enable_q;
  logic buffer_write_enable_q;
  assign normal_irq_trigger[5] = !buffer_read_enable_q && buffer_read_enable;
  assign normal_irq_trigger[4] = !buffer_write_enable_q && buffer_write_enable;
  
  always_ff @(posedge clk_i or negedge rst_dat_n) begin
    if (!rst_dat_n) begin
      cmd_issue_q <= 1'b0;
      buffer_host_ptr_q <= 0;
      w_mode_q <= 1'b0;
      w_block_count_q <= 0;
      buffer_read_enable_q <= 1'b0;
      buffer_write_enable_q <= 1'b0;
    end else begin
      cmd_issue_q <= cmd_issue;
      buffer_host_ptr_q <= buffer_host_ptr_d;
      w_mode_q <= w_mode_d;
      w_block_count_q <= w_block_count_d;
      buffer_read_enable_q <= buffer_read_enable;
      buffer_write_enable_q <= buffer_write_enable;
    end
  end

  //
  // DAT Control logic
  //

  // SD supports multi-block transfer. When performing transfer, we may run into condition where
  // the buffer is full. However we do have a reliable way of pausing transfer mid-transaction.
  // The reliable way to pause transaction would be to pause the SDCLK in the block gap.
  //
  // This however has a caveat: if there is a CMD_wo_DAT in progress, we couldn't stop SDCLK
  // anymore. To avoid this, we need to have a buffer that can store more than a block. When the
  // buffer has less than one block left, we need to prevent any CMD_wo_DAT except CMD12 from being
  // issued.

  // Host control register
  logic dat_width_q, dat_width_d;

  // Block gap control registers
  logic gap_continue_q, gap_continue_d;
  logic gap_stop_q, gap_stop_d;
  logic gap_continue_sdclk;
  prim_flop_2sync #(.Width(1)) gap_continue_sync(.clk_i(sdclk_o), .rst_ni(rst_dat_n), .d_i(gap_continue_q), .q_o(gap_continue_sdclk));

  // These registers control how we do data transfer.
  // These registers ought not to be changed when there is a transaction in progress, so we need
  // not implement clock-domain crossing for these registers.
  // logic [11:0] transfer_block_size_q, transfer_block_size_d;
  // logic [15:0] block_count_q, block_count_d;
  // logic multi_block_q, multi_block_d;
  logic auto_cmd12_enable_q, auto_cmd12_enable_d;
  // logic block_count_enable_q, block_count_enable_d;

  // We're on the last buffer block if the next chunk will coincide with the current chunk
  // available to the host.
  wire buffer_is_full_sdclk = buffer_read_left_sdclk < transfer_block_size_q;
  wire buffer_is_empty_sdclk = buffer_write_used_sdclk < transfer_block_size_q;

  logic reset_buf_r;
  logic reset_buf_w;

  logic transfer_complete_sdclk, transfer_complete;
  logic block_gap_sdclk, block_gap;
  logic card_irq_sdclk, card_irq;
  logic data_crc_error_sdclk, data_crc_error;
  logic data_end_bit_error_sdclk, data_end_bit_error;
  logic data_timeout_error_sdclk, data_timeout_error;
  prim_pulse_sync transfer_complete_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_dat_n),
    .src_pulse_i (transfer_complete_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_dat_n),
    .dst_pulse_o (transfer_complete)
  );
  prim_pulse_sync block_gap_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_dat_n),
    .src_pulse_i (block_gap_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_dat_n),
    .dst_pulse_o (block_gap)
  );
  prim_pulse_sync card_irq_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_dat_n),
    .src_pulse_i (card_irq_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_dat_n),
    .dst_pulse_o (card_irq)
  );
  prim_pulse_sync data_crc_error_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_dat_n),
    .src_pulse_i (data_crc_error_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_dat_n),
    .dst_pulse_o (data_crc_error)
  );
  prim_pulse_sync data_end_bit_error_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_dat_n),
    .src_pulse_i (data_end_bit_error_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_dat_n),
    .dst_pulse_o (data_end_bit_error)
  );
  prim_pulse_sync data_timeout_error_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_dat_n),
    .src_pulse_i (data_timeout_error_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_dat_n),
    .dst_pulse_o (data_timeout_error)
  );

  logic read_xfer_active_sdclk;
  logic write_xfer_active_sdclk;
  logic dat_line_active_sdclk;
  logic auto_cmd12_issue;

  assign normal_irq_trigger[8] = card_irq;
  assign normal_irq_trigger[2] = block_gap;

  logic w_mode_sdclk_q, w_mode_sdclk_d;
  logic buffer_is_empty_sdclk_q;

  // * If there is a read transfer in place, we cannot issue a command when the buffer is running
  //   low or when stop at block gap is requested, as we cannot stop SDCLK when command issue is
  //   in progress.
  wire cmd_issue_allowed = w_mode_sdclk_q || (buffer_read_left_sdclk >= 512 && !gap_stop_q);

  always_comb begin
    w_mode_sdclk_d = w_mode_sdclk_q;
    buffer_read_d = 1'b0;
    buffer_dat_ptr_sdclk_d = buffer_dat_ptr_sdclk_q;

    unique case (1'b1)
      reset_buf_r: begin
        w_mode_sdclk_d = 1'b0;
      end
      reset_buf_w: begin
        w_mode_sdclk_d = 1'b1;
      end
      w_mode_sdclk_q && buffer_is_empty_sdclk_q && !buffer_is_empty_sdclk: begin
        buffer_read_d = 1'b1;
      end
      buffer_read: begin
        buffer_read_d = 1'b1;
        buffer_dat_ptr_sdclk_d = buffer_dat_ptr_sdclk_q + 1;
      end
      buffer_write: begin
        buffer_dat_ptr_sdclk_d = buffer_dat_ptr_sdclk_q + 1;
      end
      default:;
    endcase
  end

  always_ff @(posedge sdclk_o or negedge rst_dat_n) begin
    if (!rst_dat_n) begin
      buffer_read_q <= 1'b0;
      buffer_dat_ptr_sdclk_q <= 0;
      buffer_is_empty_sdclk_q <= 1'b0;
      w_mode_sdclk_q <= 1'b0;
    end else begin
      buffer_read_q <= buffer_read_d;
      buffer_dat_ptr_sdclk_q <= buffer_dat_ptr_sdclk_d;
      buffer_is_empty_sdclk_q <= buffer_is_empty_sdclk;
      w_mode_sdclk_q <= w_mode_sdclk_d;
    end
  end

  ///////////////////////
  // CMD Control Logic //
  ///////////////////////

  // IO buffer
  logic cmd_o_sdclk;
  logic cmd_t_sdclk;
  logic cmd_i_sdclk;
  always_ff @(posedge sdclk_o or negedge rst_cmd_n) begin
    if (!rst_cmd_n) begin
      cmd_o <= 1'b1;
      cmd_t <= 1'b1;
      cmd_i_sdclk <= 1'b1;
    end else begin
      cmd_o <= cmd_o_sdclk;
      cmd_t <= cmd_t_sdclk;
      cmd_i_sdclk <= cmd_i;
    end
  end

  // IO buffer
  logic [3:0] dat_o_sdclk;
  logic [3:0] dat_t_sdclk;
  logic [3:0] dat_i_sdclk;
  always_ff @(posedge sdclk_o or negedge rst_dat_n) begin
    if (!rst_dat_n) begin
      dat_o <= 4'b1111;
      dat_t <= 4'b1111;
      dat_i_sdclk <= 4'b1111;
    end else begin
      dat_o <= dat_o_sdclk;
      dat_t <= dat_t_sdclk;
      dat_i_sdclk <= dat_i;
    end
  end

  // These registers indicate the message to be sent on CMD line.
  // These registers ought not to be changed when there is a transaction in progress, so we need
  // not implement clock-domain crossing for these registers.
  logic [31:0] argument_q, argument_d;
  logic [5:0] command_index_q, command_index_d;
  logic [1:0] response_type_q, response_type_d;
  logic [127:0] response_q;

  logic [1:0] command_type_q, command_type_d;
  logic command_index_check_q, command_index_check_d;
  logic command_crc_check_q, command_crc_check_d;

  logic [15:0] block_count_left;

  logic cmd_issue_sdclk;
  prim_pulse_sync cmd_issue_sync_inst (
    .clk_src_i (clk_i),
    .rst_src_ni (rst_cmd_n),
    .src_pulse_i (cmd_issue),
    .clk_dst_i (sdclk_o),
    .rst_dst_ni (rst_cmd_n),
    .dst_pulse_o (cmd_issue_sdclk)
  );

  logic command_end_sdclk;
  logic command_complete_sdclk;
  logic auto_cmd12_complete_sdclk;
  logic command_inhibit_cmd_clear_sdclk, command_inhibit_cmd_clear;
  logic cmd_index_error_sdclk, cmd_index_error;
  logic cmd_crc_error_sdclk, cmd_crc_error;
  logic cmd_end_bit_error_sdclk, cmd_end_bit_error;
  logic cmd_timeout_error_sdclk, cmd_timeout_error;
  logic auto_cmd12_error_cmd_no_ex_sdclk, auto_cmd12_error_cmd_no_ex;
  logic auto_cmd12_error_index_sdclk, auto_cmd12_error_index;
  logic auto_cmd12_error_crc_sdclk, auto_cmd12_error_crc;
  logic auto_cmd12_error_end_bit_sdclk, auto_cmd12_error_end_bit;
  logic auto_cmd12_error_timeout_sdclk, auto_cmd12_error_timeout;
  logic auto_cmd12_error_no_ex_sdclk, auto_cmd12_error_no_ex;
  prim_pulse_sync command_inhibit_cmd_clear_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_cmd_n),
    .src_pulse_i (command_inhibit_cmd_clear_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_cmd_n),
    .dst_pulse_o (command_inhibit_cmd_clear)
  );
  prim_pulse_sync cmd_index_error_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_cmd_n),
    .src_pulse_i (cmd_index_error_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_cmd_n),
    .dst_pulse_o (cmd_index_error)
  );
  prim_pulse_sync cmd_crc_error_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_cmd_n),
    .src_pulse_i (cmd_crc_error_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_cmd_n),
    .dst_pulse_o (cmd_crc_error)
  );
  prim_pulse_sync cmd_end_bit_error_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_cmd_n),
    .src_pulse_i (cmd_end_bit_error_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_cmd_n),
    .dst_pulse_o (cmd_end_bit_error)
  );
  prim_pulse_sync cmd_timeout_error_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_cmd_n),
    .src_pulse_i (cmd_timeout_error_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_cmd_n),
    .dst_pulse_o (cmd_timeout_error)
  );
  prim_pulse_sync auto_cmd12_error_cmd_no_ex_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_cmd_n),
    .src_pulse_i (auto_cmd12_error_cmd_no_ex_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_cmd_n),
    .dst_pulse_o (auto_cmd12_error_cmd_no_ex)
  );
  prim_pulse_sync auto_cmd12_error_index_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_cmd_n),
    .src_pulse_i (auto_cmd12_error_index_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_cmd_n),
    .dst_pulse_o (auto_cmd12_error_index)
  );
  prim_pulse_sync auto_cmd12_error_crc_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_cmd_n),
    .src_pulse_i (auto_cmd12_error_crc_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_cmd_n),
    .dst_pulse_o (auto_cmd12_error_crc)
  );
  prim_pulse_sync auto_cmd12_error_end_bit_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_cmd_n),
    .src_pulse_i (auto_cmd12_error_end_bit_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_cmd_n),
    .dst_pulse_o (auto_cmd12_error_end_bit)
  );
  prim_pulse_sync auto_cmd12_error_timeout_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_cmd_n),
    .src_pulse_i (auto_cmd12_error_timeout_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_cmd_n),
    .dst_pulse_o (auto_cmd12_error_timeout)
  );
  prim_pulse_sync auto_cmd12_error_no_ex_sync (
    .clk_src_i (sdclk_o),
    .rst_src_ni (rst_cmd_n),
    .src_pulse_i (auto_cmd12_error_no_ex_sdclk),
    .clk_dst_i (clk_i),
    .rst_dst_ni (rst_cmd_n),
    .dst_pulse_o (auto_cmd12_error_no_ex)
  );

  sd_cmd_ctrl #(.ConflictDetection(ConflictDetection + 2)) cmd_ctrl (
    .sdclk_i (sdclk_o),
    .rst_cmd_ni (rst_cmd_n),
    .argument_i (argument_q),
    .command_index_i (command_index_q),
    .command_type_i (command_type_q),
    .response_type_i (response_type_q),
    .command_index_check_i (command_index_check_q),
    .command_crc_check_i (command_crc_check_q),
    .response_o (response_q),
    .cmd_issue_i (cmd_issue_sdclk),
    .auto_cmd12_issue_i (auto_cmd12_issue),
    .cmd_issue_allowed_i (cmd_issue_allowed),
    .command_end_o (command_end_sdclk),
    .command_complete_o (command_complete_sdclk),
    .auto_cmd12_complete_o (auto_cmd12_complete_sdclk),
    .command_inhibit_cmd_clear_o (command_inhibit_cmd_clear_sdclk),
    .cmd_index_error_o (cmd_index_error_sdclk),
    .cmd_crc_error_o (cmd_crc_error_sdclk),
    .cmd_end_bit_error_o (cmd_end_bit_error_sdclk),
    .cmd_timeout_error_o (cmd_timeout_error_sdclk),
    .auto_cmd12_error_cmd_no_ex_o (auto_cmd12_error_cmd_no_ex_sdclk),
    .auto_cmd12_error_index_o (auto_cmd12_error_index_sdclk),
    .auto_cmd12_error_end_bit_o (auto_cmd12_error_end_bit_sdclk),
    .auto_cmd12_error_crc_o (auto_cmd12_error_crc_sdclk),
    .auto_cmd12_error_timeout_o (auto_cmd12_error_timeout_sdclk),
    .auto_cmd12_error_no_ex_o (auto_cmd12_error_no_ex_sdclk),
    .cmd_i (cmd_i_sdclk),
    .cmd_o (cmd_o_sdclk),
    .cmd_t (cmd_t_sdclk)
  );
  
  sd_dat_ctrl dat_ctrl(
    .sdclk_i (sdclk_o),
    .rst_dat_ni (rst_dat_n),
    .response_type_i (response_type_q),
    .data_present_i (data_present_q),
    .command_type_i (command_type_q),
    .dat_width_i (dat_width_q),
    .transfer_direction_i (data_transfer_direction_q),
    .auto_cmd12_enable_i (auto_cmd12_enable_q),
    .block_size_i (transfer_block_size_q),
    .block_count_i (block_count_q),
    .block_count_o (block_count_left),
    .multi_block_i (multi_block_q),
    .block_count_enable_i (block_count_enable_q),
    .gap_stop_i (gap_stop_q),
    .gap_continue_i (gap_continue_sdclk),
    .r_data_o (buffer_wdata),
    .r_valid_o (buffer_write),
    .r_buf_full_i (buffer_is_full_sdclk),
    .r_reset_o (reset_buf_r),
    .w_data_i (buffer_rdata),
    .w_ready_o (buffer_read),
    .w_buf_empty_i (buffer_is_empty_sdclk),
    .w_reset_o (reset_buf_w),
    .timeout_start_o   (timeout_start),
    .timeout_clear_o   (timeout_clear),
    .timeout_trigger_i (timeout_trigger),
    .sd_pause_req_full_o (sd_pause_req_full_sdclk),
    .sd_pause_req_gap_o (sd_pause_req_gap_sdclk),
    .sd_resumed_i (sd_resumed_sdclk),
    .block_gap_o (block_gap_sdclk),
    .read_xfer_active_o (read_xfer_active_sdclk),
    .write_xfer_active_o (write_xfer_active_sdclk),
    .dat_line_active_o (dat_line_active_sdclk),
    .auto_cmd12_issue_o (auto_cmd12_issue),
    .cmd_issue_i (cmd_issue_sdclk),
    .command_end_i (command_end_sdclk),
    .command_complete_i (command_complete_sdclk),
    .auto_cmd12_complete_i (auto_cmd12_complete_sdclk),
    .transfer_complete_o (transfer_complete_sdclk),
    .card_irq_o (card_irq_sdclk),
    .data_crc_error_o (data_crc_error_sdclk),
    .data_end_bit_error_o (data_end_bit_error_sdclk),
    .data_timeout_error_o (data_timeout_error_sdclk),
    .dat_i (dat_i_sdclk),
    .dat_o (dat_o_sdclk),
    .dat_t (dat_t_sdclk)
  );

  logic read_xfer_active;
  logic write_xfer_active;
  logic dat_line_active;
  prim_flop_2sync #(.Width(1)) read_xfer_active_sync (.clk_i, .rst_ni, .d_i(read_xfer_active_sdclk), .q_o(read_xfer_active));
  prim_flop_2sync #(.Width(1)) write_xfer_active_sync (.clk_i, .rst_ni, .d_i(write_xfer_active_sdclk), .q_o(write_xfer_active));
  prim_flop_2sync #(.Width(1)) dat_line_active_sync (.clk_i, .rst_ni, .d_i(dat_line_active_sdclk), .q_o(dat_line_active));
  logic command_inhibit_dat_q;
  wire command_inhibit_dat = read_xfer_active || dat_line_active;

  logic command_inhibit_cmd_q, command_inhibit_cmd_d;

  assign normal_irq_trigger[00] = command_inhibit_cmd_clear;
  assign normal_irq_trigger[01] = command_inhibit_dat_q && !command_inhibit_dat;
  assign error_irq_trigger[06:00] = {
    data_end_bit_error,
    data_crc_error,
    data_timeout_error,
    cmd_index_error,
    cmd_end_bit_error,
    cmd_crc_error,
    cmd_timeout_error
  };
  assign auto_cmd12_no_issue_trigger = auto_cmd12_error_cmd_no_ex;
  assign auto_cmd12_error_trigger = {
    auto_cmd12_error_index,
    auto_cmd12_error_end_bit,
    auto_cmd12_error_crc,
    auto_cmd12_error_timeout,
    auto_cmd12_error_no_ex
  };

  always_comb begin
    command_inhibit_cmd_d = command_inhibit_cmd_q;

    if (cmd_issue) begin
      command_inhibit_cmd_d = 1'b1;
    end

    if (command_inhibit_cmd_clear) begin
      command_inhibit_cmd_d = 1'b0;
    end
  end

  always_ff @(posedge clk_i or negedge rst_all_n) begin
    if (!rst_all_n) begin
      command_inhibit_cmd_q <= 1'b0;
      command_inhibit_dat_q <= 1'b0;
    end else begin
      command_inhibit_cmd_q <= command_inhibit_cmd_d;
      command_inhibit_dat_q <= command_inhibit_dat;
    end
  end

  /////////////////////////////////////////////
  // Present State and Other Readonly States //
  /////////////////////////////////////////////

  // Synchronise async signals to clk.
  // Note that this cannot be merged with cmd/data controller's signal due to clock difference.
  logic cmd_i_sync;
  logic [3:0] dat_i_sync;
  logic sdwp_n_sync;
  prim_flop_2sync #(.Width(1)) cmd_i_sync_inst (.clk_i, .rst_ni, .d_i(cmd_i), .q_o(cmd_i_sync));
  prim_flop_2sync #(.Width(4)) dat_i_sync_inst (.clk_i, .rst_ni, .d_i(dat_i), .q_o(dat_i_sync));
  prim_flop_2sync #(.Width(1)) sdwp_n_sync_inst (.clk_i, .rst_ni, .d_i(sdwp_ni), .q_o(sdwp_n_sync));

  // Present State Register (024h)
  // [24]    CMD Line Signal Level
  // [23:20] DAT[3:0] Line Signal Level
  // [19]    Write Protect Switch Pin Level
  // [18]    Card Detect Pin Level
  // [17]    Card State Stable
  // [16]    Card Inserted
  // [11]    Buffer Read Enable
  // [10]    Buffer write Enable
  // [09]    Read Transfer Active
  // [08]    Write Transfer Active
  // [02]    DAT Line Active
  // [01]    Command Inhibit (DAT)
  // [00]    Command Inhibit (CMD)
  wire [31:0] reg_present_state = {
    7'b0, cmd_i_sync, dat_i_sync, sdwp_n_sync, card_detect_pin_level, card_state_stable, card_inserted,
    4'b0, buffer_read_enable, buffer_write_enable, read_xfer_active, write_xfer_active, 5'b0, dat_line_active, command_inhibit_dat, command_inhibit_cmd_q
  };

  // Capabilities Register (040h)
  // [28]    64-bit System Bus Support
  // [26]    Voltage Support 1.8V
  // [25]    Voltage Support 3.0V
  // [24]    Voltage Support 3.3V
  // [23]    Suspend/Resume Support
  // [22]    SDMA Support
  // [21]    High Speed Support
  // [19]    ADMA2 Support
  // [17:16] Max Block Length
  // [13:08] Base Clock Frequency For SD Clock
  // [07]    Timeout Clock Unit
  // [05:00] Timeout Clock Frequency
  wire [63:0] reg_cap = {
    35'd0,
    1'b0, // ADMA not yet supported
    1'b0,
    Voltage == 3'b101,
    Voltage == 3'b110,
    Voltage == 3'b111,
    1'b0, // No Suspend/Resume Support
    1'b0, // No SDMA Support
    1'b0, // No High Speed Support
    1'b0,
    1'b0, // ADMA2 Support yet to complete
    1'b0,
    2'b00, // Max Block Length = 512 byte
    2'b0,
    6'(SdBaseClockFreq), // 25MHz
    1'b1, // MHz
    1'b0,
    6'(SdBaseClockFreq) // 25MHz
  };

  // Maximum Current Capabilities Register (048h)
  wire [63:0] req_current_cap = {
    40'd0,
    8'(Voltage == 3'b101 ? Current : 0),
    8'(Voltage == 3'b110 ? Current : 0),
    8'(Voltage == 3'b111 ? Current : 0)
  };

  // Slot Interrupt Status Register (0FCh)
  // [7:0] Interrupt Signal for Each Slot
  wire [15:0] reg_slot_irq = {8'd0, slot_irq_i};

  // Host Controller Version Register (0FEh)
  // [15:08] Vendor Version Number
  //   - 01h = SD Host Controller Specification Version 2.00
  // [07:00] Specification Version Number
  wire [15:0] reg_host_ver = {8'b0, 8'd1};

  //
  // BRAM Interfacing
  //

  logic [31:0] bram_reg_read_q, bram_reg_read_d;
  logic use_buffer_output_q, use_buffer_output_d;
  assign bram_rddata = use_buffer_output_q ? buffer_host_rdata : bram_reg_read_q;
  logic led_d;

  always_comb begin
    bram_reg_read_d = bram_reg_read_q;
    use_buffer_output_d = use_buffer_output_q;
    transfer_block_size_d = transfer_block_size_q;
    block_count_d = block_count_q;
    argument_d = argument_q;
    multi_block_d = multi_block_q;
    data_transfer_direction_d = data_transfer_direction_q;
    auto_cmd12_enable_d = auto_cmd12_enable_q;
    block_count_enable_d = block_count_enable_q;
    command_index_d = command_index_q;
    command_type_d = command_type_q;
    data_present_d = data_present_q;
    command_index_check_d = command_index_check_q;
    command_crc_check_d = command_crc_check_q;
    response_type_d = response_type_q;
    dat_width_d = dat_width_q;
    led_d = led_o;
    gap_continue_d = gap_continue_q;
    gap_stop_d = gap_stop_q;

    sd_resume_req_gap = 1'b0;

    cmd_issue = 1'b0;

    if (transfer_complete) begin
      block_count_d = block_count_left;
    end

    if (!command_inhibit_dat_q && command_inhibit_dat) begin
      gap_continue_d = 1'b0;
    end

    if (bram_en) begin
      use_buffer_output_d = 1'b0;

      unique case (bram_addr_actual)
        // SDMA System Address (000h)
        // - SDMA is not supported
        8'h00: bram_reg_read_d = 0;

        // Block Size Register (004h)
        // [14:12] Host SDMA Buffer Boundary
        // - SDMA is not supported
        // [11:00] Transfer Block Size
        //
        // Block Count Register (006h)
        // [31:16] Block Count For Current Transfer
        8'h04: begin
          if (&bram_we[1:0] && !command_inhibit_dat) transfer_block_size_d = bram_wrdata[11: 0];
          if (&bram_we[3:2] && !command_inhibit_dat) block_count_d = bram_wrdata[31:16];
          bram_reg_read_d = {block_count_q, 4'b0, transfer_block_size_q};
        end

        // Command Argument (008h)
        8'h08: begin
          if (&bram_we && !command_inhibit_cmd_q) argument_d = bram_wrdata;
          bram_reg_read_d = argument_q;
        end

        // Transfer Mode Register (00Ch)
        // [05]    Multi/Single Block Select
        // [04]    Data Transfer Direction Select
        // [02]    Auto CMD12 Enable
        // [01]    Block Count Enable
        // [00]    DMA Enable (No Support Yet)
        //
        // Command Register (00Eh)
        // [29:24] Command Index
        // [23:22] Command Type
        // [21]    Data Present Select
        // [20]    Command Index Check Enable
        // [19]    Command CRC Check Enable
        // [17:16] Response Type Select
        8'h0C: begin
          if (bram_we[0] && !command_inhibit_dat) begin
            multi_block_d = bram_wrdata[05];
            data_transfer_direction_d = bram_wrdata[04];
            auto_cmd12_enable_d = bram_wrdata[02];
            block_count_enable_d = bram_wrdata[01];
          end
          if (bram_we[3] && !command_inhibit_cmd_q) begin
            command_index_d = bram_wrdata[29:24];
            cmd_issue = 1'b1;
          end
          if (bram_we[2] && !command_inhibit_cmd_q) begin
            command_type_d = bram_wrdata[23:22];
            data_present_d = bram_wrdata[21];
            command_index_check_d = bram_wrdata[20];
            command_crc_check_d = bram_wrdata[19];
            response_type_d = bram_wrdata[17:16];
          end
          bram_reg_read_d = {
            2'b0, command_index_q, command_type_q, data_present_q, command_index_check_q, command_crc_check_q, 1'b0, response_type_q,
            10'b0, multi_block_q, data_transfer_direction_q, 1'b0, auto_cmd12_enable_q, block_count_enable_q, 1'b0
          };
        end

        // Response Registers (010h)
        8'h10: bram_reg_read_d = response_q[31:0];
        8'h14: bram_reg_read_d = response_q[63:32];
        8'h18: bram_reg_read_d = response_q[95:64];
        8'h1C: bram_reg_read_d = response_q[127:96];

        // Buffer Data (020h)
        8'h20: begin
          use_buffer_output_d = 1'b1;
          bram_reg_read_d = 0;
        end

        // Present State Register (024h)
        8'h24: bram_reg_read_d = reg_present_state;

        // Host Control Register (028h)
        // [07]    Card Detect Signal Selection
        // [06]    Card Detect Test Level
        // [04:03] DMA Select
        // [02]    High Speed Enable (Optional)
        // [01]    Data Transfer Width
        // [00]    LED Control
        //
        // Power Control Register (029h)
        // [11:09] SD Bus Voltage Select
        // [08]    SD Bus Power
        //
        // Block Gap Control Register (02Ah)
        // [19]    Interrupt At Block Gap
        // [18]    Read Wait Control
        // [17]    Continue Request
        // [16]    Stop At Block Gap Request
        //
        // Wakeup Control Register (02Bh)
        8'h28: begin
          if (bram_we[0]) begin
            dat_width_d = bram_wrdata[01];
            led_d = bram_wrdata[00];
          end
          if (bram_we[2]) begin
            gap_continue_d  = bram_wrdata[17];
            gap_stop_d      = bram_wrdata[16];
            if (sd_paused_gap_q && gap_continue_d) begin
              sd_resume_req_gap = 1'b1;
            end
          end
          bram_reg_read_d = {
            reg_wakeup_ctrl,
            5'd0, wakeup_removal_q, wakeup_insertion_q, wakeup_interrupt_q,
            4'd0, 1'b0, 1'b0, gap_continue_q, gap_stop_q,
            reg_power_ctrl,
            card_detect_signal_selection_q, card_detect_test_level_q, 1'b0, 2'b0, 1'b0, dat_width_q, led_o
          };
        end

        // Clock Control Register (02Ch)
        // Timeout Control Register (02Eh)
        // Software Reset Register (02Fh)
        8'h2C: bram_reg_read_d = {reg_sw_rst, reg_timeout_ctrl, reg_clock_ctrl};

        // Normal Interrupt Status Register (030h)
        // Error Interrupt Status Register (032h)
        8'h30: bram_reg_read_d = reg_irq_status;

        // Normal Interrupt Status Enable Register (034h)
        // Error Interrupt Status Enable Register (036h)
        8'h34: bram_reg_read_d = reg_irq_enable;

        // Normal Interrupt Signal Enable Register (038h)
        // Error Interrupt Signal Enable Register (03Ah)
        8'h38: bram_reg_read_d = reg_irq_signal;

        // Auto CMD12 Error Status Register (03Ch)
        8'h3C: bram_reg_read_d = {16'd0, reg_auto_cmd12_error};

        // Capabilities Register (040h)
        8'h40: bram_reg_read_d = reg_cap[31:0];
        8'h44: bram_reg_read_d = reg_cap[63:32];

        // Maximum Current Capabilities Register (048h)
        8'h48: bram_reg_read_d = req_current_cap[31:0];
        8'h4C: bram_reg_read_d = req_current_cap[63:32];

        // Force Event Register for Auto CMD12 Error Status (050h)
        // Force Event Register for Error Interrupt Status (052h)
        8'h50: bram_reg_read_d = 0;

        // ADMA Error Status Register (054h)
        8'h54: bram_reg_read_d = 0;

        // ADMA System Address Register (058h)
        8'h58: bram_reg_read_d = 0;
        8'h5C: bram_reg_read_d = 0;

        // Slot Interrupt Status Register (0FCh)
        // Host Controller Version Register (0FEh)
        8'hFC: bram_reg_read_d = {reg_host_ver, reg_slot_irq};

        // All other registers are reserved
        default: bram_reg_read_d = 0;
      endcase
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) begin
      bram_reg_read_q    <= 0;
      use_buffer_output_q <= 1'b0;
    end
    else begin
      bram_reg_read_q <= bram_reg_read_d;
      use_buffer_output_q <= use_buffer_output_d;
    end

  always_ff @(posedge clk_i or negedge rst_dat_n)
    if (!rst_dat_n) begin
      gap_continue_q <= 1'b0;
      gap_stop_q <= 1'b0;
    end
    else begin
      gap_continue_q <= gap_continue_d;
      gap_stop_q <= gap_stop_d;
    end

  always_ff @(posedge clk_i or negedge rst_all_n)
    if (!rst_all_n) begin
      dat_width_q <= 1'b1;
      transfer_block_size_q <= 0;
      block_count_q <= 0;
      argument_q <= 0;
      multi_block_q <= 1'b0;
      data_transfer_direction_q <= 1'b0;
      auto_cmd12_enable_q <= 1'b0;
      block_count_enable_q <= 1'b0;
      command_index_q <= 0;
      command_type_q <= 0;
      data_present_q <= 1'b0;
      command_index_check_q <= 1'b0;
      command_crc_check_q <= 1'b0;
      response_type_q <= 2'b00;
      led_o <= 1'b0;
    end
    else begin
      dat_width_q <= dat_width_d;
      transfer_block_size_q <= transfer_block_size_d;
      block_count_q <= block_count_d;
      argument_q <= argument_d;
      multi_block_q <= multi_block_d;
      data_transfer_direction_q <= data_transfer_direction_d;
      auto_cmd12_enable_q <= auto_cmd12_enable_d;
      block_count_enable_q <= block_count_enable_d;
      command_index_q <= command_index_d;
      command_type_q <= command_type_d;
      data_present_q <= data_present_d;
      command_index_check_q <= command_index_check_d;
      command_crc_check_q <= command_crc_check_d;
      response_type_q <= response_type_d;
      led_o <= led_d;
    end

endmodule
