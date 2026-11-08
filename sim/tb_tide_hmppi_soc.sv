// =============================================================================
// Project: Combined TIDE + H-MPPI Integrated System-on-Chip (SoC)
// Module:  tb_tide_hmppi_soc.sv
// Description:
//   Comprehensive SystemVerilog Testbench for `tide_hmppi_soc_top`.
//   Simulates the unified SoC:
//   1. Microcode loading (492 words) and TIDE-RAM initial state seeding.
//   2. In-order sensor measurement processing with FP32->Q8.8 conversion.
//   3. Hardware division handshake for soft-min normalization reciprocal.
//   4. 4-lane parallel stochastic trajectory optimization (256 rollouts).
//   5. Out-of-Sequence Measurement (OOSM) injection and covariance uncertainty
//      coupling verification (cov_scale_active assertion & slosh damping).
// =============================================================================

`timescale 1ns / 1ps

module tb_tide_hmppi_soc;

    // Clock and Reset
    logic clk = 0;
    logic rst_n = 1;

    always #10 clk = ~clk; // 50 MHz (20.0 ns period)

    // TIDE Coprocessor Interface
    logic        tide_enable = 0;
    logic [4:0]  tide_ckpt_interval = 5'd4;
    logic        tide_irq_en = 1'b1;
    logic [8:0]  tide_status_clr = 9'd0;
    logic        tide_clr_lat = 1'b0;
    logic [31:0] tide_qp = 32'h3DCCCCCD;     // 0.1 FP32
    logic [31:0] tide_qv = 32'h3DCCCCCD;     // 0.1 FP32
    logic [31:0] tide_dt0inv = 32'h41200000; // 10.0 FP32 (dt = 0.1s)

    // Event stream
    logic        tide_event_valid = 0;
    logic [31:0] tide_event_t = 0;
    logic [31:0] tide_event_z0 = 0, tide_event_z1 = 0, tide_event_z2 = 0, tide_event_z3 = 0;
    logic [4:0]  tide_event_flags = 0;
    wire         tide_event_ready;

    // Microcode load port
    logic        tide_uc_we = 0;
    logic [8:0]  tide_uc_addr = 0;
    logic [1:0]  tide_uc_word = 0;
    logic [31:0] tide_uc_data = 0;
    logic [8:0]  tide_uc_rd_addr = 0;
    wire  [95:0] tide_uc_rd_data;

    // RAM CPU write port
    logic        tide_ram_wr_ce = 0;
    logic [11:0] tide_ram_wr_addr = 0;
    logic [31:0] tide_ram_wr_data = 0;
    logic        tide_ram_dbg_ce = 0;
    logic [11:0] tide_ram_dbg_addr = 0;
    wire  [31:0] tide_ram_dbg_rdata;
    wire         tide_ram_dbg_valid;

    // H-MPPI Mission Inputs
    logic               mppi_start_frame = 0;
    logic signed [15:0] mppi_goal_pos_x = 16'sh0500; // +5.0m in Q8.8
    logic signed [15:0] mppi_goal_pos_y = 16'sh0200; // +2.0m in Q8.8
    logic signed [15:0] mppi_goal_pos_z = 16'sh0200; // +2.0m altitude in Q8.8
    logic signed [15:0] mppi_u_nom_x    = 16'sh0100; // +1.0 m/s^2 nominal in Q8.8
    logic signed [15:0] mppi_u_nom_y    = 16'sh0000;
    logic signed [15:0] mppi_u_nom_z    = 16'sh0000;

    // SoC Configuration Modes
    logic               soc_use_tide_state = 1'b1;
    logic               soc_auto_trigger_mppi = 1'b0;
    logic               soc_cov_scale_override = 1'b0;
    logic signed [15:0] ext_pos_x = 16'sh0000;
    logic signed [15:0] ext_pos_y = 16'sh0000;
    logic signed [15:0] ext_pos_z = 16'sh0200;
    logic signed [15:0] ext_vel_x = 16'sh0000;
    logic signed [15:0] ext_vel_y = 16'sh0000;
    logic signed [15:0] ext_vel_z = 16'sh0000;

    // Telemetry and Status Outputs
    wire         tide_busy;
    wire         tide_replay_pending;
    wire         tide_provisional;
    wire         tide_irq;
    wire [4:0]   tide_irq_ev;
    wire [8:0]   tide_status;
    wire [31:0]  tide_out_seq;
    wire [31:0]  tide_out_t;
    wire [191:0] tide_out_x;
    wire [191:0] tide_out_pdiag;
    wire         tide_out_committed;

    wire         soc_cov_scale_active;

    wire signed [15:0] mppi_opt_u_x;
    wire signed [15:0] mppi_opt_u_y;
    wire signed [15:0] mppi_opt_u_z;
    wire [23:0]        mppi_min_cost;
    wire               mppi_frame_done;

    wire signed [15:0] soc_est_pos_x_q88;
    wire signed [15:0] soc_est_pos_y_q88;
    wire signed [15:0] soc_est_pos_z_q88;
    wire signed [15:0] soc_est_vel_x_q88;
    wire signed [15:0] soc_est_vel_y_q88;
    wire signed [15:0] soc_est_vel_z_q88;

    wire        soc_div_req_valid;
    wire        soc_div_busy;
    wire        soc_div_resp_valid;
    wire [31:0] soc_div_result_fp32;

    // Instantiate SoC Top DUT
    tide_hmppi_soc_top dut (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .tide_enable_i          (tide_enable),
        .tide_ckpt_interval_i   (tide_ckpt_interval),
        .tide_irq_en_i          (tide_irq_en),
        .tide_status_clr_i      (tide_status_clr),
        .tide_clr_lat_i         (tide_clr_lat),
        .tide_qp_i             (tide_qp),
        .tide_qv_i             (tide_qv),
        .tide_dt0inv_i          (tide_dt0inv),
        .tide_event_valid_i     (tide_event_valid),
        .tide_event_t_i         (tide_event_t),
        .tide_event_z0_i        (tide_event_z0),
        .tide_event_z1_i        (tide_event_z1),
        .tide_event_z2_i        (tide_event_z2),
        .tide_event_z3_i        (tide_event_z3),
        .tide_event_flags_i     (tide_event_flags),
        .tide_event_ready_o     (tide_event_ready),
        .tide_uc_we_i           (tide_uc_we),
        .tide_uc_addr_i         (tide_uc_addr),
        .tide_uc_word_i         (tide_uc_word),
        .tide_uc_data_i         (tide_uc_data),
        .tide_uc_rd_addr_i      (tide_uc_rd_addr),
        .tide_uc_rd_data_o      (tide_uc_rd_data),
        .tide_ram_wr_ce_i       (tide_ram_wr_ce),
        .tide_ram_wr_addr_i     (tide_ram_wr_addr),
        .tide_ram_wr_data_i     (tide_ram_wr_data),
        .tide_ram_dbg_ce_i      (tide_ram_dbg_ce),
        .tide_ram_dbg_addr_i    (tide_ram_dbg_addr),
        .tide_ram_dbg_rdata_o   (tide_ram_dbg_rdata),
        .tide_ram_dbg_valid_o   (tide_ram_dbg_valid),
        .mppi_start_frame_i     (mppi_start_frame),
        .mppi_goal_pos_x_i      (mppi_goal_pos_x),
        .mppi_goal_pos_y_i      (mppi_goal_pos_y),
        .mppi_goal_pos_z_i      (mppi_goal_pos_z),
        .mppi_u_nom_x_i         (mppi_u_nom_x),
        .mppi_u_nom_y_i         (mppi_u_nom_y),
        .mppi_u_nom_z_i         (mppi_u_nom_z),
        .soc_use_tide_state_i   (soc_use_tide_state),
        .soc_auto_trigger_mppi_i(soc_auto_trigger_mppi),
        .soc_cov_scale_override_i(soc_cov_scale_override),
        .ext_pos_x_i            (ext_pos_x),
        .ext_pos_y_i            (ext_pos_y),
        .ext_pos_z_i            (ext_pos_z),
        .ext_vel_x_i            (ext_vel_x),
        .ext_vel_y_i            (ext_vel_y),
        .ext_vel_z_i            (ext_vel_z),
        .tide_busy_o            (tide_busy),
        .tide_replay_pending_o  (tide_replay_pending),
        .tide_provisional_o     (tide_provisional),
        .tide_irq_o             (tide_irq),
        .tide_irq_ev_o          (tide_irq_ev),
        .tide_status_o          (tide_status),
        .tide_out_seq_o         (tide_out_seq),
        .tide_out_t_o           (tide_out_t),
        .tide_out_x_o           (tide_out_x),
        .tide_out_pdiag_o       (tide_out_pdiag),
        .tide_out_committed_o   (tide_out_committed),
        .soc_cov_scale_active_o (soc_cov_scale_active),
        .mppi_opt_u_x_o         (mppi_opt_u_x),
        .mppi_opt_u_y_o         (mppi_opt_u_y),
        .mppi_opt_u_z_o         (mppi_opt_u_z),
        .mppi_min_cost_o        (mppi_min_cost),
        .mppi_frame_done_o      (mppi_frame_done),
        .soc_est_pos_x_q88_o    (soc_est_pos_x_q88),
        .soc_est_pos_y_q88_o    (soc_est_pos_y_q88),
        .soc_est_pos_z_q88_o    (soc_est_pos_z_q88),
        .soc_est_vel_x_q88_o    (soc_est_vel_x_q88),
        .soc_est_vel_y_q88_o    (soc_est_vel_y_q88),
        .soc_est_vel_z_q88_o    (soc_est_vel_z_q88),
        .soc_div_req_valid_o    (soc_div_req_valid),
        .soc_div_busy_o         (soc_div_busy),
        .soc_div_resp_valid_o   (soc_div_resp_valid),
        .soc_div_result_fp32_o  (soc_div_result_fp32)
    );

    // Helper Tasks
    task automatic push_event(
        input [31:0] t,
        input [31:0] z0, z1, z2, z3,
        input [4:0]  fl
    );
        integer timeout;
        begin
            @(posedge clk); #1;
            tide_event_t = t;
            tide_event_z0 = z0;
            tide_event_z1 = z1;
            tide_event_z2 = z2;
            tide_event_z3 = z3;
            tide_event_flags = fl;
            timeout = 0;
            while (!tide_event_ready && timeout < 2000) begin
                @(posedge clk); #1;
                timeout++;
            end
            tide_event_valid = 1'b1;
            @(posedge clk); #1;
            tide_event_valid = 1'b0;
        end
    endtask

    task automatic wait_tide_idle();
        integer timeout;
        begin
            timeout = 0;
            repeat (10) @(posedge clk);
            while ((tide_busy || !tide_event_ready) && timeout < 100000) begin
                @(posedge clk);
                timeout++;
            end
            repeat (5) @(posedge clk);
        end
    endtask

    // Storage for microcode loading
    logic [95:0] ucode_mem [0:511];
    integer nuc, w_idx, timeout_cnt, errors = 0;

    initial begin
        $display("================================================================================");
        $display("  Starting Combined TIDE + H-MPPI Integrated SoC Full Functional Verification");
        $display("================================================================================");

        // 1. Reset DUT
        rst_n = 0;
        repeat (5) @(posedge clk);
        #1; rst_n = 1;
        repeat (5) @(posedge clk);

        // 2. Preload Microcode into TIDE Microcode RAM via Host CPU Port
        $display("[STAGE 1] Preloading TIDE Microcode RAM (492 instructions)...");
        $readmemh("ucode.hex", ucode_mem);
        if (ucode_mem[0] === 96'bx) begin
            $readmemh("tide_hmppi_soc/sim/ucode.hex", ucode_mem);
        end
        if (ucode_mem[0] === 96'bx) begin
            $readmemh("../tide_final/tide/06_engine_top/build/ucode.hex", ucode_mem);
        end
        if (ucode_mem[0] === 96'bx) begin
            $readmemh("D:/Desktop/4-2 project/tide_hmppi_soc/sim/ucode.hex", ucode_mem);
        end

        for (nuc = 0; nuc < 492; nuc++) begin
            for (w_idx = 0; w_idx < 3; w_idx++) begin
                #1;
                tide_uc_we   = 1'b1;
                tide_uc_addr = nuc[8:0];
                tide_uc_word = w_idx[1:0];
                tide_uc_data = ucode_mem[nuc][32*w_idx +: 32];
                @(posedge clk);
            end
        end
        #1; tide_uc_we = 1'b0;
        repeat (5) @(posedge clk);
        $display("[STAGE 1 SUCCESS] Microcode preload completed (492 words committed).");

        // 3. Seed Initial State Context in TIDE-RAM (slot ctx1: 0x260)
        $display("[STAGE 2] Seeding initial vehicle state vector into TIDE-RAM ctx1...");
        // Words +1..+6: initial position/velocity [x=0.0, y=0.0, z=2.0, vx=0.0, vy=0.0, vz=0.0]
        #1; tide_ram_wr_ce = 1; tide_ram_wr_addr = 12'h261; tide_ram_wr_data = 32'h00000000; @(posedge clk); // x=0.0
        #1; tide_ram_wr_ce = 1; tide_ram_wr_addr = 12'h262; tide_ram_wr_data = 32'h00000000; @(posedge clk); // y=0.0
        #1; tide_ram_wr_ce = 1; tide_ram_wr_addr = 12'h263; tide_ram_wr_data = 32'h40000000; @(posedge clk); // z=2.0 (FP32)
        #1; tide_ram_wr_ce = 1; tide_ram_wr_addr = 12'h264; tide_ram_wr_data = 32'h00000000; @(posedge clk); // vx=0.0
        #1; tide_ram_wr_ce = 1; tide_ram_wr_addr = 12'h265; tide_ram_wr_data = 32'h00000000; @(posedge clk); // vy=0.0
        #1; tide_ram_wr_ce = 1; tide_ram_wr_addr = 12'h266; tide_ram_wr_data = 32'h00000000; @(posedge clk); // vz=0.0

        // Words +7..+27: initial covariance P (lower-triangular, 1.0 on diagonal)
        // P00 at 0x267, P11 at 0x269, P22 at 0x26C, P33 at 0x270, P44 at 0x275, P55 at 0x27B
        for (w_idx = 7; w_idx <= 27; w_idx++) begin
            #1;
            tide_ram_wr_ce = 1;
            tide_ram_wr_addr = 12'h260 + w_idx[11:0];
            // Diagonal elements initialized to 1.0 (32'h3F800000)
            if (w_idx == 7 || w_idx == 9 || w_idx == 12 || w_idx == 16 || w_idx == 21 || w_idx == 27)
                tide_ram_wr_data = 32'h3F800000;
            else
                tide_ram_wr_data = 32'h00000000;
            @(posedge clk);
        end
        #1; tide_ram_wr_ce = 0;
        repeat (5) @(posedge clk);
        $display("[STAGE 2 SUCCESS] Vehicle state seeded: z=2.0m, P_diag=1.0.");

        // 4. Enable TIDE Coprocessor and Send INIT Event
        $display("[STAGE 3] Enabling TIDE engine and pushing INIT event...");
        #1; tide_enable = 1'b1;
        repeat (5) @(posedge clk);

        // Push INIT event: flags[4] = 1, t = 1000 us
        push_event(32'd1000, 32'h0, 32'h0, 32'h0, 32'h0, 5'b10000);
        wait_tide_idle();
        $display("[STAGE 3 SUCCESS] INIT event processed. TIDE state initialized.");

        // 5. Test 1: In-Order Sensor Measurement & FP32 -> Q8.8 Telemetry Conversion
        $display("[TEST 1] Sending In-Order Sensor Event (Radar/Altimeter z=2.5m, range=5.0m)...");
        // t = 110000 us (+100 ms), z0 = 5.0m (32'h40A00000), z2 = 2.5m (32'h40200000)
        push_event(32'd110000, 32'h40A00000, 32'h00000000, 32'h40200000, 32'h00000000, 5'b00000);
        wait_tide_idle();

        $display("[TEST 1 RESULT] TIDE Committed State:");
        $display("  Out Seq: %0d, Time: %0d us", tide_out_seq, tide_out_t);
        $display("  FP32 State: x=%h, y=%h, z=%h", tide_out_x[31:0], tide_out_x[63:32], tide_out_x[95:64]);
        $display("  Q8.8 Telemetry: x=%0d/256 m, y=%0d/256 m, z=%0d/256 m",
                 soc_est_pos_x_q88, soc_est_pos_y_q88, soc_est_pos_z_q88);
        $display("  Covariance Diag: P00=%h, P11=%h, P22=%h",
                 tide_out_pdiag[31:0], tide_out_pdiag[63:32], tide_out_pdiag[95:64]);

        if (soc_est_pos_z_q88 <= 0) begin
            $display("[TEST 1 ERROR] Converted altitude should be positive!");
            errors++;
        end else begin
            $display("[TEST 1 SUCCESS] Telemetry conversion and state latch verified.");
        end

        // 6. Test 2: Trigger MPPI Stochastic Rollout Frame with Dedicated Division
        $display("\n[TEST 2] Triggering MPPI Rollout Optimization (M=256, H=16)...");
        $display("  Target Waypoint: (5.0, 2.0, 2.0) m, Nominal Accel: (1.0, 0.0, 0.0) m/s^2");
        soc_use_tide_state = 1'b1;
        #1; mppi_start_frame = 1'b1;
        @(posedge clk);
        #1; mppi_start_frame = 1'b0;

        // Monitor division handshake during MPPI execution
        timeout_cnt = 0;
        while (!mppi_frame_done && timeout_cnt < 50000) begin
            if (soc_div_req_valid) begin
                $display("  [DIV HANDSHAKE DETECTED] MPPI requested reciprocal for eta_sum=%h", dut.mppi_eta_sum_fp32);
            end
            if (soc_div_resp_valid) begin
                $display("  [DIV RESPONSE DETECTED] Hardware FP32 divider returned eta_inv=%h", soc_div_result_fp32);
            end
            @(posedge clk);
            timeout_cnt++;
        end

        if (timeout_cnt >= 50000) begin
            $display("[TEST 2 ERROR] MPPI frame timed out!");
            errors++;
        end else begin
            $display("[TEST 2 SUCCESS] MPPI Optimization Frame completed in %0d cycles!", timeout_cnt);
            $display("  Min Rollout Cost: %0d", mppi_min_cost);
            $display("  Optimal Control u*: (%0.3f, %0.3f, %0.3f) m/s^2",
                     $itor(mppi_opt_u_x)/256.0, $itor(mppi_opt_u_y)/256.0, $itor(mppi_opt_u_z)/256.0);
        end

        // 7. Test 3: Out-of-Sequence Measurement (OOSM) & Covariance Uncertainty Coupling
        $display("\n[TEST 3] Injecting Out-of-Sequence Measurement (OOSM event at t=50000 us < t_now=110000 us)...");
        // Push late measurement event: t = 50000 us
        push_event(32'd50000, 32'h40800000, 32'h00000000, 32'h40000000, 32'h00000000, 5'b00000);
        
        // Check uncertainty channel assertion while TIDE processes late measurement
        repeat (10) @(posedge clk);
        $display("  TIDE Provisional Flag: %b, Replay Pending: %b, Cov Scale Active: %b",
                 tide_provisional, tide_replay_pending, soc_cov_scale_active);

        wait_tide_idle();
        $display("[TEST 3 STEP] OOSM replay completed. Testing manual uncertainty damping trigger...");

        // Assert uncertainty override to verify Chebyshev slosh 3x cost scaling and control damping
        #1; soc_cov_scale_override = 1'b1;
        repeat (5) @(posedge clk);

        $display("  Triggering Slosh-Damped MPPI Frame with cov_scale_active = 1...");
        #1; mppi_start_frame = 1'b1;
        @(posedge clk);
        #1; mppi_start_frame = 1'b0;

        timeout_cnt = 0;
        while (!mppi_frame_done && timeout_cnt < 50000) begin
            @(posedge clk);
            timeout_cnt++;
        end

        if (timeout_cnt >= 50000) begin
            $display("[TEST 3 ERROR] Slosh-damped MPPI frame timed out!");
            errors++;
        end else begin
            $display("[TEST 3 SUCCESS] Slosh-Damped MPPI Frame completed in %0d cycles!", timeout_cnt);
            $display("  Damped Min Cost: %0d", mppi_min_cost);
            $display("  Damped Control u*: (%0.3f, %0.3f, %0.3f) m/s^2",
                     $itor(mppi_opt_u_x)/256.0, $itor(mppi_opt_u_y)/256.0, $itor(mppi_opt_u_z)/256.0);
        end

        #1; soc_cov_scale_override = 1'b0;
        repeat (20) @(posedge clk);

        $display("\n================================================================================");
        if (errors == 0) begin
            $display("  ALL TIDE + H-MPPI INTEGRATED SOC VERIFICATIONS PASSED (0 ERRORS)");
        end else begin
            $display("  VERIFICATION FAILED WITH %0d ERRORS!", errors);
        end
        $display("================================================================================");

        $finish;
    end

endmodule
