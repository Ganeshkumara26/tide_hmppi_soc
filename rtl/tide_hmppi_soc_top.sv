// =============================================================================
// Project: Combined TIDE + H-MPPI Integrated System-on-Chip (SoC)
// Module:  tide_hmppi_soc_top.sv
// Description:
//   Monolithic Top-Level SoC uniting:
//   1. Full TIDE EKF Out-of-Sequence Measurement (OOSM) Coprocessor:
//      - Temporal controller, VLIW sequencer, 128x32 register file, LSU,
//        4KB TIDE-RAM, 512x96 microcode RAM, 64x32 constant ROM,
//        pipelined IEEE-754 FP32 lanes (MUL, ADD/bypass, DIV/SQRT, IntALU).
//   2. Full 4-Lane Pipelined H-MPPI Stochastic Guidance Accelerator:
//      - 4 parallel rollout lanes (M=256 rollouts, H=16 horizon),
//        4-dimensional Gray-code Sobol QMC generators, degree-3 Chebyshev
//        polynomial fluid slosh dynamics evaluator, 1-cycle FExp2 unit.
//   3. Hardware Co-Design Interconnect Bridges:
//      - FP32-to-Q8.8 Fixed-Point State Telemetry Bridge with saturation.
//      - Shared Covariance Uncertainty Channel (cov_scale / cov_scale_in)
//        coupling TIDE provisional/replay/high-variance flags to MPPI slosh
//        damping penalty (3x cost scaling).
//      - Dedicated Hardware FP32 Restoring Divider for MPPI soft-min
//        normalization weight reciprocal calculation (eta^-1 = 1 / eta).
//      - Auto-synchronization trigger coordinating EKF state commits with
//        trajectory optimization rollout frames.
// =============================================================================

`timescale 1ns / 1ps

module tide_hmppi_soc_top #(
    parameter integer TIDE_CKPT_DEPTH = 8,
    parameter integer TIDE_JRNL_DEPTH = 64,
    parameter integer MPPI_DATA_WIDTH = 16,  // Signed Q8.8 fixed-point
    parameter integer MPPI_COST_WIDTH = 24,  // Unsigned Q16.8
    parameter integer MPPI_NUM_LANES  = 4,
    parameter integer MPPI_TOTAL_TRAJ = 256,
    parameter integer MPPI_HORIZON    = 16
)(
    input  logic                             clk,
    input  logic                             rst_n,

    // =========================================================================
    // TIDE Coprocessor Control & Configuration Port
    // =========================================================================
    input  logic                             tide_enable_i,
    input  logic [4:0]                       tide_ckpt_interval_i,
    input  logic                             tide_irq_en_i,
    input  logic [8:0]                       tide_status_clr_i,
    input  logic                             tide_clr_lat_i,
    input  logic [31:0]                      tide_qp_i,
    input  logic [31:0]                      tide_qv_i,
    input  logic [31:0]                      tide_dt0inv_i,

    // TIDE Measurement Event Ingestion Stream
    input  logic                             tide_event_valid_i,
    input  logic [31:0]                      tide_event_t_i,
    input  logic [31:0]                      tide_event_z0_i,
    input  logic [31:0]                      tide_event_z1_i,
    input  logic [31:0]                      tide_event_z2_i,
    input  logic [31:0]                      tide_event_z3_i,
    input  logic [4:0]                       tide_event_flags_i,
    output logic                             tide_event_ready_o,

    // TIDE Microcode Load Port (3x 32-bit writes per 96-bit word)
    input  logic                             tide_uc_we_i,
    input  logic [8:0]                       tide_uc_addr_i,
    input  logic [1:0]                       tide_uc_word_i,
    input  logic [31:0]                      tide_uc_data_i,
    input  logic [8:0]                       tide_uc_rd_addr_i,
    output logic [95:0]                      tide_uc_rd_data_o,

    // TIDE Host/CPU RAM Port (State Seeding & Debug Read)
    input  logic                             tide_ram_wr_ce_i,
    input  logic [11:0]                      tide_ram_wr_addr_i,
    input  logic [31:0]                      tide_ram_wr_data_i,
    input  logic                             tide_ram_dbg_ce_i,
    input  logic [11:0]                      tide_ram_dbg_addr_i,
    output logic [31:0]                      tide_ram_dbg_rdata_o,
    output logic                             tide_ram_dbg_valid_o,

    // =========================================================================
    // H-MPPI Stochastic Guidance Mission & Waypoint Port
    // =========================================================================
    input  logic                             mppi_start_frame_i,
    input  logic [MPPI_DATA_WIDTH-1:0]       mppi_goal_pos_x_i,
    input  logic [MPPI_DATA_WIDTH-1:0]       mppi_goal_pos_y_i,
    input  logic [MPPI_DATA_WIDTH-1:0]       mppi_goal_pos_z_i,
    input  logic [MPPI_DATA_WIDTH-1:0]       mppi_u_nom_x_i,
    input  logic [MPPI_DATA_WIDTH-1:0]       mppi_u_nom_y_i,
    input  logic [MPPI_DATA_WIDTH-1:0]       mppi_u_nom_z_i,

    // =========================================================================
    // SoC Interconnect & Mode Configuration
    // =========================================================================
    input  logic                             soc_use_tide_state_i,     // 1: Auto-use TIDE estimated state; 0: Use external inputs
    input  logic                             soc_auto_trigger_mppi_i,  // 1: Trigger MPPI rollout frame upon TIDE state commit
    input  logic                             soc_cov_scale_override_i, // 1: Force covariance uncertainty scale active
    input  logic [MPPI_DATA_WIDTH-1:0]       ext_pos_x_i,             // External manual override inputs
    input  logic [MPPI_DATA_WIDTH-1:0]       ext_pos_y_i,
    input  logic [MPPI_DATA_WIDTH-1:0]       ext_pos_z_i,
    input  logic [MPPI_DATA_WIDTH-1:0]       ext_vel_x_i,
    input  logic [MPPI_DATA_WIDTH-1:0]       ext_vel_y_i,
    input  logic [MPPI_DATA_WIDTH-1:0]       ext_vel_z_i,

    // =========================================================================
    // SoC Observability & Status Outputs
    // =========================================================================
    // TIDE Status & Telemetry
    output logic                             tide_busy_o,
    output logic                             tide_replay_pending_o,
    output logic                             tide_provisional_o,
    output logic                             tide_irq_o,
    output logic [4:0]                       tide_irq_ev_o,
    output logic [8:0]                       tide_status_o,
    output logic [31:0]                      tide_out_seq_o,
    output logic [31:0]                      tide_out_t_o,
    output logic [191:0]                     tide_out_x_o,
    output logic [191:0]                     tide_out_pdiag_o,
    output logic                             tide_out_committed_o,

    // Shared Covariance Uncertainty Channel Status
    output logic                             soc_cov_scale_active_o,

    // H-MPPI Synthesized Guidance Accelerations & Metric Outputs
    output logic [MPPI_DATA_WIDTH-1:0]       mppi_opt_u_x_o,
    output logic [MPPI_DATA_WIDTH-1:0]       mppi_opt_u_y_o,
    output logic [MPPI_DATA_WIDTH-1:0]       mppi_opt_u_z_o,
    output logic [MPPI_COST_WIDTH-1:0]       mppi_min_cost_o,
    output logic                             mppi_frame_done_o,

    // Converted Telemetry Outputs (FP32 -> Q8.8)
    output logic [MPPI_DATA_WIDTH-1:0]       soc_est_pos_x_q88_o,
    output logic [MPPI_DATA_WIDTH-1:0]       soc_est_pos_y_q88_o,
    output logic [MPPI_DATA_WIDTH-1:0]       soc_est_pos_z_q88_o,
    output logic [MPPI_DATA_WIDTH-1:0]       soc_est_vel_x_q88_o,
    output logic [MPPI_DATA_WIDTH-1:0]       soc_est_vel_y_q88_o,
    output logic [MPPI_DATA_WIDTH-1:0]       soc_est_vel_z_q88_o,

    // Normalization Division Channel Telemetry
    output logic                             soc_div_req_valid_o,
    output logic                             soc_div_busy_o,
    output logic                             soc_div_resp_valid_o,
    output logic [31:0]                      soc_div_result_fp32_o
);

    // =========================================================================
    // 1. TIDE Sensor Measurement R-Set Table
    // =========================================================================
    logic [1:0]  tide_rset_sel;
    logic [31:0] tide_rr0, tide_rr1, tide_rr2, tide_rr3;

    always_comb begin
        case (tide_rset_sel)
            2'd0: begin
                tide_rr0 = 32'h41c80000; // 25.0
                tide_rr1 = 32'h38d1b717; // 0.0001
                tide_rr2 = 32'h38d1b717;
                tide_rr3 = 32'h3f800000; // 1.0
            end
            2'd1: begin
                tide_rr0 = 32'h42480000; // 50.0
                tide_rr1 = 32'h3951b717;
                tide_rr2 = 32'h3951b717;
                tide_rr3 = 32'h40000000; // 2.0
            end
            2'd2: begin
                tide_rr0 = 32'h42960000; // 75.0
                tide_rr1 = 32'h399d4952;
                tide_rr2 = 32'h399d4952;
                tide_rr3 = 32'h40400000; // 3.0
            end
            default: begin
                tide_rr0 = 32'h42c80000; // 100.0
                tide_rr1 = 32'h39d1b717;
                tide_rr2 = 32'h39d1b717;
                tide_rr3 = 32'h40800000; // 4.0
            end
        endcase
    end

    // =========================================================================
    // 2. TIDE EKF Coprocessor Engine Instantiation
    // =========================================================================
    logic [191:0] tide_out_x_raw;
    logic [191:0] tide_out_pdiag_raw;
    logic         tide_committed_pulse;
    logic         tide_busy_int;
    logic         tide_prov_int;
    logic         tide_rpl_pend_int;

    tide_engine #(
        .UNIT_ENTRY(9'd0),
        .QUERY_ENTRY(9'd471),
        .CKPT_DEPTH(TIDE_CKPT_DEPTH),
        .JRNL_DEPTH(TIDE_JRNL_DEPTH)
    ) u_tide_coprocessor (
        .clk_i               (clk),
        .rst_ni              (rst_n),
        .ckpt_interval_i     (tide_ckpt_interval_i),
        .enable_i            (tide_enable_i),
        .irq_en_i            (tide_irq_en_i),
        .status_clr_i        (tide_status_clr_i),
        .clr_lat_i           (tide_clr_lat_i),
        .event_valid_i       (tide_event_valid_i),
        .event_t_i           (tide_event_t_i),
        .event_z0_i          (tide_event_z0_i),
        .event_z1_i          (tide_event_z1_i),
        .event_z2_i          (tide_event_z2_i),
        .event_z3_i          (tide_event_z3_i),
        .event_flags_i       (tide_event_flags_i),
        .event_ready_o       (tide_event_ready_o),
        .cur_flags_o         (),
        .rset_sel_o          (tide_rset_sel),
        .rset_r0_i           (tide_rr0),
        .rset_r1_i           (tide_rr1),
        .rset_r2_i           (tide_rr2),
        .rset_r3_i           (tide_rr3),
        .qp_i                (tide_qp_i),
        .qv_i                (tide_qv_i),
        .dt0inv_i            (tide_dt0inv_i),
        .uc_we_i             (tide_uc_we_i),
        .uc_addr_i           (tide_uc_addr_i),
        .uc_word_i           (tide_uc_word_i),
        .uc_data_i           (tide_uc_data_i),
        .uc_rd_addr_i        (tide_uc_rd_addr_i),
        .uc_rd_data_o        (tide_uc_rd_data_o),
        .ram_wr_ce_i         (tide_ram_wr_ce_i),
        .ram_wr_addr_i       (tide_ram_wr_addr_i),
        .ram_wr_data_i       (tide_ram_wr_data_i),
        .ram_dbg_ce_i        (tide_ram_dbg_ce_i),
        .ram_dbg_addr_i      (tide_ram_dbg_addr_i),
        .ram_dbg_rdata_o     (tide_ram_dbg_rdata_o),
        .ram_dbg_valid_o     (tide_ram_dbg_valid_o),
        .busy_o              (tide_busy_int),
        .replay_pending_o    (tide_rpl_pend_int),
        .provisional_o       (tide_prov_int),
        .irq_o               (tide_irq_o),
        .irq_ev_o            (tide_irq_ev_o),
        .status_o            (tide_status_o),
        .gpio_engine_busy_o  (),
        .gpio_late_drop_o    (),
        .gpio_provisional_o  (),
        .cnt_units_o         (),
        .cnt_late_o          (),
        .cnt_drop_o          (),
        .cnt_replay_o        (),
        .cnt_abort_o         (),
        .cnt_query_o         (),
        .cyc_last_unit_o     (),
        .cyc_last_replay_o   (),
        .lat_last_wait_o     (),
        .lat_max_wait_o      (),
        .out_seq_o           (tide_out_seq_o),
        .out_t_o             (tide_out_t_o),
        .out_x_o             (tide_out_x_raw),
        .out_pdiag_o         (tide_out_pdiag_raw),
        .out_committed_o     (tide_committed_pulse),
        .qout_x_o            (),
        .qout_t_o            (),
        .ram_conflict_o      (),
        .ram_oor_o           (),
        .rf_conflict_o       ()
    );

    assign tide_busy_o           = tide_busy_int;
    assign tide_replay_pending_o = tide_rpl_pend_int;
    assign tide_provisional_o    = tide_prov_int;
    assign tide_out_x_o          = tide_out_x_raw;
    assign tide_out_pdiag_o      = tide_out_pdiag_raw;
    assign tide_out_committed_o  = tide_committed_pulse;

    // =========================================================================
    // 3. FP32 to Q8.8 Fixed-Point Telemetry Conversion Function
    // =========================================================================
    function automatic logic signed [15:0] fp32_to_q88(input logic [31:0] fp);
        logic        s;
        logic [7:0]  e;
        logic [22:0] m;
        integer      shift;
        logic [23:0] mant;
        logic [31:0] shifted;
        logic [15:0] mag;
        begin
            s = fp[31];
            e = fp[30:23];
            m = fp[22:0];
            mant = {1'b1, m};

            if (e == 8'd0) begin
                // Zero or subnormal
                fp32_to_q88 = 16'sh0000;
            end else if (e >= 8'd134) begin // 134 - 127 = 7 => 2^7 = 128 (overflow for Q8.8 signed)
                // Saturation to max / min representable in 16-bit Q8.8
                fp32_to_q88 = s ? 16'sh8000 : 16'sh7FFF;
            end else if (e < 8'd119) begin  // 119 - 127 = -8 => 2^-8 = 1/256 (underflow for Q8.8)
                // Underflow to zero
                fp32_to_q88 = 16'sh0000;
            end else begin
                // Shift amount: S = 142 - exp, ranges from 8 to 23
                shift = 142 - e;
                // Add rounding bit before shifting right
                shifted = ({{8'd0, mant} + (32'd1 << (shift - 1))}) >> shift;
                mag = shifted[15:0];
                if (mag > 16'h7FFF) begin
                    fp32_to_q88 = s ? 16'sh8000 : 16'sh7FFF;
                end else begin
                    fp32_to_q88 = s ? (-$signed(mag)) : $signed(mag);
                end
            end
        end
    endfunction

    // Extract FP32 components: {x, y, z, vx, vy, vz}
    logic [31:0] fp_x, fp_y, fp_z, fp_vx, fp_vy, fp_vz;
    assign fp_x  = tide_out_x_raw[31:0];
    assign fp_y  = tide_out_x_raw[63:32];
    assign fp_z  = tide_out_x_raw[95:64];
    assign fp_vx = tide_out_x_raw[127:96];
    assign fp_vy = tide_out_x_raw[159:128];
    assign fp_vz = tide_out_x_raw[191:160];

    // Converted combinational wires
    logic signed [15:0] conv_pos_x, conv_pos_y, conv_pos_z;
    logic signed [15:0] conv_vel_x, conv_vel_y, conv_vel_z;
    assign conv_pos_x = fp32_to_q88(fp_x);
    assign conv_pos_y = fp32_to_q88(fp_y);
    assign conv_pos_z = fp32_to_q88(fp_z);
    assign conv_vel_x = fp32_to_q88(fp_vx);
    assign conv_vel_y = fp32_to_q88(fp_vy);
    assign conv_vel_z = fp32_to_q88(fp_vz);

    // Synchronous Registered State Snapshot (latched when TIDE commits state update)
    logic signed [15:0] reg_pos_x, reg_pos_y, reg_pos_z;
    logic signed [15:0] reg_vel_x, reg_vel_y, reg_vel_z;
    logic               tide_committed_d1;
    logic               tide_committed_edge;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tide_committed_d1 <= 1'b0;
            reg_pos_x <= 16'sh0000;
            reg_pos_y <= 16'sh0000;
            reg_pos_z <= 16'sh0200; // Default 2.0m takeoff height
            reg_vel_x <= 16'sh0000;
            reg_vel_y <= 16'sh0000;
            reg_vel_z <= 16'sh0000;
        end else begin
            tide_committed_d1 <= tide_committed_pulse;
            if (tide_committed_pulse && !tide_committed_d1) begin
                reg_pos_x <= conv_pos_x;
                reg_pos_y <= conv_pos_y;
                reg_pos_z <= conv_pos_z;
                reg_vel_x <= conv_vel_x;
                reg_vel_y <= conv_vel_y;
                reg_vel_z <= conv_vel_z;
            end
        end
    end
    assign tide_committed_edge = tide_committed_pulse && !tide_committed_d1;

    // Expose converted state telemetry
    assign soc_est_pos_x_q88_o = reg_pos_x;
    assign soc_est_pos_y_q88_o = reg_pos_y;
    assign soc_est_pos_z_q88_o = reg_pos_z;
    assign soc_est_vel_x_q88_o = reg_vel_x;
    assign soc_est_vel_y_q88_o = reg_vel_y;
    assign soc_est_vel_z_q88_o = reg_vel_z;

    // Active state multiplexer
    logic signed [15:0] mppi_in_pos_x, mppi_in_pos_y, mppi_in_pos_z;
    logic signed [15:0] mppi_in_vel_x, mppi_in_vel_y, mppi_in_vel_z;

    assign mppi_in_pos_x = soc_use_tide_state_i ? reg_pos_x : ext_pos_x_i;
    assign mppi_in_pos_y = soc_use_tide_state_i ? reg_pos_y : ext_pos_y_i;
    assign mppi_in_pos_z = soc_use_tide_state_i ? reg_pos_z : ext_pos_z_i;
    assign mppi_in_vel_x = soc_use_tide_state_i ? reg_vel_x : ext_vel_x_i;
    assign mppi_in_vel_y = soc_use_tide_state_i ? reg_vel_y : ext_vel_y_i;
    assign mppi_in_vel_z = soc_use_tide_state_i ? reg_vel_z : ext_vel_z_i;

    // =========================================================================
    // 4. Shared Covariance Uncertainty Channel
    // =========================================================================
    // Check diagonal covariance elements for elevated uncertainty
    logic [7:0] exp_p00, exp_p11, exp_p22;
    logic       high_variance;

    assign exp_p00 = tide_out_pdiag_raw[30:23];
    assign exp_p11 = tide_out_pdiag_raw[62:55];
    assign exp_p22 = tide_out_pdiag_raw[94:87];

    // High variance condition: any position variance > 4.0 m^2 (exp > 127 + 2 = 129)
    assign high_variance = (exp_p00 > 8'd129) || (exp_p11 > 8'd129) || (exp_p22 > 8'd129);

    // Active uncertainty signal driven to MPPI slosh dampening logic
    logic cov_scale_active;
    assign cov_scale_active = tide_prov_int | tide_rpl_pend_int | high_variance | soc_cov_scale_override_i;
    assign soc_cov_scale_active_o = cov_scale_active;

    // =========================================================================
    // 5. Dedicated Hardware FP32 Restoring Division Channel for MPPI
    // =========================================================================
    logic        mppi_eta_req_valid;
    logic [31:0] mppi_eta_sum_fp32;
    logic        div_busy;
    logic        div_done;
    logic [31:0] div_result;
    logic        div_in_flight_q;
    logic        div_start_pulse;

    assign div_start_pulse = mppi_eta_req_valid && !div_busy && !div_in_flight_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_in_flight_q <= 1'b0;
        end else begin
            if (div_start_pulse) begin
                div_in_flight_q <= 1'b1;
            end else if (div_done || !mppi_eta_req_valid) begin
                div_in_flight_q <= 1'b0;
            end
        end
    end

    tide_fp32_divsqrt u_mppi_div (
        .clk_i   (clk),
        .rst_ni  (rst_n),
        .start_i (div_start_pulse),
        .a_i     (32'h3F800000),       // 1.0 in IEEE-754 FP32 numerator
        .b_i     (mppi_eta_sum_fp32),  // eta denominator from MPPI
        .op_i    (1'b0),               // 0 = Divide
        .busy_o  (div_busy),
        .done_o  (div_done),
        .result_o(div_result),
        .flags_o ()
    );

    // Register division response pulse
    logic        div_resp_valid_q;
    logic [31:0] div_resp_data_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_resp_valid_q <= 1'b0;
            div_resp_data_q  <= 32'h3F800000;
        end else begin
            div_resp_valid_q <= div_done;
            if (div_done) begin
                div_resp_data_q <= div_result;
            end
        end
    end

    assign soc_div_req_valid_o   = mppi_eta_req_valid;
    assign soc_div_busy_o        = div_busy;
    assign soc_div_resp_valid_o  = div_resp_valid_q;
    assign soc_div_result_fp32_o = div_resp_data_q;

    // =========================================================================
    // 6. MPPI Rollout Frame Trigger Coordination
    // =========================================================================
    logic mppi_trigger_combined;
    assign mppi_trigger_combined = mppi_start_frame_i | (soc_auto_trigger_mppi_i & tide_committed_edge);

    // =========================================================================
    // 7. Full 4-Lane H-MPPI Guidance Accelerator Core Instantiation
    // =========================================================================
    hmppi_core_top #(
        .DATA_WIDTH (MPPI_DATA_WIDTH),
        .COST_WIDTH (MPPI_COST_WIDTH),
        .NUM_LANES  (MPPI_NUM_LANES),
        .TOTAL_TRAJ (MPPI_TOTAL_TRAJ),
        .HORIZON    (MPPI_HORIZON)
    ) u_hmppi_accelerator (
        .clk            (clk),
        .rst_n          (rst_n),
        .start_frame    (mppi_trigger_combined),
        .curr_pos_x     (mppi_in_pos_x),
        .curr_pos_y     (mppi_in_pos_y),
        .curr_pos_z     (mppi_in_pos_z),
        .curr_vel_x     (mppi_in_vel_x),
        .curr_vel_y     (mppi_in_vel_y),
        .curr_vel_z     (mppi_in_vel_z),
        .goal_pos_x     (mppi_goal_pos_x_i),
        .goal_pos_y     (mppi_goal_pos_y_i),
        .goal_pos_z     (mppi_goal_pos_z_i),
        .u_nom_x        (mppi_u_nom_x_i),
        .u_nom_y        (mppi_u_nom_y_i),
        .u_nom_z        (mppi_u_nom_z_i),
        .cov_scale_in   (cov_scale_active),
        .eta_req_valid  (mppi_eta_req_valid),
        .eta_sum_fp32   (mppi_eta_sum_fp32),
        .eta_resp_valid (div_resp_valid_q),
        .eta_inv_fp32   (div_resp_data_q),
        .opt_u_x        (mppi_opt_u_x_o),
        .opt_u_y        (mppi_opt_u_y_o),
        .opt_u_z        (mppi_opt_u_z_o),
        .min_cost_out   (mppi_min_cost_o),
        .frame_done     (mppi_frame_done_o)
    );

endmodule

