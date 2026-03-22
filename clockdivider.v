// -----------------------------------------------------------------------------
// Clock utility cells (behavioral models)
// -----------------------------------------------------------------------------
// Replace these with technology library cells (ICG/CLKMUX) in synthesis flows
// that require hardened clock path implementations.
module gf_icg (
    input  wire clk,
    input  wire en,
    input  wire test_en,
    output wire gclk
);
reg en_latch;

always @(clk or en or test_en) begin
    if (clk == 1'b0) begin
        en_latch = en | test_en;
    end
end

assign gclk = clk & en_latch;

endmodule

module gf_clk_mux2 (
    input  wire clk0,
    input  wire clk1,
    input  wire sel,
    input  wire test_en,
    output wire clk_out
);
wire clk0_g;
wire clk1_g;

gf_icg u_icg0 (
    .clk     (clk0),
    .en      (~sel),
    .test_en (test_en),
    .gclk    (clk0_g)
);

gf_icg u_icg1 (
    .clk     (clk1),
    .en      (sel),
    .test_en (test_en),
    .gclk    (clk1_g)
);

assign clk_out = clk0_g | clk1_g;

endmodule

// -----------------------------------------------------------------------------
// Module: clockdivider
// Description:
//   Integer/fractional clock divider with glitch-safe config update.
//   - cken and div_upd are asynchronous single-bit inputs, synchronized to clk
//   - div_int/div_fra are not synchronized internally; div_upd high indicates
//     they are stable and sampleable
//   - div_int/div_fra are sampled only in the output-low safe window
//   - rstn low  -> output clock forced low
//   - rstn release restores divider config to RST_DIV_INT/RST_DIV_FRA
//   - cken low  -> output clock forced low
//   - div_int=0 -> output clock forced low
//   - div_int=1, div_fra=0 -> bypass output clock
//   - div_int=1, div_fra!=0 -> dither between /1 and /2 output periods
//   - div_int>=2, div_fra!=0 -> dither between div_int and div_int+1
//   - cken is applied only in the output-low safe window
// -----------------------------------------------------------------------------
module clockdivider #(
    parameter integer INT_WIDTH = 8,
    parameter integer FRA_WIDTH = 8,
    parameter [INT_WIDTH-1:0] RST_DIV_INT = 4,
    parameter [FRA_WIDTH-1:0] RST_DIV_FRA = 0
) (
    input  wire                  clk,
    input  wire                  rstn,
    input  wire                  cken,
    input  wire                  div_upd,
    input  wire [INT_WIDTH-1:0]  div_int,
    input  wire [FRA_WIDTH-1:0]  div_fra,
    output wire                  oclk
);

localparam [INT_WIDTH-1:0] INT_ZERO = {INT_WIDTH{1'b0}};
localparam [INT_WIDTH-1:0] INT_ONE  = {{(INT_WIDTH-1){1'b0}}, 1'b1};
localparam [FRA_WIDTH-1:0] FRA_ZERO = {FRA_WIDTH{1'b0}};
localparam [INT_WIDTH-1:0] INT_RST  = RST_DIV_INT;
localparam [FRA_WIDTH-1:0] FRA_RST  = RST_DIV_FRA;
localparam integer HALF_FRA_WIDTH   = FRA_WIDTH + 1;

reg                   cken_meta_q;
reg                   cken_sync_q;
reg                   div_upd_meta_q;
reg                   div_upd_sync_q;
reg                   div_upd_busy_q;
reg                   init_cfg_pending_q;

reg                   cken_cfg;
reg [INT_WIDTH-1:0]   div_int_cfg;
reg [FRA_WIDTH-1:0]   div_fra_cfg;
reg [INT_WIDTH-1:0]   div_int_req_q;
reg [FRA_WIDTH-1:0]   div_fra_req_q;
reg                   div_req_pending_q;
reg                   exit_bypass_q;

reg [INT_WIDTH-1:0]   cnt_q;
reg [HALF_FRA_WIDTH-1:0] half_acc_q;
reg                   oclk_div_q;
reg [FRA_WIDTH-1:0]   frac1_acc_q;
reg                   frac1_cycle_en_q;

wire                  cfg_div_zero;
wire                  cfg_bypass;
wire                  cfg_frac1x;
wire                  cfg_raw_clk;
wire                  cfg_low_window_w;
wire                  div_cfg_change_w;
wire [INT_WIDTH-1:0]  half_int_cfg;
wire [HALF_FRA_WIDTH-1:0] half_fra_cfg;
wire [HALF_FRA_WIDTH:0] half_sum;
wire                  half_carry;
wire [INT_WIDTH-1:0]  half_cycles_raw;
wire [INT_WIDTH-1:0]  half_cycles_eff;
wire [INT_WIDTH-1:0]  half_limit;
wire                  div_capture_req_w;
wire                  div_capture_cfg_w;
wire                  div_capture_noop_w;
wire                  bypass_active;
wire                  cken_update_req_w;
wire                  hold_raw_low_w;
wire                  oclk_enable;
wire                  oclk_mux_w;
wire                  oclk_gate_w;
wire [FRA_WIDTH:0]    frac1_sum;

assign cfg_div_zero = (div_int_cfg == INT_ZERO);
assign cfg_bypass   = (div_int_cfg == INT_ONE) && (div_fra_cfg == FRA_ZERO);
assign cfg_frac1x   = (div_int_cfg == INT_ONE) && (div_fra_cfg != FRA_ZERO);
assign cfg_raw_clk  = cfg_bypass || cfg_frac1x;

// div = div_int + div_fra/2^F is full-cycle divider.
// Internal toggle interval uses half-cycle divider = div/2.
assign half_int_cfg    = div_int_cfg >> 1;
assign half_fra_cfg    = {div_int_cfg[0], div_fra_cfg};
assign half_sum        = {1'b0, half_acc_q} + {1'b0, half_fra_cfg};
assign half_carry      = half_sum[HALF_FRA_WIDTH];
assign half_cycles_raw = half_int_cfg + {{(INT_WIDTH-1){1'b0}}, half_carry};
assign half_cycles_eff = (half_cycles_raw == INT_ZERO) ? INT_ONE : half_cycles_raw;
assign half_limit      = half_cycles_eff - INT_ONE;
assign cfg_low_window_w = (!cken_cfg) || cfg_div_zero ||
                          (!bypass_active && (oclk_div_q == 1'b0));

assign div_cfg_change_w = (div_int != div_int_cfg) || (div_fra != div_fra_cfg);
assign div_capture_req_w = div_upd_sync_q && !div_upd_busy_q &&
                           !div_req_pending_q;
assign div_capture_cfg_w = div_capture_req_w && div_cfg_change_w;
assign div_capture_noop_w = div_capture_req_w && !div_cfg_change_w;
assign bypass_active   = cfg_raw_clk && !exit_bypass_q;
assign cken_update_req_w = (cken_cfg != cken_sync_q) &&
                           !init_cfg_pending_q && !div_req_pending_q;
assign hold_raw_low_w  = bypass_active &&
                         (div_capture_cfg_w || div_req_pending_q || cken_update_req_w);
assign frac1_sum       = {1'b0, frac1_acc_q} + {1'b0, div_fra_cfg};
assign oclk_enable     = cken_cfg && !cfg_div_zero &&
                         (!cfg_frac1x || frac1_cycle_en_q);
// Let the internal ICG/mux tree control the output clock path directly.
// Avoid combinationally gating oclk with the asynchronous reset signal.
assign oclk           = oclk_gate_w;

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        cken_meta_q      <= 1'b0;
        cken_sync_q      <= 1'b0;
        div_upd_meta_q   <= 1'b0;
        div_upd_sync_q   <= 1'b0;
        div_upd_busy_q   <= 1'b0;
        init_cfg_pending_q <= 1'b1;
        cken_cfg         <= 1'b0;
        div_int_cfg   <= INT_RST;
        div_fra_cfg   <= FRA_RST;
        div_int_req_q <= INT_RST;
        div_fra_req_q <= FRA_RST;
        div_req_pending_q <= 1'b0;
        exit_bypass_q <= 1'b0;
    end else begin
        cken_meta_q      <= cken;
        cken_sync_q      <= cken_meta_q;
        div_upd_meta_q   <= div_upd;
        div_upd_sync_q   <= div_upd_meta_q;

        if (!div_upd_sync_q) begin
            div_upd_busy_q <= 1'b0;
        end

        if (div_capture_noop_w) begin
            // No-op updates are acknowledged immediately and should not depend
            // on a low-window handoff.
            div_upd_busy_q <= 1'b1;
        end

        if (hold_raw_low_w) begin
            // In raw-clk modes, first carve out a full low window before
            // sampling new coefficients or changing cken.
            exit_bypass_q <= 1'b1;
        end else if (exit_bypass_q && !div_capture_req_w &&
                     !div_req_pending_q && cken_sync_q) begin
            exit_bypass_q <= 1'b0;
        end

        if (cfg_low_window_w) begin
            if (div_req_pending_q) begin
                div_int_cfg       <= div_int_req_q;
                div_fra_cfg       <= div_fra_req_q;
                div_req_pending_q <= 1'b0;
                init_cfg_pending_q <= 1'b0;
            end else if (div_capture_cfg_w) begin
                div_int_req_q     <= div_int;
                div_fra_req_q     <= div_fra;
                div_req_pending_q <= 1'b1;
                div_upd_busy_q    <= 1'b1;
            end else if (init_cfg_pending_q) begin
                // After reset, align the internal divider config with the
                // stable interface values once before allowing output enable.
                div_int_req_q     <= div_int;
                div_fra_req_q     <= div_fra;
                div_req_pending_q <= 1'b1;
            end

            if (cken_update_req_w) begin
                cken_cfg <= cken_sync_q;
            end
        end
    end
end

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        cnt_q       <= INT_ZERO;
        half_acc_q  <= {HALF_FRA_WIDTH{1'b0}};
        oclk_div_q  <= 1'b0;
    end else begin
        if (!cken_cfg || cfg_div_zero || cfg_raw_clk) begin
            cnt_q      <= INT_ZERO;
            half_acc_q <= {HALF_FRA_WIDTH{1'b0}};
            oclk_div_q <= 1'b0;
        end else begin
            if (cnt_q >= half_limit) begin
                cnt_q      <= INT_ZERO;
                half_acc_q <= half_sum[HALF_FRA_WIDTH-1:0];
                oclk_div_q <= ~oclk_div_q;
            end else begin
                cnt_q <= cnt_q + INT_ONE;
            end
        end
    end
end

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        frac1_acc_q      <= FRA_ZERO;
        frac1_cycle_en_q <= 1'b1;
    end else begin
        if (!cken_cfg || cfg_div_zero || !cfg_frac1x || exit_bypass_q) begin
            frac1_acc_q      <= FRA_ZERO;
            frac1_cycle_en_q <= 1'b1;
        end else if (!frac1_cycle_en_q) begin
            // Keep exactly one raw-clk cycle low to realize a /2 period.
            frac1_cycle_en_q <= 1'b1;
        end else begin
            frac1_acc_q      <= frac1_sum[FRA_WIDTH-1:0];
            frac1_cycle_en_q <= !frac1_sum[FRA_WIDTH];
        end
    end
end

// Clock path uses dedicated-style building blocks (ICG + glitch-free mux).
gf_clk_mux2 u_oclk_mux (
    .clk0    (oclk_div_q),
    .clk1    (clk),
    .sel     (bypass_active),
    .test_en (1'b0),
    .clk_out (oclk_mux_w)
);

gf_icg u_oclk_gate (
    .clk     (oclk_mux_w),
    .en      (oclk_enable),
    .test_en (1'b0),
    .gclk    (oclk_gate_w)
);

endmodule
