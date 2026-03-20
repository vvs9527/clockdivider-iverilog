`timescale 1ns/1ps

module tb_clockdivider;

parameter integer INT_WIDTH  = 8;
parameter integer FRA_WIDTH  = 8;
parameter integer CLK_PERIOD = 10;

reg                   clk;
reg                   rstn;
reg                   cken;
reg                   div_upd;
reg  [INT_WIDTH-1:0]  div_int;
reg  [FRA_WIDTH-1:0]  div_fra;
wire                  oclk;

reg                   measure_en;
integer               clk_edge_cnt;
integer               oclk_edge_cnt;
integer               err_cnt;

integer               coeff_idx;
integer               toggle_idx;
integer               rand_wait;
integer               rand_seed;

reg  [INT_WIDTH-1:0]  rand_div_int;
reg  [FRA_WIDTH-1:0]  rand_div_fra;

realtime              last_oclk_edge_t;
realtime              min_pulse_ns;
realtime              now_t;
realtime              dt_t;

clockdivider #(
    .INT_WIDTH(INT_WIDTH),
    .FRA_WIDTH(FRA_WIDTH)
) u_dut (
    .clk     (clk),
    .rstn    (rstn),
    .cken    (cken),
    .div_upd (div_upd),
    .div_int (div_int),
    .div_fra (div_fra),
    .oclk    (oclk)
);

initial begin
    clk = 1'b0;
end

always #(CLK_PERIOD/2) clk = ~clk;

task pulse_div_update_async;
    integer wait_cycles;
    begin
        // Do not start a new update until the previous request fully drains.
        wait_cycles = 0;
        while (((u_dut.div_upd_busy_q === 1'b1) ||
                (u_dut.div_upd_meta_q === 1'b1) ||
                (u_dut.div_upd_sync_q === 1'b1) ||
                (u_dut.div_req_pending_q === 1'b1) ||
                (u_dut.exit_bypass_q === 1'b1)) &&
               (wait_cycles < 512)) begin
            @(posedge clk);
            wait_cycles = wait_cycles + 1;
        end

        if ((u_dut.div_upd_busy_q === 1'b1) ||
            (u_dut.div_upd_meta_q === 1'b1) ||
            (u_dut.div_upd_sync_q === 1'b1) ||
            (u_dut.div_req_pending_q === 1'b1) ||
            (u_dut.exit_bypass_q === 1'b1)) begin
            $display("[%0t] ERROR: previous div_upd transaction did not drain", $time);
            err_cnt = err_cnt + 1;
        end

        // Hold divider bus stable first, then launch an async div_upd pulse.
        repeat (2) @(posedge clk);
        #(CLK_PERIOD/5);
        div_upd = 1'b1;

        // Keep div_upd asserted until the DUT has captured this request.
        wait_cycles = 0;
        while ((u_dut.div_upd_busy_q !== 1'b1) && (wait_cycles < 512)) begin
            @(posedge clk);
            wait_cycles = wait_cycles + 1;
        end

        if (u_dut.div_upd_busy_q !== 1'b1) begin
            $display("[%0t] ERROR: div_upd timed out before request capture", $time);
            err_cnt = err_cnt + 1;
        end

        @(posedge clk);
        #(CLK_PERIOD/5);
        div_upd = 1'b0;
    end
endtask

task pick_random_div;
    output [INT_WIDTH-1:0] o_int;
    output [FRA_WIDTH-1:0] o_fra;
    integer sel;
    begin
        // Mostly cover /2 and above, while still exercising /1.0 and 1.x.
        sel = $urandom(rand_seed) % 20;
        if (sel == 0) begin
            o_int = {{(INT_WIDTH-1){1'b0}}, 1'b1};
            o_fra = {FRA_WIDTH{1'b0}};
        end else if (sel < 4) begin
            o_int = {{(INT_WIDTH-1){1'b0}}, 1'b1};
            o_fra = ($urandom(rand_seed) % 255) + 1; // [1..255] => 1.x
        end else begin
            o_int = ($urandom(rand_seed) % 31) + 2; // [2..32]
            o_fra = $urandom(rand_seed) % 256;
        end
    end
endtask

task apply_cfg;
    input en;
    input [INT_WIDTH-1:0] i;
    input [FRA_WIDTH-1:0] f;
    input integer warmup_cycles;
    input integer measure_cycles;
    input [8*64-1:0] tag;
    input integer mode;
    integer div_x256;
    integer exp_edges;
    integer diff_edges;
    integer tol_edges;
    begin
        // mode: 0=force-low check, 1=ratio check
        cken    = en;
        div_int = i;
        div_fra = f;
        $display("[%0t] CASE: %0s  cken=%0d div_int=%0d div_fra=%0d",
                 $time, tag, en, i, f);

        pulse_div_update_async;
        repeat (warmup_cycles) @(posedge clk);

        clk_edge_cnt  = 0;
        oclk_edge_cnt = 0;
        measure_en    = 1'b1;
        repeat (measure_cycles) @(posedge clk);
        measure_en    = 1'b0;

        $display("[%0t] RESULT: clk_edges=%0d oclk_edges=%0d",
                 $time, clk_edge_cnt, oclk_edge_cnt);

        if (mode == 0) begin
            if (oclk_edge_cnt != 0) begin
                $display("[%0t] ERROR: expected oclk stuck low, got toggles=%0d",
                         $time, oclk_edge_cnt);
                err_cnt = err_cnt + 1;
            end
        end else begin
            // Ratio check: expected oclk_edges ~= clk_edges / div
            // div_x256 = div_int + div_fra/256
            div_x256 = (i << 8) + f;
            if (div_x256 <= 0) begin
                $display("[%0t] ERROR: invalid divider setting", $time);
                err_cnt = err_cnt + 1;
            end else begin
                exp_edges = (clk_edge_cnt * 256 + (div_x256 / 2)) / div_x256;
                diff_edges = (oclk_edge_cnt > exp_edges) ?
                             (oclk_edge_cnt - exp_edges) :
                             (exp_edges - oclk_edge_cnt);
                // Allow 8% + 2 edges tolerance for finite-window boundary effects.
                tol_edges = (exp_edges * 8) / 100 + 2;
                $display("[%0t] EXPECT: div=%0d.%0d exp_edges=%0d diff=%0d tol=%0d",
                         $time, i, (f * 1000) / 256, exp_edges, diff_edges, tol_edges);
                if (diff_edges > tol_edges) begin
                    $display("[%0t] ERROR: ratio mismatch, measured=%0d expected=%0d",
                             $time, oclk_edge_cnt, exp_edges);
                    err_cnt = err_cnt + 1;
                end
            end
        end
    end
endtask

task apply_cfg_long_div_upd;
    input en;
    input [INT_WIDTH-1:0] i;
    input [FRA_WIDTH-1:0] f;
    input integer warmup_cycles;
    input integer measure_cycles;
    input integer hold_cycles_after_measure;
    input [8*88-1:0] tag;
    input integer mode;
    begin
        // Software-style usage: coefficients are already stable, then div_upd
        // stays high for a long time without any internal handshake polling.
        cken    = en;
        div_int = i;
        div_fra = f;

        #(CLK_PERIOD/5);
        div_upd = 1'b1;

        if (mode == 0) begin
            expect_oclk_forced_low(warmup_cycles, measure_cycles, tag);
        end else begin
            expect_ratio_with_current_cfg(i, f, warmup_cycles, measure_cycles, tag);
        end

        repeat (hold_cycles_after_measure) @(posedge clk);
        #(CLK_PERIOD/5);
        div_upd = 1'b0;
        repeat (4) @(posedge clk);
    end
endtask

task expect_bypass_same_cfg_divupd_no_clamp;
    input integer hold_cycles;
    input [8*88-1:0] tag;
    integer sample_err;
    begin
        sample_err    = 0;
        clk_edge_cnt  = 0;
        oclk_edge_cnt = 0;

        @(negedge clk);
        #0.001;
        measure_en = 1'b1;

        #(CLK_PERIOD/5);
        div_upd = 1'b1;

        repeat (hold_cycles) begin
            @(posedge clk);
            #0.001;
            if (oclk !== 1'b1) begin
                sample_err = sample_err + 1;
            end

            @(negedge clk);
            #0.001;
            if (oclk !== 1'b0) begin
                sample_err = sample_err + 1;
            end
        end

        #(CLK_PERIOD/5);
        div_upd = 1'b0;
        @(negedge clk);
        #0.001;
        measure_en = 1'b0;

        $display("[%0t] BYPASS_NOCLAMP: %0s clk_edges=%0d oclk_edges=%0d sample_err=%0d",
                 $time, tag, clk_edge_cnt, oclk_edge_cnt, sample_err);

        if ((oclk_edge_cnt != clk_edge_cnt) || (sample_err != 0)) begin
            $display("[%0t] ERROR: bypass no-op div_upd disturbed oclk in %0s",
                     $time, tag);
            err_cnt = err_cnt + 1;
        end

        repeat (4) @(posedge clk);
    end
endtask

task expect_ratio_with_current_cfg;
    input [INT_WIDTH-1:0] i;
    input [FRA_WIDTH-1:0] f;
    input integer warmup_cycles;
    input integer measure_cycles;
    input [8*72-1:0] tag;
    integer div_x256;
    integer exp_edges;
    integer diff_edges;
    integer tol_edges;
    begin
        $display("[%0t] CASE: %0s  expect_div=%0d div_fra=%0d",
                 $time, tag, i, f);

        repeat (warmup_cycles) @(posedge clk);

        clk_edge_cnt  = 0;
        oclk_edge_cnt = 0;
        measure_en    = 1'b1;
        repeat (measure_cycles) @(posedge clk);
        measure_en    = 1'b0;

        $display("[%0t] RESULT: clk_edges=%0d oclk_edges=%0d",
                 $time, clk_edge_cnt, oclk_edge_cnt);

        div_x256 = (i << 8) + f;
        exp_edges = (clk_edge_cnt * 256 + (div_x256 / 2)) / div_x256;
        diff_edges = (oclk_edge_cnt > exp_edges) ?
                     (oclk_edge_cnt - exp_edges) :
                     (exp_edges - oclk_edge_cnt);
        tol_edges = (exp_edges * 8) / 100 + 2;
        $display("[%0t] EXPECT: div=%0d.%0d exp_edges=%0d diff=%0d tol=%0d",
                 $time, i, (f * 1000) / 256, exp_edges, diff_edges, tol_edges);
        if (diff_edges > tol_edges) begin
            $display("[%0t] ERROR: ratio mismatch, measured=%0d expected=%0d",
                     $time, oclk_edge_cnt, exp_edges);
            err_cnt = err_cnt + 1;
        end
    end
endtask

task expect_oclk_forced_low;
    input integer settle_cycles;
    input integer check_cycles;
    input [8*72-1:0] tag;
    begin
        repeat (settle_cycles) @(posedge clk);

        clk_edge_cnt  = 0;
        oclk_edge_cnt = 0;
        measure_en    = 1'b1;
        repeat (check_cycles) @(posedge clk);
        measure_en    = 1'b0;

        $display("[%0t] LOWCHK: %0s  clk_edges=%0d oclk_edges=%0d oclk=%0b",
                 $time, tag, clk_edge_cnt, oclk_edge_cnt, oclk);

        if ((oclk_edge_cnt != 0) || (oclk !== 1'b0)) begin
            $display("[%0t] ERROR: expected forced-low output in %0s",
                     $time, tag);
            err_cnt = err_cnt + 1;
        end
    end
endtask

task release_reset_async;
    begin
        @(negedge clk);
        #(CLK_PERIOD/5);
        rstn = 1'b1;
    end
endtask

always @(posedge clk or negedge clk) begin
    if (measure_en) begin
        clk_edge_cnt = clk_edge_cnt + 1;
    end
end

always @(posedge oclk or negedge oclk) begin
    if (measure_en) begin
        oclk_edge_cnt = oclk_edge_cnt + 1;
    end

    // Glitch monitor: reject pulses narrower than input half-cycle.
    if (!rstn) begin
        last_oclk_edge_t = -1.0;
    end else if ((oclk === 1'b0) || (oclk === 1'b1)) begin
        now_t = $realtime;
        if (last_oclk_edge_t >= 0.0) begin
            dt_t = now_t - last_oclk_edge_t;
            // Ignore delta-cycle (0ns) event pairs from zero-delay mux switching.
            // Only flag real short pulses with non-zero width.
            if ((dt_t > 0.001) && (dt_t + 0.001 < min_pulse_ns)) begin
                $display("[%0t] ERROR: glitch pulse too short: dt=%0.3fns (<%0.3fns)",
                         $time, dt_t, min_pulse_ns);
                err_cnt = err_cnt + 1;
            end
        end
        last_oclk_edge_t = now_t;
    end
end

initial begin
`ifdef DUMP_FSDB
    $fsdbDumpfile("waveform.fsdb");
    $fsdbDumpvars(0, tb_clockdivider);
`else
    $dumpfile("waveform.vcd");
    $dumpvars(0, tb_clockdivider);
`endif

    rstn            = 1'b0;
    cken            = 1'b0;
    div_upd         = 1'b0;
    div_int         = {INT_WIDTH{1'b0}};
    div_fra         = {FRA_WIDTH{1'b0}};
    measure_en      = 1'b0;
    clk_edge_cnt    = 0;
    oclk_edge_cnt   = 0;
    err_cnt         = 0;
    rand_seed       = 32'h20260225;
    last_oclk_edge_t = -1.0;
    min_pulse_ns    = (CLK_PERIOD / 2.0) - 0.05;

    repeat (5) @(posedge clk);
    release_reset_async;
    repeat (2) @(posedge clk);

    // Basic directed sanity.
    apply_cfg(1'b0, 8'd4, 8'd0,   8,  80, "cken=0 output forced low", 0);
    apply_cfg(1'b1, 8'd0, 8'd0,   8,  80, "div_int=0 output forced low", 0);
    apply_cfg(1'b1, 8'd1, 8'd0,  16, 200, "bypass mode (/1.0)", 1);
    apply_cfg(1'b1, 8'd1, 8'd64, 32, 320, "fractional divide (/1.25)", 1);
    apply_cfg(1'b1, 8'd1, 8'd128, 32, 320, "fractional divide (/1.5)", 1);
    apply_cfg(1'b1, 8'd1, 8'd192, 32, 320, "fractional divide (/1.75)", 1);
    apply_cfg(1'b1, 8'd2, 8'd128, 32, 320, "fractional divide (/2.5)", 1);

    // Divider bus changes must not take effect until div_upd is asserted.
    div_int = 8'd4;
    div_fra = 8'd0;
    expect_ratio_with_current_cfg(8'd2, 8'd128, 24, 240,
                                  "div change without div_upd should be ignored");
    pulse_div_update_async;
    expect_ratio_with_current_cfg(8'd4, 8'd0, 24, 240,
                                  "div change after div_upd should take effect");

    apply_cfg(1'b1, 8'd1, 8'd0, 16, 200, "bypass no-op update baseline (/1.0)", 1);
    div_int = 8'd1;
    div_fra = 8'd0;
    expect_bypass_same_cfg_divupd_no_clamp(40,
        "same bypass cfg with div_upd toggle should not clamp low");

    // Software-style long div_upd high level: coefficients are stable while
    // div_upd stays high, and the new divide must take effect before div_upd
    // deasserts.
    apply_cfg(1'b1, 8'd3, 8'd64, 16, 220, "pre-long div_upd high (/3.25)", 1);
    wait (oclk == 1'b1);
    #1;
    apply_cfg_long_div_upd(1'b1, 8'd7, 8'd64, 40, 320, 20,
                           "long div_upd high: divided to divided (/7.25)", 1);

    apply_cfg(1'b1, 8'd4, 8'd128, 16, 220, "pre-long div_upd to bypass (/4.5)", 1);
    wait (oclk == 1'b1);
    #1;
    apply_cfg_long_div_upd(1'b1, 8'd1, 8'd0, 40, 320, 20,
                           "long div_upd high: divided to bypass (/1.0)", 1);

    apply_cfg(1'b1, 8'd1, 8'd0, 16, 200, "pre-long div_upd from bypass (/1.0)", 1);
    wait (oclk == 1'b1);
    #1;
    apply_cfg_long_div_upd(1'b1, 8'd6, 8'd128, 40, 320, 20,
                           "long div_upd high: bypass to divided (/6.5)", 1);

    apply_cfg(1'b1, 8'd1, 8'd64, 24, 280, "pre-long div_upd frac1x (/1.25)", 1);
    wait (oclk == 1'b1);
    #1;
    apply_cfg_long_div_upd(1'b1, 8'd1, 8'd192, 40, 320, 20,
                           "long div_upd high: frac1x to frac1x (/1.75)", 1);

    apply_cfg(1'b1, 8'd5, 8'd192, 16, 220, "pre-cken low long div_upd high (/5.75)", 1);
    wait (oclk == 1'b1);
    #1;
    cken = 1'b0;
    expect_oclk_forced_low(8, 60, "cken low before long div_upd high should park low");
    div_int = 8'd9;
    div_fra = 8'd64;
    #(CLK_PERIOD/5);
    div_upd = 1'b1;
    expect_oclk_forced_low(8, 80, "cken=0 long div_upd high should stay low");
    cken = 1'b1;
    expect_ratio_with_current_cfg(8'd9, 8'd64, 40, 320,
                                  "re-enable during long div_upd high should use new cfg");
    repeat (20) @(posedge clk);
    #(CLK_PERIOD/5);
    div_upd = 1'b0;
    repeat (4) @(posedge clk);

    // Disable while oclk is high and confirm the clock parks low.
    apply_cfg(1'b1, 8'd5, 8'd128, 16, 220, "pre-disable running (/5.5)", 1);
    wait (oclk == 1'b1);
    #1;
    cken = 1'b0;
    expect_oclk_forced_low(8, 80, "cken low while running should park low");
    cken = 1'b1;
    expect_ratio_with_current_cfg(8'd5, 8'd128, 20, 240,
                                  "resume after cken re-enable");

    // reset/cken combination matrix
    $display("[%0t] RESET_CKEN_MATRIX: start", $time);

    // Case A: rstn=0, cken=0 -> forced low
    rstn = 1'b0;
    cken = 1'b0;
    div_int = 8'd6;
    div_fra = 8'd64;
    expect_oclk_forced_low(6, 40, "rstn=0 cken=0");

    // Case B: rstn=0, cken=1 -> still forced low
    cken = 1'b1;
    expect_oclk_forced_low(6, 40, "rstn=0 cken=1");

    // Case C: rstn=0 and toggle cken repeatedly -> always forced low
    repeat (6) begin
        cken = ~cken;
        repeat (3) @(posedge clk);
    end
    expect_oclk_forced_low(6, 40, "rstn=0 cken toggles");

    // Release reset while cken=0 -> remain low
    cken = 1'b0;
    release_reset_async;
    expect_oclk_forced_low(6, 40, "rstn release with cken=0");

    // Then enable clock and verify normal divide resumes
    apply_cfg(1'b1, 8'd3, 8'd128, 24, 300, "resume after reset release (/3.5)", 1);

    // Case D: running divider, assert reset while output is high
    apply_cfg(1'b1, 8'd2, 8'd192, 24, 220, "pre-reset running (/2.75)", 1);
    wait (oclk == 1'b1);
    #1;
    rstn = 1'b0;
    expect_oclk_forced_low(6, 40, "assert reset during active divide");

    // Case E: release reset with stable divider inputs and no div_upd.
    div_int = 8'd6;
    div_fra = 8'd64;
    cken = 1'b1;
    release_reset_async;
    @(posedge clk);
    if (u_dut.init_cfg_pending_q !== 1'b1) begin
        $display("[%0t] ERROR: init_cfg_pending_q cleared before cfg load completed",
                 $time);
        err_cnt = err_cnt + 1;
    end
    expect_ratio_with_current_cfg(8'd6, 8'd64, 24, 300,
                                  "post-reset implicit load without div_upd");

    // Case F: explicit updates after reset still work normally.
    apply_cfg(1'b1, 8'd4, 8'd0, 24, 260, "post-reset restart (/4.0)", 1);
    apply_cfg(1'b1, 8'd1, 8'd128, 24, 300, "post-reset 1.x restart (/1.5)", 1);

    // 100 random divider coefficient checks (ratio + glitch monitor).
    $display("[%0t] RANDOM_COEFF: start 100 random coefficient checks", $time);
    for (coeff_idx = 0; coeff_idx < 100; coeff_idx = coeff_idx + 1) begin
        pick_random_div(rand_div_int, rand_div_fra);

        // Force many updates while oclk is high to stress glitchless switching.
        if ((cken == 1'b1) && (($urandom(rand_seed) % 2) == 0)) begin
            wait (oclk == 1'b1);
            #1;
        end else begin
            @(posedge clk);
        end

        apply_cfg(1'b1, rand_div_int, rand_div_fra, 12, 240,
                  "random coefficient check", 1);
    end

    // 100 random cken toggles with random coefficient switching.
    $display("[%0t] RANDOM_STRESS: start 100 random cken toggles", $time);
    cken = 1'b1;
    pick_random_div(rand_div_int, rand_div_fra);
    div_int = rand_div_int;
    div_fra = rand_div_fra;
    repeat (20) @(posedge clk);

    for (toggle_idx = 0; toggle_idx < 100; toggle_idx = toggle_idx + 1) begin
        rand_wait = ($urandom(rand_seed) % 8) + 1;
        repeat (rand_wait) @(posedge clk);

        if ((cken == 1'b1) && (($urandom(rand_seed) % 2) == 0)) begin
            wait (oclk == 1'b1);
            #1;
        end else if (($urandom(rand_seed) % 2) == 0) begin
            @(posedge clk);
        end else begin
            @(negedge clk);
        end

        cken = ~cken;
        $display("[%0t] STRESS[%0d]: cken -> %0d", $time, toggle_idx, cken);

        pick_random_div(rand_div_int, rand_div_fra);
        if ((cken == 1'b1) && (($urandom(rand_seed) % 2) == 0)) begin
            wait (oclk == 1'b1);
            #1;
        end
        div_int = rand_div_int;
        div_fra = rand_div_fra;
        pulse_div_update_async;
        $display("[%0t] STRESS[%0d]: div_int=%0d div_fra=%0d",
                 $time, toggle_idx, rand_div_int, rand_div_fra);
    end

    // Post-stress functional check.
    apply_cfg(1'b1, 8'd1, 8'd192, 24, 320, "post-stress check (/1.75)", 1);
    apply_cfg(1'b1, 8'd2, 8'd128, 20, 320, "post-stress check (/2.5)", 1);

    if (err_cnt != 0) begin
        $display("[%0t] TB FAILED, errors=%0d", $time, err_cnt);
        $finish_and_return(1);
    end

`ifdef DUMP_FSDB
    $display("[%0t] TB done, waveform.fsdb generated.", $time);
`else
    $display("[%0t] TB done, waveform.vcd generated.", $time);
`endif
    $finish;
end

endmodule
