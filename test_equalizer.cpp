//============================================================================
// Equalizer Testbench (Corrected)
// Matches streaming hardware: 1 H-estimate per 1 Data-carrier
//============================================================================
#include "cofdm_hls.h"
#include <stdio.h>
#include <stdlib.h>

//============================================================================
// Test 1: No channel (H = 1.0 + 0j)
//============================================================================
int test_no_channel() {
    printf("=== Test 1: No Channel (H = 1.0 + 0j) ===\n");

    // Use 32768 for 1.0 (Power of 2 makes integer math cleaner)
    const int16_t_hls H_I = 32767;
    const int16_t_hls H_Q = 0;
    const int NUM_DATA = 10;

    hls::stream<axis_cmpx_t> hest_stream("hest");
    hls::stream<axis_cmpx_t> data_stream("data");

    for (int i = 0; i < NUM_DATA; i++) {
        // FIX: Must write one H for every Data point
        hest_stream.write({H_I, H_Q, false});

        axis_cmpx_t s = {(int16_t_hls)(1000 + i*100), (int16_t_hls)(500 + i*50), (i==NUM_DATA-1)};
        data_stream.write(s);
    }

    hls::stream<axis_cmpx_t> out_stream("out");
    equalizer(data_stream, hest_stream, out_stream, NUM_DATA);

    int errors = 0;
    for (int i = 0; i < NUM_DATA; i++) {
        axis_cmpx_t out = out_stream.read();
        int expected_i = 1000 + i*100;
        int expected_q = 500 + i*50;

        // Allow small rounding error due to 32767 vs 32768
        if (abs((int)out.i - expected_i) > 2 || abs((int)out.q - expected_q) > 2) {
            printf("  Error at index %d: Got (%d, %d), Expected (%d, %d)\n", i, (int)out.i, (int)out.q, expected_i, expected_q);
            errors++;
        }
    }

    if (errors == 0) printf("  PASS\n\n");
    else printf("  FAIL: %d errors\n\n", errors);
    return errors;
}

//============================================================================
// Test 2: Known channel (H = 0.5 + 0.5j)
//============================================================================
int test_known_channel() {
    printf("=== Test 2: Known Channel (H = 0.5 + 0.5j) ===\n");

    const int16_t_hls H_I = 16384; // 0.5 in Q15
    const int16_t_hls H_Q = 16384; // 0.5 in Q15
    const int NUM_DATA = 5;

    hls::stream<axis_cmpx_t> hest_stream("hest");
    hls::stream<axis_cmpx_t> data_stream("data");

    for (int i = 0; i < NUM_DATA; i++) {
        int X_i = 1000 + i*100;
        int X_q = 500 + i*50;

        // Emulate channel: Y = (X * H) >> 15
        int Y_i = (X_i * H_I - X_q * H_Q) >> 15;
        int Y_q = (X_i * H_Q + X_q * H_I) >> 15;

        hest_stream.write({H_I, H_Q, false});
        data_stream.write({(int16_t_hls)Y_i, (int16_t_hls)Y_q, (i==NUM_DATA-1)});
    }

    hls::stream<axis_cmpx_t> out_stream("out");
    equalizer(data_stream, hest_stream, out_stream, NUM_DATA);

    int errors = 0;
    for (int i = 0; i < NUM_DATA; i++) {
        axis_cmpx_t out = out_stream.read();
        int exp_i = 1000 + i*100;
        int exp_q = 500 + i*50;

        if (abs((int)out.i - exp_i) > 5 || abs((int)out.q - exp_q) > 5) {
            printf("  Error index %d: Got (%d, %d), Expected (%d, %d)\n", i, (int)out.i, (int)out.q, exp_i, exp_q);
            errors++;
        }
    }

    if (errors == 0) printf("  PASS\n\n");
    return errors;
}

//============================================================================
// Test 3: Zero channel (Safety check)
//============================================================================
int test_zero_channel() {
    printf("=== Test 3: Zero Channel (H = 0+0j) ===\n");

    hls::stream<axis_cmpx_t> hest_stream("hest");
    hls::stream<axis_cmpx_t> data_stream("data");

    hest_stream.write({0, 0, false});
    data_stream.write({1000, 500, true});

    hls::stream<axis_cmpx_t> out_stream("out");
    equalizer(data_stream, hest_stream, out_stream, 1);

    if (!out_stream.empty()) {
        out_stream.read();
        printf("  PASS (Hardware didn't hang)\n\n");
        return 0;
    }
    return 1;
}

int main() {
    int errors = 0;
    errors += test_no_channel();
    errors += test_known_channel();
    errors += test_zero_channel();

    if (errors == 0) printf("ALL TESTS PASSED!\n");
    else printf("FAILED: %d errors\n", errors);
    return errors;
}
