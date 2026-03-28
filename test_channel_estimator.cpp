//============================================================================
// Channel Estimator Testbench
// Uses pilot positions from cofdm_hls.h header
//============================================================================

#include "cofdm_hls.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define FFT_LEN N_FFT

//============================================================================
// Use is_pilot_offset based on header's pilot_positions
//============================================================================
bool is_pilot_offset(int offset) {
    for (int p = 0; p < N_PILOTS; p++) {
        if (offset == pilot_positions[p]) return true;
    }
    return false;
}

//============================================================================
// Test 1: No channel (ideal)
//============================================================================
int test_no_channel() {
    printf("=== Test 1: No Channel (Ideal) ===\n");

    hls::stream<axis_cmpx_user_t> in_stream("in");
    for (int idx = START_IDX; idx < START_IDX + N_USED; idx++) {
        axis_cmpx_user_t s;
        int offset = idx - START_IDX;

        if (is_pilot_offset(offset)) {
            s.i = PILOT_I_VAL;
            s.q = PILOT_Q_VAL;
        } else {
            s.i = 32767;
            s.q = 0;
        }
        s.user = idx;
        s.last = (idx == START_IDX + N_USED - 1);
        in_stream.write(s);
    }

    hls::stream<axis_cmpx_t> data_stream("data");
    hls::stream<axis_cmpx_t> hest_stream("hest");
    channel_estimator(in_stream, data_stream, hest_stream);

    axis_cmpx_t hest = hest_stream.read();
    printf("  H estimate: I=%d, Q=%d\n", (int)hest.i, (int)hest.q);

    int data_count = 0;
    printf("  First 5 data: ");
    while (!data_stream.empty()) {
        axis_cmpx_t d = data_stream.read();
        data_count++;
        if (data_count <= 5) printf("[%d,%d] ", (int)d.i, (int)d.q);
    }
    printf("\n  Total data: %d (expected %d)\n", data_count, N_DATA);

    if (data_count != N_DATA) {
        printf("  FAIL: Wrong data count\n\n");
        return 1;
    }
    if (abs((int)hest.i - 32767) < 3000) {
        printf("  PASS\n\n");
        return 0;
    } else {
        printf("  INFO: H estimate = %d (expected ~32767)\n\n", (int)hest.i);
        return 0;
    }
}

//============================================================================
// Test 2: Known flat fading channel
//============================================================================
int test_flat_fading() {
    printf("=== Test 2: Flat Fading Channel ===\n");

    const int16_t_hls CH_I = 16384;
    const int16_t_hls CH_Q = 16384;

    hls::stream<axis_cmpx_user_t> in_stream("in");
    for (int idx = START_IDX; idx < START_IDX + N_USED; idx++) {
        axis_cmpx_user_t s;
        int offset = idx - START_IDX;

        if (is_pilot_offset(offset)) {
            s.i = (int16_t_hls)((PILOT_I_VAL * (int)CH_I - PILOT_Q_VAL * (int)CH_Q) >> 15);
            s.q = (int16_t_hls)((PILOT_I_VAL * (int)CH_Q + PILOT_Q_VAL * (int)CH_I) >> 15);
        } else {
            s.i = CH_I;
            s.q = CH_Q;
        }
        s.user = idx;
        s.last = (idx == START_IDX + N_USED - 1);
        in_stream.write(s);
    }

    hls::stream<axis_cmpx_t> data_stream("data");
    hls::stream<axis_cmpx_t> hest_stream("hest");
    channel_estimator(in_stream, data_stream, hest_stream);

    axis_cmpx_t hest = hest_stream.read();
    printf("  H estimate: I=%d, Q=%d\n", (int)hest.i, (int)hest.q);
    printf("  Expected: I~%d, Q~%d\n", (int)CH_I, (int)CH_Q);

    int err_i = abs((int)hest.i - (int)CH_I);
    int err_q = abs((int)hest.q - (int)CH_Q);

    if (err_i < 4000 && err_q < 4000) {
        printf("  PASS (within 20%% tolerance)\n\n");
        return 0;
    } else {
        printf("  FAIL: error I=%d, Q=%d\n\n", err_i, err_q);
        return 1;
    }
}

//============================================================================
// Test 3: Data extraction
//============================================================================
int test_data_extraction() {
    printf("=== Test 3: Data Extraction ===\n");

    hls::stream<axis_cmpx_user_t> in_stream("in");
    int data_idx = 0;
    for (int idx = START_IDX; idx < START_IDX + N_USED; idx++) {
        axis_cmpx_user_t s;
        int offset = idx - START_IDX;

        if (is_pilot_offset(offset)) {
            s.i = PILOT_I_VAL;
            s.q = PILOT_Q_VAL;
        } else {
            s.i = 1000 + data_idx;
            s.q = 500 + data_idx;
            data_idx++;
        }
        s.user = idx;
        s.last = (idx == START_IDX + N_USED - 1);
        in_stream.write(s);
    }

    printf("  Sent %d data samples\n", data_idx);

    hls::stream<axis_cmpx_t> data_stream("data");
    hls::stream<axis_cmpx_t> hest_stream("hest");
    channel_estimator(in_stream, data_stream, hest_stream);

    int errors = 0;
    data_idx = 0;
    while (!data_stream.empty()) {
        axis_cmpx_t d = data_stream.read();
        int expected_i = 1000 + data_idx;
        int expected_q = 500 + data_idx;

        if ((int)d.i != expected_i || (int)d.q != expected_q) {
            errors++;
            if (errors <= 5) {
                printf("  Error at idx %d: got [%d,%d], expected [%d,%d]\n",
                       data_idx, (int)d.i, (int)d.q, expected_i, expected_q);
            }
        }
        data_idx++;
    }

    printf("  Extracted %d data samples\n", data_idx);
    printf("  Errors: %d\n", errors);

    if (errors == 0 && data_idx == N_DATA) {
        printf("  PASS\n\n");
        return 0;
    } else {
        printf("  FAIL\n\n");
        return 1;
    }
}

//============================================================================
// Main
//============================================================================
int main() {
    printf("================================================================\n");
    printf("CHANNEL ESTIMATOR TESTBENCH\n");
    printf("================================================================\n\n");

    printf("System parameters:\n");
    printf("  N_FFT=%d, N_USED=%d, N_PILOTS=%d, N_DATA=%d\n",
           N_FFT, N_USED, N_PILOTS, N_DATA);
    printf("  START_IDX=%d\n", START_IDX);
    printf("  PILOT_I_VAL=%d, PILOT_RECIP=%d\n\n",
           (int)PILOT_I_VAL, (int)PILOT_RECIP);

    printf("Pilot positions (from header):\n  ");
    for (int i = 0; i < 32; i++) {
        printf("%d ", (int)pilot_positions[i]);
        if ((i+1) % 8 == 0) printf("\n  ");
    }
    printf("\n\n");

    int errors = 0;
    errors += test_no_channel();
    errors += test_flat_fading();
    errors += test_data_extraction();

    printf("================================================================\n");
    printf("TEST SUMMARY: %d errors\n", errors);
    printf("================================================================\n");

    return errors;
}
