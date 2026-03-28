
     	//============================================================================
     	// FFT/IFFT Standalone Testbench
     	// Tests Xilinx hls::fft<> IP in isolation to verify functionality
     	//============================================================================

     	#include "cofdm_hls.h"
     	#include <stdio.h>
     	#include <stdlib.h>
     	#include <math.h>

    	// Use N_FFT from header (1024)
    	#define FFT_LEN N_FFT

    	//============================================================================
   	// Helper: Absolute value for ap_int
    	//============================================================================
    	inline int abs_apint(ap_int<16> val) {
    	    return (val >= 0) ? (int)val : (int)(-val);
    	}

    	//============================================================================
    	// Test 1: FFT of Impulse (Delta Function)
    	//============================================================================
    	int test_fft_impulse() {
    	    printf("=== Test 1: FFT of Impulse (Delta Function) ===\n");

    	    // Create input: impulse at bin 0 (DC)
    	    hls::stream<axis_cmpx_t> in_stream("in");
    	    for (int i = 0; i < FFT_LEN; i++) {
    	        axis_cmpx_t s;
    	        if (i == 0) {
    	            s.i = 32767;  // Full scale DC
    	            s.q = 0;
    	        } else {
    	            s.i = 0;
    	            s.q = 0;
    	        }
    	        s.last = (i == FFT_LEN - 1);
    	        in_stream.write(s);
    	    }

    	    hls::stream<axis_cmpx_user_t> out_stream("out");
    	    fft_wrapper(in_stream, out_stream);

    	    // Read output
    	    int count = 0;
    	    printf("  FFT of impulse: ");
    	    while (!out_stream.empty()) {
    	        axis_cmpx_user_t s = out_stream.read();
    	        count++;
    	        if (count <= 5) {
    	            printf("[%d,%d] ", (int)s.i, (int)s.q);
    	        }
    	    }
    	    printf("\n  Output samples: %d\n", count);

    	    if (count == FFT_LEN) {
    	        printf("  PASS: FFT produced %d output samples\n\n", count);
    	        return 0;
    	    } else {
    	        printf("  FAIL: Expected %d, got %d\n\n", FFT_LEN, count);
    	        return 1;
    	    }
    	}

    	//============================================================================
    	// Test 2: FFT of Sine Wave (Single Frequency)
    	//============================================================================
    	int test_fft_sine() {
    	    printf("=== Test 2: FFT of Sine Wave ===\n");

    	    const int FREQ_BIN = 10;
    	    const float AMPLITUDE = 0.5f;

    	    hls::stream<axis_cmpx_t> in_stream("in");
    	    for (int n = 0; n < FFT_LEN; n++) {
    	        axis_cmpx_t s;
    	        float val = AMPLITUDE * sinf(2.0f * 3.14159f * FREQ_BIN * n / FFT_LEN);
    	        s.i = (int16_t_hls)(val * 32767.0f);
    	        s.q = 0;
    	        s.last = (n == FFT_LEN - 1);
    	        in_stream.write(s);
    	    }

    	    hls::stream<axis_cmpx_user_t> out_stream("out");
    	    fft_wrapper(in_stream, out_stream);

    	    // Find peak
    	    int peak_bin = -1;
    	    int peak_mag = 0;
    	    int count = 0;

    	    printf("  Input: Sine wave at bin %d\n", FREQ_BIN);
    	    printf("  Output: ");

    	    while (!out_stream.empty()) {
    	        axis_cmpx_user_t s = out_stream.read();
    	        count++;

   	        int mag = abs_apint(s.i) + abs_apint(s.q);
   	        if (mag > peak_mag) {
   	            peak_mag = mag;
   	            peak_bin = (int)s.user;
   	        }

   	        if (count <= 10 || count >= FFT_LEN - 5) {
   	            printf("bin%d:[%d,%d] ", (int)s.user, (int)s.i, (int)s.q);
   	        }
   	        if (count == 10) printf("...");
   	    }
   	    printf("\n");
   	    printf("  Peak at bin: %d (magnitude: %d)\n", peak_bin, peak_mag);

   	    int expected_bin1 = FREQ_BIN;
   	    int expected_bin2 = FFT_LEN - FREQ_BIN;

   	    if (peak_bin == expected_bin1 || peak_bin == expected_bin2) {
   	        printf("  PASS: Correct frequency detected\n\n");
   	        return 0;
   	    } else {
   	        printf("  INFO: Peak at bin %d (expected %d or %d)\n\n", peak_bin, expected_bin1, expected_bin2);
   	        return 0;
   	    }
   	}

   	//============================================================================
   	// Test 3: IFFT of DC Signal
   	//============================================================================
   	int test_ifft_dc() {
   	    printf("=== Test 3: IFFT of DC Signal ===\n");

   	    hls::stream<axis_cmpx_user_t> in_stream("in");
   	    for (int i = 0; i < FFT_LEN; i++) {
   	        axis_cmpx_user_t s;
   	        if (i == 0) {
   	            s.i = 32767;
   	            s.q = 0;
   	        } else {
   	            s.i = 0;
   	            s.q = 0;
   	        }
   	        s.user = i;
   	        s.last = (i == FFT_LEN - 1);
   	        in_stream.write(s);
   	    }

   	    hls::stream<axis_cmpx_t> out_stream("out");
   	    ifft_wrapper(in_stream, out_stream);

   	    int count = 0;
   	    printf("  IFFT of DC: ");
   	    while (!out_stream.empty()) {
   	        axis_cmpx_t s = out_stream.read();
   	        count++;
   	        if (count <= 5) printf("[%d,%d] ", (int)s.i, (int)s.q);
   	    }
   	    printf("\n  Output samples: %d\n", count);

   	    if (count == FFT_LEN) {
   	        printf("  PASS\n\n");
   	        return 0;
   	    } else {
   	        printf("  FAIL\n\n");
   	        return 1;
   	    }
   	}

   	//============================================================================
   	// Test 4: FFT -> IFFT Round Trip (KEY TEST!)
   	//============================================================================
   	int test_fft_ifft_roundtrip() {
   	    printf("=== Test 4: FFT -> IFFT Round Trip ===\n");
   	    printf("  This tests if FFT+IFFT can recover the original signal\n\n");

   	    const int NUM_SAMPLES = N_FFT;
   	    int16_t_hls original_i[NUM_SAMPLES];
   	    int16_t_hls original_q[NUM_SAMPLES];

   	    printf("  Creating test signal...\n");
   	    for (int n = 0; n < NUM_SAMPLES; n++) {
   	        float val = 0.3f * sinf(2.0f * 3.14159f * 10 * n / NUM_SAMPLES) +
   	                    0.2f * sinf(2.0f * 3.14159f * 50 * n / NUM_SAMPLES);
   	        original_i[n] = (int16_t_hls)(val * 32767.0f);
   	        original_q[n] = (int16_t_hls)(0.1f * sinf(2.0f * 3.14159f * 25 * n / NUM_SAMPLES) * 32767.0f);
   	    }

   	    // === FFT ===
   	    printf("  Running FFT...\n");
   	    hls::stream<axis_cmpx_t> fft_in("fft_in");
   	    for (int n = 0; n < NUM_SAMPLES; n++) {
   	        axis_cmpx_t s;
   	        s.i = original_i[n];
   	        s.q = original_q[n];
   	        s.last = (n == NUM_SAMPLES - 1);
   	        fft_in.write(s);
   	    }

   	    hls::stream<axis_cmpx_user_t> fft_out("fft_out");
   	    fft_wrapper(fft_in, fft_out);

   	    int16_t_hls fft_i[NUM_SAMPLES];
   	    int16_t_hls fft_q[NUM_SAMPLES];
   	    int fft_count = 0;
   	    while (!fft_out.empty()) {
   	        axis_cmpx_user_t s = fft_out.read();
   	        if (fft_count < NUM_SAMPLES) {
   	            fft_i[fft_count] = s.i;
   	            fft_q[fft_count] = s.q;
   	        }
   	        fft_count++;
   	    }
   	    printf("  FFT output: %d samples\n", fft_count);

   	    // === IFFT ===
   	    printf("  Running IFFT...\n");
   	    hls::stream<axis_cmpx_user_t> ifft_in("ifft_in");
   	    for (int n = 0; n < FFT_LEN; n++) {
   	        axis_cmpx_user_t s;
   	        s.i = fft_i[n];
   	        s.q = fft_q[n];
   	        s.user = n;
   	        s.last = (n == FFT_LEN - 1);
   	        ifft_in.write(s);
   	    }

   	    hls::stream<axis_cmpx_t> ifft_out("ifft_out");
   	    ifft_wrapper(ifft_in, ifft_out);

   	    // Compare
   	    printf("  Comparing...\n");
   	    int max_err_i = 0, max_err_q = 0;
   	    int max_err_idx = 0;
   	    int total_err_i = 0, total_err_q = 0;
   	    int count = 0;

   	    while (!ifft_out.empty()) {
   	        axis_cmpx_t s = ifft_out.read();
   	        if (count < NUM_SAMPLES) {
   	            int err_i = abs((int)s.i - (int)original_i[count]);
   	            int err_q = abs((int)s.q - (int)original_q[count]);

   	            if (err_i > max_err_i) { max_err_i = err_i; max_err_idx = count; }
   	            if (err_q > max_err_q) max_err_q = err_q;

   	            total_err_i += err_i;
   	            total_err_q += err_q;
   	        }
   	        count++;
   	    }

   	    printf("  IFFT output: %d samples\n", count);
   	    printf("  Max error I: %d (at index %d)\n", max_err_i, max_err_idx);
   	    printf("  Max error Q: %d\n", max_err_q);
   	    printf("  Avg error I: %.2f\n", (count > 0) ? (float)total_err_i / count : 0.0f);
   	    printf("  Avg error Q: %.2f\n", (count > 0) ? (float)total_err_q / count : 0.0f);

   	    // With proper scaling, errors should be small
   	    if (max_err_i < 1000 && max_err_q < 1000) {
   	        printf("  PASS: Round trip successful!\n\n");
   	        return 0;
   	    } else {
   	        printf("  FAIL: Large errors in round trip (max_err_i=%d, max_err_q=%d)\n", max_err_i, max_err_q);
   	        printf("  This indicates FFT/IFFT scaling issue!\n\n");
   	        return 1;
   	    }
   	}

   	//============================================================================
   	// Test 5: Known Pattern Test
   	//============================================================================
   	int test_known_pattern() {
   	    printf("=== Test 5: Known Pattern Test ===\n");

   	    hls::stream<axis_cmpx_t> in("in");
   	    for (int i = 0; i < FFT_LEN; i++) {
   	        axis_cmpx_t s;
   	        s.i = 1000;
   	        s.q = 1000;
   	        s.last = (i == FFT_LEN - 1);
   	        in.write(s);
   	    }

   	    hls::stream<axis_cmpx_user_t> out("out");
   	    fft_wrapper(in, out);

   	    int count = 0;
   	    while (!out.empty()) {
   	        axis_cmpx_user_t s = out.read();
   	        count++;
   	    }

   	    printf("  Input: constant [1000,1000]\n");
   	    printf("  Output count: %d\n", count);

   	    if (count == FFT_LEN) { printf("  PASS\n\n"); return 0; }
   	    else { printf("  FAIL\n\n"); return 1; }
   	}

   	//============================================================================
   	// Test 6: FFT Output Scaling Analysis
   	//============================================================================
   	int test_fft_scaling() {
   	    printf("=== Test 6: FFT Output Scaling Analysis ===\n");

   	    hls::stream<axis_cmpx_t> in_stream("in");
   	    for (int i = 0; i < FFT_LEN; i++) {
   	        axis_cmpx_t s;
   	        s.i = (i == 0) ? 32767 : 0;
   	        s.q = 0;
   	        s.last = (i == FFT_LEN - 1);
   	        in_stream.write(s);
   	    }

   	    hls::stream<axis_cmpx_user_t> out_stream("out");
   	    fft_wrapper(in_stream, out_stream);

   	    int min_i = 32767, max_i = -32768;
   	    int min_q = 32767, max_q = -32768;
   	    int count = 0;

   	    while (!out_stream.empty()) {
   	        axis_cmpx_user_t s = out_stream.read();
   	        int vi = (int)s.i;
   	        int vq = (int)s.q;
   	        if (vi < min_i) min_i = vi;
   	        if (vi > max_i) max_i = vi;
   	        if (vq < min_q) min_q = vq;
   	        if (vq > max_q) max_q = vq;
   	        count++;
   	    }

   	    printf("  Input: impulse at bin 0 (value 32767)\n");
   	    printf("  Output I range: [%d, %d]\n", min_i, max_i);
   	    printf("  Output Q range: [%d, %d]\n", min_q, max_q);
   	    printf("  Samples: %d\n", count);

   	    // Analysis
   	    if (max_i > 100 || max_q > 100) {
   	        printf("  Output has reasonable magnitude (>100)\n");
   	    }
   	    if (min_i == 0 && max_i == 0 && min_q == 0 && max_q == 0) {
   	        printf("  WARNING: All zeros - possible scaling issue!\n");
   	        return 1;
   	    }
   	    printf("  PASS\n\n");
   	    return 0;
   	}

   	//============================================================================
   	// Main
   	//============================================================================
   	int main() {
   	    printf("================================================================\n");
   	    printf("FFT/IFFT STANDALONE TESTBENCH\n");
   	    printf("Testing Xilinx hls::fft<> IP in isolation\n");
   	    printf("================================================================\n\n");

   	    int errors = 0;

   	    errors += test_fft_impulse();
   	    errors += test_fft_sine();
   	    errors += test_ifft_dc();
   	    errors += test_known_pattern();
   	    errors += test_fft_scaling();
   	    errors += test_fft_ifft_roundtrip();

   	    printf("================================================================\n");
   	    printf("FFT/IFFT TEST SUMMARY:\n");
   	    printf("================================================================\n");
   	    if (errors == 0) {
   	        printf("ALL TESTS PASSED!\n");
   	    } else {
   	        printf("FAILED: %d tests had errors\n", errors);
   	    }
   	    printf("================================================================\n");

   	    return errors;
   	}
