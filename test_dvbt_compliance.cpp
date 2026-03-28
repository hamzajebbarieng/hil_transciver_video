        //============================================================================
     	// DVB-T Compliance Testbench
     	// Tests the fixed HLS code for proper DVB-T compliance
     	//
     	// Tests:
     	// 1. Type definitions are DVB-T compliant
     	// 2. TLAST propagation through CP insert
     	// 3. TLAST propagation through CP removal
     	// 4. Data flow integrity
    	// 5. Proper frame boundary detection
    	//============================================================================

    	#include "cofdm_hls.h"
    	#include <stdio.h>
    	#include <stdlib.h>
    	#include <string.h>

    	// Test counters
    	int tests_passed = 0;
    	int tests_failed = 0;

    	//============================================================================
    	// Test 1: Verify Type Definitions
    	//============================================================================
    	int test_type_definitions() {
    	    printf("=== Test 1: Type Definitions (DVB-T Compliant) ===\n");

    	    int errors = 0;

    	    // Test axis_bit_t
    	    axis_bit_t bit_sample;
    	    bit_sample.data = 1;
    	    bit_sample.last = true;
    	    if (bit_sample.last != true) {
    	        printf("  FAIL: axis_bit_t.last not working\n");
    	        errors++;
   	      } else {
    	        printf("  OK: axis_bit_t has data and last fields\n");
    	    }

    	    // Test axis_dibit_t
    	    axis_dibit_t dibit_sample;
    	    dibit_sample.data = 3;
    	    dibit_sample.last = false;
    	    if (dibit_sample.last != false) {
    	        printf("  FAIL: axis_dibit_t.last not working\n");
    	        errors++;
        } else {
    	        printf("  OK: axis_dibit_t has data and last fields\n");
    	    }

    	    // Test axis_cmpx_t
    	    axis_cmpx_t cmpx_sample;
    	    cmpx_sample.i = 100;
    	    cmpx_sample.q = -100;
    	    cmpx_sample.last = true;
    	    if (cmpx_sample.i != 100 || cmpx_sample.last != true) {
    	        printf("  FAIL: axis_cmpx_t not working\n");
    	        errors++;
    	    } else {
    	        printf("  OK: axis_cmpx_t has i, q, and last fields\n");
    	    }

    	    // Test axis_cmpx_user_t
    	    axis_cmpx_user_t cmpx_user_sample;
    	    cmpx_user_sample.i = 200;
    	    cmpx_user_sample.q = 300;
    	    cmpx_user_sample.user = 512;
    	    cmpx_user_sample.last = true;
    	    if (cmpx_user_sample.user != 512) {
    	        printf("  FAIL: axis_cmpx_user_t.user not working\n");
    	        errors++;
    	    } else {
    	        printf("  OK: axis_cmpx_user_t has i, q, user, and last fields\n");
    	    }

    	    // Test DVB-T parameters
    	    printf("  DVB-T Parameters:\n");
    	    printf("    N_FFT = %d (should be 1024 for 2K mode)\n", N_FFT);
    	    printf("    N_CP = %d (should be 128 for short CP)\n", N_CP);
    	    printf("    N_USED = %d (should be 768 for 2K mode)\n", N_USED);
    	    printf("    N_PILOTS = %d (should be 32 scatter pilots)\n", N_PILOTS);
    	    printf("    N_DATA = %d (should be 736 = 768-32)\n", N_DATA);

    	    if (N_FFT != 1024 || N_CP != 128 || N_USED != 768 || N_PILOTS != 32 || N_DATA != 736) {
    	        printf("  FAIL: DVB-T parameters incorrect\n");
    	        errors++;
    	    } else {
    	        printf("  OK: All DVB-T parameters correct\n");
    	    }

    	    if (errors == 0) {
    	        printf("  PASS: Type definitions test\n\n");
    	        return 0;
    	    } else {
    	        printf("  FAIL: %d errors\n\n", errors);
    	        return errors;
    	    }
    	}

   	//============================================================================
   	// Test 2: CP Insert TLAST Propagation
   	// Tests that cp_insert correctly sets TLAST at the end of 1152-sample frame
   	//============================================================================
   	int test_cp_insert_tlast() {
   	    printf("=== Test 2: CP Insert TLAST Propagation ===\n");

   	    hls::stream<axis_cmpx_t> in_stream("cp_in");
   	    hls::stream<axis_cmpx_t> out_stream("cp_out");

   	    // Generate 1024 input samples (one OFDM symbol)
   	    for (int i = 0; i < N_FFT; i++) {
   	        axis_cmpx_t sample;
   	        sample.i = i;
   	        sample.q = -i;
   	        // Set last only on the last input sample
   	        sample.last = (i == N_FFT - 1);
   	        in_stream.write(sample);
   	    }

   	    // Call cp_insert
   	    cp_insert(in_stream, out_stream);

   	    // Drain output: should get 1152 samples (128 CP + 1024 data)
   	    int last_count = 0;
   	    int total_samples = 0;

   	    // Check CP samples (first 128)
   	    for (int i = 0; i < N_CP; i++) {
   	        if (out_stream.empty()) break;
   	        axis_cmpx_t sample = out_stream.read();
   	        total_samples++;
   	        if (sample.last) {
   	            printf("  FAIL: TLAST should not be set during CP (sample %d)\n", i);
   	            return 1;
   	        }
   	        // CP should contain last 128 samples of FFT data
   	        int expected_idx = (N_FFT - N_CP) + i;
   	        if (sample.i != expected_idx) {
   	            printf("  FAIL: CP sample %d has wrong I value: got %d, expected %d\n",
   	                   i, (int)sample.i, expected_idx);
   	            return 1;
   	        }
   	    }

   	    // Check data samples (next 1024)
   	    for (int i = 0; i < N_FFT; i++) {
   	        if (out_stream.empty()) break;
   	        axis_cmpx_t sample = out_stream.read();
   	        total_samples++;


      	        // DVB-T COMPLIANT: TLAST should be set only on the LAST sample (1151)
      	        if (sample.last) {
      	            last_count++;
      	            if (i != N_FFT - 1) {
      	                printf("  FAIL: TLAST set on wrong sample: %d, expected %d\n", i, N_FFT-1);
      	                return 1;
      	            }
      	        }

      	        // Data should be in order 0 to 1023
      	        if (sample.i != i) {
      	            printf("  FAIL: Data sample %d has wrong I value: got %d, expected %d\n",
      	                   i, (int)sample.i, i);
      	            return 1;
      	        }
      	    }

      	    printf("  Total samples output: %d (expected %d)\n", total_samples, SYMBOL_LEN);
      	    printf("  TLAST count: %d (should be 1)\n", last_count);

      	    if (total_samples != SYMBOL_LEN) {
      	        printf("  FAIL: Wrong number of output samples\n");
      	        return 1;
      	    }

      	    if (last_count != 1) {
      	        printf("  FAIL: TLAST should appear exactly once\n");
      	        return 1;
      	    }

      	    printf("  PASS: CP Insert TLAST propagation test\n\n");
      	    return 0;
      	}

      	//============================================================================
      	// Test 3: CP Removal TLAST Propagation
      	// Tests that cp_removal correctly handles TLAST from upstream
      	//============================================================================
      	int test_cp_removal_tlast() {
      	    printf("=== Test 3: CP Removal TLAST Propagation ===\n");

      	    hls::stream<axis_cmpx_t> in_stream("cp_rem_in");
      	    hls::stream<axis_cmpx_t> out_stream("cp_rem_out");

      	    // Generate 1152 input samples (CP + OFDM symbol)
      	    // First 128 are CP (not real FFT data, just placeholders)
      	    for (int i = 0; i < SYMBOL_LEN; i++) {
      	        axis_cmpx_t sample;
     	        sample.i = i;
      	        sample.q = -i;
      	        // Set last only on the very last sample (1151)
      	        sample.last = (i == SYMBOL_LEN - 1);
      	        in_stream.write(sample);
      	    }

      	    // Call cp_removal
      	    cp_removal(in_stream, out_stream);

      	    // Drain output: should get 1024 samples (FFT data only)
      	    int last_count = 0;
      	    int total_samples = 0;

      	    for (int i = 0; i < N_FFT; i++) {
      	        if (out_stream.empty()) break;
      	        axis_cmpx_t sample = out_stream.read();
      	        total_samples++;

      	        // DVB-T COMPLIANT: TLAST should be set on the last FFT sample (1023)
      	        if (sample.last) {
      	            last_count++;
      	            if (i != N_FFT - 1) {
      	                printf("  FAIL: TLAST set on wrong FFT sample: %d, expected %d\n", i, N_FFT-1);
     	                return 1;
      	            }
      	        }

      	        // After CP removal, data should start from sample 128 (original index)
      	        // So output sample 0 should have i=128, output sample 1 -> i=129, etc.
      	        int expected_i = N_CP + i;
      	        if (sample.i != expected_i) {
      	            printf("  FAIL: Output sample %d has wrong I: got %d, expected %d\n",
      	                   i, (int)sample.i, expected_i);
      	            return 1;
      	        }
      	    }

      	    printf("  Total samples output: %d (expected %d)\n", total_samples, N_FFT);
      	    printf("  TLAST count: %d (should be 1)\n", last_count);

      	    if (total_samples != N_FFT) {
      	        printf("  FAIL: Wrong number of output samples\n");
      	        return 1;
      	    }

      	    if (last_count != 1) {
      	        printf("  FAIL: TLAST should appear exactly once\n");
      	        return 1;
      	    }

      	    printf("  PASS: CP Removal TLAST propagation test\n\n");
      	    return 0;
      	}

      	//============================================================================
      	// Test 4: Data Flow Integrity
      	// Tests that data passes through unchanged (except CP operations)
      	//============================================================================
      	int test_data_integrity() {
      	    printf("=== Test 4: Data Flow Integrity ===\n");

      	    // Test through cp_insert -> cp_remove (round trip)
      	    hls::stream<axis_cmpx_t> in_stream("test_in");
      	    hls::stream<axis_cmpx_t> mid_stream("test_mid");
      	    hls::stream<axis_cmpx_t> out_stream("test_out");

      	    // Create known test pattern
      	    for (int i = 0; i < N_FFT; i++) {
      	        axis_cmpx_t sample;
      	        sample.i = i * 2;  // Even numbers
      	        sample.q = i * 2 + 1;  // Odd numbers
      	        sample.last = (i == N_FFT - 1);
      	        in_stream.write(sample);
      	    }

      	    // Insert CP
      	    cp_insert(in_stream, mid_stream);

      	    // Remove CP
      	    cp_removal(mid_stream, out_stream);

      	    // Verify data integrity
      	    int errors = 0;
      	    for (int i = 0; i < N_FFT; i++) {
      	        axis_cmpx_t sample = out_stream.read();

      	        int expected_i = i * 2;
      	        int expected_q = i * 2 + 1;

      	        if (sample.i != expected_i || sample.q != expected_q) {
      	            printf("  FAIL: Sample %d: I got %d exp %d, Q got %d exp %d\n",
      	                   i, (int)sample.i, expected_i, (int)sample.q, expected_q);
      	            errors++;
      	        }
      	    }

      	    if (errors == 0) {
      	        printf("  PASS: Data integrity test (all %d samples correct)\n\n", N_FFT);
      	        return 0;
      	    } else {
      	        printf("  FAIL: %d data errors\n\n", errors);
      	        return errors;
      	    }
      	}

      	//============================================================================
      	// Test 5: Multiple OFDM Symbols
      	// Tests handling of multiple consecutive symbols
      	//============================================================================
      	int test_multiple_symbols() {
      	    printf("=== Test 5: Multiple OFDM Symbols ===\n");

      	    hls::stream<axis_cmpx_t> in_stream("multi_in");
      	    hls::stream<axis_cmpx_t> out_stream("multi_out");

      	    int num_symbols = 3;

      	    // Generate multiple OFDM symbols
      	    for (int s = 0; s < num_symbols; s++) {
      	        for (int i = 0; i < N_FFT; i++) {
      	            axis_cmpx_t sample;
      	            sample.i = s * 1000 + i;  // Unique pattern per symbol
      	            sample.q = s * 1000 + i + 500;
      	            sample.last = (i == N_FFT - 1);
      	            in_stream.write(sample);
      	        }
      	    }

      	    // Insert CP for each symbol
      	    for (int s = 0; s < num_symbols; s++) {
      	        cp_insert(in_stream, out_stream);
      	    }

      	    // Verify each symbol is properly framed
      	    int symbol_count = 0;
      	    int total_samples = 0;

      	    while (!out_stream.empty()) {
      	        axis_cmpx_t sample = out_stream.read();
      	        total_samples++;

      	        // Check if this is the last sample of a symbol
      	        if (sample.last) {
      	            symbol_count++;
      	            printf("  Symbol %d complete at sample %d\n", symbol_count, total_samples);
      	        }
      	    }

      	    int expected_total = num_symbols * SYMBOL_LEN;

         	    printf("  Total samples: %d (expected %d)\n", total_samples, expected_total);
         	    printf("  Symbols detected: %d (expected %d)\n", symbol_count, num_symbols);

         	    if (total_samples != expected_total) {
         	        printf("  FAIL: Wrong number of samples\n");
         	        return 1;
         	    }

         	    if (symbol_count != num_symbols) {
         	        printf("  FAIL: Wrong number of symbols detected\n");
         	        return 1;
         	    }

         	    printf("  PASS: Multiple symbols test\n\n");
         	    return 0;
         	}

         	//============================================================================
         	// Main Test Driver
         	//============================================================================
         	int main() {
         	    printf("================================================================\n");
         	    printf("DVB-T Compliance Testbench\n");
         	    printf("Testing HLS COFDM code fixes for DVB-T compliance\n");
         	    printf("================================================================\n\n");

         	    int total_errors = 0;

         	    total_errors += test_type_definitions();
         	    total_errors += test_cp_insert_tlast();
         	    total_errors += test_cp_removal_tlast();
         	    total_errors += test_data_integrity();
         	    total_errors += test_multiple_symbols();

         	    printf("================================================================\n");
         	    if (total_errors == 0) {
         	        printf("ALL TESTS PASSED - DVB-T COMPLIANT\n");
         	    } else {
         	        printf("TESTS FAILED: %d errors found\n", total_errors);
         	    }
         	    printf("================================================================\n");

         	    return total_errors;
         	}
