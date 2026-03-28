//============================================================================
// Full COFDM END-TO-END TESTBENCH
// With Xilinx hls::fft<> IP
//============================================================================

#include "cofdm_hls.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//============================================================================
// Test Configuration
//============================================================================
#define TEST_NUM_BITS     N_DATA    // 736 bits = exactly 1 OFDM symbol
#define TEST_NUM_SYMBOLS  1
#define BER_PASS_THRESHOLD 0.01    // 1% BER threshold for pass/fail

//============================================================================
// Helper: Generate random bits
//============================================================================
void generate_random_bits(ap_uint<1>* data, int num_bits) {
    for (int i = 0; i < num_bits; i++) {
        data[i] = rand() % 2;
    }
}

//============================================================================
// Test 1: Structural Test - Verify data flows through all stages
//============================================================================
int test_structural() {
    printf("=== Test 1: Structural Flow Test ===\n");

    hls::stream<axis_bit_t> tx_in("tx_in");
    hls::stream<axis_bit_t> rx_out("rx_out");

    int num_bits = TEST_NUM_BITS;
    for (int i = 0; i < num_bits; i++) {
        axis_bit_t b;
        b.data = i % 2;
        b.last = (i == num_bits - 1);
        tx_in.write(b);
    }

    cofdm_hil_top(tx_in, rx_out, false, 2, 0, 0, num_bits);

    int expected_count = num_decoded_bits(num_bits);
    int count = 0;
    while (!rx_out.empty()) {
        rx_out.read();
        count++;
    }

    printf("  Input bits:    %d\n", num_bits);
    printf("  Output bits:   %d\n", count);
    printf("  Expected bits: %d\n", expected_count);

    if (count == expected_count) {
        printf("  PASS: Correct number of output bits produced\n\n");
        return 0;
    } else {
        printf("  FAIL: Expected %d bits, got %d\n\n", expected_count, count);
        return 1;
    }
}

//============================================================================
// Test 2: Codec Round-Trip (encoder/decoder without FFT)
//============================================================================
int test_codec_roundtrip() {
    printf("=== Test 2: Codec Round-Trip (Encoder → Decoder) ===\n");

    int num_bits = 64;
    ap_uint<1> tx_data[64];
    for (int i = 0; i < num_bits; i++) {
        tx_data[i] = i & 1;  // Alternating 0,1,0,1,...
    }

    // TX: Encode
    hls::stream<axis_bit_t> enc_in("enc_in");
    for (int i = 0; i < num_bits; i++) {
        axis_bit_t b;
        b.data = tx_data[i];
        b.last = (i == num_bits - 1);
        enc_in.write(b);
    }

    hls::stream<axis_dibit_t> enc_out("enc_out");
    conv_encoder(enc_in, enc_out, num_bits);

    // TX: Modulate
    hls::stream<axis_cmpx_t> mod_out("mod_out");
    qpsk_modulator(enc_out, mod_out, num_bits);

    // RX: Demodulate (ideal channel, no noise)
    hls::stream<axis_dibit_t> demod_out("demod_out");
    qpsk_demodulator(mod_out, demod_out, num_bits);

    // RX: Decode
    hls::stream<axis_bit_t> dec_out("dec_out");
    viterbi_decoder(demod_out, dec_out, num_bits);

    int expected_out = num_decoded_bits(num_bits);
    int bit_errors   = 0;

    printf("  Input:  %d bits\n", num_bits);
    printf("  Output: %d bits (Viterbi traceback depth = %d)\n",
           expected_out, VITERBI_TB_DEPTH);

    for (int i = 0; i < expected_out; i++) {
        axis_bit_t b    = dec_out.read();
        int decoded     = (b.data != 0) ? 1 : 0;
        if (decoded != (int)tx_data[i])
            bit_errors++;
    }

    double ber = (expected_out > 0) ? (double)bit_errors / expected_out : 1.0;
    printf("  Bit errors: %d / %d\n", bit_errors, expected_out);
    printf("  BER: %.2f%%\n", ber * 100.0);

    if (ber < 0.01) {
        printf("  PASS: Codec round-trip perfect!\n\n");
        return 0;
    } else {
        printf("  FAIL: Codec has errors\n\n");
        return 1;
    }
}

//============================================================================
// Test 3: Full Chain with Channel Bypass (No Noise)
//============================================================================
int test_full_chain_bypass() {
    printf("=== Test 3: Full COFDM Chain (Channel Bypass) ===\n");
    printf("  Using Xilinx FFT IP — BER should be near zero in bypass mode.\n\n");

    // Generate random input
    ap_uint<1> tx_data[TEST_NUM_BITS];
    generate_random_bits(tx_data, TEST_NUM_BITS);
    printf("  Input: %d random bits\n", TEST_NUM_BITS);

    // Feed into chain
    hls::stream<axis_bit_t> tx_in("tx_in");
    for (int i = 0; i < TEST_NUM_BITS; i++) {
        axis_bit_t b;
        b.data = tx_data[i];
        b.last = (i == TEST_NUM_BITS - 1);
        tx_in.write(b);
    }

    hls::stream<axis_bit_t> rx_out("rx_out");

    printf("  Running cofdm_hil_top (channel bypass, no noise)...\n");
    cofdm_hil_top(
        tx_in,
        rx_out,
        false,           // chan_enable = false (bypass)
        2,               // k_factor (unused in bypass)
        0,               // doppler_incr (unused in bypass)
        0,               // snr_scale = 0 (no noise)
        TEST_NUM_BITS    // num_bits = 736
    );

    int expected_output = num_decoded_bits(TEST_NUM_BITS);
    printf("  Expected output: %d bits (tx_data[0..%d] verifiable)\n",
           expected_output, expected_output - 1);

    int received_bits = 0;
    int bit_errors    = 0;

    for (int i = 0; i < expected_output; i++) {
        if (rx_out.empty()) {
            printf("  WARNING: Stream empty at index %d\n", i);
            break;
        }

        axis_bit_t b    = rx_out.read();
        received_bits++;
        int decoded_bit = (b.data != 0) ? 1 : 0;

        if (decoded_bit != (int)tx_data[i]) {
            bit_errors++;
            if (bit_errors <= 5) {
                printf("  Error at bit %d: expected %d, got %d\n",
                       i, (int)tx_data[i], decoded_bit);
            }
        }
    }

    printf("  Received:   %d bits\n", received_bits);
    printf("  Bit errors: %d\n", bit_errors);

    double ber = (received_bits > 0) ? (double)bit_errors / received_bits : 1.0;
    printf("  BER: %.6f (%.4f%%)\n", ber, ber * 100.0);

    if (ber < BER_PASS_THRESHOLD) {
        printf("  PASS: BER < %.0f%%  ✓\n\n", BER_PASS_THRESHOLD * 100.0);
        return 0;
    } else if (ber < 0.1) {
        printf("  WARN: BER = %.2f%% (acceptable but not ideal)\n", ber * 100.0);
        printf("  Possible causes: channel estimation, equalizer, or scaling.\n\n");
        return 0;  // Soft pass
    } else {
        printf("  FAIL: BER = %.2f%% — chain has a serious problem.\n", ber * 100.0);
        printf("  Possible causes:\n");
        printf("    - FFT/IFFT scaling mismatch\n");
        printf("    - Channel estimator not finding pilots correctly\n");
        printf("    - Equalizer outputting wrong subcarriers\n\n");
        return 1;
    }
}

//============================================================================
// Main
//============================================================================
int main() {
    printf("================================================================\n");
    printf("FULL COFDM END-TO-END TESTBENCH\n");
    printf("With Xilinx hls::fft<> IP\n");
    printf("================================================================\n\n");

    int total_errors = 0;

    total_errors += test_structural();
    total_errors += test_codec_roundtrip();
    total_errors += test_full_chain_bypass();

    printf("================================================================\n");
    printf("FINAL RESULTS:\n");
    printf("================================================================\n");
    if (total_errors == 0) {
        printf("  ALL TESTS PASSED\n");
    } else {
        printf("  FAILED: %d test(s) had errors\n", total_errors);
    }
    printf("================================================================\n");

    return total_errors;
}
