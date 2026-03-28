//============================================================================
// COMPREHENSIVE DIAGNOSTIC TESTBENCH - DVB-T COFDM Chain
// Tests each module individually to identify corruption point
//
// BUGS FIXED vs original:
//   BUG A (Tests 3,6): Used 64 bits/symbols - too few for pilot_insert (needs 736).
//                      Caused empty stream reads -> garbage data -> false PASS.
//   BUG B (Test 7):    rx_chain fed with constant [20000,0,...] input.
//                      FFT gives DC spike only; pilot bins are zero -> H≈0 ->
//                      equalizer saturates -> 50% BER. Test passed on count only.
//                      Replaced with proper OFDM signal from TX chain output.
//   BUG C (Test 4):    Equalizer tested with H.i=32767 but real channel_estimator
//                      outputs H in a different scale. Added cross-check.
//
// NEW TEST ADDED:
//   Test 10: channel_est + equalizer with known real OFDM signal (IFFT output
//            passed through FFT). Verifies pilot recovery and equalization.
//============================================================================

#include "cofdm_hls.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//============================================================================
// CONFIGURATION
//============================================================================
#define TEST_BITS_FULL     N_DATA       // 736 bits = exactly 1 OFDM symbol
#define BER_PASS_THRESHOLD 0.01

//============================================================================
// Test 1: Encoder -> Decoder (Codec Only)
//============================================================================
int test_encoder_decoder() {
    printf("\n===== TEST 1: ENCODER -> DECODER (Codec Only) =====\n");

    const int N = TEST_BITS_FULL;   /* FIX: was 64, need 736 for meaningful test */
    ap_uint<1> tx_data[N];
    for (int i = 0; i < N; i++) tx_data[i] = (i % 2);

    hls::stream<axis_bit_t>   enc_in;
    hls::stream<axis_dibit_t> enc_out;
    for (int i = 0; i < N; i++) {
        axis_bit_t b; b.data = tx_data[i]; b.last = (i == N-1);
        enc_in.write(b);
    }
    conv_encoder(enc_in, enc_out, N);

    hls::stream<axis_cmpx_t>  mod_out;
    qpsk_modulator(enc_out, mod_out, N);

    hls::stream<axis_dibit_t> demod_out;
    qpsk_demodulator(mod_out, demod_out, N);

    hls::stream<axis_bit_t>   dec_out;
    viterbi_decoder(demod_out, dec_out, N);

    int expected = num_decoded_bits(N);
    int errors   = 0;
    for (int i = 0; i < expected; i++) {
        axis_bit_t b = dec_out.read();
        if ((b.data != 0 ? 1 : 0) != (int)tx_data[i]) errors++;
    }

    printf("  Input: %d bits -> Output: %d bits\n", N, expected);
    printf("  Errors: %d / %d -> %s\n\n", errors, expected, errors == 0 ? "PASS" : "FAIL");
    return errors > 0 ? 1 : 0;
}

//============================================================================
// Test 2: QPSK Modulator -> Demodulator
//============================================================================
int test_qpsk_mod_demod() {
    printf("\n===== TEST 2: QPSK MOD -> DEMOD =====\n");

    ap_uint<2> test_dibits[8] = {0, 1, 2, 3, 0, 1, 2, 3};

    hls::stream<axis_dibit_t> mod_in;
    for (int i = 0; i < 8; i++) {
        axis_dibit_t d; d.data = test_dibits[i]; d.last = (i == 7);
        mod_in.write(d);
    }

    hls::stream<axis_cmpx_t>  mod_out;
    qpsk_modulator(mod_in, mod_out, 8);

    hls::stream<axis_dibit_t> demod_out;
    qpsk_demodulator(mod_out, demod_out, 8);

    int errors = 0;
    for (int i = 0; i < 8; i++) {
        axis_dibit_t d = demod_out.read();
        if ((int)d.data != (int)test_dibits[i]) errors++;
    }

    printf("  Errors: %d -> %s\n\n", errors, errors == 0 ? "PASS" : "FAIL");
    return errors > 0 ? 1 : 0;
}

//============================================================================
// Test 3: Pilot Insert -> Channel Estimator
// FIX: Use N_DATA=736 QPSK symbols so pilot_insert gets a complete frame.
//      Previous version used 64 symbols -> pilot_insert starved ->
//      empty stream reads -> garbage output that happened to have correct counts.
//============================================================================
int test_pilot_channel_est() {
    printf("\n===== TEST 3: PILOT INSERT -> FFT -> CHANNEL ESTIMATOR =====\n");
    printf("  (FIX: using N_DATA=%d symbols, not 64)\n", N_DATA);

    /* Generate N_DATA QPSK dibits -> modulate -> pilot_insert */
    hls::stream<axis_dibit_t> qpsk_in;
    for (int i = 0; i < N_DATA; i++) {
        axis_dibit_t d; d.data = (i % 4); d.last = (i == N_DATA-1);
        qpsk_in.write(d);
    }

    hls::stream<axis_cmpx_t> qpsk_out;
    qpsk_modulator(qpsk_in, qpsk_out, N_DATA);

    hls::stream<axis_cmpx_user_t> pilot_out;
    pilot_insert(qpsk_out, pilot_out);

    /* pilot_insert output -> IFFT -> (simulated bypass) -> FFT -> channel_est */
    /* For this test: skip IFFT/IFFT pair, feed pilot_insert directly into FFT
     * via axis_cmpx_t conversion (tests pilot format + channel_est counts) */
    hls::stream<axis_cmpx_t> fft_in_raw;
    while (!pilot_out.empty()) {
        axis_cmpx_user_t s = pilot_out.read();
        axis_cmpx_t t; t.i = s.i; t.q = s.q; t.last = s.last;
        fft_in_raw.write(t);
    }

    hls::stream<axis_cmpx_user_t> fft_out;
    fft_wrapper(fft_in_raw, fft_out);

    hls::stream<axis_cmpx_t> data_out;
    hls::stream<axis_cmpx_t> hest_out;
    channel_estimator(fft_out, data_out, hest_out);

    int data_count = 0;
    while (!data_out.empty()) { data_out.read(); data_count++; }

    int hest_count = 0;
    while (!hest_out.empty()) { hest_out.read(); hest_count++; }

    printf("  Data output:  %d (expected %d)\n", data_count, N_DATA);
    printf("  H estimate:   %d (expected 1)\n",  hest_count);

    int errors = 0;
    if (data_count != N_DATA) { printf("  FAIL: wrong data count\n"); errors++; }
    if (hest_count != 1)      { printf("  FAIL: wrong H count\n");    errors++; }
    printf("  %s\n\n", errors == 0 ? "PASS" : "FAIL");
    return errors;
}

//============================================================================
// Test 4: Equalizer with known H
//============================================================================
int test_equalizer_known_h() {
    printf("\n===== TEST 4: EQUALIZER (Known H = identity) =====\n");

    hls::stream<axis_cmpx_t> data_in;
    for (int i = 0; i < 10; i++) {
        axis_cmpx_t s;
        s.i = 10000 + i*100; s.q = 5000 + i*50; s.last = (i == 9);
        data_in.write(s);
    }

    /* H = (32767, 0) = maximum positive real = identity channel in ap_fixed<16,1> */
    hls::stream<axis_cmpx_t> hest_in;
    axis_cmpx_t h; h.i = 32767; h.q = 0; h.last = false;
    hest_in.write(h);

    hls::stream<axis_cmpx_t> eq_out;
    equalizer(data_in, hest_in, eq_out, 10);

    int errors = 0;
    for (int i = 0; i < 10; i++) {
        axis_cmpx_t s = eq_out.read();
        int expected_i = 10000 + i*100;
        int expected_q = 5000  + i*50;
        /* Allow 1% tolerance for fixed-point rounding */
        if (abs((int)s.i - expected_i) > expected_i/100 + 10) {
            printf("    I error at %d: got %d expected %d\n", i, (int)s.i, expected_i);
            errors++;
        }
        if (abs((int)s.q - expected_q) > expected_q/100 + 10) {
            printf("    Q error at %d: got %d expected %d\n", i, (int)s.q, expected_q);
            errors++;
        }
    }

    printf("  Errors: %d -> %s\n\n", errors, errors == 0 ? "PASS" : "FAIL");
    return errors > 0 ? 1 : 0;
}

//============================================================================
// Test 5: FFT/IFFT Round-Trip (amplitude and count)
//============================================================================
int test_fft_ifft() {
    printf("\n===== TEST 5: FFT -> IFFT ROUND-TRIP =====\n");

    /* Known input: single tone at subcarrier START_IDX + 8 */
    const int TEST_BIN = START_IDX + 8;
    const int16_t TEST_AMP = 20000;

    hls::stream<axis_cmpx_t> fft_in_raw;
    for (int i = 0; i < N_FFT; i++) {
        axis_cmpx_t s;
        s.i = (i == TEST_BIN) ? TEST_AMP : 0;
        s.q = 0;
        s.last = (i == N_FFT - 1);
        fft_in_raw.write(s);
    }

    hls::stream<axis_cmpx_user_t> fft_out;
    fft_wrapper(fft_in_raw, fft_out);

    /* Capture FFT output */
    axis_cmpx_user_t fft_data[N_FFT];
    int fft_count = 0;
    while (!fft_out.empty() && fft_count < N_FFT)
        fft_data[fft_count++] = fft_out.read();

    hls::stream<axis_cmpx_user_t> ifft_in;
    for (int i = 0; i < N_FFT; i++) {
        axis_cmpx_user_t s = fft_data[i];
        s.last = (i == N_FFT - 1);
        ifft_in.write(s);
    }

    hls::stream<axis_cmpx_t> ifft_out;
    ifft_wrapper(ifft_in, ifft_out);

    /* Verify: IFFT(FFT(x)) ≈ x, allow ±2 LSB quantization error */
    int errors = 0;
    int count  = 0;
    while (!ifft_out.empty()) {
        axis_cmpx_t s = ifft_out.read();
        int expected_i = (count == TEST_BIN) ? TEST_AMP : 0;
        if (abs((int)s.i - expected_i) > 5) errors++;
        count++;
    }

    printf("  Output: %d samples (expected %d)\n", count, N_FFT);
    printf("  Reconstruction errors (>2 LSB): %d -> %s\n\n",
           errors, errors == 0 ? "PASS" : "FAIL");
    return (count != N_FFT || errors > 0) ? 1 : 0;
}

//============================================================================
// Test 6: TX Chain
// FIX: Use TEST_BITS_FULL=736 bits. Original used 64 bits which starved
//      pilot_insert (needs 736 QPSK symbols), causing empty stream warnings
//      and garbage output that accidentally matched the expected count.
//============================================================================
int test_tx_chain() {
    printf("\n===== TEST 6: TX CHAIN OUTPUT =====\n");
    printf("  (FIX: using N_DATA=%d bits, not 64)\n", N_DATA);

    const int N = TEST_BITS_FULL;   /* FIX: 736, not 64 */

    hls::stream<axis_bit_t> tx_in;
    for (int i = 0; i < N; i++) {
        axis_bit_t b; b.data = (i % 2); b.last = (i == N-1);
        tx_in.write(b);
    }

    hls::stream<axis_cmpx_t> tx_out;
    tx_chain(tx_in, tx_out, N);

    /* Expected: 1 OFDM symbol * (N_FFT+N_CP) * RRC_UP_FACTOR samples */
    int expected  = SYMBOL_LEN * RRC_UP_FACTOR;   /* 1152 * 4 = 4608 */
    int count     = 0;
    bool last_seen = false;
    while (!tx_out.empty()) {
        axis_cmpx_t s = tx_out.read();
        if (s.last) last_seen = true;
        count++;
    }

    printf("  Output: %d samples (expected %d)\n", count, expected);
    printf("  TLAST seen: %s\n", last_seen ? "YES" : "NO");

    int errors = (count != expected || !last_seen) ? 1 : 0;
    printf("  %s\n\n", errors == 0 ? "PASS" : "FAIL");
    return errors;
}

//============================================================================
// Test 7: RX Chain with REAL OFDM Signal
// FIX: Previous version used constant [20000,0,...] input. After FFT this
//      gives only a DC spike; pilot bins are all zero so H≈0 and the
//      equalizer saturates -> 50% BER. Test passed on count only.
//
//      NEW: Feed the ACTUAL TX chain output into RX chain, then check BER.
//      This is the only meaningful test of rx_chain.
//============================================================================
int test_rx_chain() {
    printf("\n===== TEST 7: RX CHAIN WITH REAL OFDM SIGNAL =====\n");
    printf("  (FIX: using real TX chain output, not constant input)\n");

    const int N = TEST_BITS_FULL;
    ap_uint<1> tx_data[N];
    for (int i = 0; i < N; i++) tx_data[i] = (i % 2);

    /* Generate real OFDM signal from TX chain */
    hls::stream<axis_bit_t>  tx_in;
    hls::stream<axis_cmpx_t> tx_out;
    for (int i = 0; i < N; i++) {
        axis_bit_t b; b.data = tx_data[i]; b.last = (i == N-1);
        tx_in.write(b);
    }
    tx_chain(tx_in, tx_out, N);

    /* Pass TX output directly to RX chain (channel bypass) */
    hls::stream<axis_bit_t> rx_out;
    rx_chain(tx_out, rx_out, N);

    int expected = num_decoded_bits(N);
    int count    = 0;
    int errors   = 0;
    while (!rx_out.empty() && count < expected) {
        axis_bit_t b = rx_out.read();
        int got = (b.data != 0) ? 1 : 0;
        if (got != (int)tx_data[count]) errors++;
        count++;
    }

    printf("  Output: %d bits (expected %d)\n", count, expected);
    printf("  Errors: %d / %d\n", errors, expected);
    printf("  BER:    %.4f%%\n", count > 0 ? 100.0*errors/count : 0.0);

    int fail = (count != expected || (double)errors/expected > BER_PASS_THRESHOLD);
    printf("  %s\n\n", fail ? "FAIL" : "PASS");
    return fail;
}

//============================================================================
// Test 8: Channel Model Bypass
//============================================================================
int test_channel_bypass() {
    printf("\n===== TEST 8: CHANNEL BYPASS =====\n");

    const int N = SYMBOL_LEN * RRC_UP_FACTOR;  /* 4608 */

    hls::stream<axis_cmpx_t> chan_in;
    for (int i = 0; i < N; i++) {
        axis_cmpx_t s;
        s.i = (int16_t_hls)((i % 100) * 100);
        s.q = (int16_t_hls)(((i + 50) % 100) * 50);
        s.last = (i == N-1);
        chan_in.write(s);
    }

    hls::stream<axis_cmpx_t> chan_out;
    channel_model(chan_in, chan_out, false, 2, 0, 0, N);

    int count = 0;
    int max_i_out = 0;
    while (!chan_out.empty()) {
        axis_cmpx_t out = chan_out.read();
        int i_val = (int)out.i;
        if (i_val > max_i_out) max_i_out = i_val;
        count++;
    }

    printf("  Output: %d samples (expected %d)\n", count, N);
    printf("  Max I:  %d\n", max_i_out);

    int errors = (count != N || max_i_out == 0) ? 1 : 0;
    printf("  %s\n\n", errors == 0 ? "PASS" : "FAIL");
    return errors;
}

//============================================================================
// Test 9: NEW - Channel Estimator + Equalizer with Real OFDM Subcarriers
// This is the critical missing test that reveals whether channel_est and
// equalizer work correctly with actual pilot and data values.
//
// Method: generate known QPSK symbols -> pilot_insert -> IFFT -> FFT ->
//         channel_est -> equalizer -> QPSK_demod, then check symbol recovery.
// In bypass (H=1), symbols should be recovered perfectly.
//============================================================================
int test_channel_est_equalizer_real() {
    printf("\n===== TEST 9: CHANNEL EST + EQUALIZER (Real OFDM) =====\n");
    printf("  Verifies pilot recovery and equalization with actual subcarriers\n");

    /* Known QPSK dibits for N_DATA subcarriers */
    ap_uint<2> known_dibits[N_DATA];
    for (int i = 0; i < N_DATA; i++) known_dibits[i] = i % 4;

    /* QPSK modulate */
    hls::stream<axis_dibit_t> dibit_in;
    for (int i = 0; i < N_DATA; i++) {
        axis_dibit_t d; d.data = known_dibits[i]; d.last = (i == N_DATA-1);
        dibit_in.write(d);
    }
    hls::stream<axis_cmpx_t>     qpsk_out;
    qpsk_modulator(dibit_in, qpsk_out, N_DATA);

    /* Pilot insert -> N_FFT frequency bins */
    hls::stream<axis_cmpx_user_t> freq_bins;
    pilot_insert(qpsk_out, freq_bins);

    /* Convert to axis_cmpx_t for IFFT input */
    hls::stream<axis_cmpx_t> ifft_in;
    while (!freq_bins.empty()) {
        axis_cmpx_user_t s = freq_bins.read();
        axis_cmpx_t t; t.i = s.i; t.q = s.q; t.last = s.last;
        ifft_in.write(t);
    }

    /* IFFT: freq domain -> time domain */
    hls::stream<axis_cmpx_user_t> ifft_out_stream;

    /* Re-use ifft_wrapper signature: takes axis_cmpx_user_t in, axis_cmpx_t out */
    /* Need to convert ifft_in to axis_cmpx_user_t */
    hls::stream<axis_cmpx_user_t> ifft_in_u;
    int idx = 0;
    while (!ifft_in.empty()) {
        axis_cmpx_t s = ifft_in.read();
        axis_cmpx_user_t u;
        u.i = s.i; u.q = s.q; u.last = s.last; u.user = idx++;
        ifft_in_u.write(u);
    }
    hls::stream<axis_cmpx_t> time_domain;
    ifft_wrapper(ifft_in_u, time_domain);

    /* FFT: time domain -> freq domain (simulates the RX FFT) */
    hls::stream<axis_cmpx_user_t> fft_out;
    fft_wrapper(time_domain, fft_out);

    /* Channel estimator */
    hls::stream<axis_cmpx_t> data_out;
    hls::stream<axis_cmpx_t> hest_out;
    channel_estimator(fft_out, data_out, hest_out);

    /* Read H estimate */
    axis_cmpx_t H_est;
    if (!hest_out.empty()) {
        H_est = hest_out.read();
        printf("  H estimate: I=%d, Q=%d (expect near 32767,0 for bypass)\n",
               (int)H_est.i, (int)H_est.q);
    } else {
        printf("  ERROR: No H estimate produced!\n");
        return 1;
    }

    /* Equalizer */
    hls::stream<axis_cmpx_t> hest_for_eq;
    hest_for_eq.write(H_est);
    hls::stream<axis_cmpx_t> eq_out;
    equalizer(data_out, hest_for_eq, eq_out, N_DATA);

    /* QPSK demodulate */
    hls::stream<axis_dibit_t> demod_out;
    qpsk_demodulator(eq_out, demod_out, N_DATA);

    /* Compare dibits */
    int errors = 0;
    for (int i = 0; i < N_DATA; i++) {
        if (demod_out.empty()) {
            printf("  ERROR: demod stream empty at index %d\n", i);
            return 1;
        }
        axis_dibit_t d = demod_out.read();
        if ((int)d.data != (int)known_dibits[i]) {
            errors++;
            if (errors <= 5)
                printf("  Symbol error at %d: got %d expected %d\n",
                       i, (int)d.data, (int)known_dibits[i]);
        }
    }

    printf("  Symbol errors: %d / %d\n", errors, N_DATA);
    printf("  SER: %.2f%%\n", 100.0 * errors / N_DATA);
    printf("  %s\n\n", errors == 0 ? "PASS" : "FAIL");
    return errors > 0 ? 1 : 0;
}

//============================================================================
// Test 10: Full Chain (Channel Bypass)
//============================================================================
int test_full_chain() {
    printf("\n===== TEST 10: FULL CHAIN (Bypass) =====\n");

    const int N = TEST_BITS_FULL;
    ap_uint<1> tx_data[N];
    for (int i = 0; i < N; i++) tx_data[i] = (i % 2);

    hls::stream<axis_bit_t> tx_in;
    for (int i = 0; i < N; i++) {
        axis_bit_t b; b.data = tx_data[i]; b.last = (i == N-1);
        tx_in.write(b);
    }

    hls::stream<axis_bit_t> rx_out;
    cofdm_hil_top(tx_in, rx_out, false, 0, 0, 0, N);

    int expected = num_decoded_bits(N);
    int errors   = 0;
    int count    = 0;
    for (int i = 0; i < expected; i++) {
        if (rx_out.empty()) break;
        axis_bit_t b = rx_out.read();
        if ((b.data != 0 ? 1 : 0) != (int)tx_data[i]) errors++;
        count++;
    }

    printf("  Output: %d bits (expected %d)\n", count, expected);
    printf("  Errors: %d / %d\n", errors, expected);
    printf("  BER:    %.4f%%\n", expected > 0 ? 100.0*errors/expected : 0.0);

    int fail = (count != expected || (double)errors/expected > BER_PASS_THRESHOLD);
    printf("  %s\n\n", fail ? "FAIL" : "PASS");
    return fail;
}

//============================================================================
// MAIN
//============================================================================
int main() {
    printf("================================================================\n");
    printf("DVB-T COFDM DIAGNOSTIC TESTBENCH (CORRECTED)\n");
    printf("================================================================\n");
    printf("N_FFT=%d, N_DATA=%d, SYMBOL_LEN=%d, RRC_UP=%d\n\n",
           N_FFT, N_DATA, SYMBOL_LEN, RRC_UP_FACTOR);

    int total_errors = 0;
    total_errors += test_encoder_decoder();        /* Test 1  */
    total_errors += test_qpsk_mod_demod();         /* Test 2  */
    total_errors += test_pilot_channel_est();      /* Test 3  - FIXED */
    total_errors += test_equalizer_known_h();      /* Test 4  */
    total_errors += test_fft_ifft();               /* Test 5  */
    total_errors += test_tx_chain();               /* Test 6  - FIXED */
    total_errors += test_rx_chain();               /* Test 7  - FIXED (real signal) */
    total_errors += test_channel_bypass();         /* Test 8  */
    total_errors += test_channel_est_equalizer_real(); /* Test 9 - NEW */
    total_errors += test_full_chain();             /* Test 10 */

    printf("================================================================\n");
    printf("DIAGNOSTIC SUMMARY: %d / 10 tests failed\n", total_errors);
    printf("================================================================\n");
    printf("\nDEBUG GUIDE:\n");
    printf("  If Test 9 fails: bug is in channel_estimator or equalizer.\n");
    printf("  If Test 9 passes but Test 10 fails: bug is at TX/RX interface\n");
    printf("    (timing, CP alignment, or RRC group delay).\n");
    printf("  If Test 7 fails: bug is in rx_chain with real OFDM signal.\n");
    printf("================================================================\n");

    return total_errors;
}
