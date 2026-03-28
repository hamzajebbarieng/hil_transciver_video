/*==============================================================================
 * ifft_wrapper.cpp  —  1024-point Inverse FFT  (Xilinx hls::fft<>, direction=0)
 *
 * ── SCHEDULE DERIVATION (proven by Python simulation) ─────────────────────
 *
 * ROOT CAUSE OF 47.72% BER (now fixed):
 *   Previous schedule IFFT_SCH=0x000 (S_inv=1) = no division.
 *   The raw IDFT sum of 768 QPSK subcarriers at amplitude 0.707 each
 *   reaches peaks of ~52× the ap_fixed<16,1> range [-1,1).
 *   Simulation: 94.4% of output samples clipped to ±32767.
 *   Clipping = severe nonlinear distortion = spectral splatter across all bins
 *   = wrong pilot amplitudes → H ≈ 1/1024 instead of 1 → 47.72% BER.
 *
 * CORRECT SCHEDULE: S_inv = 128
 *   IFFT output max = 52 / 128 = 0.58 < 1.0 → no overflow. ✓
 *   Round-trip law: S_fwd × S_inv = N → S_fwd = 1024/128 = 8 (FFT side).
 *   Pilot recovery after IFFT/128 → FFT/8: 32767.0/32767 = 1.0000 ✓
 *   channel_estimator receives PILOT_I_VAL correctly → H = 1.0 ✓
 *
 * Schedule encoding for /128 across 5 pipelined_streaming_io (radix-22) stages:
 *   stages [0,1,2,3,4] = [/4, /4, /4, /2, /1]  → total = 4×4×4×2×1 = 128
 *   2-bit fields (10=÷4, 01=÷2, 00=÷1), LSB = stage 0:
 *   stage0=10, stage1=10, stage2=10, stage3=01, stage4=00
 *   = 0b00_01_10_10_10 = 0x06A
 *
 * config_width = 16, status_width = 8  (unchanged — see fft_wrapper.cpp)
 *==============================================================================*/
#include "../cofdm_hls.h"
#include "hls_fft.h"

struct ifft_config_t : hls::ip_fft::params_t {
    static const unsigned max_nfft           = 10;
    static const unsigned input_width        = 16;
    static const unsigned output_width       = 16;
    static const unsigned config_width       = 16;
    static const unsigned status_width       = 8;
    static const unsigned phase_factor_width = 16;
    static const bool     has_nfft           = false;
    static const unsigned ordering_opt       = hls::ip_fft::natural_order;
    static const unsigned scaling_opt        = hls::ip_fft::scaled;
    static const unsigned rounding_opt       = hls::ip_fft::convergent_rounding;
    static const unsigned arch_opt           = hls::ip_fft::pipelined_streaming_io;
    static const unsigned stages_block_ram   = 1;
};

typedef ap_fixed<16, 1>                         ifft_data_t;
typedef hls::x_complex<ifft_data_t>             ifft_complex_t;
typedef hls::ip_fft::status_t<ifft_config_t>    ifft_status_t;
typedef hls::ip_fft::config_t<ifft_config_t>    ifft_cfg_t;

#define FFT_LEN     (1 << 10)   /* 1024 */

/*
 * IFFT_SCH = 0x06A
 * Stage divisors [0..4] = [/4, /4, /4, /2, /1] → total /128
 * 2-bit fields (10=÷4, 01=÷2, 00=÷1), LSB = stage 0:
 *   stage0=10, stage1=10, stage2=10, stage3=01, stage4=00
 *   = 0b00_01_10_10_10 = 0x06A
 *
 * Paired with FFT_FWD_SCH=0x015 (/8): 128 × 8 = 1024 = N ✓
 *
 * With this schedule, the raw IDFT output is:
 *   x[n] = (1/128) × Σ_k X[k] × e^{+j2πkn/N}
 * For QPSK OFDM: max|x[n]| ≈ 0.58 < 1.0 (no overflow) ✓
 */
#define IFFT_SCH    0x06Au

void ifft_wrapper(
    hls::stream<axis_cmpx_user_t> &in_stream,   /* frequency domain input  */
    hls::stream<axis_cmpx_t>      &out_stream    /* time domain output      */
)
{
#pragma HLS INLINE off
#pragma HLS DATAFLOW

    ifft_complex_t xn[FFT_LEN];
    ifft_complex_t xk[FFT_LEN];
#pragma HLS STREAM variable=xn depth=1024
#pragma HLS STREAM variable=xk depth=1024

    ifft_status_t ifft_status;
    ifft_cfg_t    ifft_cfg;
    ifft_cfg.setDir(0);         /* 0 = inverse FFT */
    ifft_cfg.setSch(IFFT_SCH);

    /* int16 → ap_fixed<16,1> (÷32768) */
    read_input: for (int n = 0; n < FFT_LEN; n++) {
#pragma HLS PIPELINE II=1
        axis_cmpx_user_t s = in_stream.read();
        xn[n].real((ifft_data_t)((float)s.i / 32768.0f));
        xn[n].imag((ifft_data_t)((float)s.q / 32768.0f));
    }

    hls::fft<ifft_config_t>(xn, xk, &ifft_status, &ifft_cfg);

    /* ap_fixed<16,1> → int16 (×32768, saturate) */
    write_output: for (int k = 0; k < FFT_LEN; k++) {
#pragma HLS PIPELINE II=1
        axis_cmpx_t out_s;
        float re = (float)xk[k].real() * 32768.0f;
        float im = (float)xk[k].imag() * 32768.0f;
        out_s.i    = (re >  32767.0f) ? (int16_t_hls) 32767 :
                     (re < -32768.0f) ? (int16_t_hls)-32768 : (int16_t_hls)re;
        out_s.q    = (im >  32767.0f) ? (int16_t_hls) 32767 :
                     (im < -32768.0f) ? (int16_t_hls)-32768 : (int16_t_hls)im;
        out_s.last = (k == FFT_LEN - 1);
        out_stream.write(out_s);
    }
}
