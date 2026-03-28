//============================================================================
// TX Chain Top-Level Wrapper
// Connects: Conv Encoder -> QPSK Mod -> Pilot Insert -> IFFT -> CP Insert
//           -> RRC Filter
//
// Fixes vs broken version:
//   1. #pragma HLS STREAM depth= must be an integer literal in HLS 2018.2.
//      Macros (N_DATA, N_FFT, SYMBOL_LEN) are NOT evaluated at pragma parse
//      time -> "undeclared identifier" errors. Replaced with numeric literals.
//   2. Added #pragma HLS INLINE off so the DATAFLOW region is self-contained
//      and avoids the "static / non-static stream" warning.
//   3. Removed the #pragma HLS INLINE off that was missing from the original.
//============================================================================
#include "../cofdm_hls.h"

void tx_chain(
    hls::stream<axis_bit_t>  &in_stream,
    hls::stream<axis_cmpx_t> &out_stream,
    int num_bits
)
{
#pragma HLS INLINE off
#pragma HLS DATAFLOW

    // -----------------------------------------------------------------------
    // Inter-module streams
    // depth values are integer literals (HLS 2018.2 requires this):
    //   enc_to_qpsk   : small FIFO, 1 bit at a time
    //   qpsk_to_pilot : must hold one full OFDM symbol of data = N_DATA = 736
    //   pilot_to_ifft : must hold one full FFT frame            = N_FFT  = 1024
    //   ifft_to_cp    : must hold one full FFT output           = N_FFT  = 1024
    //   cp_to_rrc     : must hold one full CP+data symbol       = SYMBOL_LEN = 1152
    // -----------------------------------------------------------------------
    hls::stream<axis_dibit_t>     enc_to_qpsk("enc_to_qpsk");
    hls::stream<axis_cmpx_t>      qpsk_to_pilot("qpsk_to_pilot");
    hls::stream<axis_cmpx_user_t> pilot_to_ifft("pilot_to_ifft");
    hls::stream<axis_cmpx_t>      ifft_to_cp("ifft_to_cp");
    hls::stream<axis_cmpx_t>      cp_to_rrc("cp_to_rrc");

#pragma HLS STREAM variable=enc_to_qpsk   depth=64    // small handshake FIFO
#pragma HLS STREAM variable=qpsk_to_pilot depth=736   // N_DATA
#pragma HLS STREAM variable=pilot_to_ifft depth=1024  // N_FFT
#pragma HLS STREAM variable=ifft_to_cp    depth=1024  // N_FFT
#pragma HLS STREAM variable=cp_to_rrc     depth=1152  // SYMBOL_LEN = N_FFT + N_CP

    // Number of samples fed to the RRC TX filter = one OFDM symbol length
    // (num_bits is always a multiple of N_DATA for one OFDM symbol)
    int num_ofdm_symbols = (num_bits + N_DATA - 1) / N_DATA;
    int num_cp_samples   = num_ofdm_symbols * SYMBOL_LEN; // 1 * 1152

    // 1. Convolutional encoder  (rate 1/2: 1 dibit out per input bit)
    conv_encoder(in_stream, enc_to_qpsk, num_bits);

    // 2. QPSK modulator  (1 complex symbol per dibit = 1 per input bit)
    qpsk_modulator(enc_to_qpsk, qpsk_to_pilot, num_bits);

    // 3. Pilot insertion  (outputs N_FFT = 1024 bins per OFDM symbol)
    pilot_insert(qpsk_to_pilot, pilot_to_ifft);

    // 4. IFFT  (1024-point, behavioral model for C-sim; replaced by Xilinx IP
    //           in the Vivado block design after HLS export)
    ifft_wrapper(pilot_to_ifft, ifft_to_cp);

    // 5. Cyclic prefix insertion  (outputs SYMBOL_LEN = 1152 per symbol)
    cp_insert(ifft_to_cp, cp_to_rrc);

    // 6. RRC pulse shaping + 4x upsample
    rrc_tx_filter(cp_to_rrc, out_stream, num_cp_samples);
}
