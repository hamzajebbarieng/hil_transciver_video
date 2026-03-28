//============================================================================
// RX Chain (CORRECTED)
// Now accepts num_bits like tx_chain, internally calculates correct sample counts
//============================================================================
#include "../cofdm_hls.h"

void rx_chain(
    hls::stream<axis_cmpx_t> &in_stream,
    hls::stream<axis_bit_t>  &out_stream,
    int num_bits
)
{
#pragma HLS INLINE off
#pragma HLS DATAFLOW

    // Calculate parameters
    int num_ofdm_symbols = (num_bits + N_DATA - 1) / N_DATA;

    // Upsampled samples from channel = SYMBOL_LEN * RRC_UP_FACTOR = 1152 * 4 = 4608 per symbol
    int num_rx_samples = num_ofdm_symbols * SYMBOL_LEN * RRC_UP_FACTOR;

    // After RRC RX decimation = num_rx_samples / RRC_DOWN_FACTOR = 1152 per symbol
    int num_rrc_out = num_ofdm_symbols * SYMBOL_LEN;

    // Inter-module streams
    hls::stream<axis_cmpx_t>      rrc_to_cp("rrc_to_cp");
    hls::stream<axis_cmpx_t>      cp_to_fft("cp_to_fft");
    hls::stream<axis_cmpx_user_t> fft_to_chest("fft_to_chest");
    hls::stream<axis_cmpx_t>      chest_data("chest_data");
    hls::stream<axis_cmpx_t>      chest_hest("chest_hest");
    hls::stream<axis_cmpx_t>      eq_to_demod("eq_to_demod");
    hls::stream<axis_dibit_t>     demod_to_vit("demod_to_vit");

#pragma HLS STREAM variable=rrc_to_cp    depth=1152
#pragma HLS STREAM variable=cp_to_fft    depth=1024
#pragma HLS STREAM variable=fft_to_chest depth=1024
#pragma HLS STREAM variable=chest_data   depth=736
#pragma HLS STREAM variable=chest_hest   depth=4
#pragma HLS STREAM variable=eq_to_demod  depth=736
#pragma HLS STREAM variable=demod_to_vit depth=736

    // Chain execution - now with correct sample counts
    rrc_rx_filter(in_stream, rrc_to_cp, num_rx_samples);   // Input: upsampled
    cp_removal(rrc_to_cp, cp_to_fft);                      // Output: decimated
    fft_wrapper(cp_to_fft, fft_to_chest);
    channel_estimator(fft_to_chest, chest_data, chest_hest);
    equalizer(chest_data, chest_hest, eq_to_demod, N_DATA);
    qpsk_demodulator(eq_to_demod, demod_to_vit, N_DATA);
    viterbi_decoder(demod_to_vit, out_stream, N_DATA);
}
