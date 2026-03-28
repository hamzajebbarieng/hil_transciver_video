//============================================================================
// Combined Channel Model for HIL Testing
// Rician Fading + AWGN
//
// Signal flow: input -> Rician Fading -> AWGN -> output
//
// Configuration:
//   - K-factor for Rician fading
//   - Doppler frequency
//   - SNR level (noise scale)
//   - Channel enable/bypass
//============================================================================
#include "../cofdm_hls.h"

void channel_model(
    hls::stream<axis_cmpx_t> &in_stream,
    hls::stream<axis_cmpx_t> &out_stream,
    bool chan_enable,
    uint16_t_hls k_factor,
    uint16_t_hls doppler_incr,
    int16_t_hls snr_scale,
    int num_samples
)
{
#pragma HLS DATAFLOW
    // Dataflow must be flat. Logic is handled inside the sub-blocks via 'chan_enable'
    hls::stream<axis_cmpx_t> ric_to_awgn("ric_to_awgn");
#pragma HLS STREAM variable=ric_to_awgn depth=64

    rician_channel(in_stream, ric_to_awgn, k_factor, doppler_incr, chan_enable, num_samples);
    awgn_channel(ric_to_awgn, out_stream, snr_scale, chan_enable, num_samples);
         }

