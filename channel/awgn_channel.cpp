#include "../cofdm_hls.h"

void awgn_channel(
    hls::stream<axis_cmpx_t> &in_stream,
    hls::stream<axis_cmpx_t> &out_stream,
    int16_t_hls snr_scale,
    bool enable,
    int num_samples
)
{
#pragma HLS INLINE off

    // Static ensures noise is random across OFDM symbols
    static ap_uint<32> lfsr_i[12];
    static ap_uint<32> lfsr_q[12];
    static bool initialized = false;
#pragma HLS ARRAY_PARTITION variable=lfsr_i complete
#pragma HLS ARRAY_PARTITION variable=lfsr_q complete

    if (!initialized) {
        lfsr_gauss_init(0x13579BDF, lfsr_i);
        lfsr_gauss_init(0xFDB97531, lfsr_q);
        initialized = true;
    }

    awgn_loop:
    for (int s = 0; s < num_samples; s++) {
#pragma HLS PIPELINE II=1
        axis_cmpx_t in_sample = in_stream.read();
        axis_cmpx_t out_sample;

        int16_t_hls noise_i, noise_q;
        // Corrected: Removed seed argument to match function signature
        lfsr_gauss(noise_i, lfsr_i);
        lfsr_gauss(noise_q, lfsr_q);

        if (enable) {
            uint16_t_hls scale_u = (uint16_t_hls)snr_scale;
            int32_t_hls scaled_n_i = ((int32_t_hls)noise_i * scale_u) >> 15;
            int32_t_hls scaled_n_q = ((int32_t_hls)noise_q * scale_u) >> 15;

            int32_t_hls res_i = (int32_t_hls)in_sample.i + scaled_n_i;
            int32_t_hls res_q = (int32_t_hls)in_sample.q + scaled_n_q;

            // Corrected: Cast constants to int16_t_hls to fix GCC error
            out_sample.i = (res_i > 32767) ? (int16_t_hls)32767 : (res_i < -32768) ? (int16_t_hls)-32768 : (int16_t_hls)res_i;
            out_sample.q = (res_q > 32767) ? (int16_t_hls)32767 : (res_q < -32768) ? (int16_t_hls)-32768 : (int16_t_hls)res_q;
        } else {
            out_sample = in_sample;
        }
        out_sample.last = in_sample.last;
        out_stream.write(out_sample);
    }
}
