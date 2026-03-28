#include "../cofdm_hls.h"

void rician_channel(
    hls::stream<axis_cmpx_t> &in_stream,
    hls::stream<axis_cmpx_t> &out_stream,
    uint16_t_hls k_factor,
    uint16_t_hls doppler_incr,
    bool enable,
    int num_samples
)
{
#pragma HLS INLINE off

    static ap_uint<32> lfsr_i[12];
    static ap_uint<32> lfsr_q[12];
    static int16_t_hls h_ray_i = 0;
    static int16_t_hls h_ray_q = 0;
    static bool initialized = false;
#pragma HLS ARRAY_PARTITION variable=lfsr_i complete
#pragma HLS ARRAY_PARTITION variable=lfsr_q complete

    if (!initialized) {
        lfsr_gauss_init(0xA5A5A5A5, lfsr_i);
        lfsr_gauss_init(0x5A5A5A5A, lfsr_q);
        initialized = true;
    }

    rician_loop:
    for (int s = 0; s < num_samples; s++) {
#pragma HLS PIPELINE II=1
        axis_cmpx_t in_sample = in_stream.read();
        axis_cmpx_t out_sample;

        int16_t_hls gauss_i, gauss_q;
        lfsr_gauss(gauss_i, lfsr_i);
        lfsr_gauss(gauss_q, lfsr_q);

        if (enable) {
            // Doppler filter (alpha = 1/256 via shift 8)
            h_ray_i = h_ray_i + (int16_t_hls)(((int32_t_hls)gauss_i - h_ray_i) >> 8);
            h_ray_q = h_ray_q + (int16_t_hls)(((int32_t_hls)gauss_q - h_ray_q) >> 8);

            int32_t_hls los_i = ((int32_t_hls)in_sample.i * LOS_SCALE_K2) >> 15;
            int32_t_hls los_q = ((int32_t_hls)in_sample.q * LOS_SCALE_K2) >> 15;

            int32_t_hls cm_r = ((int32_t_hls)h_ray_i * in_sample.i - (int32_t_hls)h_ray_q * in_sample.q) >> 15;
            int32_t_hls cm_i = ((int32_t_hls)h_ray_i * in_sample.q + (int32_t_hls)h_ray_q * in_sample.i) >> 15;

            int32_t_hls sc_i = ((int32_t_hls)SCATTER_SCALE_K2 * cm_r) >> 15;
            int32_t_hls sc_q = ((int32_t_hls)SCATTER_SCALE_K2 * cm_i) >> 15;

            int32_t_hls res_i = los_i + sc_i;
            int32_t_hls res_q = los_q + sc_q;

            // Corrected: Cast constants for GCC type-safety
            out_sample.i = (res_i > 32767) ? (int16_t_hls)32767 : (res_i < -32768) ? (int16_t_hls)-32768 : (int16_t_hls)res_i;
            out_sample.q = (res_q > 32767) ? (int16_t_hls)32767 : (res_q < -32768) ? (int16_t_hls)-32768 : (int16_t_hls)res_q;
        } else {
            out_sample = in_sample;
        }
        out_sample.last = in_sample.last;
        out_stream.write(out_sample);
    }
}
