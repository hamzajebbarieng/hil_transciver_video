//============================================================================
// Zero-Forcing Equalizer for DVB-T (FIXED)
//============================================================================
// DVB-T Compliance:
// - Channel estimator produces 1 H estimate per OFDM symbol (not per sample)
// - Equalizer must read H once and reuse for all 736 data samples
//============================================================================
#include "../cofdm_hls.h"

void equalizer(
    hls::stream<axis_cmpx_t> &data_stream,
    hls::stream<axis_cmpx_t> &hest_stream,
    hls::stream<axis_cmpx_t> &out_stream,
    int num_data
)
{
#pragma HLS INLINE off

    //==========================================================================
    // CRITICAL FIX: Read H estimate ONCE outside the loop!
    // The channel estimator outputs 1 H estimate per OFDM symbol,
    // but there are 736 data subcarriers. We reuse the same H for all.
    //==========================================================================
    axis_cmpx_t hest = hest_stream.read();
    int16_t_hls h_real = hest.i;
    int16_t_hls h_imag = hest.q;

    // Compute |H|^2 once (same for all iterations)
    int64_t h_mag_sq = (int64_t)h_real * (int64_t)h_real +
                       (int64_t)h_imag * (int64_t)h_imag;
    if (h_mag_sq < 1000) h_mag_sq = 1000;

    //==========================================================================
    // Process all data subcarriers using the SAME H estimate
    //==========================================================================
    eq_loop:
    for (int i = 0; i < num_data; i++) {
#pragma HLS PIPELINE II=1

        axis_cmpx_t data_in = data_stream.read();
        axis_cmpx_t eq_out;

        // Complex multiply: Data * conj(H)
        int32_t_hls num_real = (int32_t_hls)data_in.i * h_real +
                               (int32_t_hls)data_in.q * h_imag;
        int32_t_hls num_imag = (int32_t_hls)data_in.q * h_real -
                               (int32_t_hls)data_in.i * h_imag;

        // Zero-forcing division
        int64_t num_real_shifted = (int64_t)num_real << 15;
        int64_t num_imag_shifted = (int64_t)num_imag << 15;

        int32_t_hls result_i = (int32_t_hls)(num_real_shifted / h_mag_sq);
        int32_t_hls result_q = (int32_t_hls)(num_imag_shifted / h_mag_sq);

        // Saturation
        if (result_i >  32767) result_i =  32767;
        else if (result_i < -32768) result_i = -32768;
        if (result_q >  32767) result_q =  32767;
        else if (result_q < -32768) result_q = -32768;

        eq_out.i = (int16_t_hls)result_i;
        eq_out.q = (int16_t_hls)result_q;
        eq_out.last = data_in.last;
        out_stream.write(eq_out);
    }
}
