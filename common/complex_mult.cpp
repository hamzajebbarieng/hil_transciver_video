#include "../cofdm_hls.h"

void complex_mult(
    int16_t_hls ar, int16_t_hls ai,
    int16_t_hls br, int16_t_hls bi,
    int16_t_hls &pr, int16_t_hls &pi
)
{
#pragma HLS INLINE
    // Use 32-bit intermediates to handle the (ar+ai) sum safely
    int32_t_hls m1 = (int32_t_hls)ar * (int32_t_hls)br;
    int32_t_hls m2 = (int32_t_hls)ai * (int32_t_hls)bi;

    // (ar + ai) and (br + bi) can reach ~2.0, so the product can reach ~4.0
    int32_t_hls m3 = ((int32_t_hls)ar + ai) * ((int32_t_hls)br + bi);

    int32_t_hls round_val = (1 << 14);

    // Full precision results before truncation
    int32_t_hls full_r = (m1 - m2 + round_val) >> 15;
    int32_t_hls full_i = (m3 - m1 - m2 + round_val) >> 15;

    // Saturation Logic: Prevents "flipping" signs on overflow
    if (full_r > 32767)       pr = 32767;
    else if (full_r < -32768) pr = -32768;
    else                      pr = (int16_t_hls)full_r;

    if (full_i > 32767)       pi = 32767;
    else if (full_i < -32768) pi = -32768;
    else                      pi = (int16_t_hls)full_i;
}
