//============================================================================
// Gaussian Noise Generator using Central Limit Theorem
// Sums 12 uniform random numbers (from LFSRs) and subtracts 6
// to approximate a Gaussian distribution (mean=0, variance=1)
//
// Output: 16-bit signed Gaussian random sample (Q4.12 format)
// Uses 12 independent 32-bit LFSRs for uniform random generation
//============================================================================
#include "../cofdm_hls.h"

void lfsr_gauss(
    int16_t_hls &gauss_out,        // The output (ap_int<16>)
    ap_uint<32> lfsr_state[12]     // The state array (ap_uint<32> *)
)
{
#pragma HLS INLINE
    ap_uint<16> sum = 0;

    lfsr_loop:
    for (int i = 0; i < 12; i++) {
        #pragma HLS UNROLL
        // Advance LFSR
        if (lfsr_state[i][0]) {
            lfsr_state[i] = (lfsr_state[i] >> 1) ^ (ap_uint<32>)0x80200003;
        } else {
            lfsr_state[i] = lfsr_state[i] >> 1;
        }
        // Sum top bits
        sum += (ap_uint<12>)lfsr_state[i](31, 20);
    }
    // Subtract mean to center the Gaussian noise
    gauss_out = (int16_t_hls)((ap_int<16>)sum - (ap_int<16>)24576);
}


// Initialize LFSR states from seed
void lfsr_gauss_init(ap_uint<32> seed, ap_uint<32> lfsr_state[12])
{
#pragma HLS INLINE
    for (int i = 0; i < 12; i++) {
#pragma HLS UNROLL
        lfsr_state[i] = seed ^ ((ap_uint<32>)0xDEADBEEF + (ap_uint<32>)(i * 0x12345678));
    }
}
