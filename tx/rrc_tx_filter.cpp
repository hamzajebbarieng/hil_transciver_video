#include "../cofdm_hls.h"

void rrc_tx_filter(
    hls::stream<axis_cmpx_t> &in_stream,
    hls::stream<axis_cmpx_t> &out_stream,
    int num_samples
)
{
#pragma HLS INLINE off

    const int MAX_TAPS_PER_PHASE = 11;  // ceil(41/4)
    const int taps_per_phase[4] = {11, 10, 10, 10};

    // Precompute phase coefficients (zero-padded for shorter phases)
    int16_t_hls coeff_phase[4][MAX_TAPS_PER_PHASE];
#pragma HLS ARRAY_PARTITION variable=coeff_phase complete dim=2
    for (int p = 0; p < 4; p++) {
        for (int k = 0; k < MAX_TAPS_PER_PHASE; k++) {
#pragma HLS UNROLL
            int idx = p + k * RRC_UP_FACTOR;
            if (idx < RRC_NUM_TAPS)
                coeff_phase[p][k] = rrc_coeffs[idx];
            else
                coeff_phase[p][k] = 0;
        }
    }

    // Transposed FIR states for each phase (size: MAX_TAPS_PER_PHASE-1)
    acc36_t state_i[4][MAX_TAPS_PER_PHASE - 1];
    acc36_t state_q[4][MAX_TAPS_PER_PHASE - 1];
#pragma HLS ARRAY_PARTITION variable=state_i complete dim=2
#pragma HLS ARRAY_PARTITION variable=state_q complete dim=2

    // Initialize states to zero
    init_states:
    for (int p = 0; p < 4; p++) {
        for (int k = 0; k < MAX_TAPS_PER_PHASE - 1; k++) {
#pragma HLS UNROLL
            state_i[p][k] = 0;
            state_q[p][k] = 0;
        }
    }

    filter_loop:
    for (int s = 0; s < num_samples; s++) {
        axis_cmpx_t in_sample = in_stream.read();

        upsample_loop:
        for (int phase = 0; phase < RRC_UP_FACTOR; phase++) {
#pragma HLS PIPELINE II=1
            int num_taps = taps_per_phase[phase];

            // Compute products for this phase (parallel)
            acc36_t prod_i[MAX_TAPS_PER_PHASE];
            acc36_t prod_q[MAX_TAPS_PER_PHASE];
#pragma HLS ARRAY_PARTITION variable=prod_i complete
#pragma HLS ARRAY_PARTITION variable=prod_q complete
            for (int k = 0; k < MAX_TAPS_PER_PHASE; k++) {
#pragma HLS UNROLL
                int16_t_hls c = coeff_phase[phase][k];
                prod_i[k] = (acc36_t)in_sample.i * (acc36_t)c;
                prod_q[k] = (acc36_t)in_sample.q * (acc36_t)c;
            }

            // Output = h0 * x[n] + s1[n-1] (if more than 1 tap)
            acc36_t out_i, out_q;
            if (num_taps >= 1) {
                acc36_t state_val_i = (num_taps > 1) ? state_i[phase][0] : acc36_t(0);
                acc36_t state_val_q = (num_taps > 1) ? state_q[phase][0] : acc36_t(0);
                out_i = prod_i[0] + state_val_i;
                out_q = prod_q[0] + state_val_q;
            } else {
                out_i = 0; out_q = 0;
            }

            // Compute next states (only for indices 0..num_taps-2)
            acc36_t next_state_i[MAX_TAPS_PER_PHASE - 1];
            acc36_t next_state_q[MAX_TAPS_PER_PHASE - 1];
#pragma HLS ARRAY_PARTITION variable=next_state_i complete
#pragma HLS ARRAY_PARTITION variable=next_state_q complete

            for (int k = 0; k < MAX_TAPS_PER_PHASE - 1; k++) {
#pragma HLS UNROLL
                if (k < num_taps - 1) {
                    // next_state[k] = prod[k+1] + (if k+1 < num_taps-1 then state[k+1] else 0)
                    acc36_t add_i = (k+1 < num_taps-1) ? state_i[phase][k+1] : acc36_t(0);
                    acc36_t add_q = (k+1 < num_taps-1) ? state_q[phase][k+1] : acc36_t(0);
                    next_state_i[k] = prod_i[k+1] + add_i;
                    next_state_q[k] = prod_q[k+1] + add_q;
                } else {
                    // Unused states remain zero
                    next_state_i[k] = 0;
                    next_state_q[k] = 0;
                }
            }

            // Update states for next iteration
            for (int k = 0; k < MAX_TAPS_PER_PHASE - 1; k++) {
#pragma HLS UNROLL
                state_i[phase][k] = next_state_i[k];
                state_q[phase][k] = next_state_q[k];
            }

            // Write output sample
            axis_cmpx_t out_sample;
            out_sample.i = (int16_t_hls)(out_i >> 15);
            out_sample.q = (int16_t_hls)(out_q >> 15);
            out_sample.last = (in_sample.last && phase == RRC_UP_FACTOR - 1);
            out_stream.write(out_sample);
        }
    }
}
