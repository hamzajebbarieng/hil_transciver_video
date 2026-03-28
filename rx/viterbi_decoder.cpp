//============================================================================
// Viterbi Decoder - Rate 1/2, Constraint Length K=7
// Generator polynomials (lsb-current):
//   G1 = 0x79 = 1111001b
//   G2 = 0x5B = 1011011b
// Traceback depth : VITERBI_TB_DEPTH = 35
// Decision type   : Hard decision
//
// State convention (must match conv_encoder.cpp):
//   next_state = (input_bit, state[5:1])
//     next[5] = input_bit, next[4:0] = state[5:1]
//
// Design decisions (verified against xukmin/viterbi and C golden model):
//
//   decision_mem[t][s] = PREVIOUS STATE that survived into state s at time t.
//   (same structure as xukmin trellis[i][s])
//
//   Traceback depth:
//     After ACS at symbol t, the trellis has min(sym_cnt, TB_DEPTH) valid
//     entries. To recover the oldest bit in the window we do
//       lookups = min(sym_cnt, TB_DEPTH) - 1
//     NOT TB_DEPTH lookups.
//
//     Why: After N lookups starting from state S_end, we land on
//       S = states[t - N].
//     S[5] = input_bit that caused the transition INTO S, i.e. input[t-N-1].
//     We want input[t - (TB_DEPTH-1)] = input at the start of the window.
//     So we need N = TB_DEPTH - 1 lookups (not TB_DEPTH).
//
//     For short messages (sym_cnt < TB_DEPTH), we use sym_cnt - 1 lookups
//     so we land on states[0] and read its [5] = input[0] = message[0].
//
//   Decoded bit = tb_state[5] (MSB of the state after traceback)
//     = xukmin's:  state >> (K-2)  with K=7
//     = the input_bit that caused the transition INTO that state.
//============================================================================
#include "../cofdm_hls.h"

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Reproduce encoder output for (state, input_bit) -- identical to conv_encoder.cpp
static ap_uint<2> encoder_output_dec(ap_uint<6> state, ap_uint<1> input_bit)
{
#pragma HLS INLINE
    ap_uint<7> full_reg = (state, input_bit); // bit 0 = input, bits 6:1 = state

    ap_uint<1> g1 = 0, g2 = 0;
    parity_g1:
    for (int k = 0; k < VITERBI_K; k++) {
#pragma HLS UNROLL
        ap_uint<7> poly1 = (ap_uint<7>)VITERBI_G1;
        if (poly1[k]) g1 ^= full_reg[k];
    }
    parity_g2:
    for (int k = 0; k < VITERBI_K; k++) {
#pragma HLS UNROLL
        ap_uint<7> poly2 = (ap_uint<7>)VITERBI_G2;
        if (poly2[k]) g2 ^= full_reg[k];
    }
    return (g1, g2); // MSB=G1, LSB=G2
}

// Forward state transition: next[5]=input_bit, next[4:0]=state[5:1]
static ap_uint<6> next_state_dec(ap_uint<6> current_state, ap_uint<1> input_bit)
{
#pragma HLS INLINE
    return (input_bit, current_state(5, 1));
}

// Hamming distance between two dibits
static ap_uint<2> hamming_dist_dec(ap_uint<2> a, ap_uint<2> b)
{
#pragma HLS INLINE
    ap_uint<1> d0 = a[0] ^ b[0];
    ap_uint<1> d1 = a[1] ^ b[1];
    return (ap_uint<2>)(d0 + d1);
}

// ---------------------------------------------------------------------------
// Viterbi decoder top-level
// ---------------------------------------------------------------------------
void viterbi_decoder(
    hls::stream<axis_dibit_t> &in_stream,
    hls::stream<axis_bit_t>   &out_stream,
    int num_symbols
)
{
#pragma HLS INLINE off

    metric_t path_metric[VITERBI_STATES];
    metric_t path_metric_new[VITERBI_STATES];
#pragma HLS ARRAY_PARTITION variable=path_metric     complete
#pragma HLS ARRAY_PARTITION variable=path_metric_new complete

    // decision_mem[t][s] = previous state that survived into state s at time t
    ap_uint<6> decision_mem[VITERBI_TB_DEPTH][VITERBI_STATES];
#pragma HLS ARRAY_PARTITION variable=decision_mem complete dim=2

    // Initialise: only state 0 reachable at t=0
    init_metrics:
    for (int s = 0; s < VITERBI_STATES; s++) {
#pragma HLS UNROLL
        path_metric[s] = (s == 0) ? (metric_t)0 : (metric_t)0xFFF;
    }

    ap_uint<6> tb_wr_ptr  = 0;
    int        symbol_cnt = 0;

    viterbi_main:
    for (int sym = 0; sym < num_symbols; sym++) {

        axis_dibit_t in_sample = in_stream.read();
        ap_uint<2>   rx_bits   = in_sample.data;

        // Reset new metrics
        init_new_metrics:
        for (int s = 0; s < VITERBI_STATES; s++) {
#pragma HLS UNROLL
            path_metric_new[s] = (metric_t)0xFFF;
        }

        // Add-Compare-Select for every state
        acs_loop:
        for (int s = 0; s < VITERBI_STATES; s++) {
#pragma HLS UNROLL
            // Branch: input = 0
            ap_uint<6> ns0  = next_state_dec((ap_uint<6>)s, (ap_uint<1>)0);
            ap_uint<2> exp0 = encoder_output_dec((ap_uint<6>)s, (ap_uint<1>)0);
            metric_t   m0   = path_metric[s] +
                              (metric_t)hamming_dist_dec(rx_bits, exp0);

            if (m0 < path_metric_new[ns0]) {
                path_metric_new[ns0]         = m0;
                decision_mem[tb_wr_ptr][ns0] = (ap_uint<6>)s; // prev state
            }

            // Branch: input = 1
            ap_uint<6> ns1  = next_state_dec((ap_uint<6>)s, (ap_uint<1>)1);
            ap_uint<2> exp1 = encoder_output_dec((ap_uint<6>)s, (ap_uint<1>)1);
            metric_t   m1   = path_metric[s] +
                              (metric_t)hamming_dist_dec(rx_bits, exp1);

            if (m1 < path_metric_new[ns1]) {
                path_metric_new[ns1]         = m1;
                decision_mem[tb_wr_ptr][ns1] = (ap_uint<6>)s; // prev state
            }
        }

        // Commit metrics
        update_metrics:
        for (int s = 0; s < VITERBI_STATES; s++) {
#pragma HLS UNROLL
            path_metric[s] = path_metric_new[s];
        }

        symbol_cnt++;
        tb_wr_ptr = (tb_wr_ptr == VITERBI_TB_DEPTH - 1) ?
                        (ap_uint<6>)0 :
                        (ap_uint<6>)(tb_wr_ptr + 1);

        // Output one decoded bit once the traceback buffer is full,
        // or on the last symbol.
        if (symbol_cnt >= VITERBI_TB_DEPTH || in_sample.last) {

            // 1. Find survivor state (minimum path metric)
            metric_t   min_metric = (metric_t)0xFFF;
            ap_uint<6> tb_state   = 0;

            find_min:
            for (int s = 0; s < VITERBI_STATES; s++) {
#pragma HLS UNROLL
                if (path_metric_new[s] < min_metric) {
                    min_metric = path_metric_new[s];
                    tb_state   = (ap_uint<6>)s;
                }
            }

            // 2. Traceback
            //
            // Number of valid trellis entries = min(symbol_cnt, TB_DEPTH).
            // We need (valid_entries - 1) table lookups to land on the state
            // at the START of the valid window.
            //
            // After N lookups from S_end: tb_state = states[t - N]
            // tb_state[5] = input that caused transition INTO that state
            //             = input[t - N - 1]   ... we want input[t - N]
            // So we need N = valid_entries - 1 lookups to get input[0].
            //
            // This is equivalent to xukmin's loop:
            //   for i in range(trellis.size()-1, -1, -1):
            //     decoded += state >> (K-2)
            //     state = trellis[i][state]
            // which reads [5] BEFORE updating state in each step.

            int valid = (symbol_cnt < VITERBI_TB_DEPTH) ?
                            symbol_cnt : VITERBI_TB_DEPTH;
            int lookups = valid - 1;

            ap_uint<6> rd_ptr = (tb_wr_ptr == 0) ?
                                    (ap_uint<6>)(VITERBI_TB_DEPTH - 1) :
                                    (ap_uint<6>)(tb_wr_ptr - 1);

            traceback_loop:
            for (int tb = 0; tb < VITERBI_TB_DEPTH - 1; tb++) {
#pragma HLS PIPELINE II=1
                // Only perform lookups for the valid portion.
                // For tb < lookups: walk back one step.
                // For tb >= lookups: tb_state is already at the start state,
                //   skip further updates (state remains stable).
                if (tb < lookups) {
                    tb_state = decision_mem[rd_ptr][tb_state];
                    rd_ptr = (rd_ptr == 0) ?
                                 (ap_uint<6>)(VITERBI_TB_DEPTH - 1) :
                                 (ap_uint<6>)(rd_ptr - 1);
                }
            }

            // 3. Decoded bit = MSB (bit 5) of the state at the start of window
            //    = input_bit that caused the transition INTO that state
            //    = xukmin's: state >> (K-2) with K=7
            //
            // CRITICAL: bit_t = ap_int<1> is a SIGNED 1-bit integer.
            // Its only representable values are 0 and -1 (two's complement).
            // Casting uint value 1 directly to ap_int<1> gives -1, not 1.
            // Always extract via ap_uint<1> first to stay unsigned, then
            // assign to bit_t — the bit pattern is preserved correctly.
            ap_uint<1> decoded_bit = (ap_uint<1>)((tb_state >> 5) & (ap_uint<6>)1);
            axis_bit_t out_sample;
            out_sample.data = (bit_t)decoded_bit;
            out_sample.last = in_sample.last;
            out_stream.write(out_sample);
        }
    }
}
