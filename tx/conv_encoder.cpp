//============================================================================
// Convolutional Encoder - Rate 1/2, Constraint Length K=7
// Generator polynomials (lsb-current / Spiral convention):
//   G1 = 0x79 = 1111001b  -> taps at positions 0,3,4,5,6
//   G2 = 0x5B = 1011011b  -> taps at positions 0,1,3,4,6
//
// State register: 6 bits (K-1 = 6)
//   state[5] = oldest bit, state[0] = most recently shifted-in bit
//
// For each input bit: outputs 2 coded bits packed as a dibit (G1, G2)
//
// MATLAB equivalent:
//   trellis = poly2trellis(7, [171 133])
//   encoded = convenc(data, trellis)
//   Note: MATLAB uses msb-current polynomials (171oct=0x79, 133oct=0x5B).
//   After bit-reversal to lsb-current these become 0x4F and 0x6D, but
//   because our full_reg construction places the input at bit 0 (lsb),
//   using 0x79 / 0x5B directly with lsb indexing gives the correct taps.
//============================================================================
#include "../cofdm_hls.h"

// Compute one parity bit given the 7-bit shift register content and
// a 7-bit generator polynomial (lsb-current: bit 0 = current input).
static ap_uint<1> compute_parity(ap_uint<7> full_reg, ap_uint<7> poly)
{
#pragma HLS INLINE
    ap_uint<1> result = 0;
    parity_loop:
    for (int k = 0; k < VITERBI_K; k++) {
#pragma HLS UNROLL
        if (poly[k])
            result ^= full_reg[k];
    }
    return result;
}

void conv_encoder(
    hls::stream<axis_bit_t>   &in_stream,
    hls::stream<axis_dibit_t> &out_stream,
    int num_bits
)
{
#pragma HLS INLINE off

    // 6-bit shift register, initialised to zero (known encoder start state)
    // Bit layout after loading: full_reg[6:1] = state, full_reg[0] = input_bit
    ap_uint<6> state = 0;

    encode_loop:
    for (int i = 0; i < num_bits; i++) {
#pragma HLS PIPELINE II=1
        axis_bit_t in_sample = in_stream.read();
        ap_uint<1> input_bit = in_sample.data;

        // Build the full 7-bit register (lsb-current convention):
        //   bit 0 = current input (newest)
        //   bits 6:1 = shift register contents (state[5] = oldest)
        ap_uint<7> full_reg = (state, input_bit);

        // Compute two parity bits using generator polynomials
        ap_uint<1> g1_out = compute_parity(full_reg, (ap_uint<7>)VITERBI_G1);
        ap_uint<1> g2_out = compute_parity(full_reg, (ap_uint<7>)VITERBI_G2);

        // Output dibit: MSB = G1, LSB = G2  (matches decoder's ACS expectation)
        axis_dibit_t out_sample;
        out_sample.data = (g1_out, g2_out);
        out_sample.last = in_sample.last;
        out_stream.write(out_sample);

        // Advance shift register: push input_bit into MSB, drop oldest (LSB)
        state = (input_bit, state(5, 1));
    }
}
