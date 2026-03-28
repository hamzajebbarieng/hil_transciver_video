#include "../cofdm_hls.h"

void qpsk_demodulator(
    hls::stream<axis_cmpx_t>  &in_stream,
    hls::stream<axis_dibit_t> &out_stream,
    int num_symbols
)
{
#pragma HLS INLINE off

    demod_loop:
    for (int i = 0; i < num_symbols; i++) {
#pragma HLS PIPELINE II=1
        axis_cmpx_t in_sample = in_stream.read();
        axis_dibit_t out_sample;

        // Bit 1: 0 if I > 0, 1 if I < 0
        // Bit 0: 0 if Q > 0, 1 if Q < 0
        ap_uint<1> b1 = (in_sample.i < 0) ? 1 : 0;
        ap_uint<1> b0 = (in_sample.q < 0) ? 1 : 0;

        // Reconstruct dibit to match corrected Gray table:
        // (+,+) -> 00
        // (-,+) -> 01
        // (-,-) -> 10
        // (+,-) -> 11
        if (b1 == 0 && b0 == 0)      out_sample.data = 0; // 00
        else if (b1 == 1 && b0 == 0) out_sample.data = 1; // 01
        else if (b1 == 1 && b0 == 1) out_sample.data = 2; // 10
        else                         out_sample.data = 3; // 11

        out_sample.last = in_sample.last;
        out_stream.write(out_sample);
    }
}
