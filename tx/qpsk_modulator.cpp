#include "../cofdm_hls.h"

void qpsk_modulator(
    hls::stream<axis_dibit_t> &in_stream,
    hls::stream<axis_cmpx_t>  &out_stream,
    int num_symbols
)
{
#pragma HLS INLINE off

    qpsk_loop:
    for (int i = 0; i < num_symbols; i++) {
#pragma HLS PIPELINE II=1
        axis_dibit_t in_sample = in_stream.read();
        axis_cmpx_t  out_sample;

        // Corrected Gray Mapping to match MATLAB 'gray'
        // 00 (0) -> (+,+)
        // 01 (1) -> (-,+)
        // 11 (3) -> (+,-)
        // 10 (2) -> (-,-)
        switch ((ap_uint<2>)in_sample.data) {
            case 0: // 00
                out_sample.i = QPSK_POS_VAL;
                out_sample.q = QPSK_POS_VAL;
                break;
            case 1: // 01
                out_sample.i = QPSK_NEG_VAL;
                out_sample.q = QPSK_POS_VAL;
                break;
            case 2: // 10
                out_sample.i = QPSK_NEG_VAL;
                out_sample.q = QPSK_NEG_VAL;
                break;
            case 3: // 11
                out_sample.i = QPSK_POS_VAL;
                out_sample.q = QPSK_NEG_VAL;
                break;
            default:
                out_sample.i = 0; out_sample.q = 0;
                break;
        }
        out_sample.last = in_sample.last;
        out_stream.write(out_sample);
    }
}
