//============================================================================
// Pilot Insertion Module
// Inserts 32 pilot symbols among 768 active subcarriers
// Data carriers: 736, Pilot carriers: 32
// Pilot value: 1.3 + 0j (Q2.14: 21299 + j*0)
//
// Subcarrier mapping for 1024-FFT:
//   - Active range: indices 128..895 (768 subcarriers centered)
//   - 32 pilots evenly spaced within active range
//   - 736 data subcarriers fill remaining positions
//
// Input:  736 QPSK symbols per OFDM symbol (streamed)
// Output: 1024-point frequency-domain vector (streamed)
//============================================================================
#include "../cofdm_hls.h"

void pilot_insert(
    hls::stream<axis_cmpx_t>      &in_stream,
    hls::stream<axis_cmpx_user_t> &out_stream
)
{
#pragma HLS INLINE off
    static int16_t_hls data_buf_i[N_DATA];
    static int16_t_hls data_buf_q[N_DATA];

    for (int i = 0; i < N_DATA; i++) {
#pragma HLS PIPELINE II=1
        axis_cmpx_t sample = in_stream.read();
        data_buf_i[i] = sample.i;
        data_buf_q[i] = sample.q;
    }

    int data_rd_idx = 0;
    int pilot_ptr = 0;
    // Cache the first pilot position in a register
    index10_t next_pilot_pos = pilot_positions[0];

    output_fft: for (int out_idx = 0; out_idx < N_FFT; out_idx++) {
#pragma HLS PIPELINE II=1
        axis_cmpx_user_t out_sample;
        out_sample.user = out_idx;
        out_sample.last = (out_idx == N_FFT - 1);

        int active_offset = out_idx - START_IDX;
        bool in_range = (out_idx >= START_IDX && out_idx < START_IDX + N_USED);

        if (in_range && pilot_ptr < N_PILOTS && active_offset == next_pilot_pos) {
            out_sample.i = PILOT_I_VAL;
            out_sample.q = PILOT_Q_VAL;
            pilot_ptr++;
            if(pilot_ptr < N_PILOTS) next_pilot_pos = pilot_positions[pilot_ptr];
        } else if (in_range) {
            out_sample.i = data_buf_i[data_rd_idx];
            out_sample.q = data_buf_q[data_rd_idx];
            data_rd_idx++;
        } else {
            out_sample.i = 0;
            out_sample.q = 0;
        }
        out_stream.write(out_sample);
    }
}
