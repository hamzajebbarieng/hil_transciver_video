//============================================================================
// COFDM HIL Top Level - CORRECTED
//============================================================================
#include "../cofdm_hls.h"

void cofdm_hil_top(
    hls::stream<axis_bit_t>  &tx_bits_in,
    hls::stream<axis_bit_t>  &rx_bits_out,
    bool     chan_enable,
    uint16_t_hls k_factor,
    uint16_t_hls doppler_incr,
    int16_t_hls  snr_scale,
    int      num_bits
) {
#pragma HLS INTERFACE axis port=tx_bits_in
#pragma HLS INTERFACE axis port=rx_bits_out
#pragma HLS INTERFACE s_axilite port=return bundle=ctrl

    // REMOVE DATAFLOW - run sequentially
    //#pragma HLS DATAFLOW

    // Step 1: Run TX completely first
    hls::stream<axis_cmpx_t> tx_to_chan("tx_to_chan");
    tx_chain(tx_bits_in, tx_to_chan, num_bits);

    // Drain TX output completely before starting RX
    // (This ensures all data is ready)
    hls::stream<axis_cmpx_t> tx_drained("tx_drained");
    while (!tx_to_chan.empty()) {
        tx_drained.write(tx_to_chan.read());
    }

    // Step 2: Run channel (or bypass)
    hls::stream<axis_cmpx_t> chan_to_rx("chan_to_rx");
    if (chan_enable) {
        int num_samples = ((num_bits + N_DATA - 1) / N_DATA) * SYMBOL_LEN * RRC_UP_FACTOR;
        channel_model(tx_drained, chan_to_rx, chan_enable, k_factor, doppler_incr, snr_scale, num_samples);
    } else {
        // Bypass - just copy
        while (!tx_drained.empty()) {
            chan_to_rx.write(tx_drained.read());
        }
    }

    // Step 3: Run RX
    rx_chain(chan_to_rx, rx_bits_out, num_bits);
}
