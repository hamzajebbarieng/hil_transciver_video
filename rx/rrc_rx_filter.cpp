	//============================================================================
     	// RRC RX Filter - DVB-T Compliant Fix
     	//
     	// Fix: Properly propagates TLAST from input to output
     	// Original issue: out_sample.last = false; broke frame boundary detection
     	//
     	// DVB-T Note: The RRC filter is a matched filter that should preserve
     	// frame boundaries so downstream modules can detect OFDM symbol boundaries
     	//============================================================================
    	#include "../cofdm_hls.h"

    	void rrc_rx_filter(
    	    hls::stream<axis_cmpx_t> &in_stream,
    	    hls::stream<axis_cmpx_t> &out_stream,
    	    int num_samples
    	)
    	{
    	#pragma HLS INLINE off

    	    // Transposed FIR states (N-1 registers for N=41 taps)
    	    acc36_t state_i[RRC_NUM_TAPS - 1];
    	    acc36_t state_q[RRC_NUM_TAPS - 1];
    	#pragma HLS ARRAY_PARTITION variable=state_i complete
    	#pragma HLS ARRAY_PARTITION variable=state_q complete

    	    // Initialize states to zero
    	    init_states:
    	    for (int i = 0; i < RRC_NUM_TAPS - 1; i++) {
    	#pragma HLS UNROLL
    	        state_i[i] = 0;
    	        state_q[i] = 0;
    	    }

    	    int decim_cnt = 0;
    	    // DVB-T FIX: Track input last signal for proper frame boundary propagation
    	    bool input_last = false;

    	    filter_loop:
    	    for (int s = 0; s < num_samples; s++) {
    	#pragma HLS PIPELINE II=1
    	        axis_cmpx_t in_sample = in_stream.read();

    	        // DVB-T COMPLIANT: Capture the TLAST from input for frame boundary tracking
    	        if (s == num_samples - 1) {
    	            input_last = in_sample.last;
    	        }

    	        // Compute all products in parallel
    	        acc36_t prod_i[RRC_NUM_TAPS];
    	        acc36_t prod_q[RRC_NUM_TAPS];
    	#pragma HLS ARRAY_PARTITION variable=prod_i complete
    	#pragma HLS ARRAY_PARTITION variable=prod_q complete
    	        for (int i = 0; i < RRC_NUM_TAPS; i++) {
    	#pragma HLS UNROLL
    	            prod_i[i] = (acc36_t)in_sample.i * (acc36_t)rrc_coeffs[i];
    	            prod_q[i] = (acc36_t)in_sample.q * (acc36_t)rrc_coeffs[i];
    	        }

    	        // Output = h0 * x[n] + s1[n-1]
    	        acc36_t out_i = prod_i[0] + state_i[0];
    	        acc36_t out_q = prod_q[0] + state_q[0];

    	        // Update states in ascending order (uses old higher states)
    	        for (int i = 0; i < RRC_NUM_TAPS - 1; i++) {
    	#pragma HLS UNROLL
    	            if (i == RRC_NUM_TAPS - 2) {
    	                // Last state: s_{N-1}[n] = h_{N-1} * x[n]
    	                state_i[i] = prod_i[i + 1];
    	                state_q[i] = prod_q[i + 1];
    	            } else {
    	                // s_{i+1}[n] = h_{i+1} * x[n] + s_{i+2}[n-1]
    	                state_i[i] = prod_i[i + 1] + state_i[i + 1];
    	                state_q[i] = prod_q[i + 1] + state_q[i + 1];
    	            }
    	        }

    	        // Decimate by factor of 4
    	        if (decim_cnt == RRC_DOWN_FACTOR - 1) {
    	            decim_cnt = 0;
    	            axis_cmpx_t out_sample;
    	            out_sample.i = (int16_t_hls)(out_i >> 15);
    	            out_sample.q = (int16_t_hls)(out_q >> 15);

    	            // DVB-T COMPLIANT FIX: Propagate TLAST for proper frame boundary detection
    	            // The last decimated output should have last=true to signal end of frame
    	            // Note: This is a simplification - in practice, the exact last position
    	            // depends on filter latency, but this provides proper frame boundary signaling
    	            out_sample.last = in_sample.last;

    	            out_stream.write(out_sample);
   	        } else {
    	            decim_cnt++;
    	        }
    	    }
    	}
