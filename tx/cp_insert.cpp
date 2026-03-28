	//============================================================================
     	// CP Insert Module - DVB-T Compliant Fix
     	//
     	// Fix: Properly sets TLAST at the end of the full OFDM symbol (1152 samples)
     	// Original issue: Only set last at sample 1023, not at sample 1151
     	//============================================================================
     	#include "../cofdm_hls.h"

    	void cp_insert(
    	    hls::stream<axis_cmpx_t> &in_stream,
    	    hls::stream<axis_cmpx_t> &out_stream
    	)
    	{
    	#pragma HLS INLINE off

    	    // Use static arrays to infer BRAM
    	    static int16_t_hls buf_i[N_FFT];
    	    static int16_t_hls buf_q[N_FFT];

    	    // Step 1: Buffer the full OFDM symbol (1024 samples)
    	    // Also capture the 'last' signal from input to know when input frame ends
    	    bool input_last = false;
    	    load_symbol:
    	    for (int n = 0; n < N_FFT; n++) {
    	#pragma HLS PIPELINE II=1
    	        axis_cmpx_t sample = in_stream.read();
    	        buf_i[n] = sample.i;
    	        buf_q[n] = sample.q;
    	        if (n == N_FFT - 1) {
    	            input_last = sample.last;
    	        }
    	    }

    	    // Step 2: Output Cyclic Prefix (First 128 samples of the 1152-symbol)
    	    output_cp:
    	    for (int n = 0; n < N_CP; n++) {
    	#pragma HLS PIPELINE II=1
    	        axis_cmpx_t out_sample;
    	        int idx = (N_FFT - N_CP) + n; // 1024 - 128 = index 896
    	        out_sample.i = buf_i[idx];
    	        out_sample.q = buf_q[idx];
    	        // TLAST is NOT set during CP output - we have more data coming
    	        out_sample.last = false;
    	        out_stream.write(out_sample);
    	    }

    	    // Step 3: Output Original Symbol (1024 samples)
    	    output_data:
    	    for (int n = 0; n < N_FFT; n++) {
    	#pragma HLS PIPELINE II=1
    	        axis_cmpx_t out_sample;
    	        out_sample.i = buf_i[n];
    	        out_sample.q = buf_q[n];
    	        // DVB-T COMPLIANT FIX: Set TLAST at the END of the full 1152-sample packet
    	        // This is sample index 1151 (0-indexed), which is when n == N_FFT - 1
    	        // The downstream RRC filter can now properly detect frame boundaries
    	        out_sample.last = (n == N_FFT - 1);
    	        out_stream.write(out_sample);
    	    }
    	}
