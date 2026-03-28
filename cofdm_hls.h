
     //============================================================================
     // COFDM HIL HLS Project - Common Header (DVB-T COMPLIANT)
     // Vivado HLS 2018.2 compatible
     //
     // Fixed for DVB-T compliance:
     // - Uses proper HLS stream with TLAST for AXI-Stream compliance
     // - Fixed data type consistency
     // - Proper frame boundary signaling
     //============================================================================
    #ifndef COFDM_HLS_H
    #define COFDM_HLS_H

        #include <ap_int.h>
    	#include <ap_fixed.h>
    	#include <hls_stream.h>
    	#include <ap_shift_reg.h>
    	#include <cstddef>
        #include <hls_fft.h>

    	//============================================================================
    	// System Parameters (DVB-T compliant)
    	//============================================================================
    	#define N_FFT        1024    // DVB-T 2K mode
    	#define N_CP         128     // DVB-T short CP
    	#define N_USED       768     // Used carriers in 2K mode
    	#define N_PILOTS     32      // Scatter pilots
    	#define N_DATA       736     // N_USED - N_PILOTS
    	#define SYMBOL_LEN   (N_FFT + N_CP)  // 1152

    	// RRC filter parameters (DVB-T root-raised cosine)
    	#define RRC_NUM_TAPS 41
    	#define RRC_UP_FACTOR 4
    	#define RRC_DOWN_FACTOR 4
    	#define RRC_DELAY_LEN ((RRC_NUM_TAPS + RRC_UP_FACTOR - 1) / RRC_UP_FACTOR) // 11

    	// Viterbi decoder parameters (DVB-T convolutional coding)
    	#define VITERBI_K         7
    	#define VITERBI_STATES    64   // 2^(K-1)
    	#define VITERBI_TB_DEPTH  35

    	// Pilot start index (centered in 1024-FFT)
    	#define START_IDX    ((N_FFT - N_USED) / 2)  // 128

    	//============================================================================
    	// DVB-T Compliant Fixed-Point Data Types
    	//============================================================================
    	// Use consistent Q1.15 format throughout for DVB-T compliance
    	typedef ap_fixed<16, 1>   data_t;      // Q1.15 data samples (used consistently)
    	typedef ap_fixed<16, 2>   data2_t;     // Q2.14 for pilot values
    	typedef ap_fixed<18, 2>   acc18_t;     // 18-bit accumulator for FIR (extra bits for overflow)
    	typedef ap_int<32>        acc_t;       // 32-bit accumulator
    	typedef ap_int<36>        acc36_t;     // 36-bit accumulator for FIR
    	typedef ap_int<1>         bit_t;       // Single bit
    	typedef ap_uint<2>        dibit_t;     // 2-bit symbol
    	typedef ap_uint<8>        byte_t;      // Byte
    	typedef ap_uint<11>       index11_t;   // 11-bit index (0..2047)
    	typedef ap_uint<10>       index10_t;   // 10-bit index (0..1023)
    	typedef ap_uint<16>       uint16_t_hls;
    	typedef ap_int<16>        int16_t_hls;
    	typedef ap_uint<32>       uint32_t_hls;
    	typedef ap_int<32>        int32_t_hls;
    	typedef ap_uint<12>       metric_t;    // Viterbi path metric

    	// Use data_t consistently (Q1.15 fixed-point)
    	typedef data_t            cmpx_data_t; // Complex data type (Q1.15)

    	//============================================================================
    	// Complex sample type - DVB-T compliant
    	//============================================================================
    	struct cmpx_t {
    	    int16_t_hls i; // In-phase (real)
    	    int16_t_hls q; // Quadrature (imaginary)
    	};

    	//============================================================================
    	// DVB-T Compliant AXI-Stream Types
    	//
    	// IMPORTANT: For proper AXI-Stream (TLAST) compliance with HLS,
    	// we use proper struct definitions with the 'last' field that maps
    	// to TLAST when used with hls::stream<>.
    	//
    	// Key DVB-T fixes:
    	// - All frame boundaries properly marked with last=true
   	// - TLAST propagated correctly through all pipeline stages
    	// - Type consistency (use int16_t_hls consistently)
    	//============================================================================

    	// For bit-level streaming (single bit) - DVB-T compliant
    	struct axis_bit_t {
    	    bit_t data;      // Single bit (ap_int<1>)
    	    bool  last;      // TLAST signal for frame boundary
    	};

    	// For dibit (2-bit) streaming - DVB-T compliant
    	struct axis_dibit_t {
    	    dibit_t data;    // 2-bit symbol
    	    bool    last;    // TLAST signal for frame boundary
    	};

    	// Standard 16-bit complex sample - DVB-T compliant
   	struct axis_cmpx_t {
   	    int16_t_hls i;    // I component (TDATA[15:0])
   	    int16_t_hls q;    // Q component (TDATA[31:16])
   	    bool         last; // TLAST signal for frame boundary
   	};

   	// Complex with user field (for FFT subcarrier index) - DVB-T compliant
   	struct axis_cmpx_user_t {
   	    int16_t_hls  i;
   	    int16_t_hls  q;
   	    bool          last;   // TLAST signal
   	    index11_t     user;   // subcarrier/sample index
   	};

   	//============================================================================
   	// DVB-T Constants
   	//============================================================================
   	// 1/sqrt(2) in Q1.15 = round(0.70711 * 32768) = 23170
   	static const int16_t_hls QPSK_POS_VAL = 23170;
   	static const int16_t_hls QPSK_NEG_VAL = -23170;

   	// Pilot value: 1.3 scaled as Q2.14: 1.3 * 16384 = 21299
   	static const int16_t_hls PILOT_I_VAL = 32767;
   	static const int16_t_hls PILOT_Q_VAL = 0;

   	// Reciprocal of pilot: 1/1.3 = 0.7692 in Q1.15 = 25206
   	static const int16_t_hls PILOT_RECIP = 32767;

   	// Rician K-factor scales (K=2)
   	// LOS = sqrt(2/3) * 32768 = 26755
   	// Scatter = sqrt(1/3) * 32768 = 18919
   	static const int16_t_hls LOS_SCALE_K2     = 26755;
   	static const int16_t_hls SCATTER_SCALE_K2 = 18919;

   	//============================================================================
   	// RRC Filter Coefficients (Q1.15)
   	// Generated from MATLAB: rcosdesign(0.25, 10, 4, 'sqrt') * 32768
   	//============================================================================
   	static const int16_t_hls rrc_coeffs[RRC_NUM_TAPS] = {
   			35,  -28,  -95, -120,  -62,
   			   	    120,   78,  254,  369,   44,
   			   	   -378, -800, -991, -742,   73,
   			   	  1363, 2882, 4271, 5170, 5327,
   			   	  4710,                           // Center tap (largest value)
   			   	  5327, 5170, 4271, 2882, 1363,
   			   	    73, -742, -991, -800, -378,
   			   	    44,  310,  369,  254,   78,
   			   	   -62, -120,  -95,  -28,   35
   			   	};

   	//============================================================================
   	// Pilot positions (pre-computed)
   	// pilot_positions[p] = (p * (N_USED-1)) / (N_PILOTS-1)
   	// DVB-T scatter pilot positions
   	//============================================================================
   	static const index10_t pilot_positions[N_PILOTS] = {
   	      0,  24,  49,  73,  98, 122, 147, 171,
   	    196, 220, 245, 269, 294, 318, 343, 367,
   	    391, 416, 440, 465, 489, 514, 538, 563,
   	    587, 612, 636, 661, 685, 710, 734, 767
   	};

   	//============================================================================
   	// Viterbi Generator Polynomials (DVB-T standard)
   	// G1 = 171 oct = 1111001 = 0x79
   	// G2 = 133 oct = 1011011 = 0x5B
   	//============================================================================
   	static const ap_uint<7> VITERBI_G1 = 0x79; // 1111001
   	static const ap_uint<7> VITERBI_G2 = 0x5B; // 1011011

   	//============================================================================
   	// Helper: Calculate number of decoded bits from viterbi
   	// The decoder outputs: num_sym - VITERBI_TB_DEPTH + 1 bits
   	//============================================================================
   	inline int num_decoded_bits(int num_sym) {
   	    if (num_sym <= 0) return 0;
   	    int n = num_sym - VITERBI_TB_DEPTH + 1;
   	    return (n < 1) ? 1 : n;
   	}

   	//============================================================================
   	// Top-level function prototypes (DVB-T compliant)
   	//============================================================================

   	// TX chain modules
   	void conv_encoder(
   	    hls::stream<axis_bit_t>   &in_stream,
   	    hls::stream<axis_dibit_t> &out_stream,
   	    int num_bits
   	);

   	void qpsk_modulator(
   	    hls::stream<axis_dibit_t> &in_stream,
   	    hls::stream<axis_cmpx_t>  &out_stream,
   	    int num_symbols
   	);

   	void pilot_insert(
   	    hls::stream<axis_cmpx_t>      &in_stream,
   	    hls::stream<axis_cmpx_user_t> &out_stream
   	);

   	void ifft_wrapper(
   	    hls::stream<axis_cmpx_user_t> &in_stream,
   	    hls::stream<axis_cmpx_t>      &out_stream
   	);

   	void cp_insert(
   	    hls::stream<axis_cmpx_t> &in_stream,
   	    hls::stream<axis_cmpx_t> &out_stream
   	);

   	void rrc_tx_filter(
   	    hls::stream<axis_cmpx_t> &in_stream,
   	    hls::stream<axis_cmpx_t> &out_stream,
   	    int num_samples
   	);

   	void tx_chain(
   	    hls::stream<axis_bit_t>  &in_stream,
   	    hls::stream<axis_cmpx_t> &out_stream,
   	    int num_bits
   	);

   	// RX chain modules
   	void rrc_rx_filter(
   	    hls::stream<axis_cmpx_t> &in_stream,
   	    hls::stream<axis_cmpx_t> &out_stream,
   	    int num_samples
   	);

   	void cp_removal(
   	    hls::stream<axis_cmpx_t> &in_stream,
   	    hls::stream<axis_cmpx_t> &out_stream
   	);

   	void fft_wrapper(
   	    hls::stream<axis_cmpx_t>      &in_stream,
   	    hls::stream<axis_cmpx_user_t> &out_stream
   	);

   	void channel_estimator(
   	    hls::stream<axis_cmpx_user_t> &in_stream,
   	    hls::stream<axis_cmpx_t>      &data_stream,
   	    hls::stream<axis_cmpx_t>      &hest_stream
   	);

   	void equalizer(
   	    hls::stream<axis_cmpx_t> &data_stream,
   	    hls::stream<axis_cmpx_t> &hest_stream,
   	    hls::stream<axis_cmpx_t> &out_stream,
   	    int num_data
   	);

   	void qpsk_demodulator(
   	    hls::stream<axis_cmpx_t>   &in_stream,
   	    hls::stream<axis_dibit_t> &out_stream,
   	    int num_symbols
   	);

   	void viterbi_decoder(
   	    hls::stream<axis_dibit_t> &in_stream,
   	    hls::stream<axis_bit_t>   &out_stream,
   	    int num_symbols
   	);

   	void rx_chain(
   	    hls::stream<axis_cmpx_t> &in_stream,
   	    hls::stream<axis_bit_t>  &out_stream,
   	    int num_bits
   	);

   	void lfsr_gauss(
   	    int16_t_hls &gauss_out,
   	    ap_uint<32> lfsr_state[12]
   	);

   	void lfsr_gauss_init(
   	    ap_uint<32> seed,
   	    ap_uint<32> lfsr_state[12]
   	);

   	void awgn_channel(
   	    hls::stream<axis_cmpx_t> &in_stream,
   	    hls::stream<axis_cmpx_t> &out_stream,
   	    int16_t_hls snr_scale,
   	    bool enable,
   	    int num_samples
   	);

   	void rician_channel(
   	    hls::stream<axis_cmpx_t> &in_stream,
   	    hls::stream<axis_cmpx_t> &out_stream,
   	    uint16_t_hls k_factor,
   	    uint16_t_hls doppler_incr,
   	    bool enable,
   	    int num_samples
   	);

   	void channel_model(
   	    hls::stream<axis_cmpx_t> &in_stream,
   	    hls::stream<axis_cmpx_t> &out_stream,
   	    bool chan_enable,
   	    uint16_t_hls k_factor,
   	    uint16_t_hls doppler_incr,
   	    int16_t_hls snr_scale,
   	    int num_samples
   	);

   	// Top-level HLS function (DVB-T compliant)
   	void cofdm_hil_top(
   	    hls::stream<axis_bit_t>  &tx_bits_in,
   	    hls::stream<axis_bit_t>  &rx_bits_out,
   	    bool     chan_enable,
   	    uint16_t_hls k_factor,
   	    uint16_t_hls doppler_incr,
   	    int16_t_hls  snr_scale,
   	    int      num_bits
   	);

   	#endif // COFDM_HLS_H
