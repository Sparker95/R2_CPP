#pragma once

// Utility functions for probability theory math

namespace Utils
{
	// Utils for Log2 probability ratio operations
	constexpr unsigned int LOG2_PROB_QUANTIZATION = 8;
	float ProbToL2Float(float prob);
	float L2FloatToProb(float l2_float);
	float L2IntToL2Float(int l2_int);
	int L2FloatToL2Int(float l2_float);
}
