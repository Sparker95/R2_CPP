#include "ProbUtils.h"

#include <cmath>

float Utils::ProbToL2Float(float prob)
{
	return std::log2f(prob / (1.0f - prob));
}

float Utils::L2FloatToProb(float l2_float)
{
	float _pow2 = std::exp2f(l2_float);
	return _pow2 / (1.0f + _pow2);
}

float Utils::L2IntToL2Float(int l2_int)
{
	return static_cast<float>(l2_int) / static_cast<float>(LOG2_PROB_QUANTIZATION);
}

int Utils::L2FloatToL2Int(float l2_float)
{
	return static_cast<int>(l2_float * static_cast<float>(LOG2_PROB_QUANTIZATION));
}
