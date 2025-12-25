#include "Vector2.h"

#include <cmath>

template<class T>
T Vector2Base<T>::Distance(Vector2Base<T> v0, Vector2Base<T> v1)
{
	T dx = v1.x - v0.x;
	T dy = v1.y - v0.y;
	return std::sqrt(dx*dx + dy*dy);
}

template struct Vector2Base<float>;