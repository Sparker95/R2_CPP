#pragma once

#include <cmath>
//#include <unordered_set>
#include <string_view>

template <class T>
struct Vector2Base
{
	T x = 0;
	T y = 0;

	constexpr Vector2Base(T x, T y)
		: x(x), y(y)
	{
	}

	Vector2Base() = default;

	Vector2Base operator+(const Vector2Base& r) const { return Vector2Base(x + r.x, y + r.y); }
	Vector2Base& operator+=(const Vector2Base& r) { x += r.x; y += r.y; return *this; }
	Vector2Base operator-(const Vector2Base& r) const { return Vector2Base(x - r.x, y - r.y); }
	Vector2Base& operator-=(const Vector2Base& r) { x -= r.x; y -= r.y; return *this; }
	Vector2Base operator*(T r) const { return Vector2Base(x * r, y * r); }
	Vector2Base& operator*=(T r) { x *= r; y *= r; return *this; }
	Vector2Base operator/(T r) const { return Vector2Base(x / r, y / r); }
	Vector2Base& operator/=(T r) { x /= r; y /= r; return *this; }
	bool operator==(const Vector2Base<T>& r) const { return x == r.x && y == r.y; }
	bool operator!=(const Vector2Base<T>& r) const { return x != r.x || y != r.y; }

	static T Distance(Vector2Base<T> v0, Vector2Base<T> v1);
	static T DistanceSq(Vector2Base<T> v0, Vector2Base<T> v1);
	static T DotProduct(Vector2Base<T> v0, Vector2Base<T> v1);
	
	T Length() const
	{
		return std::sqrt(x * x + y * y);
	}

	static const Vector2Base ZERO;
};

// Common constants
template <class T>
const Vector2Base<T> Vector2Base<T>::ZERO(0, 0);

// Common types
using Vector2 = Vector2Base<float>;

// Hash function for cell positions, to use in std::unordered_set
template<>
struct std::hash<Vector2Base<int>>
{
	std::size_t operator()(const Vector2Base<int>& hk) const noexcept
	{
		// WIP find something better, now we abuse hash for strings which is perhaps not fast
		std::u32string_view sv(reinterpret_cast<const char32_t*>(&hk.x), 2);
		auto hashFunctor = std::hash<std::u32string_view>();
		std::size_t hashValue = hashFunctor(sv);
		return hashValue;

		//std::size_t _x = static_cast<std::size_t>(hk.x);
		//std::size_t _y = static_cast<std::size_t>(hk.y);
		//return _x | (_y << 32);
	}
};