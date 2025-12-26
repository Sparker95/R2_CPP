#pragma once

template <class T>
struct Vector2Base
{
	T x = 0;
	T y = 0;

	Vector2Base(T x, T y)
		: x(x), y(y)
	{
	}

	Vector2Base() = default;

	Vector2Base operator+(const Vector2Base& r) { return Vector2Base(x + r.x, y + r.y); }
	Vector2Base& operator+=(const Vector2Base& r) { x += r.x; y += r.y; return *this; }
	Vector2Base operator-(const Vector2Base& r) { return Vector2Base(x - r.x, y - r.y); }
	Vector2Base& operator-=(const Vector2Base& r) { x -= r.x; y -= r.y; return *this; }
	Vector2Base operator*(T r) { return Vector2Base(x * r, y * r); }
	Vector2Base& operator*=(T r) { x *= r; y *= r; return *this; }
	Vector2Base operator/(T r) { return Vector2Base(x / r, y / r); }
	Vector2Base& operator/=(T r) { x /= r; y /= r; return *this; }
	bool operator==(const Vector2Base<T>& r) const { return x == r.x && y == r.y; }

	static T Distance(Vector2Base<T> v0, Vector2Base<T> v1);
};

using Vector2 = Vector2Base<float>;
