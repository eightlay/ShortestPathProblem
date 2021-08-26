#pragma once

template<typename T = double>
struct Point
{
	Point(T x_ = 0, T y_ = 0) : x(x_), y (y_) {}
	T x;
	T y;
};