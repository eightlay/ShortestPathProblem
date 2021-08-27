#pragma once

#include "Point.h"

#include <cmath>

// Function to calculate Euclidian distance
template <typename T>
inline T eucl_dist(Point<T>* from, Point<T>* to)
{
	return (T)std::sqrt(
		std::pow(from->x - to->x, 2)
		+
		std::pow(from->y - to->y, 2)
	);
}

// Function to calculate Manhattan distance
template <typename T>
inline T manh_dist(Point<T>* from, Point<T>* to)
{
	return abs(from->x - to->x) + abs(from->y - to->y);
}
