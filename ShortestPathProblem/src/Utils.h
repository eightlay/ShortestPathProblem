#pragma once

#include <cmath>

// Function to calculate distance
template <typename T>
double dist(std::pair<T, T>* v1, std::pair<T, T>* v2)
{
	return std::sqrt(
		std::pow(v1->first - v2->first, 2)
		+
		std::pow(v1->second - v2->second, 2)
	);
}