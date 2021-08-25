#pragma once

#include <cmath>

// Function to calculate distance
template <typename T>
double dist(std::pair<T, T>* n1, std::pair<T, T>* n2)
{
	return std::sqrt(
		std::pow(n1->first - n2->first, 2)
		+
		std::pow(n1->second - n2->second, 2)
	);
}