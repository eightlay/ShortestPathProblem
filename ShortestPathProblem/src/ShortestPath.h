#pragma once

#include <list>

// Result of shortest path finding algorithm
template <typename T>
struct ShortestPath
{
	ShortestPath(std::list<size_t> path_, T distance_)
	{
		path = path_;
		distance = distance_;
	}

	std::list<size_t> path;
	T distance;
};