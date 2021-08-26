#pragma once

#include "Utils.h"
#include "Arc.h"
#include "Point.h"

#include <list>
#include <vector>
#include <functional>

// Graph class
template <typename T>
class Graph
{
public:
	// Constructor
	Graph(Point<T>* nodes_, size_t nodes_count_, Arc* arcs_, size_t arcs_count_,
		std::function<T(Point<T>*, Point<T>*)> distance_ = eucl_dist<T>)
	{
		// Distance function
		distance = distance_;

		// Nodes
		nodes_count = nodes_count_;

		nodes = new Point<T>[nodes_count];

		for (size_t node = 0; node < nodes_count; node++)
		{
			nodes[node] = nodes_[node];
			arcs.push_back({});
		}

		// Arcs
		arcs_count = arcs_count_;

		for (size_t arc = 0; arc < arcs_count; arc++)
		{
			arcs[arcs_[arc].from].push_back(arcs_[arc].to);
		}

		for (size_t node = 0; node < nodes_count; node++)
		{
			arcs[node].sort();
		}
	}

	// Deconstructor
	~Graph() 
	{
		delete[] nodes;
	}

	// Get distance from n1 to n2
	inline T get_distance(size_t from, size_t to) const
	{
		return distance(&nodes[from], &nodes[to]);
	}

	// Get arcs
	inline std::list<size_t> get_neighbours(size_t node_from) const
	{
		return arcs[node_from];
	}

protected:
	mutable Point<T>* nodes;
	mutable std::vector<std::list<size_t>> arcs;
	mutable size_t nodes_count;
	mutable size_t arcs_count;

	mutable std::function<T(Point<T>*, Point<T>*)> distance;
};