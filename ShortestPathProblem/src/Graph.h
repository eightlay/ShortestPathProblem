#pragma once

#include "Utils.h"
#include "VertexPair.h"

#include <list>
#include <unordered_map>

// Graph class
template <typename T>
class Graph
{
public:
	// Constructor
	Graph(std::pair<T, T>* points, size_t* points_num,
		std::pair<size_t, size_t>* connections = nullptr, size_t* connections_num = 0)
	{
		node_num = *points_num;

		if (connections != nullptr)
		{
			arcs_num = *connections_num;

			if (arcs_num == 0)
			{
				throw std::invalid_argument("Connections list is not empty, but connections number is zero");
			}
			
			for (size_t i = 0; i < node_num; i++)
			{
				arcs.insert({ i, {} });
			}

			for (size_t c = 0; c < arcs_num; c++)
			{
				const size_t from = connections[c].first;
				const size_t to = connections[c].second;

				distances[{from, to}] = (T)dist<T>(&points[from], &points[to]);

				arcs[from].push_back(to);
				arcs[to].push_back(from);
			}

			for (size_t i = 0; i < node_num; i++)
			{
				arcs[i].sort();
			}
		}
		else
		{
			arcs_num = node_num * (node_num - 1) / 2;

			for (size_t i = 0; i < node_num; i++)
			{
				arcs.insert({ i, {} });

				for (size_t j = 0; j < i; j++)
				{
					arcs[i].push_back(j);
				}

				for (size_t j = i + 1; j < node_num; j++)
				{
					distances[{i, j}] = (T)dist<T>(&points[i], &points[j]);
					arcs[i].push_back(j);
				}
			}
		}
	}

	// Deconstructor
	~Graph() {}

	// Get distance from n1 to n2
	T get_distance(size_t n1, size_t n2)
	{
		if (n1 == n2) return 0;
		if (n1 < 0 || n2 < 0) throw std::out_of_range("Index out of graph nodes must be non-negative");
		if (n1 > node_num || n2 > node_num) throw std::out_of_range("Index out of graph nodes range");

		const size_t from = (size_t)std::min(n1, n2);
		const size_t to = (size_t)std::max(n1, n2);
		
		if (distances.find({ from, to }) == distances.end())
		{
			throw std::out_of_range("No such node");
		}

		return distances[{from, to}];
	}

	// Get distance from n1 to n2
	T operator()(size_t n1, size_t n2)
	{
		return get_distance(n1, n2);
	}

	// Get arc
	std::list<size_t> get_arcs(size_t node_from) const
	{
		return arcs[node_from];
	}

protected:
	mutable std::unordered_map<VertexPair, T> distances;
	mutable std::unordered_map<size_t, std::list<size_t>> arcs;
	mutable size_t node_num;
	mutable size_t arcs_num;
};