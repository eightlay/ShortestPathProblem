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
		vertex_num = *points_num;

		if (connections != nullptr)
		{
			arcs_num = *connections_num;

			if (arcs_num == 0)
			{
				throw std::invalid_argument("Connections list is not empty, but connections number is zero");
			}
			
			for (size_t i = 0; i < vertex_num; i++)
			{
				__vertices.insert({ i, {} });
			}

			for (size_t c = 0; c < arcs_num; c++)
			{
				const size_t from = connections[c].first;
				const size_t to = connections[c].second;

				__distances[{from, to}] = (T)dist<T>(&points[from], &points[to]);

				__vertices[from].push_back(to);
				__vertices[to].push_back(from);
			}

			for (size_t i = 0; i < vertex_num; i++)
			{
				__vertices[i].sort();
			}
		}
		else
		{
			arcs_num = vertex_num * (vertex_num - 1) / 2;

			for (size_t i = 0; i < vertex_num; i++)
			{
				__vertices.insert({ i, {} });

				for (size_t j = 0; j < i; j++)
				{
					__vertices[i].push_back(j);
				}

				for (size_t j = i + 1; j < vertex_num; j++)
				{
					__distances[{i, j}] = (T)dist<T>(&points[i], &points[j]);
					__vertices[i].push_back(j);
				}
			}
		}
	}

	// Deconstructor
	~Graph() {}

	// Get distance from v1 to v2
	T operator()(T v1, T v2)
	{
		if (v1 == v2) return 0;
		if (v1 < 0 || v2 < 0) throw std::out_of_range("Index out of graph vertices must be non-negative");
		if (v1 > vertex_num || v2 > vertex_num) throw std::out_of_range("Index out of graph vertices range");

		const size_t from = (size_t)std::min(v1, v2);
		const size_t to = (size_t)std::max(v1, v2);
		
		if (__distances.find({ from, to }) == __distances.end())
		{
			throw std::out_of_range("No such vertice");
		}

		return __distances[{from, to}];
	}

	// Get vertices
	std::list<size_t> get_vertices(size_t from) const
	{
		return __vertices[from];
	}

private:
	mutable std::unordered_map<VertexPair, T> __distances;
	mutable std::unordered_map<size_t, std::list<size_t>> __vertices;
	mutable size_t vertex_num;
	mutable size_t arcs_num;
};