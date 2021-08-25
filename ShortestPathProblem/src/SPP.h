#pragma once

#include "Graph.h"
#include "ShortestPath.h"

#include <cstring>
#include <limits>

template <typename T>
class SPP : public Graph<T>
{
	using Graph::Graph;

public:
	//SPP(std::pair<T, T>* points, size_t* points_num,
	//	std::pair<size_t, size_t>* connections = nullptr, size_t* connections_num = 0):
	//	Graph(points, points_num, connections, connections_num)
	//{
	//	
	//}
	
	// Find a shortes path between node_from and node_to using specifed method
	ShortestPath<T> find_path(size_t node_from, size_t node_to, char* method = "dijkstra")
	{
		if (std::strcmp(method, "dijkstra") == 0)
		{
			return dijkstra(node_from, node_to);
		}

		return ShortestPath<T>({}, 0);
	}

private:
	// Find a shortes path between node_from and node_to using Dijkstra's algorithm
	ShortestPath<T> dijkstra(size_t node_from, size_t node_to)
	{
		// Is node unvisited or not
		bool* unvisited = new bool[node_num];

		// Tentative distance to the node
		T* tentative_dist = new T[node_num];

		// Node we came from to current the node
		size_t* came_from = new size_t[node_num];

		// Current node
		size_t current = node_from;

		// Initial values
		for (size_t i = 0; i < node_num; i++)
		{
			if (node_from == i)
			{
				unvisited[i] = false;
				tentative_dist[i] = 0;
			}
			else
			{
				unvisited[i] = true;
				tentative_dist[i] = std::numeric_limits<T>::max();
			}

			came_from[i] = i;
		}

		// Dijkstra's algorithm
		while (current != node_to)
		{
			// Mark current node as visited
			unvisited[current] = false;

			// Updating tentative distances
			// Iterate through all current's node neighbours
			for (const size_t& neighbour : arcs[current])
			{
				// If neighbour is unvisited
				if (unvisited[neighbour])
				{
					// Calculate new tentative distance
					T dist = tentative_dist[current] + get_distance(current, neighbour);

					// Compare current and newly calculated tentative distances
					if (dist < tentative_dist[neighbour])
					{
						// Assign if the new one is the smaller one
						tentative_dist[neighbour] = dist;
						came_from[neighbour] = current;
					}
				}
			}

			// Choosing the next node
			// Find unvisited node with minimal tentative distance
			size_t min_node = current;
			T min_dist = std::numeric_limits<T>::max();

			for (size_t i = 0; i < node_num; i++)
			{
				if (unvisited[i])
				{
					if (tentative_dist[i] < min_dist)
					{
						min_node = i;
						min_dist = tentative_dist[i];
					}
				}
			}
			
			// If tentative distances are the same take the first unvisited node
			if (min_node == current)
			{
				for (size_t i = 0; i < node_num; i++)
				{
					if (unvisited[i])
					{
						min_node = i;
						min_dist = tentative_dist[i];
						break;
					}
				}
			}

			// Assign new current node
			current = min_node;
		}

		// Construct the path
		std::list<size_t> path = { node_to };

		while (current != node_from)
		{
			size_t prev_node = came_from[current];
			path.push_back(prev_node);
			current = prev_node;
		}

		path.reverse();

		// Distance to node_to
		T path_distance = tentative_dist[node_to];

		// Free memory
		delete[] unvisited;
		delete[] tentative_dist;
		delete[] came_from;

		return ShortestPath<T>(path, path_distance);
	}
};