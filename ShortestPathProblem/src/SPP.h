#pragma once

#include "Graph.h"
#include "ShortestPath.h"

#include <cstring>
#include <limits>
#include <queue>

template <typename T>
class SPP : public Graph<T>
{
	using Graph::Graph;

public:
	// Find a shortes path between node_from and node_to using specifed method
	ShortestPath<T> find_path(size_t node_from, size_t node_to, char* method = "dijkstra")
	{
		if (std::strcmp(method, "dijkstra") == 0)
		{
			return dijkstra(node_from, node_to);
		}

		return ShortestPath<T>({}, 0, false);
	}

private:
	// Find a shortes path between node_from and node_to using Dijkstra's algorithm
	ShortestPath<T> dijkstra(size_t node_from, size_t node_to)
	{
		// Is node unvisited or not
		bool* unvisited = new bool[nodes_count];

		// Tentative distance to the node
		T* tentative_dist = new T[nodes_count];

		// Node we came from to current the node
		size_t* came_from = new size_t[nodes_count];

		// Current node
		size_t current = node_from;

		// Initial values
		for (size_t node = 0; node < nodes_count; node++)
		{
			unvisited[node] = true;
			tentative_dist[node] = std::numeric_limits<T>::max();
			came_from[node] = node;
		}

		unvisited[node_from] = false;
		tentative_dist[node_from] = 0;

		// Dijkstra's algorithm
		while (true)
		{
			// Mark current node as visited
			unvisited[current] = false;

			// Updating tentative distances
			// Iterate through all current's node neighbours
			for (const size_t& neighbour : get_neighbours(current))
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

			for (size_t node = 0; node < nodes_count; node++)
			{
				if (unvisited[node])
				{
					if (tentative_dist[node] < min_dist)
					{
						min_node = node;
						min_dist = tentative_dist[node];
					}
				}
			}
			
			// If tentative distances are the same take the first unvisited node
			if (min_node == current)
			{
				for (size_t node = 0; node < nodes_count; node++)
				{
					if (unvisited[node])
					{
						min_node = node;
						min_dist = tentative_dist[node];
						break;
					}
				}
			}

			// Assign new current node
			current = min_node;

			// If the destination is reached, stop the algorithm
			if (current == node_to) break;
			
			// Check if there still are unvisited nodes
			bool has_unvisited = false;

			for (size_t node = 0; node < nodes_count; node++)
			{
				has_unvisited |= unvisited[node];
			}

			// If there is no unvisited nodes, there is no path between node_from and node_to
			return ShortestPath<T>({}, 0, false);
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

		return ShortestPath<T>(path, path_distance, true);
	}

	// Find a shortes path between node_from and node_to using Greedy heuristic algorithm
	ShortestPath<T> greedy(size_t node_from, size_t node_to,
		T(*heuristic)(std::pair<T, T>*, std::pair<T, T>*) = manh_dist<T>)
	{
		//// Compare function for priority queue
		//auto cmp = [](std::pair<T, T>* left, std::pair<T, T>* right) { return heuristic(left, right); };

		//// Is node unvisited or not
		//bool* unvisited = new bool[nodes_count];

		//// Node we came from to current the node
		//size_t* came_from = new size_t[nodes_count];

		//// Current node
		//size_t current = node_from;

		//// Initial values
		//for (size_t i = 0; i < nodes_count; i++)
		//{
		//	unvisited[i] = true;
		//	came_from[i] = i;
		//}

		//unvisited[node_from] = false;

		//std::priority_queue<size_t, std::vector<size_t>, decltype(cmp)> q(cmp);
		//
		//// TODO: greedy algo

		//// Construct the path
		//std::list<size_t> path = { node_to };

		//while (current != node_from)
		//{
		//	size_t prev_node = came_from[current];
		//	path.push_back(prev_node);
		//	current = prev_node;
		//}

		//path.reverse();

		//// Distance to node_to
		//T path_distance = tentative_dist[node_to];

		//// Free memory
		//delete[] unvisited;
		//delete[] tentative_dist;
		//delete[] came_from;

		//return ShortestPath<T>(path, path_distance);
		return ShortestPath<T>({}, 0);
	}
};