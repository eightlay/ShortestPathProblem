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
	ShortestPath<T> find_path(size_t node_from, size_t node_to, char* method = "dijkstra",
		std::function<T(Point<T>*, Point<T>*)> heuristic = nullptr)
	{
		if (node_from == node_to)
		{
			return ShortestPath<T>({node_from}, 0, true, true);
		}

		if (std::strcmp(method, "dijkstra") == 0)
		{
			return dijkstra(node_from, node_to);
		}

		if (std::strcmp(method, "greedy") == 0)
		{
			return greedy(node_from, node_to, heuristic);
		}

		if (std::strcmp(method, "astar") == 0)
		{
			return astar(node_from, node_to, heuristic);
		}

		return ShortestPath<T>({}, 0, false, false);
	}

private:
	// Find a shortes path between node_from and node_to using Dijkstra's algorithm
	ShortestPath<T> dijkstra(size_t node_from, size_t node_to)
	{
		// Is node unvisited or not
		bool* unvisited = new bool[nodes_count];

		// Tentative distance to the node
		T* tentative_dist = new T[nodes_count];

		// Node we came from to the current one
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

			// Stop algorithm if the smallest tentative distance 
			// among the nodes in the unvisited set is infinity.
			// It means, that there is no path from node_from to node_to
			if (current == min_node) return ShortestPath<T>({}, 0, false, false);

			// Assign new current node
			current = min_node;

			// If the destination is reached, stop the algorithm
			if (current == node_to) break;

			// Mark current node as visited
			unvisited[current] = false;
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

		return ShortestPath<T>(path, path_distance, true, true);
	}

	// Find a shortes path between node_from and node_to using Greedy heuristic algorithm
	ShortestPath<T> greedy(size_t node_from, size_t node_to,
		std::function<T(Point<T>*, Point<T>*)> heuristic)
	{
		// If heuristic is not specified, use distance function of the graph
		if (heuristic == nullptr)
		{
			heuristic = distance;
		}

		// Priority queue
		auto cmp = [&](size_t left, size_t right) 
		{ return heuristic(&nodes[left], &nodes[node_to]) > heuristic(&nodes[right], &nodes[node_to]); };

		std::priority_queue<size_t, std::vector<size_t>, decltype(cmp)> frontier(cmp);

		// Is node unvisited or not
		bool* unvisited = new bool[nodes_count];

		// Node we came from to the current one
		size_t* came_from = new size_t[nodes_count];

		// Current node
		size_t current = node_from;

		// Initial values
		for (size_t i = 0; i < nodes_count; i++)
		{
			unvisited[i] = true;
			came_from[i] = i;
		}

		unvisited[node_from] = false;
		
		// Greedy heuristic algorithm
		while (true)
		{
			// Iterate through all current's node neighbours
			for (const size_t& neighbour : get_neighbours(current))
			{
				// If neighbour is unvisited
				if (unvisited[neighbour])
				{
					frontier.push(neighbour);
					came_from[neighbour] = current;
				}
			}

			// If there is no other nodes in frontier,
			// there is no path from node_from to node_to
			if (frontier.size() == 0) return ShortestPath<T>({}, 0, false, false);

			// Assign new current node
			current = frontier.top();
			frontier.pop();

			// If the destination is reached, stop the algorithm
			if (current == node_to) break;

			// Mark current node as visited
			unvisited[current] = false;
		}

		// Construct the path
		std::list<size_t> path = { node_to };
		T path_distance = 0;

		while (current != node_from)
		{
			size_t prev_node = came_from[current];
			
			path.push_back(prev_node);
			path_distance += get_distance(prev_node, current);

			current = prev_node;
		}

		path.reverse();

		// Free memory
		delete[] unvisited;
		delete[] came_from;

		return ShortestPath<T>(path, path_distance, true, false);
	}

	// Find a shortes path between node_from and node_to using A* algorithm
	ShortestPath<T> astar(size_t node_from, size_t node_to,
		std::function<T(Point<T>*, Point<T>*)> heuristic)
	{
		// If heuristic is not specified, use distance function of the graph
		if (heuristic == nullptr)
		{
			heuristic = distance;
		}

		// Is node unvisited or not
		bool* unvisited = new bool[nodes_count];

		// Tentative distance to the node
		T* tentative_dist = new T[nodes_count];

		// Priority queue
		auto cmp = [&](size_t left, size_t right)
		{ 
			return 
				heuristic(&nodes[left], &nodes[node_to]) + tentative_dist[left] 
				> 
				heuristic(&nodes[right], &nodes[node_to]) + tentative_dist[right];
		};

		std::priority_queue<size_t, std::vector<size_t>, decltype(cmp)> frontier(cmp);

		// Node we came from to the current one
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
						frontier.push(neighbour);
					}
				}
			}

			// If there is no other nodes in frontier,
			// there is no path from node_from to node_to
			if (frontier.size() == 0) return ShortestPath<T>({}, 0, false, false);

			// Assign new current node
			current = frontier.top();
			frontier.pop();

			// If the destination is reached, stop the algorithm
			if (current == node_to) break;

			// Mark current node as visited
			unvisited[current] = false;
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

		return ShortestPath<T>(path, path_distance, true, true);
	}
};