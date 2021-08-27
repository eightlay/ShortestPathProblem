#include "ShortestPathProblem.h"

using namespace std;

using std::chrono::high_resolution_clock;
using std::chrono::duration;
using std::chrono::milliseconds;

int main()
{
	Point<double> nodes[] = {
		{0, 0}, {7, 0}, {0, 9}, {11, 9}, {9, 11}, {0, 11}
	};
	size_t nodes_count = 6;

	Arc arcs[] = {
		{0, 1}, {0, 2}, {0, 5}, 
		{1, 0}, {1, 2}, {1, 3},
		{2, 0}, {2, 1}, {2, 3}, {2, 5},
		{3, 1}, {3, 2}, {3, 4},
		{4, 3}, {4, 5},
		{5, 0}, {5, 2}, {5, 4}
	};
	size_t arcs_count = 18;

	size_t node_from = 0;
	size_t node_to = 4;

	SPP<double> g(nodes, nodes_count, arcs, arcs_count);
	
	for (int i = 0; i < 3; i++)
	{
		// Choose algorithm
		char* method;
		if (i == 0) method = "dijkstra";
		else if (i == 1) method = "greedy";
		else method = "astar";

		// Timer start
		auto t1 = high_resolution_clock::now();
		
		// Find the path
		ShortestPath<double> result = g.find_path(node_from, node_to, method);
		
		// Timer end
		auto t2 = high_resolution_clock::now();

		// Number of milliseconds algorithm ran
		duration<double, std::milli> runtime = t2 - t1;

		// Print info
		cout << "Method: " << method << endl;
		cout << "Runtime: " << runtime.count() << " ms"  << endl;
		cout << "Path found: " << ((result.exist) ? "yes" : "no") << endl;
		cout << "Path is shortest: " << ((result.shortest) ? "yes" : "no") << endl;
		cout << "Shortest path: ";

		for (const size_t& node : result.path)
		{
			cout << node << " ";
		}

		cout << "\nDistance of the shortest path: " << result.distance << endl << endl;
	}

	return 0; 
}
