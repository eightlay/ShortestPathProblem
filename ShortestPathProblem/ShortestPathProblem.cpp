// ShortestPathProblem.cpp: определяет точку входа для приложения.
//

#include "ShortestPathProblem.h"

using namespace std;

int main()
{
	Point<int> nodes[] = { {-61, 0}, {156, 0}, {0, 61}, {12, 1} };
	size_t nodes_count = 4;

	Arc arcs[] = { {2, 3}, {1, 2}, {0, 1} };
	size_t arcs_count = 3;

	SPP<int> g(nodes, nodes_count, arcs, arcs_count);
	
	for (int i = 0; i < 1; i++)
	{
		char* method;
		if (i == 0) method = "dijkstra";
		else if (i == 1) method = "greedy";
		else method = "astar";

		ShortestPath<int> result = g.find_path(0, 3, method);
	
		cout << "Method: " << method << endl;
		cout << "Path found: " << ((result.exist) ? "yes" : "no") << endl;
		cout << "Shortest path: ";

		for (const size_t& node : result.path)
		{
			cout << node << " ";
		}

		cout << "\nDistance of the shortest path: " << result.distance << endl << endl;
	}

	return 0; 
}
