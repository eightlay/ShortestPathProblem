// ShortestPathProblem.cpp: определяет точку входа для приложения.
//

#include "ShortestPathProblem.h"

using namespace std;

int main()
{
	pair<int, int> points[] = { {-61, 0}, {156, 0}, {0, 61}, {12, 1} };
	size_t points_num = 4;

	pair<size_t, size_t> connections[] = { {2, 3}, {1, 2}, {0, 1} };
	size_t connections_num = 3;

	SPP<int> g(points, &points_num, connections, &connections_num);
	
	ShortestPath<int> result = g.find_path(0, 3);
	
	cout << "Shortest path: ";

	for (const size_t& node : result.path)
	{
		cout << node << " ";
	}

	cout << "\nDistance of the shortest path: " << result.distance << endl;

	return 0; 
}
