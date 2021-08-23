// ShortestPathProblem.cpp: определяет точку входа для приложения.
//

#include "ShortestPathProblem.h"
#include <unordered_map>

using namespace std;

int main()
{
	pair<int, int> points[] = { {-61, 0}, {156, 0}, {0, 61}, {12, 1} };
	size_t points_num = 4;

	pair<size_t, size_t> connections[] = { {1, 3}, {1, 2}, {0, 1}, {0, 2} };
	size_t connections_num = 4;

	Graph<int> g(points, &points_num, connections, &connections_num);
	
	cout << g(0, 3);

	return 0;
}
