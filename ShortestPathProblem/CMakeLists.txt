# CMakeList.txt: проект CMake для ShortestPathProblem; включите исходный код и определения,
# укажите здесь логику для конкретного проекта.
#
cmake_minimum_required (VERSION 3.8)

# Добавьте источник в исполняемый файл этого проекта.
add_executable (ShortestPathProblem "ShortestPathProblem.cpp" "ShortestPathProblem.h" "src/Graph.h" "src/SPP.h" "src/VertexPair.h" "src/Utils.h" "src/ShortestPath.h")

# TODO: Добавьте тесты и целевые объекты, если это необходимо.
