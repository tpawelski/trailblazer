# Trailblazer

Stanford CS106B assignment (Fall, 2016). As per the assignment guide (https://web.stanford.edu/class/archive/cs/cs106b/cs106b.1172//assn/trailblazer.html):

This assignment focuses on graphs, specifically on searching for paths in a graph. You will write several path finding algorithms.
We provide you with several support files, but you should not modify them. Turn in the following files;

-trailblazer.cpp , code to perform graph path searches
-map-custom.txt, a world map of your own creation.
-map-custon.jpg, a world map of your own creation.


## Overview
This program displays various road maps and allows users to find the shortest path between any two nodes. For example, if the user loaded the Stanford map and selected a node at the top of the Oval and another node at FroSoCo, your program should display the best route:

If you click on any two nodes in the world, the program will find a path from the starting position to the ending position. As it does so, it will color the vertexes green and yellow based on the colors assigned to them by the algorithm. Once the path is found, the program will highlight it and display information about the path cost in the console. The user can select one of four path-searching algorithms in the top menu:

-Breadth-first search (BFS)
-Dijkstra's algorithm
-A* search
-Alternative Route

The window contains several controls. You can load world maps by selecting them from the bottom drop-down menu and then clicking the "Load" button.

In your trailblazer.cpp file, you must write the following 4 functions for finding paths and creating mazes in a graph:

`Path breadthFirstSearch(RoadGraph& graph, Vertex* start, Vertex* end)`
`Path dijkstrasAlgorithm(RoadGraph& graph, Vertex* start, Vertex* end)`
`Path aStar(RoadGraph& graph, Vertex* start, Vertex* end)`
`Path alternativeRoute(RoadGraph& graph, Vertex* start, Vertex* end)`

Each of the first three implements a path-searching algorithm taught in class. You should search the given graph for a path from the given start vertex to the given end vertex. If you find such a path, the path you return should be a list of all vertexes along that path, with the starting vertex first (index 0 of the vector) and the ending vertex last.

A Path is a typedef of a Vector<Vertex *>. It is an alias for the same exact type.

If no path is found, return an empty path. If the start and end vertexes are the same, return a one-element vector containing only that vertex. Though some graphs may be undirected (all edges go both ways), your code should not assume this. You may assume that the graph passed in is not corrupt.

Our provided main client program will allow you to test each algorithm one at a time before moving on to the next. You can add more functions as helpers if you like, particularly to remove redundancy between some algorithms containing similar code.

The road network world is represented by a RoadGraph, which is a thin wrapper around the BasicGraph we saw in class. Each vertex represents a specific location on the world. An edge between two verticies means that there is a direct road between the two. The cost of the path is the time it takes to traverse that road.

Your algorithm should work on any RoadGraph instance, such as this map of Middle Earth that can help Frodo get from the Shire to Mount Doom.




