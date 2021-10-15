# Assignment2_A-_BFS_DFS_UCS
In this project we see multiple running scenarios with memory usage and run time of each algorithm in the pathfinding.cs file
This work was made using sebastian Lague first three videos as pillars for UCS, BFS, DFS
Main File containing algorithms	: Pathfinding.cs
Created by	: Driss Jaidi and Mohammed Chaouni
Version #	: 4.0
Updated on 	: October 14, 2021
References: { Sebastian Lague Youtube channel and github code
 https://www.youtube.com/watch?v=-L-WgKMFuhE&list=PLFt_AvWsXl0cq5Umv3pMC9SPnKjfp9eGW  
 https://github.com/SebLague/Pathfinding
 			}
This program contains 10 functions:
FindPath: finds shortest path using A*, the code can be modified for different A* heuristics
FindPathUCS: finds shortest path using uniform cost search, the code can be modified for different gCosts
FindPathBFS: finds shortest path using Breadth first search.
FindPathDFS: finds shortest path using Depth first search.
RetracePath: marks the nodes that constitue the shortest path of A*
RetracePathUCS: marks the nodes that constitue the shortest path of A*
RetracePathBFS: marks the nodes that constitue the shortest path of A*
RetracePathDFS: marks the nodes that constitue the shortest path of A*
GetDistance: gets the optimal distance between the two parameters of the function considering that some directions have a higher cost that normal movement
GetDistanceUCS: gets the optimal distance between the two parameters of the function considering a straight line distance using sqrt(par1, par2) all to the power two

File containing colorisation and grid structure: Grid.cs
File containing the node structure: Node.cs
File containing the scenes from screenshots: scenes
