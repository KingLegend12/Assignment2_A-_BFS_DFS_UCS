using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using Debug = UnityEngine.Debug;
public class Pathfinding : MonoBehaviour {

	public Transform seekertargetAstar, targetAstar,seekertargetUCS, targetUCS,seekertargetBFS, targetBFS, seekertargetDFS, targetDFS;
	public Stopwatch timerAstar = new Stopwatch();
	public Stopwatch timerUCS = new Stopwatch();
	public Stopwatch timerBFS = new Stopwatch();
	public Stopwatch timerDFS = new Stopwatch();
	Grid grid;

	void Awake() {
		grid = GetComponent<Grid> ();
	}

	void Update() {  //remove below comments to update the path 
	//FindPath (seekertargetAstar.position, targetAstar.position);
	//	FindPathUCS (seekertargetUCS.position, targetUCS.position);
	//	FindPathBFS (seekertargetBFS.position, targetBFS.position);
	//	FindPathDFS (seekertargetDFS.position, targetDFS.position);

		
	}
	void Start() { //only prints the times once at after the completion of each algorithm
		FindPath (seekertargetAstar.position, targetAstar.position);
		Debug.Log("A* runtime:"+timerAstar.Elapsed.ToString());
		FindPathUCS (seekertargetUCS.position, targetUCS.position);
		Debug.Log("UCS runtime:"+timerUCS.Elapsed.ToString());
		FindPathBFS (seekertargetBFS.position, targetBFS.position);
		Debug.Log("BFS runtime:"+timerBFS.Elapsed.ToString());
		FindPathDFS (seekertargetDFS.position, targetDFS.position);
		Debug.Log("DFS runtime:"+timerDFS.Elapsed.ToString());

		
		
		
		
		
	}
	void FindPath(Vector3 startPos, Vector3 targetPos) {
		
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		timerAstar.Start();
		openSet.Add(startNode);
        
		
		while (openSet.Count > 0) {
			
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost) {
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) {
				timerAstar.Stop();
				RetracePath(startNode,targetNode);
				
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistance(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
		timerAstar.Stop();
		
	}

void FindPathUCS(Vector3 startPos, Vector3 targetPos) {
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		timerUCS.Start();
		openSet.Add(startNode);
		
		while (openSet.Count > 0) {
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].gCost < node.gCost || openSet[i].gCost == node.gCost) {
					
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) {
				timerUCS.Stop();
				RetracePathUCS(startNode,targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistanceUCS(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
		timerUCS.Stop();
	}
     

	void FindPathBFS(Vector3 startPos, Vector3 targetPos) { 
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
      Queue<Node> queueBFS = new Queue<Node>();
	  HashSet<Node> closedSet = new HashSet<Node>();
	  timerBFS.Start();
		queueBFS.Enqueue(startNode);
        
		while(queueBFS.Count!=0) { 
			Node currentNode= queueBFS.Dequeue();
			if (currentNode == targetNode) {
				timerBFS.Stop();
				RetracePathBFS(startNode,targetNode);
				return;
			}
			closedSet.Add(currentNode);
		foreach (Node neighbour in grid.GetNeighbours(currentNode))
		{
			if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}
			if (neighbour.walkable || !queueBFS.Contains(neighbour)) {
					closedSet.Add(neighbour);
					neighbour.parent = currentNode; 
					queueBFS.Enqueue(neighbour);
				}
		}
		}
		timerBFS.Stop();
	}

	void FindPathDFS(Vector3 startPos, Vector3 targetPos) { 
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
      Stack<Node> StackDFS = new Stack<Node>();
	  HashSet<Node> closedSet = new HashSet<Node>();
	    timerDFS.Start();
		StackDFS.Push(startNode);
        
		while(StackDFS.Count!=0) { 
			Node currentNode= StackDFS.Pop();
			if (currentNode == targetNode) {
				RetracePathDFS(startNode,targetNode);
				return;
			}
			closedSet.Add(currentNode);
		foreach (Node neighbour in grid.GetNeighbours(currentNode))
		{
			if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}
			if (neighbour.walkable || !StackDFS.Contains(neighbour)) {
					closedSet.Add(neighbour);
					neighbour.parent = currentNode; 
					StackDFS.Push(neighbour);
				}
		}
		}
		timerDFS.Stop();
	}

	void RetracePath(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		grid.path = path;

	}
    void RetracePathUCS(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		grid.pathUCS = path;

	}
    void RetracePathBFS(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		grid.pathBFS = path;

	}
    void RetracePathDFS(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		grid.pathDFS = path;

	}
	int GetDistance(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 14*dstY + 10* (dstX-dstY);
		return 14*dstX + 10 * (dstY-dstX);
	}
	int GetDistanceUCS(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 10*dstY + 10* (dstX-dstY);
		return 10*dstX + 10 * (dstY-dstX);
	}
}