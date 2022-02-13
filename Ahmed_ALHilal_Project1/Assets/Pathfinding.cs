using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class Pathfinding : MonoBehaviour
{

    public Transform seeker, target;

    Grid_Class grid;

    int C_A = 0; // global vars
    int C_UCS = 0;
    int C_BFS = 0;
    int C_DFS = 0;

    void Awake()
    {
        grid = GetComponent<Grid_Class>();
    }

    void Update()
    {
        C_A = 0;
        C_UCS = 0;
        C_BFS = 0;
        C_DFS = 0;

        var clock_A = new System.Diagnostics.Stopwatch();
        var clock_UCS = new System.Diagnostics.Stopwatch();
        var clock_BFS = new System.Diagnostics.Stopwatch();
        var clock_DFS = new System.Diagnostics.Stopwatch();

        clock_A.Start();
        FindPath(seeker.position, target.position);
        clock_A.Stop();

        clock_UCS.Start();
        FindPath_UCS(seeker.position, target.position);
        clock_A.Stop();

        clock_BFS.Start();
        FindPath_BFS(seeker.position, target.position);
        clock_A.Stop();

        clock_DFS.Start();
        FindPath_DFS(seeker.position, target.position);
        clock_A.Stop();

        Debug.Log($"A* Execution Time: {clock_A.ElapsedMilliseconds} ms\nUCS Execution Time: {clock_UCS.ElapsedMilliseconds} ms\nBFS Execution Time: {clock_BFS.ElapsedMilliseconds} ms\nDFS Execution Time: {clock_DFS.ElapsedMilliseconds} ms");

    }

    void FindPath(Vector3 startPos, Vector3 targetPos) // A* Algo Pathfinding
    {
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        openSet.Add(startNode);
        
        while(openSet.Count > 0)
        {
            Node currentNode = openSet[0];
            for(int i=1; i < openSet.Count; i++)
            {
                if(openSet[i].fCost < currentNode.fCost || openSet[i].fCost == currentNode.fCost && openSet[i].hCost < currentNode.hCost)
                {
                    currentNode = openSet[i];
                }
            }

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            if(currentNode == targetNode)
            {
                RetracePath(startNode, targetNode);
                return;
            }

            foreach(Node neighbour in grid.GetNeighbours(currentNode))
            {
                if(!neighbour.walkable || closedSet.Contains(neighbour))
                {
                    continue;
                }

                int newMovementCostToNeighbour = currentNode.gCost + GetDistance(currentNode, neighbour);

                if(newMovementCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
                {
                    neighbour.gCost = newMovementCostToNeighbour;
                    neighbour.hCost = GetDistance(neighbour, targetNode);
                    neighbour.parent = currentNode;

                    if(!openSet.Contains(neighbour))
                    {
                        openSet.Add(neighbour);
                    }
                }
            }
        }
    }

    void FindPath_DFS(Vector3 startPos, Vector3 targetPos)
    {
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        Stack<Node> Stack_DFS = new Stack<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();

        Stack_DFS.Push(startNode);

        while (Stack_DFS.Count != 0)
        {

            Node currentNode = Stack_DFS.Pop();

            if (currentNode == targetNode)
            {
                RetracePath_DFS(startNode, targetNode);
                return;
            }

            closedSet.Add(currentNode);

            foreach (Node neighbour in grid.GetNeighbours(currentNode))
            {

                if (!neighbour.walkable || closedSet.Contains(neighbour))
                {
                    continue;
                }

                if (neighbour.walkable || !Stack_DFS.Contains(neighbour))
                {
                    closedSet.Add(neighbour);
                    neighbour.parent = currentNode;
                    Stack_DFS.Push(neighbour);
                }

            }
        }
    }

    void FindPath_BFS(Vector3 startPos, Vector3 targetPos)
    {

        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        Queue<Node> Queue_BFS = new Queue<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();

        Queue_BFS.Enqueue(startNode);

        while (Queue_BFS.Count != 0)
        {

            Node currentNode = Queue_BFS.Dequeue();

            if (currentNode == targetNode)
            {
                RetracePath_BFS(startNode, targetNode);
                return;
            }

            closedSet.Add(currentNode);

            foreach (Node neighbour in grid.GetNeighbours(currentNode))
            {

                if (!neighbour.walkable || closedSet.Contains(neighbour))
                {
                    continue;
                }

                if (neighbour.walkable || !Queue_BFS.Contains(neighbour))
                {
                    closedSet.Add(neighbour);
                    neighbour.parent = currentNode;
                    Queue_BFS.Enqueue(neighbour);
                }

            }
        }
    }

    void FindPath_UCS(Vector3 startPos, Vector3 targetPos)
    {

        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();

        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            Node currentNode = openSet[0];

            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].fCost < currentNode.fCost || openSet[i].fCost == currentNode.fCost)
                {
                    if (openSet[i].hCost < currentNode.hCost)
                        currentNode = openSet[i];
                }
            }

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            if (currentNode == targetNode)
            {
                RetracePath_UCS(startNode, targetNode);
                return;
            }

            foreach (Node neighbour in grid.GetNeighbours(currentNode))
            {

                if (!neighbour.walkable || closedSet.Contains(neighbour))
                {
                    continue;
                }

                int newCostToNeighbour = currentNode.gCost;

                if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
                {

                    neighbour.gCost = newCostToNeighbour;
                    neighbour.hCost = 0;
                    neighbour.parent = currentNode;

                    if (!openSet.Contains(neighbour))
                    {
                        openSet.Add(neighbour);
                    }
                }
            }
        }
    }

    void RetracePath(Node startNode, Node endNode) // A* Algo Retrace
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while(currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
            C_A++;
        }

        path.Reverse();
        grid.path = path;
    }

    void RetracePath_DFS(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
            C_DFS++;
        }
        path.Reverse();
        grid.path_DFS = path;
    }

    void RetracePath_BFS(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
            C_BFS++;
        }
        path.Reverse();
        grid.path_BFS = path;
    }

    void RetracePath_UCS(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
            C_UCS++;
        }
        path.Reverse();
        grid.path_UCS = path;
    }

    int GetDistance(Node nodeA, Node nodeB)
    {
        int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int dstY = Mathf.Abs(nodeB.gridY - nodeB.gridY);

        if(dstX > dstY)
        {
            return 14 * dstY + 10 * (dstX - dstY);
        }
        else
            return 14 * dstX + 10 * (dstY - dstX);
    }
}
