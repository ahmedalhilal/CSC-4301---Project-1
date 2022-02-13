using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Grid_Class : MonoBehaviour
{
    //public Transform player; The greyed code in this script is for testing purposes.
    public LayerMask unwalkableMask;
    public Vector2 gridWorldSize;
    public float nodeRadius;
    Node[,] grid;

    float nodeDiameter; 
    int gridSizeX, gridSizeY;


    void Start()
    {
        nodeDiameter = nodeRadius*2;
        gridSizeX = Mathf.RoundToInt(gridWorldSize.x/nodeDiameter);
        gridSizeY = Mathf.RoundToInt(gridWorldSize.y/nodeDiameter);
        CreateGrid();
    }

    void CreateGrid()
    {
        grid = new Node[gridSizeX,gridSizeY]; 
        Vector3 worldBottomLeft = transform.position - Vector3.right * gridWorldSize.x/2 - Vector3.forward * gridWorldSize.y/2;

        for(int x = 0; x < gridSizeX; x++)
        {
            for(int y = 0; y < gridSizeY; y++)
            {
                Vector3 worldPoint = worldBottomLeft + Vector3.right * (x * nodeDiameter + nodeRadius) + Vector3.forward * (y * nodeDiameter + nodeRadius);
                bool walkable = !(Physics.CheckSphere(worldPoint, nodeRadius, unwalkableMask));
                grid [x,y] = new Node(walkable, worldPoint, x, y);
            }
        }
    }

    public List<Node> GetNeighbours(Node node)
    {
        List<Node> neighbours = new List<Node>();

        for(int x = -1; x <= 1; x++)
        {
            for(int y = -1; y <= 1; y++)
            {
                if(x == 0 && y == 0)
                {
                    continue;
                }

                int checkX = node.gridX + x;
                int checkY = node.gridY + y;

                if(checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
                {
                    neighbours.Add(grid[checkX, checkY]);
                }
            }
        }
        
        return neighbours;
    }

    public Node NodeFromWorldPoint(Vector3 worldPosition)
    {
        float PercentX = (worldPosition.x + gridWorldSize.x/2) / gridWorldSize.x;
        float PercentY = (worldPosition.z + gridWorldSize.y/2) / gridWorldSize.y;
        PercentX = Mathf.Clamp01(PercentX);
        PercentY = Mathf.Clamp01(PercentY);

        int x = Mathf.RoundToInt((gridSizeX - 1) * PercentX);
        int y = Mathf.RoundToInt((gridSizeY - 1) * PercentY);

        return grid[x,y];
    }

    public List<Node> path;
    public List<Node> path_DFS;
    public List<Node> path_BFS;
    public List<Node> path_UCS;

    void OnDrawGizmos()
    {
        Gizmos.DrawWireCube(transform.position, new Vector3(gridWorldSize.x, 1, gridWorldSize.y));
        if (grid != null) // The Color Black is for A*
        {
            //Node playerNode = NodeFromWorldPoint(player.position);
            foreach (Node n in grid)
            {
                Gizmos.color = (n.walkable)? Color.white : Color.red;
                /*if(playerNode == n)
                {
                    Gizmos.color = Color.cyan;
                }*/

                if(path != null)
                {
                    if(path.Contains(n))
                    {
                        Gizmos.color = Color.black;
                    }
                }

                Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - 0.1f));
            }
        }

        if (path_BFS != null) // The Color Yellow is for BFS
        {
            foreach (Node n in path_BFS)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - .1f));
            }
        }

        if (path_DFS != null) // The Color Green is for DFS
        {
            foreach (Node n in path_DFS)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - .1f));
            }
        }

        if (path_UCS != null) // The Color Cyan is for UCS
        {
            foreach (Node n in path_UCS)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - .1f));
            }
        }
    }
}
