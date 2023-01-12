#include "stdafx.h"
#include "ENavGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAStar.h"

using namespace Elite;

Elite::NavGraph::NavGraph(const Polygon& contourMesh, float playerRadius = 1.0f) :
	Graph2D(false),
	m_pNavMeshPolygon(nullptr)
{
	//Create the navigation mesh (polygon of navigatable area= Contour - Static Shapes)
	m_pNavMeshPolygon = new Polygon(contourMesh); // Create copy on heap

	//Get all shapes from all static rigidbodies with NavigationCollider flag
	auto vShapes = PHYSICSWORLD->GetAllStaticShapesInWorld(PhysicsFlags::NavigationCollider);

	//Store all children
	for (auto shape : vShapes)
	{
		shape.ExpandShape(playerRadius);
		m_pNavMeshPolygon->AddChild(shape);
	}

	//Triangulate
	m_pNavMeshPolygon->Triangulate();

	//Create the actual graph (nodes & connections) from the navigation mesh
	CreateNavigationGraph();
}

Elite::NavGraph::~NavGraph()
{
	delete m_pNavMeshPolygon; 
	m_pNavMeshPolygon = nullptr;
}

int Elite::NavGraph::GetNodeIdxFromLineIdx(int lineIdx) const
{
	auto nodeIt = std::find_if(m_Nodes.begin(), m_Nodes.end(), [lineIdx](const NavGraphNode* n) { return n->GetLineIndex() == lineIdx; });
	if (nodeIt != m_Nodes.end())
	{
		return (*nodeIt)->GetIndex();
	}

	return invalid_node_index;
}

Elite::Polygon* Elite::NavGraph::GetNavMeshPolygon() const
{
	return m_pNavMeshPolygon;
}

void Elite::NavGraph::CreateNavigationGraph()
{
	//Graph2D graph{true};
	//1. Go over all the edges of the navigationmesh and create nodes
	
	//A. loop over all the lines of the polygon, Tip: use GetLines()
	//for each line
	for (const auto& line : m_pNavMeshPolygon->GetLines())
	{
		//check if line is connected to another triangle, Tip: use GetTrainglesFromLineIdx
		if (m_pNavMeshPolygon->GetTrianglesFromLineIndex(line->index).size() > 1)
		{
			//create NavGraphNode place in middle of line and give lineIdx
			NavGraphNode* edgeNode{ new NavGraphNode{GetNextFreeNodeIndex(), line->index, (line->p1 + line->p2) / 2.f}};
			AddNode(edgeNode);
		}
	}
	//B. for each triangle in the NavigationMesh, find the nodes
	for (const auto& triangle : m_pNavMeshPolygon->GetTriangles())
	{
		//loop over all the lineIdx Tip: look at the metaData property of triangle
		std::vector<int> validNodes{};
		for (const int& lineIdx : triangle->metaData.IndexLines)
		{
			if (GetNodeIdxFromLineIdx(lineIdx) != invalid_node_index)
			{
				validNodes.push_back(GetNodeIdxFromLineIdx(lineIdx));
			}
		}
		if (validNodes.size() == 2)
		{
			GraphConnection2D* connection{ new GraphConnection2D{} };
			connection->SetFrom(validNodes[0]);
			connection->SetTo(validNodes[1]);
			//graph.SetConnectionCostsToDistance();
			connection->SetCost(abs(Elite::Distance(GetNodePos(validNodes[0]), GetNodePos(validNodes[1]))));
			AddConnection(connection);
		}
		else if (validNodes.size() == 3)
		{
			for (int i{}; i < validNodes.size(); ++i)
			{
				GraphConnection2D* connection{ new GraphConnection2D{} };
				connection->SetFrom(validNodes[i]);
				if (i == 2)
				{
					connection->SetTo(validNodes[i - 2]);
					connection->SetCost(abs(Elite::Distance(GetNodePos(validNodes[0]), GetNodePos(validNodes[i - 2]))));
					AddConnection(connection);
				}
				else
				{
					connection->SetTo(validNodes[i + 1]);
					connection->SetCost(abs(Elite::Distance(GetNodePos(validNodes[0]), GetNodePos(validNodes[i + 1]))));
					AddConnection(connection);
				}
			}
			
		}
	}

	
	
	//2. Create connections now that every node is created
	
	//3. Set the connections cost to the actual distance
}

