#pragma once
#include <vector>
#include <iostream>
#include "framework/EliteMath/EMath.h"
#include "framework\EliteAI\EliteGraphs\ENavGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAStar.h"

namespace Elite
{
	class NavMeshPathfinding
	{
	public:
		static std::vector<Vector2> FindPath(Vector2 startPos, Vector2 endPos, NavGraph* pNavGraph, std::vector<Vector2>& debugNodePositions, std::vector<Portal>& debugPortals)
		{
			//Create the path to return
			std::vector<Vector2> finalPath{};

			//A. Get the start and endTriangle
			const Triangle* startTriangle = pNavGraph->GetNavMeshPolygon()->GetTriangleFromPosition(startPos);
			const Triangle* endTriangle = pNavGraph->GetNavMeshPolygon()->GetTriangleFromPosition(endPos);
			//check if they exist otherwise return
			if (!startTriangle) return finalPath;
			if (!endTriangle) return finalPath;

			if (startTriangle == endTriangle)
			{
				finalPath.push_back(endPos);
				return finalPath;
			}

			//We have valid start/end triangles and they are not the same
			//=> Start looking for a path
			//Copy the graph

			auto graphClone{ pNavGraph->Clone() };
			
			//Create extra node for the Start Node (Agent's position
			NavGraphNode* startNode{new NavGraphNode(graphClone->GetNextFreeNodeIndex(),-1, startPos)};
			graphClone->AddNode(startNode);
			for (const int& edgeIdx : startTriangle->metaData.IndexLines)
			{
				int nodeIdx{ pNavGraph->GetNodeIdxFromLineIdx(edgeIdx) };
				if (nodeIdx == invalid_node_index) continue;
				GraphConnection2D* connection{ new GraphConnection2D };
				connection->SetFrom(nodeIdx);
				connection->SetTo(startNode->GetIndex());
				connection->SetCost(startPos.Distance(graphClone->GetNodePos(nodeIdx)));
				graphClone->AddConnection(connection);
			}

			NavGraphNode* endNode{ new NavGraphNode(graphClone->GetNextFreeNodeIndex(),-1, endPos) };
			graphClone->AddNode(endNode);
			for (const int& edgeIdx : endTriangle->metaData.IndexLines)
			{
				int nodeIdx{ pNavGraph->GetNodeIdxFromLineIdx(edgeIdx) };
				if (nodeIdx == invalid_node_index) continue;
				GraphConnection2D* connection{ new GraphConnection2D };
				connection->SetFrom(nodeIdx);
				connection->SetTo(endNode->GetIndex());
				connection->SetCost(endPos.Distance(graphClone->GetNodePos(nodeIdx)));
				graphClone->AddConnection(connection);
			}

			auto pathfinder = AStar<NavGraphNode, GraphConnection2D>(graphClone.get(), Elite::HeuristicFunctions::Chebyshev);

			std::vector<NavGraphNode*> pathNodes{ pathfinder.FindPath(startNode, endNode) };

			for (NavGraphNode*& node : pathNodes)
			{
				finalPath.push_back(node->GetPosition());
			}
			
			debugNodePositions = finalPath;
			
			//Create extra node for the endNode
			
			//Run A star on new graph
			
			//OPTIONAL BUT ADVICED: Debug Visualisation

			//Run optimiser on new graph, MAKE SURE the A star path is working properly before starting this section and uncommenting this!!!
			const auto portals = SSFA::FindPortals(pathNodes, pNavGraph->GetNavMeshPolygon());
			finalPath = SSFA::OptimizePortals(portals);

			debugPortals = portals;

			return finalPath;
		}
	};
}
