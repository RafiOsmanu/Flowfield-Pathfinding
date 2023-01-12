#pragma once
#include "framework/EliteAI/EliteNavigation/ENavigation.h"
#include <iterator>
namespace Elite
{
	template <class T_NodeType, class T_ConnectionType>
	class AStar
	{
	public:
		AStar(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction);

		// stores the optimal connection to a node and its total costs related to the start and end node of the path
		struct NodeRecord
		{
			T_NodeType* pNode = nullptr;
			T_ConnectionType* pConnection = nullptr;
			float costSoFar = 0.f; // accumulated g-costs of all the connections leading up to this one
			float estimatedTotalCost = 0.f; // f-cost (= costSoFar + h-cost)

			bool operator==(const NodeRecord& other) const
			{
				return pNode == other.pNode
					&& pConnection == other.pConnection
					&& costSoFar == other.costSoFar
					&& estimatedTotalCost == other.estimatedTotalCost;
			};

			bool operator<(const NodeRecord& other) const
			{
				return estimatedTotalCost < other.estimatedTotalCost;
			};
		};

		std::vector<T_NodeType*> FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode);
		void AStar<T_NodeType, T_ConnectionType>::FlowMap(T_NodeType* pStartNode, T_NodeType* pGoalNode);

	private:
		float GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const;

		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
		Heuristic m_HeuristicFunction;
	};

	template <class T_NodeType, class T_ConnectionType>
	AStar<T_NodeType, T_ConnectionType>::AStar(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction)
		: m_pGraph(pGraph)
		, m_HeuristicFunction(hFunction)
	{
	}

	template <class T_NodeType, class T_ConnectionType>
	std::vector<T_NodeType*> AStar<T_NodeType, T_ConnectionType>::FindPath(T_NodeType* pStartNode, T_NodeType* pGoalNode)
	{
		std::vector<T_NodeType*> path;
		std::vector<NodeRecord> openList;
		std::vector<NodeRecord> closedList;
		
		

		
		NodeRecord currentRecord;

		NodeRecord startRecord{};
		startRecord.pNode = pStartNode;
		startRecord.pConnection == nullptr;
		startRecord.estimatedTotalCost = startRecord.costSoFar + GetHeuristicCost(pStartNode, pGoalNode);
		//currentRecord = startRecord;

		openList.push_back(startRecord);

		while (!openList.empty())
		{
			currentRecord = *std::min_element(openList.begin(), openList.end());
			if (currentRecord.pNode == pGoalNode)
			{
				break;
			}
			else
			{
				
				/*std::vector<NodeRecord>::iterator closedListIt;
				std::vector<NodeRecord>::iterator openListIt;*/
				
				for (auto& connections : m_pGraph->GetNodeConnections(currentRecord.pNode))
				{
					bool isNotCheaper{false};
					float costSoFar{};
					costSoFar = connections->GetCost() + 1 + currentRecord.costSoFar;
					/*T_NodeType* nextNode = m_pGraph->GetNode(connections->GetTo());*/
					//closedListIt = std::find_if(closedList.begin(), closedList.end(), [nextNode](NodeRecord record) {return record.pNode == nextNode; });
					for (const NodeRecord& clRecord : closedList)
					{

						if (clRecord.pNode->GetIndex() == connections->GetTo())
						{
							if (clRecord.costSoFar < costSoFar)
							{
								isNotCheaper = true;
								break;
							}
							else
							{
								closedList.erase(std::remove(closedList.begin(), closedList.end(), clRecord));
							}
						}
					}
					//if (isNotCheaper) continue;
					for (const NodeRecord& olRecord : openList)
					{
						//openListIt = std::find_if(openList.begin(), openList.end(), [nextNode](NodeRecord record) {return record.pNode == nextNode; });
						if (olRecord.pNode->GetIndex() == connections->GetTo())
						{
							if (olRecord.costSoFar < costSoFar)
							{
								isNotCheaper = true;
								break;
							}
							else
							{
								openList.erase(std::remove(openList.begin(), openList.end(), olRecord));
							}
						}
					}
					if (isNotCheaper) continue;

					//expensive records removed, make new node and add it to the open list
					NodeRecord newRecord{};
					newRecord.pConnection = connections;
					newRecord.pNode = m_pGraph->GetNode(connections->GetTo());
					newRecord.costSoFar = costSoFar;
					newRecord.estimatedTotalCost = costSoFar + GetHeuristicCost(m_pGraph->GetNode(connections->GetTo()), pGoalNode);

					openList.push_back(newRecord);

					
				}

				openList.erase(std::remove(openList.begin(), openList.end(), currentRecord));
				closedList.push_back(currentRecord);
			}

		}

		if (currentRecord.pNode != pGoalNode)
			return std::vector<T_NodeType*>();

		while (currentRecord.pNode != startRecord.pNode)
		{
			path.emplace_back(currentRecord.pNode);
			for (const NodeRecord& clRecord : closedList)
			{
				if (clRecord.pNode->GetIndex() == currentRecord.pConnection->GetFrom())
				{
					currentRecord = clRecord;
					//path.push_back(currentRecord.pNode);
					break;
				}
			}
		}

		path.emplace_back(pStartNode);
		std::reverse(path.begin(), path.end());


		return path;
		//return std::vector<T_NodeType*>();
	}

	template <class T_NodeType, class T_ConnectionType>
	float AStar<T_NodeType, T_ConnectionType>::GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const
	{
		Vector2 toDestination = m_pGraph->GetNodePos(pEndNode) - m_pGraph->GetNodePos(pStartNode);
		return m_HeuristicFunction(abs(toDestination.x), abs(toDestination.y));
	}

	template <class T_NodeType, class T_ConnectionType>
	void AStar<T_NodeType, T_ConnectionType>::FlowMap(T_NodeType* pStartNode, T_NodeType* pGoalNode) 
	{
		std::vector<T_NodeType*> path;
		std::vector<NodeRecord> openList, closedList;

		NodeRecord startRecord{};
		startRecord.pNode = pGoalNode;
		startRecord.pConnection = nullptr;
		startRecord.costSoFar = 0;

		NodeRecord currentRecord;

		// place the goal node as the first node in the open list
		openList.push_back(startRecord);

		while (!openList.empty())
		{
			// get the node with the lowest cost from the open list and set it as the current node
			currentRecord = *std::min_element(openList.begin(), openList.end(), [](const auto& a, const auto& b) { return a.costSoFar < b.costSoFar; });

			// move the current node from the open list to the closed list
			closedList.push_back(currentRecord);
			openList.erase(std::remove(openList.begin(), openList.end(), currentRecord));

			// get all current node's neighbours
			for (auto& connection : m_pGraph->GetNodeConnections(currentRecord.pNode))
			{
				// if neighbour node is water, skip that node
				if (m_pGraph->GetNode(connection->GetTo())->GetTerrainType() == TerrainType::Water) continue;

				auto neighbour = m_pGraph->GetNode(connection->GetTo());
				// check if the neighbour is already in the closed list
				auto closedIter = std::find_if(closedList.begin(), closedList.end(), [neighbour](const auto& record) { return record.pNode == neighbour; });
				if (closedIter != closedList.end())
				{
					// if neighbour is already in closed list and has a higher cost, update its cost and connection
					if (currentRecord.costSoFar + 1 < closedIter->costSoFar)
					{
						closedIter->costSoFar = currentRecord.costSoFar + 1;
						closedIter->pConnection = connection;
					}
				}
				else
				{
					// go through open list to see if neighbour node is already visited
					bool nodeFound = false;
					for (auto& olRecord : openList)
					{
						if (olRecord.pNode == neighbour)
						{
							nodeFound = true;
							// if node is already in openList and cost is lower update openlist cost and connection
							if (currentRecord.costSoFar + 1 < olRecord.costSoFar)
							{
								// set current node distance to that of the neighbour to target
								olRecord.costSoFar = currentRecord.costSoFar + 1;
								olRecord.pConnection = connection;
							}
							break;
						}
					}
					if (!nodeFound)
					{
						// create a new record for this neighbour node
						NodeRecord neighbourRecord{};
						neighbourRecord.pNode = neighbour;
						neighbourRecord.pConnection = connection;
						neighbourRecord.costSoFar = currentRecord.costSoFar + 1;
						// put the neighbour node in the open list
						openList.push_back(neighbourRecord);
					}
				}
			}
			//draw distance for every grid to goalNode
			DEBUGRENDERER2D->DrawString(m_pGraph->GetNodeWorldPos(currentRecord.pNode->GetIndex()), std::to_string(static_cast<int>(currentRecord.costSoFar)).c_str());
		}
	}
}