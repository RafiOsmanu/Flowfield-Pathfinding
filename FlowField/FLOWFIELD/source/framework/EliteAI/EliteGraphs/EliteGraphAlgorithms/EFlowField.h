#pragma once
#include "framework/EliteAI/EliteNavigation/ENavigation.h"
#include <iterator>
#include "projects/Movement/SteeringBehaviors/SteeringAgent.h"
namespace Elite
{
	template <class T_NodeType, class T_ConnectionType>
	class FlowField
	{
	public:
		FlowField(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction);

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

		void FlowField<T_NodeType, T_ConnectionType>::CalculateFlowField(T_NodeType* pGoalNode, std::vector<SteeringAgent*>& pAgents);

	private:
		float GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const;

		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
		Heuristic m_HeuristicFunction;
	};

	template <class T_NodeType, class T_ConnectionType>
	FlowField<T_NodeType, T_ConnectionType>::FlowField(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction)
		: m_pGraph(pGraph)
		, m_HeuristicFunction(hFunction)
	{
	}

	template <class T_NodeType, class T_ConnectionType>
	float FlowField<T_NodeType, T_ConnectionType>::GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const
	{
		Vector2 toDestination = m_pGraph->GetNodePos(pEndNode) - m_pGraph->GetNodePos(pStartNode);
		return m_HeuristicFunction(abs(toDestination.x), abs(toDestination.y));
	}

	//FlowField Algorithm
	template <class T_NodeType, class T_ConnectionType>
	void FlowField<T_NodeType, T_ConnectionType>::CalculateFlowField(T_NodeType* pGoalNode, std::vector<SteeringAgent*>& pAgents)
	{
		//1.HeatMap
		//-------------------------------------------------------------
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
				if (m_pGraph->GetNode(connection->GetTo())->GetTerrainType() == TerrainType::Wall) continue;

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
			currentRecord.pNode->SetDistance(static_cast<int>(currentRecord.costSoFar));
		}

		std::vector<Vector2> vectorMap;
		int i{};
		//2.Vector Kernel Convolution
		//Combined direction given by the gradient + direction given by minimum distance value neighbour
		//-------------------------------------------------------------
		for (auto& node : closedList)
		{
			Vector2 flowFieldVector{};
			int ClosestDistance{100};
			bool wallAmongNeighbour{};
			int neighbourAmount{};
			for (auto& neighbour : m_pGraph->GetNodeConnections(node.pNode))
			{
				i = m_pGraph->GetNodeConnections(node.pNode).size();
				if (m_pGraph->GetNode(neighbour->GetTo())->GetTerrainType() == TerrainType::Wall || m_pGraph->GetNodeConnections(node.pNode).size() < 4) wallAmongNeighbour = true;
			}

			if (wallAmongNeighbour)
			{
				for (auto& neighbour : m_pGraph->GetNodeConnections(node.pNode))
				{
					//vectors pointing from center to neighbours
					Vector2 vectorToNeighbour = m_pGraph->GetNodeWorldPos(m_pGraph->GetNode(neighbour->GetTo())) - m_pGraph->GetNodeWorldPos(node.pNode);
					if (m_pGraph->GetNode(neighbour->GetTo())->GetDistance() < ClosestDistance)
					{
						ClosestDistance = m_pGraph->GetNode(neighbour->GetTo())->GetDistance();
						flowFieldVector = vectorToNeighbour. GetNormalized();
					}
				}
			}
			else
			{
				for (auto& neighbour : m_pGraph->GetNodeConnections(node.pNode))
				{
					//vectors pointing from center to neighbours
					Vector2 vectorToNeighbour = m_pGraph->GetNodeWorldPos(node.pNode) - m_pGraph->GetNodeWorldPos(m_pGraph->GetNode(neighbour->GetTo()));
					float distance = static_cast<float>(m_pGraph->GetNode(neighbour->GetTo())->GetDistance());
					flowFieldVector += vectorToNeighbour.GetNormalized() * distance;
					++neighbourAmount;
				}
			}
			
			//calculate average -> kernel convolution technique
			if (neighbourAmount > 0)
			{
				flowFieldVector /= static_cast<float>(neighbourAmount);
			}

			//set the direction in the node 
			node.pNode->SetDirection(flowFieldVector);
		}


		//3. Set agent direction to flow map
		//-------------------------------------------------------------
		
		for (auto& agent : pAgents)
		{
			if (m_pGraph->GetNodeAtWorldPos(agent->GetPosition()))
			{
				if (m_pGraph->GetNodeAtWorldPos(agent->GetPosition()) != pGoalNode)
					agent->SetLinearVelocity(m_pGraph->GetNodeAtWorldPos(agent->GetPosition())->GetDirection() * agent->GetMaxLinearSpeed());
				else
					agent->SetLinearVelocity({ 0.f, 0.f });
			}
		}
	}
}