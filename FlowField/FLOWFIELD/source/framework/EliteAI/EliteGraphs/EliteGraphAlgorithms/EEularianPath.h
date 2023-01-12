#pragma once
#include <stack>

namespace Elite
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	template <class T_NodeType, class T_ConnectionType>
	class EulerianPath
	{
	public:

		EulerianPath(IGraph<T_NodeType, T_ConnectionType>* pGraph);

		Eulerianity IsEulerian() const;
		std::vector<T_NodeType*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(int startIdx, std::vector<bool>& visited) const;
		bool IsConnected() const;

		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
	};

	template<class T_NodeType, class T_ConnectionType>
	inline EulerianPath<T_NodeType, T_ConnectionType>::EulerianPath(IGraph<T_NodeType, T_ConnectionType>* pGraph)
		: m_pGraph(pGraph)
	{
	}

	template<class T_NodeType, class T_ConnectionType>
	inline Eulerianity EulerianPath<T_NodeType, T_ConnectionType>::IsEulerian() const
	{

		// If the graph is not connected, there can be no Eulerian Trail
		if (IsConnected() == false)
			return Eulerianity::notEulerian;

		// Count nodes with odd degree 
		auto nodes = m_pGraph->GetAllNodes();
		int oddCount{};
		for (auto n : nodes)
		{
			auto connections = m_pGraph->GetNodeConnections(n);
			if (connections.size() & 1)
				oddCount++;
		}


		// A connected graph with more than 2 nodes with an odd degree (an odd amount of connections) is not Eulerian
		if (oddCount > 2)
			return Eulerianity::notEulerian;

		// A connected graph with exactly 2 nodes with an odd degree is Semi-Eulerian (unless there are only 2 nodes)
		// An Euler trail can be made, but only starting and ending in these 2 nodes
		if (oddCount == 2 && nodes.size() != 2)
			return Eulerianity::semiEulerian;


		// A connected graph with no odd nodes is Eulerian

		return Eulerianity::eulerian; // REMOVE AFTER IMPLEMENTING
	}

	template<class T_NodeType, class T_ConnectionType>
	inline std::vector<T_NodeType*> EulerianPath<T_NodeType, T_ConnectionType>::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		auto graphCopy = m_pGraph->Clone();
		auto path = std::vector<T_NodeType*>();
		int nrOfNodes = graphCopy->GetNrOfNodes();

		// Check if there can be an Euler path
		// If this graph is not eulerian, return the empty path
		// Else we need to find a valid starting index for the algorithm
		if (eulerianity == Eulerianity::notEulerian)
			return path;
		
		
		// Start algorithm loop
		int nodeidx{};
		std::stack<int> nodeStack;
		auto nodes = m_pGraph->GetAllNodes();
		if (eulerianity == Eulerianity::semiEulerian)
		{
			for (auto n : nodes)
			{
				auto connections = m_pGraph->GetNodeConnections(n);
				if (connections.size() & 1)
				{
					nodeidx = n->GetIndex();
					break;
				}
			}
		}
		else if (eulerianity == Eulerianity::eulerian)
		{
			nodeidx = nodes[0]->GetIndex();
		}
		
		
		//nodeStack.push(69);
		//std::cout << graphCopy->GetAllConnections().size() << "/n";
		do
		{
			auto connections = graphCopy->GetNodeConnections(nodeidx);
			auto node = graphCopy->GetNode(nodeidx);

			

			if (connections.size() != 0)
			{
				nodeStack.push(nodeidx);
				nodeidx = connections.front()->GetTo();

				graphCopy->RemoveConnection(node->GetIndex(), nodeidx);
				if (graphCopy->GetNodeConnections(nodeidx).size() == 0)
				{
					nodeStack.push(nodeidx);
				}

			}
			else
			{

				path.push_back(node);
				if (nodeStack.size() > 0)
				{
					nodeStack.pop();
					if (nodeStack.size() > 0)
					{
						nodeidx = nodeStack.top();
					}
				}
			}
		}
		while (graphCopy->GetAllConnections().size() != 0 && nodeStack.size() != 0);
		

		/*if (nodeStack.size() > 0)
		{
			path.push_back(m_pGraph->GetNode(nodeStack.top()));
		}*/

		std::reverse(path.begin(), path.end()); // reverses order of the path
		for (auto pathelement : path)
		{
			std::cout << pathelement->GetIndex() << " ";
			
		}
		std::cout << '\n';
		return path;
	}

	template<class T_NodeType, class T_ConnectionType>
	inline void EulerianPath<T_NodeType, T_ConnectionType>::VisitAllNodesDFS(int startIdx, std::vector<bool>& visited) const
	{
		// mark the visited node
		visited[startIdx] = true;


		// recursively visit any valid connected nodes that were not visited before
		for (T_ConnectionType* connection : m_pGraph->GetNodeConnections(startIdx))
			if (visited[connection->GetTo()] == false)
				VisitAllNodesDFS(connection->GetTo(), visited);


	}

	template<class T_NodeType, class T_ConnectionType>
	inline bool EulerianPath<T_NodeType, T_ConnectionType>::IsConnected() const
	{
		auto nodes = m_pGraph->GetAllNodes();
		vector<bool> visited(m_pGraph->GetNrOfNodes(), false);

		// find a valid starting node that has connections
		int connectedIndex = invalid_node_index;

		for (auto n : nodes)
		{
			auto connections = m_pGraph->GetNodeConnections(n);
			if (connections.size() != 0)
			{
				connectedIndex = n->GetIndex();
				break;
			}
		}

		
		// if no valid node could be found, return false

		if (connectedIndex == invalid_node_index) return false;


		// start a depth-first-search traversal from the node that has at least one connection
		VisitAllNodesDFS(connectedIndex, visited);

		// if a node was never visited, this graph is not connected
		for (auto n : nodes)
		{
			if (!visited[n->GetIndex()])
			return false;
		}


		return true;
	}

}