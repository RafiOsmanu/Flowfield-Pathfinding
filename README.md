# <div align="center">Flow Field For Agent PathFinding</div>
<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="Flow Field"></a>

<!-- ABOUT THE PROJECT -->
## About the project
![FlowFieldGiphy](https://user-images.githubusercontent.com/104839344/212175190-50d1a982-c059-4a62-834f-fd2484f53796.gif)


In this project I implement a pathfinding algorithm called Flow Field 
that is very efficient for handling a big group of agents when it comes to pathfinding. 
I created this project for gameplay programming, a course at Howest [DAE].

Written in c++, using a framework provided by DAE

<p align="right">(<a href="#readme-top">back to top</a>)</p>



### Built With

* [C++]




<!-- GETTING STARTED -->
## What are Flow Fields

A flow field is also known as vector field pathfinding, this is a technique that calculates the path from the goal to every node in the graph. 
It can be explained in three steps.

**1. First we create a heat map that calculates the distance between the goal node and every other node on the graph.**

![HeatMap](https://user-images.githubusercontent.com/104839344/212181669-48c5e57c-56f5-4446-b43f-afaf9c6d7808.JPG)

The idea is that by creating this heat map we calculate the length of the path to the goal for every node so that 
later on (in step 2) we can use these distances to create direction vectors

**2. When we have created our heatmap we can use that information to create a vector map.**

#### To calculate these vectors:
2.1 For every node we create vectors that points towards their neighbours. 
2.2 Then set the length of each vector to the normalized distance (we calculated this in step 1) of the neigbour node it points to.
2.3 Finally we take the average of all these vectors combined and save this direction in our node.

You will have a vector that points in the direction of the upward gradient, towards the neighboring node with the lowest distance. This process is repeated for every node until every node has a vector.

![VectorMap](https://user-images.githubusercontent.com/104839344/212182718-876c6758-bf49-4097-8adb-6d7e3f428a86.JPG)

**3. Set the velocity of your agent to the vectors u have created**

![agent to goal](https://user-images.githubusercontent.com/104839344/212186247-268b802b-1eb9-455d-8d62-4f17dda19a1d.JPG)

The last step is pretty straightforward, you have to check which node is at the position of your agent
and then set the agents velocity to that of the vector that is present in that node multiplied by your agents maximun speed.

## Design/Implementation 

#### Heatmap Implementation :

- Typically the goal node is used as the starting point for the search, the algorithm works by expanding out from the goal node and evaluating the cost of reaching other nodes in the map. The algorithm is able to efficiently find the shortest path to the goalNode by working backwards from the goal to the starting point.
```cpp
		//1.HeatMap
		//-------------------------------------------------------------
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
			//save The distance inside the corresponding node 
			currentRecord.pNode->SetDistance(static_cast<int>(currentRecord.costSoFar));
		}
```

#### Vector Implementation :

- Now that we calculated all the distances, we can use them to make the direction vectors that will lead our agent to the goal.

```cpp

		//2.Vector Kernel Convolution
		//Combined direction given by the gradient + direction given by minimum distance value neighbour
		//-------------------------------------------------------------
		for (auto& node : closedList)
		{
			Vector2 flowFieldVector{};
			int ClosestDistance{100};
			bool wallAmongNeighbour{};
			int neighbourAmount{};

			//check if any of the neighbours is a wall or node is on edge of grid
			for (auto& neighbour : m_pGraph->GetNodeConnections(node.pNode))
			{
				if (m_pGraph->GetNode(neighbour->GetTo())->GetTerrainType() == TerrainType::Wall || 
				m_pGraph->GetNodeConnections(node.pNode).size() < 4) wallAmongNeighbour = true;
			}

			//if true
			if (wallAmongNeighbour)
			{
				//make vector from node to all its neighbours
				for (auto& neighbour : m_pGraph->GetNodeConnections(node.pNode))
				{
					//vectors pointing from center to neighbours
					Vector2 vectorToNeighbour = m_pGraph->GetNodeWorldPos(m_pGraph->GetNode(neighbour->GetTo())) - 
					m_pGraph->GetNodeWorldPos(node.pNode);

					//calculate the neighbour with lowest cost 
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
					Vector2 vectorToNeighbour = m_pGraph->GetNodeWorldPos(node.pNode) 
					- m_pGraph->GetNodeWorldPos(m_pGraph->GetNode(neighbour->GetTo()));
					
					float distance = static_cast<float>(m_pGraph->GetNode(neighbour->GetTo())->GetDistance());

					//add to the vector and calulate average later 
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
```

#### Design Choice VectorMap 
- I chose not to only use the technique i described before, i also make use of a technique where u take the neighbour with lowest cost.
- I combined these techniques because taking the average gives u a more organic flow, but is not so accurate when you have neighbours that are walls.
- Taking the lowest distance isn't very organic, but it's less accurate when you have a wall among your neighbours.

#### Setting the agents velocity to the calculated direction vectors:

- Finally we check which node our agent is on and use that node's velocity to set the velocity of the agent.
- Then we multiply it by the max speed of the agent to create our final velocity.
```cpp
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
```


### Result

- As finished result u get a pathfinder that can take in a lot of agents. 

![ezgif com-gif-maker (1)](https://user-images.githubusercontent.com/104839344/212194558-a351a909-3419-42f0-950c-9429bc57dec4.gif)

## Conclusion 

This research project has helped me become a better c++ programmer and getting a better understanding about concepts we discussed this semester.
Flow Field is not a "one size fits all" problem solver, but it is definitely useful and interesting for game development.

I wanted to do crowd simulation/crowd avoiding algorithms first but didn't have the time, so I am interested in researching these topics in the future.

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[DAE]: https://www.digitalartsandentertainment.be/
[C++]: https://cplusplus.com/

