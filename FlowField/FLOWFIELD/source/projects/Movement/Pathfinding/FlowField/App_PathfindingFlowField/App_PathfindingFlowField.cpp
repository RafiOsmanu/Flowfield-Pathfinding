//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "App_PathfindingFlowField.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EFlowField.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EBFS.h"

using namespace Elite;

//Destructor
App_PathfindingFlowField::~App_PathfindingFlowField()
{
	SAFE_DELETE(m_pGridGraph);
	SAFE_DELETE(m_pGraphRenderer);
	SAFE_DELETE(m_pGraphEditor);
	for (int i{}; i < m_AgentsAmount; ++i)
	{
		SAFE_DELETE(m_psteeringAgents[i]);
	}
	m_psteeringAgents.clear();
}

//Functions
void App_PathfindingFlowField::Start()
{
	m_pGraphEditor = new GraphEditor();
	m_pGraphRenderer = new GraphRenderer();
	//Set Camera
	DEBUGRENDERER2D->GetActiveCamera()->SetZoom(39.0f);
	DEBUGRENDERER2D->GetActiveCamera()->SetCenter(Elite::Vector2(73.0f, 35.0f));

	//Create Graph
	MakeGridGraph();

	//Setup default start path
	startPathIdx = 44;
	endPathIdx = 88;
	CalculatePath();

	m_AgentsAmount = 300;
	//initialize all steeringAgents
	m_psteeringAgents.resize(m_AgentsAmount);
	m_BottomleftWorld = Vector2{0, 0};
	m_TopRightWorld = Vector2{static_cast<float>(COLUMNS) *  static_cast<float>(m_SizeCell), static_cast<float>(ROWS) *  static_cast<float>(m_SizeCell)};

	for (int i{}; i < m_AgentsAmount; ++i)
	{
		m_psteeringAgents[i] = new SteeringAgent();
		m_psteeringAgents[i]->SetAutoOrient(true);
		m_psteeringAgents[i]->SetLinearVelocity({ 1, 0 });
		m_psteeringAgents[i]->SetMaxLinearSpeed(80.f);
		m_psteeringAgents[i]->SetPosition({ randomFloat(2.f, m_TopRightWorld.x - 2.f), randomFloat(2.f, m_TopRightWorld.y - 2.f) });
		m_psteeringAgents[i]->SetMass(1.f);
	}
}

void App_PathfindingFlowField::Update(float deltaTime)
{
	//Update Agent
	for (size_t i{}; i < m_psteeringAgents.size(); ++i)
	{
		m_psteeringAgents[i]->Update(deltaTime);
		m_psteeringAgents[i]->TrimToWorld(m_BottomleftWorld, m_TopRightWorld, true);
	}
	CalculatePath();
	
	UNREFERENCED_PARAMETER(deltaTime);

	//INPUT
	bool const middleMousePressed = INPUTMANAGER->IsMouseButtonUp(InputMouseButton::eMiddle);
	if (middleMousePressed)
	{
		MouseData mouseData = { INPUTMANAGER->GetMouseData(Elite::InputType::eMouseButton, Elite::InputMouseButton::eMiddle) };
		Elite::Vector2 mousePos = DEBUGRENDERER2D->GetActiveCamera()->ConvertScreenToWorld({ (float)mouseData.X, (float)mouseData.Y });

		//Find closest node to click pos
		int closestNode = m_pGridGraph->GetNodeIdxAtWorldPos(mousePos);
		endPathIdx = closestNode;
		
	}

	if (m_pGraphEditor->UpdateGraph(m_pGridGraph))
	{
		CalculatePath();
	}
	//IMGUI
	UpdateImGui();
}

void App_PathfindingFlowField::Render(float deltaTime) const
{
	/*Vector2 worldBounds[4]{};
	worldBounds[0] = m_BottomleftWorld;
	worldBounds[1] = { m_TopRightWorld.x, m_BottomleftWorld.y };
	worldBounds[2] = m_BottomleftWorld + m_TopRightWorld;
	worldBounds[3] = { m_BottomleftWorld.x , m_TopRightWorld.y  };
	DEBUGRENDERER2D->DrawSegment(worldBounds[0], worldBounds[1], { 1.0f, 0.f, 0.f }, -1);
	DEBUGRENDERER2D->DrawSegment(worldBounds[1], worldBounds[2], { 1.0f, 0.f, 0.f }, -1);
	DEBUGRENDERER2D->DrawSegment(worldBounds[2], worldBounds[3], { 1.0f, 0.f, 0.f }, -1);
	DEBUGRENDERER2D->DrawSegment(worldBounds[3], worldBounds[0], { 1.0f, 0.f, 0.f }, -1);*/

	UNREFERENCED_PARAMETER(deltaTime);
	//Render grid
	m_pGraphRenderer->RenderGraph(m_pGridGraph, m_DebugSettings.DrawNodes, m_DebugSettings.DrawNodeDistance, m_DebugSettings.DrawrenderVectors, m_DebugSettings.DrawConnectionCosts);

	//Render end node on top if applicable
	if (endPathIdx != invalid_node_index)
	{
		m_pGraphRenderer->HighlightNodes(m_pGridGraph, { m_pGridGraph->GetNode(endPathIdx) }, END_NODE_COLOR);
	}

	//render path below if applicable
	if (m_vPath.size() > 0)
	{
		m_pGraphRenderer->HighlightNodes(m_pGridGraph, m_vPath);
	}
	
}

void App_PathfindingFlowField::MakeGridGraph()
{
	m_pGridGraph = new GridGraph<GridTerrainNode, GraphConnection>(COLUMNS, ROWS, m_SizeCell, false, false, 1.f, 1.5f);

	//Setup default terrain
	m_pGridGraph->GetNode(86)->SetTerrainType(TerrainType::Wall);
	m_pGridGraph->GetNode(66)->SetTerrainType(TerrainType::Wall);
	m_pGridGraph->GetNode(67)->SetTerrainType(TerrainType::Wall);
	m_pGridGraph->GetNode(47)->SetTerrainType(TerrainType::Wall);
	m_pGridGraph->RemoveConnectionsToAdjacentNodes(86);
	m_pGridGraph->RemoveConnectionsToAdjacentNodes(66);
	m_pGridGraph->RemoveConnectionsToAdjacentNodes(67);
	m_pGridGraph->RemoveConnectionsToAdjacentNodes(47);
}

void App_PathfindingFlowField::UpdateImGui()
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		int menuWidth = 200;
		int const width = DEBUGRENDERER2D->GetActiveCamera()->GetWidth();
		int const height = DEBUGRENDERER2D->GetActiveCamera()->GetHeight();
		bool windowActive = true;
		ImGui::SetNextWindowPos(ImVec2((float)width - menuWidth - 10, 10));
		ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)height - 20));
		ImGui::Begin("Flow Field", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
		ImGui::PushAllowKeyboardFocus(false);

		////Elements
		//ImGui::Text("CONTROLS");
		//ImGui::Indent();
		//ImGui::Text("LMB: target");
		//ImGui::Text("RMB: start");
		//ImGui::Unindent();

		/*Spacing*/ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing(); ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		/*Spacing*/ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing(); ImGui::Spacing();

		ImGui::Text("Flow Field Pathfinding");
		ImGui::Spacing();

		ImGui::Text("controls:");
		ImGui::Text("Middle Mouse Click");
		ImGui::Text("To Set Goal");
		std::string buttonText{ "" };
		m_StartSelected = false;
		
		ImGui::Checkbox("Grid", &m_DebugSettings.DrawNodes);
		ImGui::Checkbox("NodeDistance", &m_DebugSettings.DrawNodeDistance);
		ImGui::Checkbox("VectorMap", &m_DebugSettings.DrawrenderVectors);
		
		ImGui::Spacing();

		//End
		ImGui::PopAllowKeyboardFocus();
		ImGui::End();
	}
#pragma endregion
#endif
}

void App_PathfindingFlowField::CalculatePath()
{
	if (startPathIdx != invalid_node_index
		&& endPathIdx != invalid_node_index
		&& startPathIdx != endPathIdx)
	{
		//BFS Pathfinding
		//auto pathfinder = BFS<GridTerrainNode, GraphConnection>(m_pGridGraph);
		auto heatMap = FlowField<GridTerrainNode, GraphConnection>(m_pGridGraph, m_pHeuristicFunction);
		auto startNode = m_pGridGraph->GetNode(startPathIdx);
		auto endNode = m_pGridGraph->GetNode(endPathIdx);

		heatMap.CalculateFlowField(endNode, m_psteeringAgents);
	}
	else
	{
		std::cout << "Agents Have No Goal" << std::endl;
	}
}
