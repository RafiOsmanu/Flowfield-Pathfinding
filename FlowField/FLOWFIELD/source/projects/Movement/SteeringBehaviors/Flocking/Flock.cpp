#include "stdafx.h"
#include "Flock.h"

#include "../SteeringAgent.h"
#include "../Steering/SteeringBehaviors.h"
#include "../CombinedSteering/CombinedSteeringBehaviors.h"
#include "../SpacePartitioning/SpacePartitioning.h"

using namespace Elite;

//Constructor & Destructor
Flock::Flock(
	int flockSize /*= 50*/, 
	float worldSize /*= 100.f*/, 
	SteeringAgent* pAgentToEvade /*= nullptr*/, 
	bool trimWorld /*= false*/)

	: m_WorldSize{ worldSize }
	, m_FlockSize{ flockSize }
	, m_pAgentToEvade{pAgentToEvade}
	, m_NeighborhoodRadius{ 15 }
	, m_NrOfNeighbors{0}
	, m_SeekWeight{ 0.5f }
	, m_SeparationWeight{ 0.5f }
{
	m_Agents.resize(m_FlockSize);
	m_Neighbors.resize(m_FlockSize);
	
	m_pCohesionBehavior = new Cohesion(this);
	m_pSeparationBehavior = new Separation(this);
	m_pVelMatchBehavior = new VelocityMatch(this);
	m_pWanderBehavior = new Wander();
	m_pSeekBehavior = new Seek();
	m_pBlendedSteering = new BlendedSteering({ {m_pSeekBehavior, m_SeekWeight},
		{m_pCohesionBehavior, 0.4f},
		{m_pSeparationBehavior, 0.5f},
		{m_pVelMatchBehavior , 0.0f },
		{m_pWanderBehavior , 0.4f } });

	m_pFleeBehavior = new Flee();

	//priortySteering
	m_pPrioritySteering = new PrioritySteering({ m_pFleeBehavior, m_pBlendedSteering });

	//agent to evade
	m_pAgentToEvade = new SteeringAgent();
	m_pAgentToEvade->SetSteeringBehavior(m_pSeekBehavior);
	m_pAgentToEvade->SetBodyColor({ 1, 0, 0 });
	m_pAgentToEvade->SetAutoOrient(true);
	m_pAgentToEvade->SetMaxLinearSpeed(15.f);
	//cellSpace
	m_pCellSpace = new CellSpace(m_WorldSize, m_WorldSize, 25, 25, m_FlockSize);

	// TODO: initialize the flock and the memory pool
	for (int i{}; i < flockSize; ++i)
	{
		m_Agents[i] = new SteeringAgent();
		m_Agents[i]->SetSteeringBehavior(m_pPrioritySteering);
		m_Agents[i]->SetPosition(Elite::Vector2(randomFloat(0, m_WorldSize), randomFloat(0, m_WorldSize)));
		m_Agents[i]->SetMass(0.f);
		m_Agents[i]->SetAutoOrient(true);
		m_Agents[i]->SetMaxLinearSpeed(20.f);
		m_pCellSpace->AddAgent(m_Agents[i]);
		m_OldPos.push_back(m_Agents[i]->GetPosition());
	}
}

Flock::~Flock()
{
	// TODO: clean up any additional data

	SAFE_DELETE(m_pBlendedSteering);
	SAFE_DELETE(m_pPrioritySteering);
	SAFE_DELETE(m_pSeekBehavior);
	SAFE_DELETE(m_pCohesionBehavior);
	SAFE_DELETE(m_pSeparationBehavior);
	SAFE_DELETE(m_pVelMatchBehavior);
	SAFE_DELETE(m_pFleeBehavior);
	SAFE_DELETE(m_pVelMatchBehavior);
	SAFE_DELETE(m_pAgentToEvade);
	SAFE_DELETE(m_pCellSpace);
	SAFE_DELETE(m_pWanderBehavior);

	for(auto pAgent: m_Agents)
	{
		SAFE_DELETE(pAgent);
	}
	m_Agents.clear();

	
}

void Flock::Update(float deltaT)
{
	// TODO: update the flock
	// loop over all the agents
		// register its neighbors	(-> memory pool is filled with neighbors of the currently evaluated agent)
		// update it				(-> the behaviors can use the neighbors stored in the pool, next iteration they will be the next agent's neighbors)
		// trim it to the world

	for (unsigned int i{}; i < m_Agents.size(); ++i)
	{
		RegisterNeighbors(m_Agents[i]);
		
		
		m_Agents[i]->Update(deltaT);
		/*for (signed int n{}; n < m_NrOfNeighbors; ++n)
		{
			m_Neighbors[n]->Update(deltaT);
		}*/
		if (m_TrimWorld)
		{
			m_Agents[i]->TrimToWorld(m_WorldSize);
		}
		
		m_pCellSpace->UpdateAgentCell(m_Agents[i], m_OldPos[i]);
		m_OldPos[i] = m_Agents[i]->GetPosition();
	}
	TargetData evadeTarget;
	evadeTarget.LinearVelocity = m_pAgentToEvade->GetLinearVelocity();
	evadeTarget.Position = m_pAgentToEvade->GetPosition();

	m_pFleeBehavior->SetTarget(evadeTarget);
	m_pAgentToEvade->Update(deltaT);
}

void Flock::Render(float deltaT)
{
	if(m_SpacialPartitioningDebug) m_pCellSpace->RenderCells();


	RegisterNeighbors((m_Agents[0]));
	m_Agents[0]->SetRenderBehavior(true);
	m_Agents[0]->SetBodyColor({ 1, 0, 0 });
	m_Agents[0]->Render(deltaT);
	
	/*for (signed int i{}; i < m_NrOfNeighbors; ++i)
	{
		m_Neighbors[i]->Render(deltaT);
	}*/
	// TODO: render the flock
	Vector2 SeperationDirection{m_pSeparationBehavior->CalculateSteering(deltaT, m_Agents[0]).LinearVelocity};
	Vector2 SeekDirection{m_pSeekBehavior->CalculateSteering(deltaT, m_Agents[0]).LinearVelocity};
	Vector2 CohesionDirection{m_pCohesionBehavior->CalculateSteering(deltaT, m_Agents[0]).LinearVelocity};
	Vector2 WanderDirection{m_pWanderBehavior->CalculateSteering(deltaT, m_Agents[0]).LinearVelocity};
	
	
	DEBUGRENDERER2D->DrawDirection(m_Agents[0]->GetPosition(), SeperationDirection, 10, {1,1,0});
	DEBUGRENDERER2D->DrawDirection(m_Agents[0]->GetPosition(), SeekDirection, 10, {0,1,1});
	DEBUGRENDERER2D->DrawDirection(m_Agents[0]->GetPosition(), WanderDirection, 10, {1,0,0});
	


	DEBUGRENDERER2D->DrawCircle(m_Agents[0]->GetPosition(), m_NeighborhoodRadius, {0,1,0}, 0.f);

	//for (unsigned int i{}; i < m_Agents.size(); ++i)
	//{
	//	//m_Agents[i]->Render(deltaT);
	//	if (i == 0) continue;
	//	if (m_Agents[0]->GetPosition().Distance(m_Agents[i]->GetPosition()) <= m_NeighborhoodRadius)
	//	{
	//		m_Agents[i]->SetBodyColor({ 0, 1, 0 });
	//	}
	//	else
	//	{
	//		m_Agents[i]->SetBodyColor({ 1, 1, 0 });
	//	}
	//}

	m_pAgentToEvade->Render(deltaT);

	

}

void Flock::UpdateAndRenderUI()
{
	//Setup
	int menuWidth = 235;
	int const width = DEBUGRENDERER2D->GetActiveCamera()->GetWidth();
	int const height = DEBUGRENDERER2D->GetActiveCamera()->GetHeight();
	bool windowActive = true;
	ImGui::SetNextWindowPos(ImVec2((float)width - menuWidth - 10, 10));
	ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)height - 20));
	ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	ImGui::PushAllowKeyboardFocus(false);

	//Elements
	ImGui::Text("CONTROLS");
	ImGui::Indent();
	ImGui::Text("LMB: place target");
	ImGui::Text("RMB: move cam.");
	ImGui::Text("Scrollwheel: zoom cam.");
	ImGui::Unindent();

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();
	ImGui::Spacing();

	ImGui::Text("STATS");
	ImGui::Indent();
	ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
	ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
	ImGui::Unindent();

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();

	ImGui::Text("Flocking");
	ImGui::Spacing();

	// TODO: Implement checkboxes for debug rendering and weight sliders here
	ImGui::SliderFloat("Seek", &m_pBlendedSteering->GetWeightedBehaviorsRef()[0].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Cohesion", &m_pBlendedSteering->GetWeightedBehaviorsRef()[1].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Separation", &m_pBlendedSteering->GetWeightedBehaviorsRef()[2].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("VelocityMatch", &m_pBlendedSteering->GetWeightedBehaviorsRef()[3].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Wander", &m_pBlendedSteering->GetWeightedBehaviorsRef()[4].weight, 0.f, 1.f, "%.2");

	//ImGui::Checkbox("Debug Rendering", &m_CanDebugRender);
	ImGui::Checkbox("Trim World", &m_TrimWorld);

	ImGui::Checkbox("Spacial Partitioning", &m_SpacialPartitioning);
	ImGui::Checkbox("Spacial Partitioning Debug", &m_SpacialPartitioningDebug);

	//End
	ImGui::PopAllowKeyboardFocus();
	ImGui::End();
	
}

void Flock::RegisterNeighbors(SteeringAgent* pAgent)
{
	// TODO: Implement
	if (m_SpacialPartitioning)
	{
		m_NrOfNeighbors = m_pCellSpace->RegisterNeighbors(pAgent, m_NeighborhoodRadius, m_Neighbors);
		return;
	}

	m_NrOfNeighbors = 0;
	//m_Neighbors.clear();
	for (size_t i{}; i < m_Agents.size(); ++i)
	{
		if (pAgent == m_Agents[i]) continue;
		
			if (pAgent->GetPosition().Distance(m_Agents[i]->GetPosition()) <= m_NeighborhoodRadius)
			{
				m_Neighbors[m_NrOfNeighbors] = m_Agents[i];
				++m_NrOfNeighbors;

			}
	}
	
	
	
}

Elite::Vector2 Flock::GetAverageNeighborPos() const
{
	if (m_NrOfNeighbors == 0) return Vector2{};
	// TODO: Implement
	Vector2 averagePos{0.f, 0.f};

	for (signed int i{}; i < m_NrOfNeighbors; ++i)
	{
		averagePos += m_Neighbors[i]->GetPosition();
	}
	averagePos /= float(m_NrOfNeighbors);

	return averagePos;
}

Elite::Vector2 Flock::GetAverageNeighborVelocity() const
{
	// TODO: Implement
	if (m_NrOfNeighbors == 0) return Vector2{};
	Vector2 averageVelocity{};

	for (int i{}; i < m_NrOfNeighbors; ++i)
	{
		averageVelocity += m_Neighbors[i]->GetLinearVelocity();
	}
	averageVelocity /= float(m_NrOfNeighbors);

	return averageVelocity;
}

void Flock::SetTarget_Seek(TargetData target)
{
	// TODO: Set target for seek behavior
	m_pSeekBehavior->SetTarget(target);
}


float* Flock::GetWeight(ISteeringBehavior* pBehavior) 
{
	if (m_pBlendedSteering)
	{
		auto& weightedBehaviors = m_pBlendedSteering->GetWeightedBehaviorsRef();
		auto it = find_if(weightedBehaviors.begin(),
			weightedBehaviors.end(),
			[pBehavior](BlendedSteering::WeightedBehavior el)
			{
				return el.pBehavior == pBehavior;
			}
		);

		if(it!= weightedBehaviors.end())
			return &it->weight;
	}

	return nullptr;
}


float Flock::GetNeighborhoodRadius()
{
	return m_NeighborhoodRadius;
}