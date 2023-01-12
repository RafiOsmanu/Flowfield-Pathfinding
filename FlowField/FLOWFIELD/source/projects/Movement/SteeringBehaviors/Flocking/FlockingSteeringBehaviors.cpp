#include "stdafx.h"
#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{


	SteeringOutput steering{};
	if (m_pFlock->GetNrOfNeighbors() == 0)
		return steering;

	Elite::Vector2 agentPos{ pAgent->GetPosition() };

	//m_Target.Position = m_pFlock->GetAverageNeighborPos() - agentPos;
	steering.LinearVelocity = m_pFlock->GetAverageNeighborPos() - agentPos;
	const Elite::Color firstAgentColor{ 1, 0, 0, 1 };


	if (pAgent->CanRenderBehavior())
	{
		//DEBUGRENDERER2D->DrawSegment(pAgent->GetPosition(), steering.LinearVelocity, steering.LinearVelocity.Magnitude(), {1,0,1});
		DEBUGRENDERER2D->DrawSegment(pAgent->GetPosition(), m_pFlock->GetAverageNeighborPos(), { 1,0,1 });
	}
	
	//steering.LinearVelocity = Seek::CalculateSteering(deltaT, pAgent).LinearVelocity;
	
	return steering;
}

//*********************
//SEPARATION (FLOCKING)

SteeringOutput Separation::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	if (m_pFlock->GetNrOfNeighbors() == 0) return SteeringOutput{};
	SteeringOutput steering{};
	const std::vector<SteeringAgent*> m_Neighbors{m_pFlock->GetNeighbors()};
	const int NrNeighbors{m_pFlock->GetNrOfNeighbors()};
	Elite::Vector2 totalPushForce{};
	
	
	for (int i{}; i < NrNeighbors; ++i)
	{
		Elite::Vector2 pushForce{ pAgent->GetPosition() - m_Neighbors[i]->GetPosition() };
		pushForce /= pushForce.MagnitudeSquared();
		totalPushForce += pushForce;
	}

	totalPushForce = totalPushForce.GetNormalized() * pAgent->GetMaxLinearSpeed();

	steering.LinearVelocity = totalPushForce;

	return steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)

SteeringOutput VelocityMatch::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	


	steering.LinearVelocity = m_pFlock->GetAverageNeighborVelocity();

	return steering;
}