//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "SteeringBehaviors.h"
#include "../SteeringAgent.h"
#include "../Obstacle.h"
#include "framework\EliteMath\EMatrix2x3.h"

//SEEK
//****
SteeringOutput Seek::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	/*if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(
			pAgent->GetPosition(),
			steering.LinearVelocity, 5, { 0, 1,0 });
	}*/

	return steering;
}

SteeringOutput Flee::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	Elite::Vector2 fromTarget = pAgent->GetPosition() - m_Target.Position;
	float distance = fromTarget.Magnitude();

	SteeringOutput steering = {};

	if (distance > m_FleeRadius)
	{
		steering.IsValid = false;
		return steering;
	}

	steering.LinearVelocity =  pAgent->GetPosition() - m_Target.Position;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	/*if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(
			pAgent->GetPosition(),
			steering.LinearVelocity, 5, { 0, 1,0 });
	}*/

	return steering;
}

SteeringOutput Arrive::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	//pAgent->SetAutoOrient(false);
	SteeringOutput steering = {};
	const float arrivalRadius = 1.f;
	const float slowRadius = 15.f;

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	const float distance{ steering.LinearVelocity.Magnitude() };
	
	if (distance < arrivalRadius)
	{
		steering.LinearVelocity = { 0.f, 0.f };
		return steering;
	}
	
	steering.LinearVelocity.Normalize();
	if (distance < slowRadius)
	{
		steering.LinearVelocity *= pAgent->GetMaxLinearSpeed() * (distance / slowRadius);

	}
	else
	{
		steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
	}
	
	return steering;
}

SteeringOutput Face::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	pAgent->SetAutoOrient(false);
	SteeringOutput steering = {};
	const Elite::Vector2 vecToTarget = m_Target.Position - pAgent->GetPosition();
	const float angleToTarget = atan2(vecToTarget.y, vecToTarget.x);


	if (abs(angleToTarget - pAgent->GetRotation()) <= 0.1f)
	{
		return steering;
	}

	if (angleToTarget - pAgent->GetRotation() > 0)
	{
		steering.AngularVelocity = pAgent->GetMaxAngularSpeed();
	}
	else
	{
		steering.AngularVelocity = -pAgent->GetMaxAngularSpeed();
	}
	return steering;
}

SteeringOutput Wander::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};

	Elite::Vector2 cirleCenter{ pAgent->GetPosition() };

	//apply offset
	cirleCenter += pAgent->GetDirection() * m_OffsetDistance;

	m_WanderAngle += Elite::randomFloat(-m_MaxAngleChange, m_MaxAngleChange);
	//get random pos on circle
	cirleCenter.x += m_Radius * cosf(m_WanderAngle);
	cirleCenter.y += m_Radius * sinf(m_WanderAngle);
	m_Target.Position = cirleCenter;

	steering.LinearVelocity = Seek::CalculateSteering(deltaT, pAgent).LinearVelocity;


	Elite::Vector2 debugCircle{ pAgent->GetPosition() + pAgent->GetDirection() * m_OffsetDistance };
	/*if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), pAgent->GetLinearVelocity(), pAgent->GetLinearVelocity().Magnitude(), { 1,1,1 });
		DEBUGRENDERER2D->DrawCircle(debugCircle, m_Radius, {1,0,0}, 0.f);
	}*/

	return steering;
}

void Wander::SetWanderOffset(float offset)
{
	m_OffsetDistance = offset;
}



