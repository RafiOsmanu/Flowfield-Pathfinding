/*=============================================================================*/
// Copyright 2021-2022 Elite Engine
// Authors: Matthieu Delaere, Thomas Goussaert
/*=============================================================================*/
// SteeringBehaviors.h: SteeringBehaviors interface and different implementations
/*=============================================================================*/
#ifndef ELITE_STEERINGBEHAVIORS
#define ELITE_STEERINGBEHAVIORS

//-----------------------------------------------------------------
// Includes & Forward Declarations
//-----------------------------------------------------------------
#include "../SteeringHelpers.h"
class SteeringAgent;
class Obstacle;

#pragma region **ISTEERINGBEHAVIOR** (BASE)
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	virtual SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) = 0;

	//Seek Functions
	void SetTarget(const TargetData& target) { m_Target = target; }

	template<class T, typename std::enable_if<std::is_base_of<ISteeringBehavior, T>::value>::type* = nullptr>
	T* As()
	{ return static_cast<T*>(this); }

protected:
	TargetData m_Target;
};
#pragma endregion

///////////////////////////////////////
//SEEK
//****
class Seek : public ISteeringBehavior
{
public:
	Seek() = default;
	virtual ~Seek() = default;

	//Seek Behaviour
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
};


///////////////////////////////////////
//FLEE
//****
class Flee : public ISteeringBehavior
{
public:
	Flee() = default;
	virtual ~Flee() = default;

	//Flee Behaviour
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
	void SetFleeRadius(float fleeRadius) { m_FleeRadius = fleeRadius; }

private:
	float m_FleeRadius = 10.f;

};

///////////////////////////////////////
//Arrive
//****
class Arrive : public ISteeringBehavior
{
public:

	Arrive() = default;
	virtual ~Arrive() = default;

	//arrive behaviour
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
};




///////////////////////////////////////
//Face
//****
class Face : public ISteeringBehavior
{
public:
	Face() = default;
	virtual ~Face() = default;

	//Face Behaviour
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;

};


///////////////////////////////////////
//Wander
//****
class Wander : public Seek
{
public:
	Wander() = default;
	virtual ~Wander() = default;

	//Face Behaviour
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
	void SetWanderOffset(float offset);

private:
	float m_OffsetDistance = 6.0f;
	float m_Radius = 4.f;
	float m_MaxAngleChange = Elite::ToRadians(45.f);
	float m_WanderAngle = 0.f;

};
#endif


