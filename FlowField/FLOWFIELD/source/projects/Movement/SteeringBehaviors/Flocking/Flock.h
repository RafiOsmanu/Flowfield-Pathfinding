#pragma once
#include "../SteeringHelpers.h"
#include "FlockingSteeringBehaviors.h"

class ISteeringBehavior;
class SteeringAgent;
class BlendedSteering;
class PrioritySteering;
class CellSpace;

class Flock final
{
public:
	Flock(
		int flockSize = 50, 
		float worldSize = 100.f, 
		SteeringAgent* pAgentToEvade = nullptr, 
		bool trimWorld = false);

	~Flock();

	void Update(float deltaT);
	void UpdateAndRenderUI() ;
	void Render(float deltaT);

	void RegisterNeighbors(SteeringAgent* pAgent);
	int GetNrOfNeighbors() const { return m_NrOfNeighbors; }
	const std::vector<SteeringAgent*>& GetNeighbors() const { return m_Neighbors; }

	Elite::Vector2 GetAverageNeighborPos() const;
	Elite::Vector2 GetAverageNeighborVelocity() const;

	void SetTarget_Seek(TargetData target);
	void SetWorldTrimSize(float size) { m_WorldSize = size; }
	float GetNeighborhoodRadius();
	bool m_TrimWorld = false;
	float m_WorldSize = 0.f;
	bool m_SpacialPartitioning = true;
	bool m_SpacialPartitioningDebug = true;
	//CellSpace
	CellSpace* m_pCellSpace = nullptr;

private:
	//Datamembers
	int m_FlockSize = 0;
	std::vector<SteeringAgent*> m_Agents;
	std::vector<SteeringAgent*> m_Neighbors;


	float m_NeighborhoodRadius = 10.f;
	int m_NrOfNeighbors = 0;

	SteeringAgent* m_pAgentToEvade = nullptr;
	
	//Steering Behaviors
	Seek* m_pSeekBehavior = nullptr;
	Separation* m_pSeparationBehavior = nullptr;
	Cohesion* m_pCohesionBehavior = nullptr;
	VelocityMatch* m_pVelMatchBehavior = nullptr;
	Wander* m_pWanderBehavior = nullptr;
	//Evade* m_pEvadeBehavior = nullptr;
	Flee* m_pFleeBehavior = nullptr;

	BlendedSteering* m_pBlendedSteering = nullptr;
	PrioritySteering* m_pPrioritySteering = nullptr;

	float* GetWeight(ISteeringBehavior* pBehaviour);


	std::vector<Elite::Vector2> m_OldPos;

private:
	Flock(const Flock& other);
	Flock& operator=(const Flock& other);
	float m_SeekWeight;
	float m_SeparationWeight;
};