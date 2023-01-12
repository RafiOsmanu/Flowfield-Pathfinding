#include "stdafx.h"
#include "SpacePartitioning.h"
#include "projects\Movement\SteeringBehaviors\SteeringAgent.h"

// --- Cell ---
// ------------
Cell::Cell(float left, float bottom, float width, float height)
{
	boundingBox.bottomLeft = { left, bottom };
	boundingBox.width = width;
	boundingBox.height = height;
}

std::vector<Elite::Vector2> Cell::GetRectPoints() const
{
	auto left = boundingBox.bottomLeft.x;
	auto bottom = boundingBox.bottomLeft.y;
	auto width = boundingBox.width;
	auto height = boundingBox.height;

	std::vector<Elite::Vector2> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	

	

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(float width, float height, int rows, int cols, int maxEntities)
	: m_SpaceWidth(width)
	, m_SpaceHeight(height)
	, m_NrOfRows(rows)
	, m_NrOfCols(cols)
	, m_Neighbors(maxEntities)
	, m_NrOfNeighbors(0)
	, m_RadiusRect()
	, m_AmountOfAgents(0)
{
	float left{};
	float bottom{};
	m_CellWidth = m_SpaceWidth / float(m_NrOfCols);
	m_CellHeight = m_SpaceHeight / float(m_NrOfRows);
	Cell cell{0, 0, 0, 0};
	m_Neighbors.resize(maxEntities);
	
	for (int r{}; r < (m_NrOfRows); ++r)
	{
		for (int c{}; c < (m_NrOfCols); ++c)
		{
			cell.boundingBox.bottomLeft = { left, bottom };
			cell.boundingBox.width = m_CellWidth;
			cell.boundingBox.height = m_CellHeight;
			m_Cells.push_back(cell);
			left += m_SpaceWidth / float(m_NrOfCols);
		}
		bottom += m_CellHeight;
		left = 0;
	}
	
	
}

void CellSpace::AddAgent(SteeringAgent* agent)
{
	if (m_AmountOfAgents == 0)
	{
		m_RenderAgent = agent;
	}
	++m_AmountOfAgents;
	m_Cells[PositionToIndex(agent->GetPosition())].agents.push_back(agent);
}

void CellSpace::UpdateAgentCell(SteeringAgent* agent, Elite::Vector2 oldPos)
{
	if (PositionToIndex(agent->GetPosition()) != PositionToIndex(oldPos))
	{
		m_Cells[PositionToIndex(oldPos)].agents.remove(agent);
		AddAgent(agent);
	}
	
}

int CellSpace::RegisterNeighbors(SteeringAgent* agent, float queryRadius, std::vector<SteeringAgent*>& neighbours)
{
	Elite::Rect radiusRect = Elite::Rect({ agent->GetPosition().x - queryRadius, agent->GetPosition().y - queryRadius}, queryRadius * 2, queryRadius * 2);

	m_NrOfNeighbors = 0;
	//m_Neighbors.clear();
	for (int i{}; i < m_Cells.size(); ++i)
	{
		if (Elite::IsOverlapping(m_Cells[i].boundingBox, radiusRect))
		{
			for (SteeringAgent* pAgent : m_Cells[i].agents)
			{
				if (pAgent == agent) continue;

				if (agent->GetPosition().Distance(pAgent->GetPosition()) <= queryRadius)
				{
					//m_Neighbors[m_NrOfNeighbors] = pAgent;
					neighbours[m_NrOfNeighbors] = pAgent;
					++m_NrOfNeighbors;
				}
			}
		}
	}
	//m_Neighbors = neighbours;
	m_RadiusRect = Elite::Rect({ m_RenderAgent->GetPosition().x - queryRadius, m_RenderAgent->GetPosition().y - queryRadius}, queryRadius * 2 , queryRadius * 2);
	return m_NrOfNeighbors;
}

void CellSpace::EmptyCells()
{
	for (Cell& c : m_Cells)
		c.agents.clear();
}

void CellSpace::RenderCells() const
{
	float depth{ DEBUGRENDERER2D->NextDepthSlice()};
	
	Elite::Polygon* cells{};
	for (int i{}; i < m_Cells.size(); ++i)
	{
		cells = new Elite::Polygon(m_Cells[i].GetRectPoints());
		if (Elite::IsOverlapping(m_Cells[i].boundingBox, m_RadiusRect))
		{
			DEBUGRENDERER2D->DrawPolygon(cells, { 0, 0, 1 }, 0);
		}
		else
		{
			DEBUGRENDERER2D->DrawPolygon(cells, {1, 0, 0}, depth);
		}
		delete cells;
		
		std::string nrOfAgentsString{ std::to_string(m_Cells[i].agents.size()) };
		const char* nrOfAgents{ nrOfAgentsString.c_str()};

		Elite::Vector2 nrOfAgentsPos{ m_Cells[i].boundingBox.bottomLeft.x + 0.5f, m_Cells[i].boundingBox.bottomLeft.y + 15.f };

		DEBUGRENDERER2D->DrawString(nrOfAgentsPos, nrOfAgents);

		
	}
	//queryRect debug
	std::vector<Elite::Vector2> queryRectPoints;
	queryRectPoints.push_back(m_RadiusRect.bottomLeft);
	queryRectPoints.push_back({ m_RadiusRect.bottomLeft.x + m_RadiusRect.width , m_RadiusRect.bottomLeft.y});
	queryRectPoints.push_back({ m_RadiusRect.bottomLeft.x + m_RadiusRect.width , m_RadiusRect.bottomLeft.y + m_RadiusRect.height});
	queryRectPoints.push_back({ m_RadiusRect.bottomLeft.x , m_RadiusRect.bottomLeft.y + m_RadiusRect.height });

	Elite::Polygon* queryRect{ new Elite::Polygon(queryRectPoints) };
	DEBUGRENDERER2D->DrawPolygon(queryRect, { 0, 1, 0 });

	delete queryRect;
}

int CellSpace::PositionToIndex(const Elite::Vector2 pos) const
{
	const int col{ posToIndexConvert(pos.x, m_CellWidth, m_NrOfCols)};
	const int row{ posToIndexConvert(pos.y, m_CellHeight, m_NrOfRows)};
	int index{ row * m_NrOfCols + col };
	//for (int i{}; i < m_Cells.size(); ++i)
	
	if (Elite::IsOverlapping(m_Cells[index].boundingBox, Elite::Rect(pos, 1.f, 1.f))) return index;
	
	
	return 0;
	
}

int CellSpace::posToIndexConvert(const float pos, const float CellMeasure, const int nrOfCell) const
{
	int index{};
	index = trunc((pos / CellMeasure ));
	index = Elite::Clamp(index, 0, nrOfCell - 1);
	/*if (index < nrOfCell)
	{
		return index;
	}*/
	return index;

}