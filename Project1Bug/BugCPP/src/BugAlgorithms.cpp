#include <math.h>
#include "BugAlgorithms.hpp"

BugAlgorithms::BugAlgorithms(Simulator * const simulator)
{
    m_simulator = simulator;   
    //add your initialization of other variables
    //that you might have declared

    m_mode = STRAIGHT;
    m_hit[0] = m_hit[1] = HUGE_VAL;
    m_leave[0] = m_leave[1] = 0;
    m_distLeaveToGoal = HUGE_VAL; 
	m_distance = 0; // This is the breadcrumb which shows how far the robot
                    // traveled in the "right" direction
	m_perimeter = 0; // Perimeter of given boundary
    m_direction = NONE;
}

BugAlgorithms::~BugAlgorithms(void)
{
    //do not delete m_simulator  
}

Move BugAlgorithms::Bug0(Sensor sensor)
{
    Move move;
    if (sensor.m_dmin <= m_simulator->GetWhenToTurn())
    {
		if(m_mode == STRAIGHT)
		{
			m_hit[0] =  m_simulator->GetRobotCenterX();
            m_hit[1] = m_simulator->GetRobotCenterY();
		}
        move = GetMoveAlongBoundary(sensor);
		m_mode = AROUND;
    }
    else
    {
        move = GetMoveTowardsGoal();
		m_mode = STRAIGHT;
    }
    return move;
}

Move BugAlgorithms::Bug1(Sensor sensor)
{
    Move move;
    double curX = m_simulator->GetRobotCenterX();
    double curY = m_simulator->GetRobotCenterY();

    switch (m_mode)
    {
    case STRAIGHT:
    case AROUND_AND_AWAY_FROM_HIT_POINT:
    case STRAIGHT_AND_AWAY_FROM_LEAVE_POINT:
    {
        // Logic common between Bug1 and Bug2
		m_distance = 0;
		m_perimeter = 0;
        m_direction = NONE;
        MoveTowardsGoalOrNextBoundary(sensor, &move);
        break;
    }
    case AROUND:
    {
        // Keep moving around the boundary until the hit point is reached again
        if (m_simulator->ArePointsNear(curX, curY, m_hit[0], m_hit[1]))
        {
            m_mode = AROUND_AND_TOWARD_LEAVE_POINT;
        }

        UpdateLeavePoint();
        move = GetMoveAroundBoundary(sensor);
        break;
    }
    case AROUND_AND_TOWARD_LEAVE_POINT:
    {
        // Keep moving around the boundary until the leave point is reached
        if (m_simulator->ArePointsNear(curX, curY, m_leave[0], m_leave[1]))
        {
            move = GetMoveTowardsGoal();
            m_mode = STRAIGHT_AND_AWAY_FROM_LEAVE_POINT;
        }
        else
        {
            if(m_direction == NONE)
            {
			    if(m_distance < (m_perimeter - m_distance)) // Is going right faster than going left?
			    {
				    move = GetMoveAroundBoundary(sensor, RIGHT);
                    m_direction = RIGHT;
			    }
			    else
			    {
				    move = GetMoveAroundBoundary(sensor, LEFT);
                    m_direction = LEFT;
			    }
            }
            else
            {
		        move = GetMoveAroundBoundary(sensor, m_direction);
            }

        }
        break;
    }
    default:
    {
        // Should not get here!
    }
    }

    return move;
}

Move BugAlgorithms::Bug2(Sensor sensor)
{
    Move move;
    double curX = m_simulator->GetRobotCenterX();
    double curY = m_simulator->GetRobotCenterY();

    switch (m_mode)
    {
    case STRAIGHT:
    case AROUND_AND_AWAY_FROM_HIT_POINT:
    case STRAIGHT_AND_AWAY_FROM_LEAVE_POINT:
    {
        // Logic common between Bug1 and Bug2
        MoveTowardsGoalOrNextBoundary(sensor, &move);
        break;
    }
    case AROUND:
    {
        double goalX = m_simulator->GetGoalCenterX();
        double goalY = m_simulator->GetGoalCenterY();
        double initX = m_simulator->GetRobotInitX();
        double initY = m_simulator->GetRobotInitY();

        // Calculate the distance from hit point to goal. Variable is static
        // so the calculation is only done once.
        static double distHitToGoal =
            sqrt( pow(goalX - m_hit[0], 2) + pow(goalY - m_hit[1], 2));

        // Keep moving around the boundary until the m-line is reached
        // at a point closer to the goal
        if (m_simulator->IsPointNearLine(curX, curY, initX, initY,
                                         goalX, goalY) &&
            m_simulator->GetDistanceFromRobotToGoal() < distHitToGoal)
        {
            move = GetMoveTowardsGoal();
            UpdateLeavePoint();
            m_mode = STRAIGHT_AND_AWAY_FROM_LEAVE_POINT;
        }
        else
        {
            move = GetMoveAroundBoundary(sensor);
        }
        break;
    }
    case AROUND_AND_TOWARD_LEAVE_POINT:
    default:
    {
        // Should not get here!
    }
    }
    
    return move;
}

// Private utility methods

/**
 *@brief Returns the appropriately-sized (step) move in the specified direction
 *@param dx x component of directional vector
 *@param dy y component of directional vector
 *@param dist (optional) Pre-calculated distance if available. If not specified
 *            then it is calculated.
 */
Move BugAlgorithms::GetMove(const double &dx, const double &dy,
                            const double &dist)
{
    double distance = dist;

    // If distance wasn't provided, calculate it
    if (distance < 0)
    {
        distance = sqrt( dx*dx + dy*dy );
    }

    Move move = { m_simulator->GetStep() / distance * dx,
                  m_simulator->GetStep() / distance * dy };
    return move;
}


/**
 *@brief Returns the move to go along (parallel with) the boundary edge
 *@param sensor provides closest point from obstacle boundary to robot center
 *@param dir (optional) The direction (right/left) the robot should go. Defaults right.
 */
Move BugAlgorithms::GetMoveAlongBoundary(const Sensor &sensor, Direction dir)
{
    // Calculate turn perpendicular to boundary hit point direction. Arbitrarily
    // chose right turn.
    double xToBoundary = sensor.m_xmin - m_simulator->GetRobotCenterX();
    double yToBoundary = sensor.m_ymin - m_simulator->GetRobotCenterY();

	if(dir == RIGHT) //Navigate Right
	{
		double xPerpBoundary = yToBoundary;
		double yPerpBoundary = -1 * xToBoundary;
		return GetMove(xPerpBoundary, yPerpBoundary);
	}
	else //Navigate Left
	{
		double xPerpBoundary = -1 * yToBoundary;
		double yPerpBoundary = xToBoundary;
		return GetMove(xPerpBoundary, yPerpBoundary);
	}
}

/**
 *@brief Returns the move to go around the boundary
 *@param sensor provides closest point from obstacle boundary to robot center
 *@param dir (optional) The direction (right/left) the robot should go. Defaults right.
 */
Move BugAlgorithms::GetMoveAroundBoundary(const Sensor &sensor, Direction dir)
{
    // If sensor detects a hit, move along (parallel with) boundary edge.
    // Otherwise, move towards (perpendicular to) the boundary edge.
    Move move;
    if (sensor.m_dmin <= m_simulator->GetWhenToTurn())
    {
        move = GetMoveAlongBoundary(sensor, dir);
    }
    else
    {
        move = GetMoveTowardsBoundary(sensor);
    }
    double moveDist = sqrt(move.m_dx*move.m_dx + move.m_dy*move.m_dy);
	m_perimeter += moveDist;
    return move;
}

/**
 *@brief Returns the move to go towards the boundary
 *@param sensor provides closest point from obstacle boundary to robot center
 */
Move BugAlgorithms::GetMoveTowardsBoundary(const Sensor &sensor)
{
    double xToBoundary = sensor.m_xmin - m_simulator->GetRobotCenterX();
    double yToBoundary = sensor.m_ymin - m_simulator->GetRobotCenterY();
    return GetMove(xToBoundary, yToBoundary);
}

/**
 *@brief Returns the move to go towards the goal
 */
Move BugAlgorithms::GetMoveTowardsGoal()
{
    double xToGoal =
        m_simulator->GetGoalCenterX() - m_simulator->GetRobotCenterX();
    double yToGoal =
        m_simulator->GetGoalCenterY() - m_simulator->GetRobotCenterY();
    double distToGoal = m_simulator->GetDistanceFromRobotToGoal();
    return GetMove(xToGoal, yToGoal, distToGoal);
}

/**
 *@brief Performs the work necessary to move towards the goal or next boundary
 *       (Common code between Bug1 and Bug2 algorithms)
 *@param sensor provides closest point from obstacle boundary to robot center
 *@param move the returned move generated by this method
 */
void BugAlgorithms::MoveTowardsGoalOrNextBoundary(const Sensor &sensor,
                                                  Move *move)
{
    double curX = m_simulator->GetRobotCenterX();
    double curY = m_simulator->GetRobotCenterY();

    switch (m_mode)
    {
    case STRAIGHT:
    {
        if (sensor.m_dmin <= m_simulator->GetWhenToTurn())
        {
            // Robot detected a hit, save the hit point, and start moving
            // around the boundary
            m_hit[0] = curX;
            m_hit[1] = curY;
            m_mode = AROUND_AND_AWAY_FROM_HIT_POINT;
            *move = GetMoveAlongBoundary(sensor);
        }
        else
        {
            // No hit detected, move towards the goal
            *move = GetMoveTowardsGoal();
        }
        break;
    }
    case AROUND_AND_AWAY_FROM_HIT_POINT:
    {
        // Keep moving around the boundary while robot is still near the hit
        // point
        if (!m_simulator->ArePointsNear(curX, curY, m_hit[0], m_hit[1]))
        {
            m_mode = AROUND;
        }

        *move = GetMoveAroundBoundary(sensor);
        break;
    }
    case STRAIGHT_AND_AWAY_FROM_LEAVE_POINT:
    {
        // Keep moving straight towards the goal while robot is still near the
        // boundary. But if the robot is getting closer to the boundary, switch
        // to STRAIGHT so the boundary can be detected again.
        static double lastSensorDist = sensor.m_dmin;
        if (sensor.m_dmin > m_simulator->GetWhenToTurn() ||
            sensor.m_dmin < lastSensorDist)
        {
            m_mode = STRAIGHT;
        }

        *move = GetMoveTowardsGoal();
        break;
    }
    case AROUND:
    case AROUND_AND_TOWARD_LEAVE_POINT:
    default:
    {
        // Should not get here as these are Bug1 and Bug2 specific!
    }
    }
}

/**
 *@brief Updates the leave point if current position is closer to the goal
 */
void BugAlgorithms::UpdateLeavePoint()
{
    // If robot has discovered a point closer to the goal than previously known,
    // save it.
    double distToGoal = m_simulator->GetDistanceFromRobotToGoal();
    if (distToGoal < m_distLeaveToGoal)
    {
        m_distLeaveToGoal = distToGoal;
        m_leave[0] = m_simulator->GetRobotCenterX();
        m_leave[1] = m_simulator->GetRobotCenterY();
		m_distance = m_perimeter;
    }
}

