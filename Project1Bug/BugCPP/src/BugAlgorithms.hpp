/**
 *@file BugAlgorithms.hpp
 *@brief Prototype for the Bug algorithms required in this assignment
 */

#ifndef BUG_ALGORITHMS_HPP_
#define BUG_ALGORITHMS_HPP_

#include "Simulator.hpp"

/**
 * @brief Bug algorithm computes a small move (m_dx, m_dy) that the robot needs to make
 */
struct Move
{
    double m_dx;
    double m_dy;    
};


/**
 *@brief Prototype for the different Bug algorithms required in this assignment
 *
 *@remark
 *  Feel free to add additional functions/data members as you see fit
 */
class BugAlgorithms
{
public:
    /**
     *@brief Set simulator
     *@param simulator pointer to simulator
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator
     */
    BugAlgorithms(Simulator * const simulator);
            
    /**
     *@brief Free memory and delete objects allocated by this instance
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator.
     */
    ~BugAlgorithms(void);
     
    
    /**
     *@brief Select the appropriate move so that the robot behaves
     *       as described in the respective bug algorithms.
     *@param sensor provides closest point from obstacle boundary to robot center
     */
    Move Bug0(Sensor sensor);
    Move Bug1(Sensor sensor);
    Move Bug2(Sensor sensor);
    
protected:
    /**
     *@brief Pointer to simulator
     */
    Simulator  *m_simulator;

    enum Mode
    {
        STRAIGHT,
        STRAIGHT_AND_AWAY_FROM_LEAVE_POINT,
        AROUND_AND_AWAY_FROM_HIT_POINT,
        AROUND_AND_TOWARD_LEAVE_POINT,
        AROUND
    };

    enum Direction
    {
        RIGHT,
        LEFT,
        NONE,
    };

    double m_hit[2], m_leave[2], m_distLeaveToGoal, m_distance, m_perimeter;
    int    m_mode;
    Direction m_direction;
    
private:

    /**
     *@brief Returns the appropriately-sized (step) move in the specified direction
     *@param dx x component of directional vector
     *@param dy y component of directional vector
     *@param dist (optional) Pre-calculated distance if available. If not specified
     *            then it is calculated.
     */
    Move GetMove(const double &dx, const double &dy,
                 const double &dist = -1);

    /**
     *@brief Returns the move to go along (parallel with) the boundary edge
     *@param sensor provides closest point from obstacle boundary to robot center
     *@param dir (optional) The direction (right/left) the robot should go. Defaults right.
     */
    Move GetMoveAlongBoundary(const Sensor &sensor, Direction dir = RIGHT);

    /**
     *@brief Returns the move to go around the boundary
     *@param sensor provides closest point from obstacle boundary to robot center
     *@param dir (optional) The direction (right/left) the robot should go. Defaults right.
     */
    Move GetMoveAroundBoundary(const Sensor &sensor, Direction dir = RIGHT);

    /**
     *@brief Returns the move to go towards the boundary
     *@param sensor provides closest point from obstacle boundary to robot center
     */
    Move GetMoveTowardsBoundary(const Sensor &sensor);

    /**
     *@brief Returns the move to go towards the goal
     */
    Move GetMoveTowardsGoal();

    /**
     *@brief Performs the work necessary to move towards the goal or next boundary
     *@param sensor provides closest point from obstacle boundary to robot center
     *@param move the returned move generated by this method
     */
    void MoveTowardsGoalOrNextBoundary(const Sensor &sensor, Move *move);

    /**
     *@brief Updates the leave point if current position is closer to the goal (For Bug2)
     */
    void UpdateLeavePoint();

    friend class Graphics;
};

#endif
