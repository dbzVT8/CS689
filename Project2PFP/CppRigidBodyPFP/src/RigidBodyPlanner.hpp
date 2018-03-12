#ifndef RIGID_BODY_PLANNER_HPP_
#define RIGID_BODY_PLANNER_HPP_

#include "RigidBodySimulator.hpp"
#include <list>

struct RigidBodyMove
{
    double m_dx;
    double m_dy;
    double m_dtheta;
};

class RigidBodyPlanner
{
public:
    RigidBodyPlanner(RigidBodySimulator * const simulator);
            
    ~RigidBodyPlanner(void);

    /*
     * This is the function that you should implement.
     * This function needs to compute by how much the position (dx, dy) 
     * and orientation (dtheta) should change so that the robot makes a small 
     * move toward the goal while avoiding obstacles, 
     * as guided by the potential field.
     *
     * You have access to the simulator.
     * You can use the methods available in simulator to get all the information
     * you need to correctly implement this function
     *
     */
    RigidBodyMove ConfigurationMove(void);
    
protected:
    RigidBodySimulator *m_simulator;

    void checkIfRobotStuck(const Point &rjw);
    void addFakeObstacle(const Point &rjw,
                         std::vector<double> rep_force);
    int getFurthestVertexFromGoal();
    std::vector<double> computeRepulsiveForce(const Point &rjw);
    std::vector<double> getObstacleRepForce(const Point &rjw,
                                            const Point &obstacle);

private:
    int m_vertexFurthestFromGoal; // The robot vertex furthest from goal
    std::vector<double> m_localVertices; // The robot's local vertices
    bool m_robotStuck; // Designates whether robot is stuck
    std::list<Point> m_averageMoves; // List of moves to average dist traveled
    std::list<Point> m_fakeObstacles; // List of fake obstacles
    bool m_furthestVertexCalculated; // Designates whether furthest vertex from
                                     // goal has been calculated after being stuck
};

#endif
