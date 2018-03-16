#include "RigidBodyPlanner.hpp"
#include <iostream>

// Robot's step size for moves
static const double STEP_SIZE = 0.05;

// Repulsive scaling factor for real and fake obstacles. (higher = stronger force)
static const int REAL_OBSTACLE_SENS = 20; 
static const int FAKE_OBSTACLE_SENS = 3;

// Num moves to track average distance traveled
static const int NUM_MOVES_TO_TRACK = 30;

// Num moves traveled before robot is considered stuck
static const int NUM_MOVES_BEFORE_STUCK = 5;

// Minimum distance for obstacles to be visible by robot
static const int OBSTACLE_AWARENESS_DIST = 5;

// Method that returns the magnitude of the given vector
double GetVectorMagnitude(std::vector<double> v)
{
    double sumOfSquares = 0;
    for (int i=0; i<v.size(); ++i)
    {
        sumOfSquares += v[i]*v[i];
    }
    return sqrt(sumOfSquares);
}

// Method that returns the distance between two points a and b.
double GetDistBetweenPoints(const double &a_x, const double &a_y,
                            const double &b_x, const double &b_y)
{
    return sqrt(pow(b_y - a_y, 2) + pow(b_x - a_x, 2));
}

// Method to convert a vector to unit length
void ToUnitVector(std::vector<double> *v)
{
    double mag = GetVectorMagnitude(*v);
    for (int i=0; i<v->size(); ++i)
    {
        v->at(i) *= 1/mag;
    }
}

/* Constructor
   @param simulator
*/
RigidBodyPlanner::RigidBodyPlanner(RigidBodySimulator * const simulator) :
    m_simulator(simulator), m_vertexFurthestFromGoal(0), m_robotStuck(false),
    m_furthestVertexCalculated(false)
{
    // Calculate and store local coordinates
    int numVerts = m_simulator->GetNrRobotVertices();
    m_localVertices.resize(2 * numVerts);
    for (int i=0; i<m_localVertices.size(); ++i) {
        m_localVertices[i] = m_simulator->GetRobotVertices()[i];
    }
}

RigidBodyPlanner::~RigidBodyPlanner(void)
{
    //do not delete m_simulator  
}

/* Method to calculate the robot's next move
*/
RigidBodyMove RigidBodyPlanner::ConfigurationMove(void)
{
    RigidBodyMove move;
    move.m_dtheta = 0;
    move.m_dx = 0;
    move.m_dy = 0;

    // If the robot is stuck, calculate and save off the vertex furthest
    // away from the goal
    if (m_robotStuck)
    {
        m_vertexFurthestFromGoal = getFurthestVertexFromGoal();
        m_furthestVertexCalculated = true;
    }

    // total repulsive force used for creating fake obstacles
    std::vector<double> force_rep(2,0);

    // For each vertex of the robot...
    for (int j=0; j<m_simulator->GetNrRobotVertices()*2; j+=2)
    {
        // Get the current vertex's global coordinates
        Point rjw = {m_simulator->GetRobotVertices()[j],
                     m_simulator->GetRobotVertices()[j+1]};

        // If the robot is currently not stuck, track the movement of one
        // vertex (vertex 0) to check if it's stuck.
        if (!m_robotStuck && j == 0)
        {
            checkIfRobotStuck(rjw);
        }

        // Calculate the attractive force to the goal
        std::vector<double> force_j_att(2,0);
        force_j_att[0] = (m_simulator->GetGoalCenterX() - rjw.m_x);
        force_j_att[1] = (m_simulator->GetGoalCenterY() - rjw.m_y);
        ToUnitVector(&force_j_att);

        // For all obstacles real and fake, calculate the repulsive force and
        // add it to the overall repulsive force for this vertex
        std::vector<double> force_j_rep = computeRepulsiveForce(rjw);

        // add to the total repulsive force for this move to be used for
        // creating fake obstacles
        force_rep[0] += force_j_rep[0];
        force_rep[1] += force_j_rep[1];

        // Calculate the overall force for this vertex and make it unit length
        std::vector<double> force(2,0);
        force[0] = force_j_att[0] + force_j_rep[0];
        force[1] = force_j_att[1] + force_j_rep[1];
        ToUnitVector(&force);

        // Get the local coordinates for this vertex, and compute the Jacobian
        // components
        double xj = m_localVertices[j];
        double yj = m_localVertices[j+1];
        double theta = m_simulator->GetRobotTheta();
        double del_delTheta_rjw_x = -xj*sin(theta) - yj*cos(theta);
        double del_delTheta_rjw_y = xj*cos(theta) + yj*sin(theta);

        // "Multiply" the Jacobian with the force and add to the overall move
        move.m_dx += force[0];
        move.m_dy += force[1];
        move.m_dtheta += (force[0] * del_delTheta_rjw_x) +
                         (force[1] * del_delTheta_rjw_y);
    }

    // Make the move unit length then scale it to a step size
    std::vector<double> move_xy(2,0);
    move_xy[0] = move.m_dx;
    move_xy[1] = move.m_dy;
    ToUnitVector(&move_xy);
    double thetaStep = 0.005;
    move.m_dtheta > 0 ? move.m_dtheta = thetaStep : move.m_dtheta = -thetaStep;
    move.m_dx *= STEP_SIZE;
    move.m_dy *= STEP_SIZE;

    // If the furthest vertex from the goal has been calculated, this means
    // the robot is stuck. Create a fake obstacle behind the robot to help
    // push it towards the goal.
    if (m_furthestVertexCalculated)
    {
        int j = m_vertexFurthestFromGoal;
        Point rjw = {m_simulator->GetRobotVertices()[j],
                     m_simulator->GetRobotVertices()[j+1]};
        addFakeObstacle(rjw, force_rep);
        m_averageMoves.clear();
        m_robotStuck = false;
        m_furthestVertexCalculated = false;
    }

    return move;
}

/* Method to compute the repulsive force
   @param rjw global coordinates of a particular robot vertex
*/
std::vector<double> RigidBodyPlanner::computeRepulsiveForce(const Point &rjw)
{
    std::vector<double> force_j_rep(2,0);

    for (int i=0; i<m_simulator->GetNrObstacles(); ++i)
    {
        Point obstacle =
            m_simulator->ClosestPointOnObstacle(i, rjw.m_x, rjw.m_y);
        std::vector<double> obst_rep_force = getObstacleRepForce(rjw, obstacle);
        force_j_rep[0] += obst_rep_force[0];
        force_j_rep[1] += obst_rep_force[1];
    }

    // For each fake obstacle, calculate it's repulsive force and add it
    // to the overall repulsive force for this vertex. Scale the repulsive
    // force of fake obstacles less than real obstacles to ensure real
    // obstacles are avoided.
    for (std::list<Point>::iterator it=m_fakeObstacles.begin();
         it != m_fakeObstacles.end(); ++it)
    {
        Point obstacle = *it;
        std::vector<double> obst_rep_force = getObstacleRepForce(rjw, obstacle);
        force_j_rep[0] += obst_rep_force[0];
        force_j_rep[1] += obst_rep_force[1];
    }

    return force_j_rep;
}

/* Method to get the repulsive force of an obstacle
   @param rjw global coordinates of a particular robot vertex
   @param obstacle the position of the obstacle
*/
std::vector<double>
   RigidBodyPlanner::getObstacleRepForce(const Point &rjw, const Point &obstacle)
{
    double obstacleDist = GetDistBetweenPoints(rjw.m_x, rjw.m_y,
                                               obstacle.m_x, obstacle.m_y);
    std::vector<double> rep_force(2,0);

    // Ignore any obstacles that are OBSTACLE_AWARENESS_DIST  units or
    // more away
    if (obstacleDist < OBSTACLE_AWARENESS_DIST)
    {
        rep_force[0] += FAKE_OBSTACLE_SENS/
                              obstacleDist * (rjw.m_x - obstacle.m_x);
        rep_force[1] += FAKE_OBSTACLE_SENS/
                              obstacleDist * (rjw.m_y - obstacle.m_y);
    }

   return rep_force;
}

/* Method to check whether the robot is stuck
   @param rjw global coordinates of a particular robot vertex
*/
void RigidBodyPlanner::checkIfRobotStuck(const Point &rjw)
{
    // If robot is not currently stuck and there are at least
    // 10 moves tallied, get the distance moved between the first
    // and 10th move.
    if (!m_robotStuck && m_averageMoves.size() >= NUM_MOVES_TO_TRACK)
    {
        Point front = m_averageMoves.front();
        Point back = m_averageMoves.back();
        double dist = GetDistBetweenPoints(front.m_x, front.m_y,
                                           back.m_x, back.m_y);

        // If distance traveled is less than NUM_MOVES_BEFORE_STUCK step sizes
        // of NUM_MOVES_TO_TRACK away, the robot is considered stuck
        if (dist < NUM_MOVES_BEFORE_STUCK*STEP_SIZE) {
            m_robotStuck = true;
        }

        // Remove first point to make room for next one
        m_averageMoves.pop_front();
    }

    Point pt = {rjw.m_x, rjw.m_y};
    m_averageMoves.push_back(pt);
}

/* Method to add a fake obstacle when the robot gets stuck
   @param rjw global coordinates of a particular robot vertex
   @param rep_force repulsive force for all robot vertices
*/
void RigidBodyPlanner::addFakeObstacle(const Point &rjw,
                                       std::vector<double> rep_force)
{
    // Only add an obstacle if the repulsive force has a nonzero magnitude
    if (GetVectorMagnitude(rep_force) > 0)
    {
        ToUnitVector(&rep_force);

        // Create a fake obstacle that is force_x and force_y away from the
        // robot's current position. This will put the fake obstacle behind
        // the robot and push it towards the goal.
        Point obstacle;
        obstacle.m_x = rjw.m_x + rep_force[0];
        obstacle.m_y = rjw.m_y + rep_force[1];
        m_fakeObstacles.push_back(obstacle);
    }
}

// Calculate which vertex on the robot is furthest away from the goal.
// This is used to help determine the location of fake obstacles.
int RigidBodyPlanner::getFurthestVertexFromGoal()
{
    double furthestDist = 0;
    int furthestVertex = 0;
    for (int j=0; j<m_simulator->GetNrRobotVertices()*2; j+=2)
    {
        double rjw_x = m_simulator->GetRobotVertices()[j];
        double rjw_y = m_simulator->GetRobotVertices()[j+1];
        double distToGoal = GetDistBetweenPoints(rjw_x, rjw_y,
            m_simulator->GetGoalCenterX(), m_simulator->GetGoalCenterY());

        if (distToGoal > furthestDist)
        {
            furthestDist = distToGoal;
            furthestVertex = j;
        }
    }

    return furthestVertex;
}
