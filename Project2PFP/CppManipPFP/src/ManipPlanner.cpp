#include "ManipPlanner.hpp"

ManipPlanner::ManipPlanner(ManipSimulator * const manipSimulator)
{
    m_manipSimulator = manipSimulator;   
}

ManipPlanner::~ManipPlanner(void)
{
    //do not delete m_simulator  
}

double jacobianTransposeAtIndex(int i)
{
    //figure this out
}

Point getLocalPoint(Point origin, double x, double y)
{
    Point local;
    local.m_x = x - origin.m_x;
    local.m_y = y - origin.m_y;
    return local;
}

Point getRepulsiveForce(double xJoint, double yJoint, int numberOfObstacles, ManipSimulator * const manipSimulator)
{
    Point sumOfRep;
    sumOfRep.m_x = 0;
    sumOfRep.m_y = 0;
    for(int i = 0; i < numberOfObstacles; i = i+1)
    {
        Point p = manipSimulator ->ClosestPointOnObstacle(i, xJoint, yJoint);
        double x = xJoint - p.m_x;
        double y = yJoint - p.m_y;
        sumOfRep.m_x = sumOfRep.m_x + x;
        sumOfRep.m_y = sumOfRep.m_y + y;
    }
    return sumOfRep;
}
void ManipPlanner::ConfigurationMove(double allLinksDeltaTheta[])
{
    double totalForce = 0;
    double x_goal = m_manipSimulator ->GetGoalCenterX();
    double y_goal = m_manipSimulator ->GetGoalCenterY();
    double numberOfLinks = m_manipSimulator ->GetNrLinks();
    double numberOfObstacles = m_manipSimulator ->GetNrObstacles();
    for(int i = 0; i < numberOfLinks; i = i+1)
    {
        Point forceAtIndex;
        forceAtIndex.m_x = 0;
        forceAtIndex.m_y = 0;
        int x_joint = m_manipSimulator ->GetLinkEndX(i);
        int y_joint = m_manipSimulator ->GetLinkEndY(i);
        if(i == numberOfLinks - 1)
        {
            double attractiveX = x_goal - x_joint;
            double attractiveY = y_goal - y_joint;
            Point repulsiveForce = getRepulsiveForce(x_joint, y_joint, numberOfObstacles, m_manipSimulator);
            forceAtIndex.m_x = repulsiveForce.m_x + attractiveX;
            forceAtIndex.m_y = repulsiveForce.m_y + attractiveY;
        }
        else
        {
            forceAtIndex = getRepulsiveForce(x_joint, y_joint, numberOfObstacles,  m_manipSimulator);
        }
        double u = jacobianTransposeAtIndex(i) * forceAtIndex;

    }
}



