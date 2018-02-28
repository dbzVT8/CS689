#include "ManipPlanner.hpp"

ManipPlanner::ManipPlanner(ManipSimulator * const manipSimulator)
{
    m_manipSimulator = manipSimulator;   
}

ManipPlanner::~ManipPlanner(void)
{
    //do not delete m_simulator  
}


double getDerivativeShortCut( int j, int i,ManipSimulator * const manipSimulator, bool isX )
{
    if(isX)
    {
        return ((manipSimulator ->GetLinkEndY(j)) * -1) + (manipSimulator ->GetLinkStartY(i));
    }
    else
    {
        return (manipSimulator ->GetLinkEndX(j)) - (manipSimulator ->GetLinkStartX(i));
    }
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
    double U[sizeof(allLinksDeltaTheta)];
    for(int i = 0; i < numberOfLinks; i++)
    {
        U[i] = 0; 
    }
    for(int j = 0; j < numberOfLinks; j = j+1)
    {
        Point forceAtIndex;
        forceAtIndex.m_x = 0;
        forceAtIndex.m_y = 0;
        double x_joint = m_manipSimulator ->GetLinkEndX(j);
        double y_joint = m_manipSimulator ->GetLinkEndY(j);
        if(j == numberOfLinks - 1)
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

        double force[2];
        force[0] = forceAtIndex.m_x;
        force[1] = forceAtIndex.m_y;

        double jacobianTranspose[sizeof(allLinksDeltaTheta)][2];
        for(int i = 0; i < sizeof(allLinksDeltaTheta); i++)
        {
            jacobianTranspose[i][0] = getDerivativeShortCut(j, i, m_manipSimulator, true);
            jacobianTranspose[i][1] = getDerivativeShortCut(j, i, m_manipSimulator, false);
        }

        for(int i = 0; i < numberOfLinks; i++)
        {
            U[i] = U[i] + force[0] * jacobianTranspose[i][0] + force[1] * jacobianTranspose[i][1];
        }
    }

    //make it a unit vector
    double magnitude = 0;
    for(int i = 0; i < numberOfLinks; i++)
    {
        magnitude = magnitude + pow(U[i], 2);
    }
    magnitude = sqrt(magnitude);

    for(int i = 0; i < numberOfLinks; i++)
    {
        U[i] = U[i] / magnitude;
    }

    for(int i = 0; i < numberOfLinks; i++)
    {
        allLinksDeltaTheta[i] = U[i];
    }
}



