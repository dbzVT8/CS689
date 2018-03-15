#include "ManipPlanner.hpp"
#include <vector>
#include <iostream>
using namespace std;

ManipPlanner::ManipPlanner(ManipSimulator * const manipSimulator)
{
    m_manipSimulator = manipSimulator;  
    m_attractiveFactor = 10;
    m_repulsiveFactor = 15;
    m_repulsiveThreshold = 3;
    m_numberOfIterations = 0;
    m_stepSize = 0.005;
}

ManipPlanner::~ManipPlanner(void)
{
    //do not delete m_simulator  
}

std::vector<double> makeUnitVector(std::vector<double> input)
{
    double magnitude = 0;
    std::vector<double> output(input.size());
    for(int i = 0; i < input.size(); i++)
    {
        magnitude += pow(input[i], 2);
    }
    magnitude = sqrt(magnitude);

    for(int i = 0; i < input.size(); i++)
    {
        output[i] = input[i] / magnitude;
    }
    return output;
}

double getDistanceBetweenPoints(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1,2));
}

std::vector<double> ManipPlanner::getRepulsiveForce(int joint)
{
    std::vector<double> sumOfRep(2);
    double xJoint = m_manipSimulator->GetLinkEndX(joint);
    double yJoint = m_manipSimulator->GetLinkEndY(joint);
    const int numberOfObstacles = m_manipSimulator->GetNrObstacles();
    for(int i = 0; i < numberOfObstacles; i++)
    {
        Point p = m_manipSimulator->ClosestPointOnObstacle(i, xJoint, yJoint);
        double distanceToObstacle = getDistanceBetweenPoints(p.m_x, p.m_y,
                                                             xJoint, yJoint);
        if(distanceToObstacle < m_repulsiveThreshold)
        {
            double x = (xJoint - p.m_x) * m_repulsiveFactor/(distanceToObstacle);
            double y = (yJoint - p.m_y) * m_repulsiveFactor/(distanceToObstacle);
            sumOfRep[0] = (sumOfRep[0] + x);
            sumOfRep[1] = (sumOfRep[1] + y);
        }
    }
    return sumOfRep;
}

void ManipPlanner::ConfigurationMove(double allLinksDeltaTheta[])
{
    double x_goal = m_manipSimulator->GetGoalCenterX();
    double y_goal = m_manipSimulator->GetGoalCenterY();
    const int numberOfLinks = m_manipSimulator->GetNrLinks();
    double currentX = m_manipSimulator->GetLinkEndX(numberOfLinks - 1);
    double currentY = m_manipSimulator->GetLinkEndY(numberOfLinks - 1);
    std::vector<double> U(numberOfLinks);
    double distanceToGoal = getDistanceBetweenPoints(currentX, currentY,
                                                     x_goal, y_goal);

    if(m_numberOfIterations < c_MaxNumberOfIterations)
    {
        m_numberOfIterations = m_numberOfIterations + 1;
    }
    else 
    {
        //has reached goal but is still jerky
        if(m_manipSimulator->HasRobotReachedGoal()) 
        {
            if(m_stepSize > .01)
            {
                m_stepSize = m_stepSize * .9;
            }
            m_numberOfIterations = 0;
        }
        else //arm is stuck
        {
            if(m_attractiveFactor < 3)
            {
                m_attractiveFactor = m_attractiveFactor * 1.2;
            }

            if(m_repulsiveThreshold > 1.5)
            {
                m_repulsiveThreshold = m_repulsiveThreshold  * .9;
            }
                
            if(m_stepSize > .01)
            {
                m_stepSize = m_stepSize * .9;
            }
            m_numberOfIterations = 0;
        }
    }
    
    for(int j = 0; j < numberOfLinks; j++)
    {
        double x_joint = m_manipSimulator ->GetLinkEndX(j);
        double y_joint = m_manipSimulator ->GetLinkEndY(j);
        
        std::vector<double> force(2);
        if(j == numberOfLinks - 1)
        {
            double attractiveX = (x_goal - x_joint) *
                (m_attractiveFactor/distanceToGoal);
            double attractiveY = (y_goal - y_joint) *
                (m_attractiveFactor/distanceToGoal);

            std::vector<double> repulsiveForce = getRepulsiveForce(j);
            force[0] = repulsiveForce[0] + attractiveX;
            force[1] = repulsiveForce[1] + attractiveY;
        }
        else
        {
             std::vector<double> repulsiveForce = getRepulsiveForce(j);
             force[0] = repulsiveForce[0];
             force[1] = repulsiveForce[1];
        }

        for(int i = 0; i < numberOfLinks; i++)
        {
            double jacobianTransposei0 =
                ((m_manipSimulator ->GetLinkEndY(j)) * -1) +
                 (m_manipSimulator ->GetLinkStartY(i));
            double jacobianTransposei1 =
                (m_manipSimulator ->GetLinkEndX(j)) -
                (m_manipSimulator ->GetLinkStartX(i));
            
            U[i] += force[0] * jacobianTransposei0 +
                    force[1] * jacobianTransposei1;
        }

    }

    U = makeUnitVector(U);

    for(int i = 0; i < numberOfLinks; i++)
    {
        allLinksDeltaTheta[i] = U[i] * m_stepSize;
    }
}
