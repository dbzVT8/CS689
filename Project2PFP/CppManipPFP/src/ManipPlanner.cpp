#include "ManipPlanner.hpp"
#include <vector>

ManipPlanner::ManipPlanner(ManipSimulator * const manipSimulator)
{
    m_manipSimulator = manipSimulator;   
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
        magnitude = magnitude + pow(input[i], 2);
    }
    magnitude = sqrt(magnitude);

    for(int i = 0; i < input.size(); i++)
    {
        output[i] = input[i] / magnitude;
    }
    return output;
}

std::vector<double> getRepulsiveForce(double xJoint, double yJoint, int numberOfObstacles, ManipSimulator * const manipSimulator)
{
    std::vector<double> sumOfRep(2);
    for(int i = 0; i < numberOfObstacles; i = i+1)
    {
        Point p = manipSimulator ->ClosestPointOnObstacle(i, xJoint, yJoint);
        double x = xJoint - p.m_x;
        double y = yJoint - p.m_y;
        sumOfRep[0] = sumOfRep[0] + x;
        sumOfRep[1] = sumOfRep[1] + y;
    }
    return sumOfRep;
}


void ManipPlanner::ConfigurationMove(double allLinksDeltaTheta[])
{
    double totalForce = 0;
    double x_goal = m_manipSimulator ->GetGoalCenterX();
    double y_goal = m_manipSimulator ->GetGoalCenterY();
    const int numberOfLinks = m_manipSimulator ->GetNrLinks();
    const int numberOfObstacles = m_manipSimulator ->GetNrObstacles();
    std::vector<double> U(numberOfLinks);
    
    for(int j = 0; j < numberOfLinks; j = j+1)
    {
        double x_joint = m_manipSimulator ->GetLinkEndX(j);
        double y_joint = m_manipSimulator ->GetLinkEndY(j);
        
        std::vector<double> force(2);
        if(j == numberOfLinks - 1)
        {
            double attractiveX = x_goal - x_joint;
            double attractiveY = y_goal - y_joint;
            std::vector<double> repulsiveForce = getRepulsiveForce(x_joint, y_joint, numberOfObstacles, m_manipSimulator);
            force[0] = repulsiveForce[0] + attractiveX;
            force[1] = repulsiveForce[1]  + attractiveY;
        }
        else
        {
             force = getRepulsiveForce(x_joint, y_joint, numberOfObstacles,  m_manipSimulator);
        }

        for(int i = 0; i < numberOfLinks; i++)
        {
            double jacobianTransposei0 = ((m_manipSimulator ->GetLinkEndY(j)) * -1) + (m_manipSimulator ->GetLinkStartY(i));
            double jacobianTransposei1 = (m_manipSimulator ->GetLinkEndX(j)) - (m_manipSimulator ->GetLinkStartX(i));
            
            U[i] = U[i] + force[0] * jacobianTransposei0 + force[1] * jacobianTransposei1;
        }

    }

    U = makeUnitVector(U);

    for(int i = 0; i < numberOfLinks; i++)
    {
        allLinksDeltaTheta[i] = U[i] * .01;
    }
}



