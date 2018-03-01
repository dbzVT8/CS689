#include "RigidBodyPlanner.hpp"
#include <iostream>

void ToUnitVector(double *dx, double *dy, double *dtheta = NULL)
{
    if (dx != NULL && dy != NULL)
    {
        double tempX = *dx;
        double tempY = *dy;
        double tempTheta = 0;

        if (dtheta != NULL)
        {
            tempTheta = *dtheta;
        }

        double magnitude = sqrt(pow(tempX, 2) +
                                pow(tempY, 2) + pow(tempTheta, 2));
        *dx = 1/magnitude * tempX;
        *dy = 1/magnitude * tempY;
        *dtheta = 1/magnitude * tempTheta;
    }
}

RigidBodyPlanner::RigidBodyPlanner(RigidBodySimulator * const simulator)
{
    m_simulator = simulator;   

    // Calculate and store local coordinates
    int numVerts = m_simulator->GetNrRobotVertices();
    m_localVertices.resize(2 * numVerts);
    std::cout << "Init vertices: ";
    for (int i=0; i<m_localVertices.size(); ++i)
    {
        m_localVertices[i] = m_simulator->GetRobotVertices()[i];
        std::cout << m_localVertices[i] << " ";
    }
    std::cout << std::endl;
}

RigidBodyPlanner::~RigidBodyPlanner(void)
{
    //do not delete m_simulator  
}


RigidBodyMove RigidBodyPlanner::ConfigurationMove(void)
    
{
    RigidBodyMove move;

    for (int j=0; j<m_simulator->GetNrRobotVertices()*2; j+=2)
    {
        double rjw_x = m_simulator->GetRobotVertices()[j];
        double rjw_y = m_simulator->GetRobotVertices()[j+1];
        double force_jx_att = m_simulator->GetGoalCenterX() - rjw_x; 
        double force_jy_att = m_simulator->GetGoalCenterY() - rjw_y;
        double force_jx_rep = 0;
        double force_jy_rep = 0;

        for (int i=0; i<m_simulator->GetNrObstacles(); ++i)
        {
            Point obstacle =
                m_simulator->ClosestPointOnObstacle(i, rjw_x, rjw_y);
            force_jx_rep += (rjw_x - obstacle.m_x);
            force_jy_rep += (rjw_y - obstacle.m_y);
        }

        double force_jx = force_jx_att + force_jx_rep;
        double force_jy = force_jy_att + force_jy_rep;
        ToUnitVector(&force_jx, &force_jy);

        double xj = m_localVertices[j];
        double yj = m_localVertices[j+1];
        double theta = m_simulator->GetRobotTheta();
        double del_delTheta_rjw_x = -xj*sin(theta) - yj*cos(theta);
        double del_delTheta_rjw_y = xj*cos(theta) + yj*sin(theta);

        move.m_dx += force_jx;
        move.m_dy += force_jy;
        move.m_dtheta += (force_jx * del_delTheta_rjw_x) +
                         (force_jy * del_delTheta_rjw_y);
    }

    ToUnitVector(&move.m_dx, &move.m_dy, &move.m_dtheta);
    double alpha = 0.1;
    double beta = 0.01;
    move.m_dx *= alpha;
    move.m_dy *= alpha;
    move.m_dtheta *= beta;

    return move;
}


