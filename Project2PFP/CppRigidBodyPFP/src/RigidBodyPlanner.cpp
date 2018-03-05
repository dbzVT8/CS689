#include "RigidBodyPlanner.hpp"
#include <iostream>

double GetVectorMagnitude(const double &dx, const double &dy, const double &dz = 0)
{
    return sqrt(dx*dx + dy*dy + dz*dz);
}

double GetDistBetweenPoints(const double &a_x, const double &a_y,
                            const double &b_x, const double &b_y)
{
    return sqrt(pow(b_y - a_y, 2) + pow(b_x - a_x, 2));
}

void ToUnitVector(double *dx, double *dy, double *dz = 0)
{
    if (dx != 0 && dy != 0)
    {
        double tempX = *dx;
        double tempY = *dy;
        double tempZ = 0;

        if (dz != 0)
        {
            tempZ = *dz;
        }

        double magnitude = GetVectorMagnitude(tempX, tempY, tempZ);
        *dx = 1/magnitude * tempX;
        *dy = 1/magnitude * tempY;

        if (dz != 0)
        {
            *dz = 1/magnitude * tempZ;
        }
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
        double distToGoal = GetDistBetweenPoints(rjw_x, rjw_y,
            m_simulator->GetGoalCenterX(), m_simulator->GetGoalCenterY());
        double force_jx_att = 1/distToGoal * (m_simulator->GetGoalCenterX() - rjw_x); 
        double force_jy_att = 1/distToGoal * (m_simulator->GetGoalCenterY() - rjw_y);
        //double force_jx_att = (m_simulator->GetGoalCenterX() - rjw_x); 
        //double force_jy_att = (m_simulator->GetGoalCenterY() - rjw_y);
        //ToUnitVector(&force_jx_att, &force_jy_att);
        double force_jx_rep = 0;
        double force_jy_rep = 0;

        for (int i=0; i<m_simulator->GetNrObstacles(); ++i)
        {
            Point obstacle =
                m_simulator->ClosestPointOnObstacle(i, rjw_x, rjw_y);
            double obstacleDist = GetDistBetweenPoints(rjw_x, rjw_y,
                obstacle.m_x, obstacle.m_y);

            if (obstacleDist < 5)
            {
                force_jx_rep += 1/obstacleDist * (rjw_x - obstacle.m_x);
                force_jy_rep += 1/obstacleDist * (rjw_y - obstacle.m_y);
            }
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

    ToUnitVector(&move.m_dx, &move.m_dy);
    double alpha = 0.05;
    double epsilon = 0.0005;

    if (move.m_dtheta > 0)
    {
        move.m_dtheta = epsilon;
    }
    else
    {
        move.m_dtheta = -epsilon;
    }

    move.m_dx *= alpha;
    move.m_dy *= alpha;

    return move;
}


