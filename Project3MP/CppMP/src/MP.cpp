#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>
#include <iostream>

MotionPlanner::MotionPlanner(Simulator * const simulator)
{
    m_simulator = simulator;   

    std::vector<double> robot(2, 0);
    robot[0] = m_simulator->GetRobotCenterX();
    robot[1] = m_simulator->GetRobotCenterY();
    AddVertex(robot, -1);
    m_vidAtGoal = -1;
    m_totalSolveTime = 0;
    const double c_MIN_THRESHOLD = 2;
}

MotionPlanner::~MotionPlanner(void)
{
    //do not delete m_simulator  

    const int n = m_vertices.size();
    for(int i = 0; i < n; ++i)
	delete m_vertices[i];
}

// Method that returns the magnitude of the given vector
double GetVectorMagnitude(const std::vector<double> &v)
{
    double sumOfSquares = 0;
    for (int i=0; i<v.size(); ++i)
    {
        sumOfSquares += v[i]*v[i];
    }
    return sqrt(sumOfSquares);
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

double getDistanceBetweenPoints(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1,2));
}

int getRandomInteger(int min, int max)
{
    return min + (rand() % (max - min + 1));
}

void MotionPlanner::ExtendTree(int vid, const double sto[])
{
    double step = m_simulator->GetDistOneStep();
    // Get qNear state
    while(getDistanceBetweenPoints(sto[0], sto[1], m_vertices[vid]->m_state[0], m_vertices[vid]->m_state[1]) > 2)
    {

        std::vector<double> qNear(2,0);
        qNear[0] = m_vertices[vid]->m_state[0];
        qNear[1] = m_vertices[vid]->m_state[1];

        // Get step size vector in direction of sto
        std::vector<double> stateDir(2, 0);
        stateDir[0] = sto[0] - qNear[0];
        stateDir[1] = sto[1] - qNear[1];
        ToUnitVector(&stateDir);
        stateDir[0] *= step;
        stateDir[1] *= step;

        std::vector<double> qNew = qNear;
        qNew[0] += stateDir[0];
        qNew[1] += stateDir[1];

        m_simulator->SetRobotCenter(qNew[0], qNew[1]);

        // Only add qNew to tree if it's a valid state (no collision)
        if (m_simulator->IsValidState())
        {
            if (m_simulator->HasRobotReachedGoal())
            {
                AddVertex(qNew, vid, Vertex::TYPE_GOAL);
            }
            else
            {
                AddVertex(qNew, vid);
            }
            vid = vid + 1;
        }
        else
        {
            return;
        }
    }
}
bool isAlreadyAVertex(double s[], std::vector<Vertex *> vertices)
{
    for(int i = 0; i < vertices.size(); i++)
    {
        double vX = vertices[i] ->m_state[0];
        double vY = vertices[i] ->m_state[1];
        double distance = getDistanceBetweenPoints(s[0], s[1], vX, vY) ;
        
        if(distance < 2)
        {
            return true;
        }
    }
    return false;
}
void getRandomState(Simulator * const simulator, double s[], std::vector<Vertex *> vertices)
{
    int rand = getRandomInteger(1, 10);
    if(rand == getRandomInteger(1, 10)) //get around goal region 10%
    {
        double goalCenterX = simulator ->GetGoalCenterX();
        double goalCenterY = simulator ->GetGoalCenterY();
        double radius = simulator ->GetGoalRadius();
        
        do{
           s[0] = PseudoRandomUniformReal(goalCenterX - radius, goalCenterX + radius);
           s[1] = PseudoRandomUniformReal(goalCenterY - radius, goalCenterY + radius);
        }while(isAlreadyAVertex(s, vertices));
    }
    else //get random state 90%
    {
        do{
            simulator ->SampleState(s);
        }while(isAlreadyAVertex(s, vertices));
    }
}

void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);

    double sto[2];
    getRandomState(m_simulator, sto, m_vertices);
    
    int vid = getRandomInteger(0, m_vertices.size() - 1);
    ExtendTree(vid, sto);

    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);
 
    double sto[2];
    getRandomState(m_simulator, sto, m_vertices);

    int vid;
    double shortestDistance = -1;
    for(int i = 0; i < m_vertices.size(); i++)
    {
        double distance = getDistanceBetweenPoints(sto[0], sto[1],
                                                   m_vertices[i] ->m_state[0],
                                                   m_vertices[i] ->m_state[1]);
        if(shortestDistance == -1 || distance < shortestDistance)
        {
            vid = i;
            shortestDistance = distance;
        }
    }
    ExtendTree(vid, sto);
    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendEST(void)
{
    Clock clk;
    StartTime(&clk);

    //your code
    double sto[2];
    getRandomState(m_simulator, sto, m_vertices);
    
    std::vector<double> weights(m_vertices.size(), 0);
    double weightSum = 0;

    for(int i = 0; i < m_vertices.size(); i++)
    {
        int numChildren = m_vertices[i]->m_nchildren;
        weights[i] = 1.0 / (1.0 + numChildren);
        weightSum += weights[i];
    }

    //TODO: rand should include 0 and go just up to weightSum but not including
    double rand = PseudoRandomUniformReal(0, weightSum);
    int vid = -1;

    //UNCOMMENT ARROW SECTIONS TO SELECT RANDOM STARTING POINT IN FOR LOOP
    int start = getRandomInteger(0, m_vertices.size() - 1);   

    for (int i = 0; i < m_vertices.size(); i++)
    {
        double weight = weights[i];

        if (rand < weight)
        {
            vid = i;
            break;
        }
        rand -= weight;
    }

    if (vid == -1)                                             
    {
        for (int i = 0; i < start; i++)
        {
            double weight = weights[i];

            if (rand < weight)
            {
                vid = i;
                break;
            }
            rand -= weight;
        }
    }

    if (vid == -1)
    {
        vid == m_vertices.size() - 1;
    }
    ExtendTree(vid, sto);
//end code

    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendMyApproach(void)
{
    Clock clk;
    StartTime(&clk);
 
    const int c_MAX_NODES = 2;
//your code
    // going with balanced tree. Each node should have at most MAX_NODES
    double sto[2];
    getRandomState(m_simulator, sto, m_vertices);

    int vid;
    double shortestDistance = -1;
    for(int i = 0; i < m_vertices.size(); i++)
    {
        if(m_vertices[i] ->m_nchildren < c_MAX_NODES)
        {
            double distance = getDistanceBetweenPoints(sto[0], sto[1],
                                                       m_vertices[i] ->m_state[0],
                                                       m_vertices[i] ->m_state[1]);
            if(shortestDistance == -1 || distance < shortestDistance)
            {
                vid = i;
                shortestDistance = distance;
            }
        }
    }
    ExtendTree(vid, sto);
    
    
    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::AddVertex(const std::vector<double> &state, int parent,
                              int type)
{
    Vertex *vinit = new Vertex();

    vinit->m_parent   = parent;   
    vinit->m_state[0] = state[0];
    vinit->m_state[1] = state[1];
    vinit->m_type = type;
    vinit->m_nchildren= 0;    

    if(vinit->m_type == Vertex::TYPE_GOAL)
    {
	    m_vidAtGoal = m_vertices.size();
    }

    m_vertices.push_back(vinit); 

    if(vinit->m_parent >= 0)
    {
	    (++m_vertices[vinit->m_parent]->m_nchildren);
    }
}

void MotionPlanner::GetPathFromInitToGoal(std::vector<int> *path) const
{
    std::vector<int> rpath;
    
    rpath.clear();
    
    int i = m_vidAtGoal;
    do
    {
        rpath.push_back(i);
        i = m_vertices[i]->m_parent;	
    } 
    while(i >= 0);
    
    path->clear();

    for(int i = rpath.size() - 1; i >= 0; --i)
    {
	    path->push_back(rpath[i]);
    }
}
