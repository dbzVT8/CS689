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

void MotionPlanner::ExtendTree(const int vid, const double sto[])
{
    // Get qNear state
    std::vector<double> qNear(2,0);
    qNear[0] = m_vertices[vid]->m_state[0];
    qNear[1] = m_vertices[vid]->m_state[1];
    std::cout << "qNear: (" << qNear[0] << "," << qNear[1] << ")  -->  ";

    // Get step size vector in direction of sto
    double step = m_simulator->GetDistOneStep();
    std::vector<double> stateDir(2, 0);
    stateDir[0] = sto[0] - qNear[0];
    stateDir[1] = sto[1] - qNear[1];
    ToUnitVector(&stateDir);
    stateDir[0] *= step;
    stateDir[1] *= step;

    bool done = false;
    std::vector<double> qNew = qNear;
    double distToSto = getDistanceBetweenPoints(qNew[0], qNew[1],
                                                sto[0], sto[1]); 
    int counter = 0;

    while (!done && distToSto > step)
    {
        qNew[0] += stateDir[0];
        qNew[1] += stateDir[1];
        distToSto = getDistanceBetweenPoints(qNew[0], qNew[1],
                                             sto[0], sto[1]);

        // Only add qNew to tree if it's a valid state
        m_simulator->SetRobotCenter(qNew[0], qNew[1]);

        if (m_simulator->IsValidState())
        {
            // If robot reached the goal, add the vertex and stop. Otherwise,
            // just add the vertex and keep going
            if (m_simulator->HasRobotReachedGoal())
            {
                counter++;
                AddVertex(qNew, vid, Vertex::TYPE_GOAL);
                done = true;
                std::cout << "Reached goal, ";
            }
            else
            {
                counter++;
                AddVertex(qNew, vid);
            }
        }
        else
        {
            // Reached an invalid state, stop
            done = true;
            std::cout << "Invalid state, ";
        }
    }

    if (!done)
    {
        std::cout << "Reached STO, ";
    }

    std::cout << "added " << counter << " vertices." << std::endl;
}

void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);

//your code

    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);
 
//your code
    double sto[2];
    m_simulator ->SampleState(sto); //this isn't right. Need to re-write to be the way the teacher wants
    int vid;
    double shortestDistance = -1;
    for(int i = 0; i < m_vertices.size(); i++)
    {
        double distance = getDistanceBetweenPoints(sto[0], sto[1], m_vertices[i] ->m_state[0], m_vertices[i] ->m_state[1]); 
        if(shortestDistance == -1 || distance < shortestDistance)
        {
            vid = i;
            shortestDistance = distance;
        }
    }
    ExtendTree(vid, sto);
//end code
    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendEST(void)
{
    Clock clk;
    StartTime(&clk);

    //your code
    double sto[2];
    m_simulator ->SampleState(sto); //this isn't right. Need to re-write to be the way the teacher wants
    int vid;
    std::vector<int> tempVertices;
    for(int i = 0; i < m_vertices.size(); i++)
    {
       Vertex *v = m_vertices[i];
       for(int j = 0; j < v ->m_nchildren; j++)
       {
           tempVertices.push_back(i);
       }
    }
    vid = tempVertices[getRandomInteger(0, tempVertices.size())];
    ExtendTree(vid, sto);
//end code

    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendMyApproach(void)
{
    Clock clk;
    StartTime(&clk);
 
//your code
    
    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::AddVertex(const std::vector<double> &state, int type,
                              int parent)
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
