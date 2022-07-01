//
// Created by carol on 12/17/18.
//

#ifndef MAPD_AGENT_H
#define MAPD_AGENT_H

#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <functional>  // for std::hash (c++11 and above)
#include <map>
#include <algorithm>
#include <queue>
#include <limits.h>
#include <iostream>
#include <vector>
#include <list>
#include <utility>
#include <boost/heap/fibonacci_heap.hpp>
#include "LLNode.h"
#include "egraph_reader.h"
#include "map_loader.h"
//#include "Node.h"
#include "Endpoint.h"
class Task;
class Token;


using namespace std;


class Agent {
public:

    typedef dense_hash_map<LLNode*, LLNode*, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;

    int loc, park_loc, goal_loc;
    int id;
    int row, col;

    int finish_time;
    vector<int> path;
    vector<int> cost;
    vector<int> list_task_completed;
    vector<int> ongoing_tasks;
    vector<pair<int, bool>> path_principal;

    bool hold;
    pair<int,int> holding;

    bool only_dummy;
    int start_time;
    vector<int> non_dummy_path;
    bool debug;
    Endpoint *ep_park_loc;
    Endpoint* next_ep;
    bool delivering;
    Task *task;
    int min_cost;
    int start_task;
    double timepathPlan;
    int release_time;
    long num_conflito;

    int AStar_sCol(int start_loc, Endpoint *goal, int begin_time, Token token, bool evaluate, bool coleta);

    void setAgent(int loc, int col, int row, int id);
    bool TP( Token &token);
    Task* bestTask(Token token);

    int AStar(int start_loc, Endpoint *goal, int begin_time, Token token, bool hold, bool coleta);
    bool isConstrained(int curr_id, int next_id, int next_timestep, Token token, int ag_hide);
    void updatePath(Node *goal);
    void releaseClosedListNodes(map<unsigned int, Node*> &allNodes_table);
    bool Move2EP(Token &token, bool constraint);
    int fastEvaluateTP(Token &token);
    int planPath(int start_loc, Endpoint *goal, int begin_time, Token &token);
    int planPathPark(int start_loc, Endpoint *goal, int begin_time, Token &token);


    int planFastPath(int start_loc, int begin_time, Token &token);

    int AStar_certo(int start_loc, Endpoint *goal, int begin_time, Token token, bool evaluate, bool coleta, Endpoint *next_goal);

    void releaseClosedListLLNodes(hashtable_t *allNodes_table);

    void updatePath(LLNode *goal);

    int planPath2EP(int start_loc, Endpoint *goal, int begin_time, Token &token);
};


class Task{
public:

    int id;
  //  int state; //0: livre | 1:com agente;
    Agent *agent{};

    int seq_id{}; // TSP sequences id

    int agent_id{};
    int release_time;//tempo que a tarefa chegou no sistema;
    int start_time;
    int goal_time;
    int dist;
    int atraso_goal;

    int ag_arrive_start;
    int ag_arrive_goal;

    Endpoint *start;
    Endpoint *goal;

    bool delivering;


    Task() {};
    Task(int id, int t, Endpoint *s, Endpoint *g, int s_t, int g_t, int dist): id(id), release_time(t), start(s), goal(g), start_time(s_t), goal_time(g_t), dist(dist), ag_arrive_goal(0), ag_arrive_start(0), atraso_goal(0), delivering(false) {}
};


class PathPlanning{
public:
    typedef dense_hash_map<LLNode*, LLNode*, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;

    int timestep;
    int seed;

    string problem_name;

    vector<bool> map;
    vector<bool> endpoints;
    vector<Agent*> agents;
    int col;
    long num_collision;
    vector<vector<pair<int, bool>>>path_agents;

    vector<Task*> taskset;

    PathPlanning(){};

    int Agent_AStar(Agent *ag, int start_loc, Endpoint *goal, int begin_time, bool hold, Endpoint *next_goal,
                    Token &token);

    int Agent_AStar1(Agent *ag, int start_loc, Endpoint *goal, int begin_time, bool hold, Endpoint *next_goal);

    void updatePathAgent(LLNode *goal, Agent *ag);
    bool isConstrained(int curr_loc, int next_loc, int next_timestep, int id);
    void releaseClosedLLNodes(hashtable_t* allNodes_table);

    void updatePathAgent(Node *goal, Agent *ag);


    void releaseClosedListNodes(std::map<unsigned int, Node *> &allNodes_table);

    bool Move2EP(Agent *agent, int begin_time, bool constraint, Token &token);

    virtual ~PathPlanning();
};

class Token{
public:
    int timestep;

    vector<bool> map;
    vector<bool> endpoints;
    vector<Agent*> agents;

    vector<vector<int>> path; //path[agent][time] = loc (id)

    vector<Task*> taskset;
    int final_timestep; //o ultimo tempo que o agente cchega na park location

    Token(){timestep = 0;}


};

#endif //MAPD_AGENT_H
