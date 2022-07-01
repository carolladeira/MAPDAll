//
// Created by carol on 12/17/18.
//

#ifndef MAPD_SIMULATION_H
#define MAPD_SIMULATION_H

#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <cstring>
#include <cassert>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <climits>
#include "dlib/optimization/max_cost_assignment.h"
#include "ICBSSearch.h"
#include "egraph_reader.h"

#include "Agent.h"
#include "Endpoint.h"
#include "GA.h"
#include "map_loader.h"

using namespace std;

class Simulation {
public:
    int time;
    float computation_time;

    Token token;
    PathPlanning pp;

    int row, col;
    int maxtime;
    int num_agents;
    int num_endpoint;

    string problem_name;

    vector<bool> my_map;
    int timestep;

    double tempoTotalGA;
    double tempoTotalPathPlan;

    string tour_file;
    string out_file;
    string tsp_file;

    vector<queue<Task*> > TSP_seqs;
    vector<int> TSP_len;
    vector<int> TSP_agent;
    vector<vector<int> > Dis;
    vector<Task> tasks_total;

    int t_task, num_task, sizeTaskset;

    vector<vector<int>> list_agents;

    vector<Agent> agents_order;

    vector<Agent> agents;
    vector<Endpoint> endpoints;

    vector<vector<Task>> taskset;
    vector<Task*> list_taskset;
    vector<int>score;

    int freq, seed;

    uint64_t num_conflito_CBS = 0;
    uint64_t num_conflito_CBS_total = 0;
    uint64_t conflito_CSB_final = 0;

    long num_conflito_PP =0;

    int fitness_m;
    float fitness_st;

    void readTourFile(string tourFile);
    void BFS();

    Simulation(string map, string task, int frequency, int seed, string nome);

    void LoadMap(string arq);
    void LoadTask(string arq, float frequency);

    void run_AS(bool bestInd, int tipo_GA);
    void run_SCol(bool bestInd, int tipo_GA);
    void run_CBS(bool bestInd, int tipo_GA);
    void runNSGA();
    void run_CBS_heuristic();
    void run_AS_teste();
    void run_AS_heuristic(bool bestInd);
    void run_AS_Fast(bool bestInd, int tipo_GA);
    void run_HungarianMethod();
    int nearestAgent();

    bool PathFinding(vector<Agent*> &ags, const vector<vector<int> > &cons_paths);

    vector<int> CalcCost(Agent* ag);

    void printPaths();
    void printMap();
    void printTasks();
    void printList_taskset();

    int getShortTime();

    void run_TP();

    void showTask(string arq, float t, int c, string tam);
    void showPathAgents(const string& arq);

    ///save
    void SavePath();
    void SaveTask(string file, string arq);

    int mkspn() const;
    float service_time();
    long tempoGA() { return this->tempoTotalGA; }
    double tempoPathPlan()  { return this->tempoTotalPathPlan; }

    bool sort_assignment(Chromosome i, Chromosome j) { return (i.fitness < j.fitness); }

    void deleteTask_id(int id);
    vector<vector<int>> heuristic(vector<Agent> copy_agents, vector<Task*> taskset);

    ///Para eu conferir
    bool testCollision();
    bool TestConstraints();

    int SingleFindPath(Agent *ag, int t);


    void run_AS_Hold(bool bestInd, int tipo_GA, int objetivo);

    int planPathAStar(Agent *ag, int begin_time, Endpoint *goal);
    int planPathAStar1(Agent *ag, int begin_time, Endpoint *goal);

    void run_AS2EP(bool bestInd, int tipo_GA);

    bool check_reachable(int id, int loc_finish, int tempo);
};


#endif //MAPD_SIMULATION_H
