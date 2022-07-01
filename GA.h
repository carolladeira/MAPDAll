
#ifndef MAPDAG_GA_H
#define MAPDAG_GA_H

#include "Agent.h"
#include <iostream>
#include "dlib/optimization/max_cost_assignment.h"


#define sizePopulation_TA           20
#define sizePopulation_E            20
//#define sizePopulation              20

#define CROSSOVER_RATE            0.8
#define MUTATION_RATE             0.5
#define TOURNAMENT                0.01
#define ELITISM                   0.04
#define MUTATION_AGENT            0.3
#define GENERATIONS               50
#define TIMEOUT                   180


class Gene{
public:
    int agent;
    int task;
    int order;

    Gene(int ag, int t, int o):  agent(ag), task(t), order(o) {}

    Gene() {}
};
class Chromosome{
public:
    vector<float> fitness;
    vector<Gene> Individual;
    double distance;
    int id;
    int rank;

    Chromosome(vector<Gene> Individual): Individual(Individual) {}
    Chromosome () {};
};

class GA {
public:
    int sizeTaskset;
    int sizeAgents;
    int sizePopulation;
    int fitness_makespan;
    float fitness_st;
    int seed;
    int timestep;

    vector<Agent>copy_agents;
    vector<Agent>agentes;
    vector<Task *> taskset;

    vector<Chromosome> pop;
    vector<int>score;

    vector<vector<int> > Dis;

    vector<Task> task_total;

    float tempo_GA;
    int cont=0;

    void Crossover(vector<Gene> &offspring1, vector<Gene> &offspring2);
    void Crossover_PB(vector<Gene> &offspring1, vector<Gene> &offspring2);
    float getFitness(Chromosome &ind);

    //void createBestInd();

    void showPopulation();
    void showPopulation(vector<Chromosome> temp);
    void showIndividual(vector<Gene> ind);

    double distance(Chromosome individual1, Chromosome individual2);

    bool dominates(Chromosome ind1, Chromosome ind2);

    void crowdingDistance(vector<Chromosome> front);
    void crowdingDistancePop(vector<Chromosome> &newPop);

    vector<vector<Chromosome>> nonDominatedSort(vector<Chromosome> p);

    vector<vector<int>>
    run_GA(vector<Agent> agents, vector<Task *> taskset, vector<Task> task_total, vector<vector<int>> Dis, bool bestInd,
           int seed, int tipo, int timestep, int obj);

    int NSGA(bool bestInd, int objetivo);

    void Initialize(vector<Agent> agents, vector<Task *> taskset);

    int Tournament(int i);
    int getBestChromosomeMakespan();

    virtual void Mutation(vector<Gene> &individual);

    void run_NSGA();
    void createBestInd();

    int crowdingTournament();
    int Tournament();
    int getBestChromosomeServiceTime();
    static int getBestChromosomeMakespan(vector<Chromosome> popul);
    int escalonamento(vector<Gene> &individual);
    int findAgentCloser(int release_time, int loc_start, vector<Agent> agents);


    int run_GA_TA(bool bestInd);
    int steadyStateGA(bool bestInd);
    int run_GA_Elitista(bool bestInd);
    int run_GA_E(bool bestInd);

    void correlacao(int tipo, int obj);

    int getBestChromosomeTo0();
};

class GA_E: public GA{
public:
    int cont = 0;


    void Crossover(vector<Gene> &offspring1, vector<Gene> &offspring2);
    void Mutation(vector<Gene> &individual) override;

};



#endif //MAPDAG_GA_H
