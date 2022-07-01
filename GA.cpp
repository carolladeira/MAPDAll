//
// Created by carol on 1/21/19.
//

#include <random>
#include <cfloat>
#include "GA.h"

//#define IMPRIME
bool myfunction(Gene i, Gene j) { return (i.task < j.task); }

bool sort_fitness0(Chromosome i, Chromosome j) { return (i.fitness[0] < j.fitness[0]); }

bool sort_fitness1(Chromosome i, Chromosome j) { return (i.fitness[1] < j.fitness[1]); }

bool sort_Distance(Chromosome i, Chromosome j) { return (i.distance > j.distance); }

void GA::Initialize(vector<Agent> agents, vector<Task *> taskset) {
    this->copy_agents = agents;
    sizeTaskset = taskset.size();
    sizeAgents = agents.size();
    int j = 0;
    vector<int> t;

//    int x = random();
//    cout<<x<<endl;
    for (int i = 0; i < sizePopulation; i++) {
        t.clear();

        for (int k = 0; k < sizeTaskset; k++) {
            int p = taskset[k]->id;
            t.push_back(p);

        }
        std::random_shuffle(begin(t), end(t));

        vector<Gene> genes;
        for (auto it = taskset.begin(); it != taskset.end(); it++) {
            int id_agent = rand() % agents.size();
            Gene ind = Gene(id_agent, t[j], j);
            j++;

            genes.push_back(ind);
        }
        j = 0;
        Chromosome c = Chromosome(genes);
        c.fitness.resize(2);
        c.id = i;
        pop.push_back(c);
    }

}

int GA::Tournament(int i) {
    float k = TOURNAMENT; //5% da populacao
    int size = (int) ceil(sizePopulation * k);
    int ind;
    int best = rand() % sizePopulation;
    if (i != -1) {
        best = i;
    }
    for (int it = 0; it < size -1; it++) {
        ind = rand() % sizePopulation;
        if (pop[ind].fitness[0] < pop[best].fitness[0]) {
            best = ind;
        }
    }
    return best;
}

int GA::Tournament() {
    float k = TOURNAMENT;
    int size = (int) ceil(sizePopulation * k);
    int ind, best = rand() % sizePopulation;
    for (int i = 0; i < size; i++) {
        ind = rand() % sizePopulation;
        if (pop[ind].fitness[0] < pop[best].fitness[0]) {
            best = ind;
        }
    }
    return best;
}

void GA::showPopulation() {

    for (int i = 0; i < pop.size(); i++) {
        cout << endl << "Cromossomo " << i << " -> ";
        for (auto &j : pop[i].Individual) {
           // cout << j.task << "\t";
           // cout << j.agent << "\t";

        }
        cout << " " << pop[i].fitness[0] << " " << pop[i].fitness[1];

    }
    cout << endl;
}

void GA::showPopulation(vector<Chromosome> p) {

    for (auto &i : p) {
        // cout << endl << "Cromossomo " << i << " -> ";
        for (int j = 0; j < i.Individual.size(); j++) {
            //  cout <<pop[i].Individual[j].task<<"\t";
            // cout <<pop[i].Individual[j].agent<<"\t";

        }
        cout << " " << i.fitness[0] << " " << i.fitness[1];

    }
    cout << endl;
}

void GA::showIndividual(vector<Gene> ind) {
    cout << endl;

    for (auto &i : ind) {
        cout << i.task << "\t";

    }
    cout << "\t |";
    for (auto &i : ind) {
        cout << i.agent << "\t";


    }
    cout << endl;
}

int GA::getBestChromosomeMakespan() {

    int best_ind = 0;
    int best_score = -1;

    for (int i = 0; i < sizePopulation; i++) {
        if (best_score == -1) {
            best_score = pop[i].fitness[0];
        }
        if (pop[i].fitness[0] < best_score) {
            best_ind = i;
            best_score = pop[i].fitness[0];
        }
    }
    return best_ind;
}

int GA::getBestChromosomeMakespan(vector<Chromosome> popul) {

    int best_ind = 0;
    int best_score = -1;

    for (int i = 0; i < popul.size(); i++) {
        if (best_score == -1) {
            best_score = popul[i].fitness[0];
        }
        if (popul[i].fitness[0] < best_score) {
            best_ind = i;
            best_score = popul[i].fitness[0];
        }
    }
    return best_ind;
}

int GA::getBestChromosomeServiceTime() {

    int best_ind = 0;
    int best_score = -1;

    for (int i = 0; i < sizePopulation; i++) {
        if (best_score == -1) {
            best_score = pop[i].fitness[1];
        }
        if (pop[i].fitness[1] < best_score) {
            best_ind = i;
            best_score = pop[i].fitness[1];
        }
    }
    return best_ind;
}

int GA::getBestChromosomeTo0() {

    int best_ind = 0;
    pair<int, int> best_score;
    best_score.first = pop[0].fitness[0];
    best_score.second = pop[0].fitness[1];
    double dist = 100000.0;
    double dist_at;
    /// formula -> x = (x - xmin)/(xmax - xmin)

    float xmin, xmax, ymin, ymax;
    std::sort(pop.begin(), pop.end(), sort_fitness0);
    xmin = pop[0].fitness[0];
    xmax = pop[sizePopulation-1].fitness[0];
    std::sort(pop.begin(), pop.end(), sort_fitness1);

    ymin = pop[0].fitness[1];
    ymax = pop[sizePopulation-1].fitness[1];
    for (int i = 0; i < sizePopulation; i++) {
        float x = (pop[i].fitness[0] - xmin)/(xmax - xmin);
        float y = (pop[i].fitness[1] - ymin)/(ymax - ymin);
//        cout << endl << "Cromossomo " << i << " -> ";
//        cout << " " << pop[i].fitness[0] << " " << pop[i].fitness[1];
//        cout << "  - " << x << " " << y;
        dist_at = pow((x), 2) +
                  pow((y), 2);
        dist_at = sqrt(dist_at);
        if (dist_at < dist) {
            best_ind = i;
            dist = dist_at;
        }
    }
    return best_ind;
}

void GA::Mutation(vector<Gene> &individual) {
    double prob = ((double) rand() / ((double) RAND_MAX + 1));

    if (prob < MUTATION_RATE) {
        int pos1 = rand() % individual.size();
        int pos2 = rand() % individual.size();

        int task = individual[pos1].task;
        individual[pos1].task = individual[pos2].task;
        individual[pos2].task = task;
        //   cout<<endl<<"\t Mutacao de tarefa --------> Pos1: "<<pos1<<" Pos2: "<<pos2<<" Task1: "<<individual[pos1].task<<" Task2: "<<individual[pos2].task<<" ";

    } else {

        int pos1 = rand() % individual.size();
//        int ag = rand() % sizeAgents; ///aleatorio
//       // cout<<endl<<"\t Mutacao de agente --------> Pos1: "<<pos1<<" antes: "<<individual[pos1].agent<<" agora: "<<ag<<" ";
//        individual[pos1].agent = ag;

 //       FAZENDO POR VIZINHOS
        unsigned seed = rand();
        std::default_random_engine e(seed);
        int viz = (MUTATION_AGENT * sizeAgents);
        std::uniform_int_distribution<int> dis(-viz, viz);
        int myNumber = 0;
        while (myNumber == 0) myNumber = dis(e);
        //  int t  = rand() %  viz;           //entre os vizinhos de agente
        int ag = individual[pos1].agent;
        int id = ag + myNumber;
       // cout<<endl<<"\t Mutacao de agente --------> Pos1: "<<pos1<<" antes: "<<individual[pos1].agent<<" vizinho: "<<myNumber<<" "<<" id: "<<id;
        if (id < 0) {
            int id_agent = sizeAgents + id;
            individual[pos1].agent = copy_agents[id_agent].id;
        } else{
            int id_agent = id % sizeAgents;
            individual[pos1].agent = copy_agents[id_agent].id;
        }
      //  cout<<" final: "<<individual[pos1].agent;
    }

}

void GA_E::Mutation(vector<Gene> &individual) {
    double prob = ((double) rand() / ((double) RAND_MAX + 1));

    if (prob < MUTATION_RATE) {
        int pos1 = rand() % individual.size();
        int pos2 = rand() % individual.size();

        int task = individual[pos1].task;
        individual[pos1].task = individual[pos2].task;
        individual[pos2].task = task;
        //   cout<<endl<<"\t Mutacao de tarefa --------> Pos1: "<<pos1<<" Pos2: "<<pos2<<" Task1: "<<individual[pos1].task<<" Task2: "<<individual[pos2].task<<" ";

    }
}

void GA::Crossover(vector<Gene> &offspring1, vector<Gene> &offspring2) {

    double prob = ((double) rand() / ((double) RAND_MAX + 1));
    if (prob < CROSSOVER_RATE) {
        //    cout<<endl<<" -----------CROSSOVER-------- "<<" ";
//
//        showIndividual(offspring1);
//        showIndividual(offspring2);

        vector<Gene> temp;
        temp.resize(sizeTaskset);
        int p = rand() % sizeTaskset;
        temp = offspring1;

        int number1 = rand() % sizeTaskset;
        int number2 = rand() % sizeTaskset;

        int start = fmin(number1, number2);
        int end = fmax(number1, number2);

        std::vector<int> child1;
        std::vector<int> child2;
        // cout<<start<<" "<<end<<endl;

        for (int i = start; i < end; i++) {
            child1.push_back(offspring1[i].task);
            child2.push_back(offspring2[i].task);
        }

        int geneIndex = 0;
        int geneInparent1 = 0;
        int geneInparent2 = 0;

        for (int i = 0; i < sizeTaskset; i++) {
            geneIndex = (end + i) % sizeTaskset;
            geneInparent1 = offspring1[geneIndex].task;
            geneInparent2 = offspring2[geneIndex].task;

            bool is_there = false;
            for (int i1 : child1) {
                if (i1 == geneInparent2) {
                    is_there = true;
                }
            }
            if (!is_there) {
                child1.push_back(geneInparent2);
            }

            bool is_there1 = false;
            for (int i1 : child2) {
                if (i1 == geneInparent1) {
                    is_there1 = true;
                }
            }
            if (!is_there1) {
                child2.push_back(geneInparent1);
            }
        }

        std::rotate(child1.rbegin(), child1.rbegin() + start, child1.rend());
        std::rotate(child2.rbegin(), child2.rbegin() + start, child2.rend());

        for (int i = 0; i < start; i++) {
            offspring1[i].agent = offspring2[i].agent;
            offspring2[i].agent = temp[i].agent;
        }

        for (int i = 0; i < sizeTaskset; i++) {
            offspring1[i].task = child2[i];
            offspring2[i].task = child1[i];
        }

//        showIndividual(offspring1);
//        showIndividual(offspring2);

    }
}

void GA::Crossover_PB(vector<Gene> &offspring1, vector<Gene> &offspring2) {

    double prob = ((double) rand() / ((double) RAND_MAX + 1));
    if (prob < CROSSOVER_RATE) {
        //          cout<<endl<<" -----------CROSSOVER-------- "<<" ";

        //    showIndividual(offspring1);
        //  showIndividual(offspring2);

        vector<Gene> temp;
        temp.resize(sizeTaskset);
        temp = offspring1;
        int start = rand() % sizeTaskset;


        std::vector<int> child1;
        std::vector<int> child2;
        // cout<<start<<" "<<end<<endl;

        for (int i = 0; i < sizeTaskset; i++) {
            child1.push_back(-1);
            child2.push_back(-1);
            int number1 = rand() % 2;
            if (number1 == 1) child1[i] = offspring1[i].task;
            int number2 = rand() % 2;
            if (number2 == 1) child2[i] = offspring2[i].task;;

        }

        int geneIndex = 0;
        int geneInparent1 = 0;
        int geneInparent2 = 0;
        std::list<int> temp1;
        std::list<int> temp2;
        for (int i = 0; i < sizeTaskset; i++) {
            geneIndex = i;
            geneInparent1 = offspring1[geneIndex].task;
            geneInparent2 = offspring2[geneIndex].task;

            bool is_there = false;
            for (int i1 = 0; i1 < child1.size(); i1++) {
                if (child1[i1] == geneInparent2) {
                    is_there = true;
                }
            }
            if (!is_there) {
                temp1.push_back(geneInparent2);
            }

            bool is_there1 = false;
            for (int i1 = 0; i1 < child2.size(); i1++) {
                if (child2[i1] == geneInparent1) {
                    is_there1 = true;
                }
            }
            if (!is_there1) {
                temp2.push_back(geneInparent1);
            }
        }
        ///  std::rotate(temp1.rbegin(), temp1.rbegin() + temp1.size() -1, temp1.rend());
        //  std::rotate(child1.rbegin(), child1.rbegin() + start, child1.rend());

        for (int i = 0; i < sizeTaskset; i++) {
            if (child1[i] == -1) {
                child1[i] = temp1.front();
                temp1.pop_front();
            }
            if (child2[i] == -1) {
                child2[i] = temp2.front();
                temp2.pop_front();
            }

        }


        for (int i = 0; i < start; i++) {
            offspring1[i].agent = offspring2[i].agent;
            offspring2[i].agent = temp[i].agent;
        }

        for (int i = 0; i < sizeTaskset; i++) {
            offspring1[i].task = child2[i];
            offspring2[i].task = child1[i];
        }

        //     showIndividual(offspring1);
        //   showIndividual(offspring2);

    }
}

void GA_E::Crossover(vector<Gene> &offspring1, vector<Gene> &offspring2) {

    double prob = ((double) rand() / ((double) RAND_MAX + 1));
    if (prob < CROSSOVER_RATE) {
        //cout<<endl<<" -----------CROSSOVER-------- "<<" ";
        //showIndividual(offspring1);
        //showIndividual(offspring2);

        vector<Gene> temp;
        temp.resize(sizeTaskset);
        int p = rand() % sizeTaskset;
        temp = offspring1;

        int number1 = rand() % sizeTaskset;
        int number2 = rand() % sizeTaskset;

        int start = fmin(number1, number2);
        int end = fmax(number1, number2);

        std::vector<int> child1;
        std::vector<int> child2;
        //cout<<start<<" "<<end<<endl;

        for (int i = start; i < end; i++) {
            child1.push_back(offspring1[i].task);
            child2.push_back(offspring2[i].task);
        }

        int geneIndex = 0;
        int geneInparent1 = 0;
        int geneInparent2 = 0;

        for (int i = 0; i < sizeTaskset; i++) {
            geneIndex = (end + i) % sizeTaskset;
            geneInparent1 = offspring1[geneIndex].task;
            geneInparent2 = offspring2[geneIndex].task;

            bool is_there = false;
            for (int i1 = 0; i1 < child1.size(); i1++) {
                if (child1[i1] == geneInparent2) {
                    is_there = true;
                }
            }
            if (!is_there) {
                child1.push_back(geneInparent2);
            }

            bool is_there1 = false;
            for (int i1 = 0; i1 < child2.size(); i1++) {
                if (child2[i1] == geneInparent1) {
                    is_there1 = true;
                }
            }
            if (!is_there1) {
                child2.push_back(geneInparent1);
            }
        }

        std::rotate(child1.rbegin(), child1.rbegin() + start, child1.rend());
        std::rotate(child2.rbegin(), child2.rbegin() + start, child2.rend());

        for (int i = 0; i < sizeTaskset; i++) {
            offspring1[i].task = child2[i];
            offspring2[i].task = child1[i];
        }

        //   showIndividual(offspring1);
        //  showIndividual(offspring2);

    }
}

float GA::getFitness(Chromosome &ind) {

    vector<vector<int>> list_task;
    list_task.resize(sizeAgents);
    vector<Gene> individual = ind.Individual;
    int qtd_task = 0;
    for (auto &i : individual) {
        int n_task = i.task;
        int n_agent = i.agent;
        list_task[n_agent].push_back(n_task);
        qtd_task++;
    }

    Task n;
    int start = 0, makespan = 0, soc = 0;
    double st = 0.0;
    for (int i = 0; i < list_task.size(); i++) {
        //   cout<<endl<<"Agente "<<i;
        start = 0;
        int loc = copy_agents[i].loc;
        int time = copy_agents[i].finish_time;
        for (int j = 0; j < list_task[i].size(); j++) {
            n = task_total[list_task[i][j]];
             //  cout<<endl<<" pega tarefa "<<n.id <<" start "<<n.start->loc<<" goal "<<n.goal->loc<<endl;
            if (j == 0) {
                //int release_time = n.release_time;
                int dist = n.start->h_val[loc];
                start = time + dist;
                loc = n.start->loc;
              //  cout<<" vai pra start "<<loc<<" no tempo "<<start;
                if (j + 1 != list_task[i].size()) continue;
            } else if (j <= list_task[i].size() - 1) {
                Task n1 = task_total[list_task[i][j - 1]];
                Task n2 = task_total[list_task[i][j]];
                int release_time = n2.release_time;
                int dist_t1 = n1.goal->h_val[loc];
                st += dist_t1 + start - n1.release_time;
                loc = n1.goal->loc;
              //   cout<<" vai pra goal "<<loc<<" no tempo "<<start + dist_t1;
                int dist_t2 = n2.start->h_val[loc];
                loc = n2.start->loc;
                start = start + dist_t1 + dist_t2;
               //   cout<<" vai pra start "<<loc<<" no tempo "<<start;
                // start = max(start + dist_t1 + dist_t2, release_time);
                if (j + 1 != list_task[i].size()) continue;

            }
            int dist_t1 = n.goal->h_val[loc];
            start = start + dist_t1;
            st += start - n.release_time;
            //  cout<<" vai de "<<loc<<" pra goal "<<n.goal->loc<<" no tempo "<<start<<endl;
            // makespan = start;
        }
        //   cout<<endl<<"Agente "<<i<<" st "<<st;
        soc = start + soc;
        if (start > makespan) makespan = start;

    }
   // cout<<" st:  "<<st<<" qtd_task:  "<< qtd_task<<endl;

    st = st / qtd_task;
   // cout<<endl<<" makespan: "<<makespan ;
   // cout<<" st: "<< st ;

    ind.fitness[0] = makespan;
    ind.fitness[1] = st;
    return makespan;
}

int GA::escalonamento(vector<Gene> &individual) {
    vector<Agent> agents = copy_agents;
    Task n;
    int id_agent;
    int qtd_task = 0;
    float st =0;
    for (auto &i : individual) {
        int id_task = i.task;
        qtd_task++;
        n = task_total[id_task];
        int release_time = n.release_time;
        int loc_start_task = n.start->loc;
        int loc_goal_task = n.goal->loc;
        id_agent = findAgentCloser(release_time, loc_start_task, agents);
        i.agent = id_agent;
        int ft = Dis[agents[id_agent].loc][loc_start_task] + agents[id_agent].finish_time;
        // int ft =  max(Dis[agents[id_agent].loc][loc_start_task] + agents[id_agent].finish_time, release_time);
        // cout<<endl<<"Agente "<<id_agent<<" pega tarefa "<<id_task <<" Rt: "<<release_time<<" loc: "<<agents[id_agent].loc<<" -> "<<loc_start_task<<"[";
        // cout<<ft<<"] -> "<<loc_goal_task<<"["<<Dis[loc_start_task][loc_goal_task]<<"] = "<< ft+ Dis[loc_start_task][loc_goal_task];
        agents[id_agent].finish_time = ft + Dis[loc_start_task][loc_goal_task];
        agents[id_agent].loc = loc_goal_task;
        st += agents[id_agent].finish_time - release_time;
    }
    int fitness = 0;
    Agent ag;

    for (auto &agent : agents) {
        ag = agent;
        int finish_time = ag.finish_time;
        if (finish_time > fitness) {
            fitness = finish_time;
        }
    }
    float ser = st/qtd_task;
  //  cout<<"ft m: "<<fitness ;
   // cout<<" ft st: "<<ser <<endl;
    return fitness;
}

int GA::findAgentCloser(int release_time, int loc_start, vector<Agent> agents) {
    Agent ag;
    int best_ag, best_t = 1000000;
    for (int i = 0; i < copy_agents.size(); i++) {
        ag = agents[i];
        int finish_time = ag.finish_time;
        int loc = ag.loc;
        int dist = Dis[loc][loc_start];
        // int t_ateTask = max(dist + finish_time, release_time);
        int t_ateTask = dist + finish_time;
        if (t_ateTask < best_t) {
            best_ag = ag.id;
            best_t = t_ateTask;
        }
    }
    return best_ag;
}

void GA::createBestInd() {
    int menor = INT_MAX;
    int id_agent;
    int id_task;
    int loc_i, loc_f, c = 0;
    vector<Task *> copy_taskset = taskset;
    vector<Gene> genes;
    vector<Agent> agents = copy_agents;

    while (c < sizeTaskset) {
        for (int i = 0; i < copy_taskset.size(); i++) {
            for (int j = 0; j < sizeAgents; j++) {
                loc_i = agents[j].loc;
                loc_f = copy_taskset[i]->start->loc;
                if (Dis[loc_i][loc_f] + agents[j].finish_time < menor) {
                    menor = Dis[loc_i][loc_f] + agents[j].finish_time;
                    id_agent = j;
                    id_task = i;
                }
            }
        }
        menor = INT_MAX;
        Gene ind;
        int id_tarefa = copy_taskset[id_task]->id;
        //  cout<<"Agente "<<id_agent<<" com tarefa "<<id;
        ind = Gene(id_agent, id_tarefa, c);
        genes.push_back(ind);
        c++;
        agents[id_agent].finish_time = Dis[agents[id_agent].loc][copy_taskset[id_task]->start->loc] +
                                       Dis[copy_taskset[id_task]->start->loc][copy_taskset[id_task]->goal->loc] +
                                       agents[id_agent].finish_time;
        agents[id_agent].loc = copy_taskset[id_task]->goal->loc;
        //  cout<<" tempo final "<<copy_agents[id_agent].finish_time<<endl;
        copy_taskset.erase(copy_taskset.begin() + id_task);
    }
    Chromosome novo = Chromosome(genes);
    novo.fitness.resize(2);
    novo.distance = 0.0;
    novo.id = pop.size();
    pop.push_back(novo);
}

double GA::distance(Chromosome individual1, Chromosome individual2) {
    double dist = 0.0;
    dist = pow((individual1.fitness[0] - individual2.fitness[0]), 2) +
           pow((individual1.fitness[1] - individual2.fitness[1]), 2);
    dist = sqrt(dist);
    return dist;
}

bool GA::dominates(Chromosome ind1, Chromosome ind2) {
    bool dominates = false;
    if ((ind1.fitness[0] <= ind2.fitness[0] && ind1.fitness[1] <= ind2.fitness[1]) &&
        (ind1.fitness[0] < ind2.fitness[0] || ind1.fitness[1] < ind2.fitness[1])) {
        return true;
    } else {
        return false;
    }
}

void GA::crowdingDistancePop(vector<Chromosome> &newPop) {

    //so tenho 2 objetivos
    for (int i = 0; i < 2; i++) {
        //   cout<<endl<<"Objetivo "<<i<<endl;
        if (i == 0) {
            std::sort(newPop.begin(), newPop.end(), sort_fitness0);
        }
        if (i == 1) {
            std::sort(newPop.begin(), newPop.end(), sort_fitness1);
        }
        newPop[0].distance = std::numeric_limits<double>::max();
        newPop[newPop.size() - 1].distance = DBL_MAX;

        for (int c = 1; c < newPop.size() - 1; c++) {
            if (i == 0) {
                //  const auto [min,max] = minmax_element(pop.begin()->fitness_m[0])
                double min = newPop[0].fitness[0];
                double max = newPop[newPop.size() - 1].fitness[0];
                //cout<<"c+1: "<<(newPop[c+1].fitness_m[i] - newPop[c-1].fitness_m[i])/(max-min)<<" -> "<<(newPop[c+1].fitness_m[i] - newPop[c-1].fitness_m[i])<<" / "<<(max-min)<<" + "<<newPop[c].distance<<endl;
                double valor =
                        newPop[c].distance + ((newPop[c + 1].fitness[i] - newPop[c - 1].fitness[i]) / (max - min));
                //  cout<<"c: "<<newPop[c].id<<" - valor: "<<valor<<endl;
                newPop[c].distance =
                        newPop[c].distance + ((newPop[c + 1].fitness[i] - newPop[c - 1].fitness[i]) / (max - min));
            }
            if (i == 1) {
                double min = newPop[0].fitness[1];
                double max = newPop[newPop.size() - 1].fitness[1];
                // cout<<"c+1: "<<(newPop[c+1].fitness_m[i] - newPop[c-1].fitness_m[i])/(max-min)<<" -> "<<(newPop[c+1].fitness_m[i] - newPop[c-1].fitness_m[i])<<" / "<<(max-min)<<" + "<<newPop[c].distance<<endl;
                double valor =
                        newPop[c].distance + ((newPop[c + 1].fitness[i] - newPop[c - 1].fitness[i]) / (max - min));
                // cout<<"c: "<<newPop[c].id<<" - valor: "<<valor<<endl;
                newPop[c].distance =
                        newPop[c].distance + ((newPop[c + 1].fitness[i] - newPop[c - 1].fitness[i]) / (max - min));
            }

        }
    }

}


void GA::crowdingDistance(vector<Chromosome> front) {

    //so tenho 2 objetivos
    for (int i = 0; i < 2; i++) {
        if (i == 0) {
            std::sort(front.begin(), front.end(), sort_fitness0);
        }
        if (i == 1) {
            std::sort(front.begin(), front.end(), sort_fitness1);
        }
        showPopulation();
        pop[0].distance = INT16_MAX;
        pop[pop.size() - 1].distance = INT16_MAX;

        for (int c = 1; c < pop.size() - 1; c++) {
            if (i == 0) {
                //  const auto [min,max] = minmax_element(pop.begin()->fitness_m[0])
                int min = pop[0].fitness[0];
                int max = pop[pop.size() - 1].fitness[0];
                pop[c].distance = pop[c].distance + ((pop[c + 1].fitness[i] - pop[c - 1].fitness[i]) / (max - min));
            }
            if (i == 1) {
                int min = pop[0].fitness[1];
                int max = pop[pop.size() - 1].fitness[1];
                pop[c].distance = pop[c].distance + ((pop[c + 1].fitness[i] - pop[c - 1].fitness[i]) / (max - min));
            }

        }
    }

}

vector<vector<Chromosome>> GA::nonDominatedSort(vector<Chromosome> p) {
    vector<int> n;
    vector<vector<Chromosome>> fronts;
    //  cout<<"--------------"<<endl;
    vector<vector<Chromosome *>> s;
    s.resize(p.size());
    fronts.resize(p.size());
    n.resize(p.size());
    for (int i = 0; i < p.size(); i++) {
        // cout<<"ID:  "<<p[i].id<<" : "<<endl;
        int id = p[i].id;
        n[id] = 0;
        for (int j = 0; j < p.size(); j++) {
            if (i != j) {
                //  cout<<"\t"<<p[j].fitness_m[0]<<" "<<p[j].fitness_m[1];
                if (dominates(p[i], p[j])) {
                    //cout<<" domina "<<p[j].id<<endl;
                    s[id].push_back(&p[j]);
                } else if (dominates(p[j], p[i])) {
                    //cout<<" Eh Dominado "<<p[j].id<<endl;
                    n[id] += 1;
                }
            }
        }
        if (n[id] == 0) {
            p[i].rank = 0;
            fronts[0].push_back(p[i]);
        }
    }
    int w = 0;
    while (fronts[w].size() != 0) {
        vector<Chromosome> nextFront;
        for (int i = 0; i < fronts[w].size(); i++) {
            //   cout<<"i"<<i<<endl;
            int id_q = fronts[w][i].id;
            for (int j = 0; j < s[id_q].size(); j++) {
                int id = s[id_q][j]->id;
                n[id] -= 1;
                if (n[id] == 0) {
                    s[id_q][j]->rank = w + 1;
                    nextFront.push_back(*s[id_q][j]);

                }
            }
        }
        w += 1;
        fronts[w] = nextFront;
    }

    return fronts;
}

int GA::crowdingTournament() {
    float k = TOURNAMENT; //5% da populacao
    int size = (int) ceil(sizePopulation * k);
    int ind, best = rand() % sizePopulation;
    for (int i = 0; i < size; i++) {
        ind = rand() % sizePopulation;
        if (pop[ind].rank < pop[best].rank) {
            best = ind;
        } else if (pop[ind].rank == pop[best].rank) {
            if (pop[ind].distance > pop[best].distance) {
                best = ind;

            }
        }
    }
    return best;
}

void GA::correlacao(int tipo, int obj) {

    this->sizePopulation = 1000;

    Initialize(agentes, taskset);

    for (int i = 0; i < sizePopulation; i++) {
        pop[i].fitness[0] = getFitness(pop[i]);
    }
    string nome = "";
    if (tipo == 0) {
        nome = "GA_TA";
    } else if (tipo == 1) {
        nome = "Elitist";
    } else if (tipo == 2) {
        nome = "steadyState";
    } else if (tipo == 3) {
        nome = "GA_E";
    } else if (tipo == 4) {
        if (obj == 0) {
            nome = "NSGA_M";
        } else if (obj == 1) {
            nome = "NSGA_ST";
        } else if (obj == 2) {
            nome = "NSGA_O";
        }
    }

    string res_file =
            "/home/carolina/MAPDAll/correlacao/corr-"+nome+"-1000-" + to_string(sizeAgents) + "-" + to_string(timestep) + ".csv";
    ofstream fout(res_file, ios::app);
    if (!fout) cout << "ERR0 - salvar aquivo correlacao"<<endl;
    for (int i = 0; i < pop.size(); i++) {
        fout << i << "," << pop[i].fitness[0] << "," << pop[i].fitness[1] << "," << seed << endl;
    }
    pop.clear();
    fout.close();
}

int GA::run_GA_E(bool bestInd) {
    //  cout<<"GA_E "<<endl;
    this->sizePopulation = sizePopulation_TA;


    Initialize(agentes, taskset);

    if (bestInd) {
        pop.pop_back();
        createBestInd();
    }

#ifdef IMPRIME
    string res_file = "/home/carol/Documents/result-ga-e.csv ";
    ofstream fout(res_file, ios::app);
    if (!fout)
        cout << "ERR0";
#endif

    for (int i = 0; i < sizePopulation; i++) {
        pop[i].fitness[0] = escalonamento(pop[i].Individual);

    }
//    for (int i = 0; i < sizePopulation; i++) {
//        pop[i].fitness[0] = getFitness(pop[i]);
//    }
    int geracoes = 0;
    while (geracoes < GENERATIONS) {
        int cPop = 0;
        vector<Chromosome> temp;
        temp = pop;
        vector<Chromosome> newPop;
        Chromosome off1;
        Chromosome off2;

        while (cPop < sizePopulation_E) {
            int offspring1 = Tournament();
            int offspring2 = Tournament();

            off1 = pop[offspring1];
            off2 = pop[offspring2];
            Crossover(off1.Individual, off2.Individual);
            Mutation(off1.Individual);
            Mutation(off2.Individual);
            cPop++;
            cPop++;
            newPop.push_back(off1);
            newPop.push_back(off2);
        }

        for (int i = 0; i < sizePopulation; i++) {
            newPop[i].fitness[0] = escalonamento(newPop[i].Individual);
        }

//        for (int i = 0; i < sizePopulation; i++) {
//            newPop[i].fitness[0] = getFitness(newPop[i]);
//        }

        std::sort(pop.begin(), pop.end(), sort_fitness0);
        std::sort(newPop.begin(), newPop.end(), sort_fitness0);


        int e = 0;
        for (int i = ceil(sizePopulation * ELITISM); i < sizePopulation; i++) {
            pop[i] = newPop[e];
            e++;
        }
        geracoes++;
#ifdef IMPRIME
        int id = getBestChromosomeMakespan();
        double best_fitness = pop[id].fitness_m[0];
        fout << cont << "," << i << "," << best_fitness << "," << GENERATIONS << "," << sizePopulation_E << ","
             << CROSSOVER_RATE << "," << MUTATION_RATE << "," << MUTATION_AGENT << "," << TOURNAMENT << "," << ELITISM
             << endl;
        //fout.close();
#endif
    }
    int id = getBestChromosomeMakespan();
    //cout <<"Duration: "<<duration<< " Fitness: " << pop[id].fitness_m <<" Generations:" <<i<<endl;
    this->fitness_makespan = pop[id].fitness[0];
    this->fitness_st = pop[id].fitness[1];
   // cout<<"st: "<<this->fitness_st<<endl;
    return id;
}

void GA::run_NSGA() {

    vector<vector<Chromosome>> fronts;
    // fronts = nonDominatedSort();
    //crowdingDistance();
    int P = sizePopulation;
    int g = 0;
    while (g < GENERATIONS) {
        int cPop = 0;
        vector<Chromosome> newPop;
        crowdingDistancePop(pop);
        while (cPop < P) {
            Chromosome off1;
            Chromosome off2;
            int offspring1 = crowdingTournament();
            int offspring2 = crowdingTournament();

            off1 = pop[offspring1];
            off2 = pop[offspring2];

            Crossover(off1.Individual, off2.Individual);
            Mutation(off1.Individual);
            Mutation(off2.Individual);
            off1.id = cPop + P;
            off1.distance = 0.0;

            cPop++;
            off2.id = cPop + P;
            off2.distance = 0.0;

            cPop++;
            newPop.push_back(off1);
            newPop.push_back(off2);
        }
        // cout<<"newPop "<<newPop.size()<<endl;

        for (auto &i : newPop) {
            i.fitness[0] = getFitness(i);
        }

        int e = 0;
        for (auto &i : pop) {
            newPop.push_back(i);
            e++;
        }
        for (int i = 0; i < newPop.size(); i++) {
            newPop[i].id = i;
            newPop[i].distance = 0.0;
        }
        crowdingDistancePop(newPop);
        fronts = nonDominatedSort(newPop);

//        string res_file =
//                "/home/carol/Desktop/correlacao/Fronts/fo-si-" + to_string(sizeAgents) + "-" + to_string(timestep) + "-"+to_string(g)+
//                ".csv";
//        ofstream fout(res_file, ios::app);
//        if (!fout) cout << "ERR0";
//        for (int i = 0; i < fronts.size(); i++) {
//            for (int j = 0; j < fronts[i].size(); j++) {
//                fout << fronts[i][j].fitness[0] << "," << fronts[i][j].fitness[1] << "," << i << endl;
//            }
//        }
//        for (int i = 0; i < newPop.size(); i++) {
//            fout << newPop[i].fitness[0] << "," << newPop[i].fitness[1] << "," << -1 << endl;
//        }
//        fout.close();



        pop.clear();

        //cout<<" New Pop "<< newPop.size()<<" Pop "<<pop.size()<<endl;
        for (int i = 0; pop.size() < P; i++) {
            //sort solutions from bu Crowding Distance
            std::sort(fronts[i].begin(), fronts[i].end(), sort_Distance);
            if (fronts[i].empty()) {
                break;
            }
            for (auto &j : fronts[i]) {
                if (pop.size() < P) {
                    pop.push_back(j);
                }
            }
        }
//        if(g == 0 | g == 15 | g == 30 | g ==49){
//            string res_file =
//                    "/home/carol/Desktop/eaf/frequencia 10/fo-" + to_string(sizeAgents) + "-" + to_string(timestep) + "-"+to_string(g)+
//                    ".csv";
//            ofstream fout(res_file, ios::app);
//            if (!fout) cout << "ERR0";
//            for (int i = 0; i < pop.size(); i++) {
//                fout << pop[i].fitness_m[0] << "," << pop[i].fitness_m[1] << "," << pop[i].rank << endl;
//            }
//            fout.close();
//        }

        g++;
    }


}

int GA::NSGA(bool bestInd, int objetivo) {

    this->sizePopulation = sizePopulation_TA;
    this->pop.clear();

    Initialize(agentes, taskset);
    if (bestInd) {
        pop.pop_back();
        createBestInd();
    }

    for (int i = 0; i < sizePopulation; i++) {
        pop[i].fitness[0] = getFitness(pop[i]);
    }

    vector<vector<Chromosome>> fronts;

    fronts = nonDominatedSort(pop);
    pop.clear();
    for (int i = 0; i < fronts.size(); i++) {
        for (int j = 0; j < fronts[i].size(); j++) {
            pop.push_back(fronts[i][j]);

        }
    }
    run_NSGA();
    int id =0;
    if(objetivo == 0){
        id = getBestChromosomeMakespan();
     //   cout<<"mks: "<<pop[id].fitness[0]<<endl;

    }else if (objetivo == 1){
        id = getBestChromosomeServiceTime();
       // cout<<"st: "<<pop[id].fitness[1]<<endl;

    }
    else if (objetivo == 2){
        id = getBestChromosomeTo0();
        // cout<<"st: "<<pop[id].fitness[1]<<endl;

    }
    this->fitness_makespan = pop[id].fitness[0];
    this->fitness_st = pop[id].fitness[1];
   // cout<<"st: "<<this->fitness_st<<endl;

    return id;
}

int GA::steadyStateGA(bool bestInd) {
    // cout<<"SteadyState "<<endl;

    this->sizePopulation = sizePopulation_TA;

    Initialize(agentes, taskset);
    if (bestInd) {
        pop.pop_back();
        createBestInd();
    }
    for (int i = 0; i < sizePopulation; i++) {
        pop[i].fitness[0] = getFitness(pop[i]);
    }

    int geracoes = 0;
    int contador =0;
    while (geracoes < 500) {
        int cPop = 0;
        vector<Chromosome> temp;
        temp = pop;
        vector<Chromosome> newPop;
        Chromosome off1;
        Chromosome off2;
        //cout<<endl<<" generations: "<<i<<endl;
        // showPopulation();

        int p = contador % sizePopulation;
        int hu = (p + 1) % sizePopulation;
        //cout<<p<<" "<<hu<<endl;
        contador = hu+1;
        // cout<<p<<" - "<<(p+1)%20<<endl;
        int offspring1 = Tournament(p);
        int offspring2 = Tournament(hu);

        off1 = pop[offspring1];
        off2 = pop[offspring2];

        Crossover(off1.Individual, off2.Individual);
        Mutation(off1.Individual);
        Mutation(off2.Individual);

        off1.fitness[0] = getFitness(off1);
        off2.fitness[0] = getFitness(off2);

//        int pior = 0;
//        int id;
//        for (int i = 0; i < pop.size(); i++) {
//            if (pop[i].fitness[0] > pior) {
//                pior = pop[i].fitness[0];
//                id = i;
//            }
//        }
//        pop[id] = off1;
//        pior = 0;
//        for (int i = 0; i < pop.size(); i++) {
//            if (pop[i].fitness[0] > pior) {
//                pior = pop[i].fitness[0];
//                id = i;
//            }
//        }
//        pop[id] = off2;

        newPop = pop;
        newPop.push_back(off1);
        newPop.push_back(off2);
        std::sort(newPop.begin(), newPop.end(), sort_fitness0);
        pop.clear();
        for (int i = 0; i < sizePopulation; i++) {
            pop.push_back(newPop[i]);
        }

#ifdef IMPRIME
        int id_melhor = getBestChromosomeMakespan();
        double best_fitness = pop[id_melhor].fitness_m[0];
        fout << seed << "," << timestep << "," << g << "," << best_fitness << "," << GENERATIONS << ","
             << sizePopulation_TA << "," << CROSSOVER_RATE << "," << MUTATION_RATE << "," << MUTATION_AGENT << ","
             << TOURNAMENT << endl;
#endif
        geracoes++;
    }

    int id = getBestChromosomeMakespan();
    this->fitness_makespan = pop[id].fitness[0];
    this->fitness_st = pop[id].fitness[1];
   // cout<<"st: "<<this->fitness_st<<endl;

#ifdef IMPRIME

    fout.close();
#endif
    return id;

}

int GA::run_GA_TA(bool bestInd) {
    //  cout<<"GA_TA "<<endl;


    this->sizePopulation = sizePopulation_TA;
    Initialize(agentes, taskset);
    if (bestInd) {
        pop.pop_back();
        createBestInd();
    }
//    createBestInd();
//    pop[0].fitness[0] = getFitness(pop[0]);

    for (int i = 0; i < sizePopulation; i++) {
        pop[i].fitness[0] = getFitness(pop[i]);
    }

    int geracoes = 0;
    while (geracoes < GENERATIONS) {
        int cPop = 0;
        vector<Chromosome> temp;
        temp = pop;
        vector<Chromosome> newPop;
        Chromosome off1;
        Chromosome off2;
        //  cout<<endl<<" generations: "<<geracoes<<endl;


        while (cPop < sizePopulation) {
            //  cout<<endl<<" cPop: "<<cPop<<endl;

            int offspring1 = Tournament();
            int offspring2 = Tournament();

            off1 = pop[offspring1];
            off2 = pop[offspring2];
            //   cout<<" off1: "<<offspring1<<" off2: "<<offspring2<<endl;

            Crossover(off1.Individual, off2.Individual);
            Mutation(off1.Individual);
            Mutation(off2.Individual);
            cPop++;
            cPop++;
            newPop.push_back(off1);
            newPop.push_back(off2);
        }

        for (int i = 0; i < sizePopulation; i++) {
            newPop[i].fitness[0] = getFitness(newPop[i]);
        }
        std::sort(pop.begin(), pop.end(), sort_fitness0);
        std::sort(newPop.begin(), newPop.end(), sort_fitness0);
        int e = 0;
        for (int i = ceil(sizePopulation * ELITISM); i < sizePopulation; i++) {
            pop[i] = newPop[e];
            e++;
        }
        geracoes++;
    }

    int id = getBestChromosomeMakespan();
    // cout <<endl<< " Fitness: " << pop[id].fitness_m[0] <<endl;
    this->fitness_makespan = pop[0].fitness[0];
    this->fitness_st = pop[0].fitness[1];
   // cout<<"st: "<<this->fitness_st<<endl;
    return 0;
}

int GA::run_GA_Elitista(bool bestInd) {
    //cout<<"Elitista "<<endl;

    this->sizePopulation = sizePopulation_TA;

    Initialize(agentes, taskset);

    if (bestInd) {
        pop.pop_back();
        createBestInd();
    }

    for (int i = 0; i < sizePopulation; i++) {
        pop[i].fitness[0] = getFitness(pop[i]);
    }
    int geracoes = 0;
    while (geracoes < GENERATIONS) {
        int cPop = 0;
        vector<Chromosome> temp;
        temp = pop;
        vector<Chromosome> newPop;
        Chromosome off1;
        Chromosome off2;

        while (cPop < sizePopulation) {
            int offspring1 = Tournament();
            int offspring2 = Tournament();

            off1 = pop[offspring1];
            off2 = pop[offspring2];

            Crossover(off1.Individual, off2.Individual);
            Mutation(off1.Individual);
            Mutation(off2.Individual);
            cPop++;
            cPop++;
            newPop.push_back(off1);
            newPop.push_back(off2);
        }

        for (int i = 0; i < sizePopulation; i++) {
            newPop[i].fitness[0] = getFitness(newPop[i]);
        }
        for (auto &i : pop) {
            newPop.push_back(i);

        }
        pop.clear();
        std::sort(newPop.begin(), newPop.end(), sort_fitness0);
        for (int i = 0; i < sizePopulation; i++) {
            pop.push_back(newPop[i]);
        }
        geracoes++;

    }
    int id = getBestChromosomeMakespan();
    // cout<<"T: "<<timestep<< " Fitness: " << pop[id].fitness_m[0] <<endl;
    this->fitness_makespan = pop[id].fitness[0];
    this->fitness_st = pop[id].fitness[1];
  //  cout<<"st: "<<this->fitness_st<<endl;
    return id;
}

vector<vector<int>>
GA::run_GA(vector<Agent> agents, vector<Task *> taskset, vector<Task> task_total, vector<vector<int>> Dis, bool bestInd,
           int seed, int tipo, int timestep, int obj) {


    this->pop.clear();
    this->task_total = task_total;
    this->taskset = taskset;
    this->Dis = Dis;
    this->agentes = agents;
    this->seed = seed;
    this->timestep = timestep;

    std::clock_t start;
    float duration = 0.0;
    start = std::clock();

    int id = 0;
    correlacao(tipo, obj);


    if (tipo == 0) {
        id = run_GA_TA(bestInd);
    } else if (tipo == 1) {
        id = run_GA_Elitista(bestInd);
    } else if (tipo == 2) {
        id = steadyStateGA(bestInd);
    } else if (tipo == 3) {
        id = run_GA_E(bestInd);
    } else if (tipo == 4) {
        id = NSGA(bestInd, obj);
    }

    duration = (float)(std::clock() - start) / CLOCKS_PER_SEC;
    this->tempo_GA = duration;

    vector<vector<int>> list_task;
    list_task.resize(sizeAgents);
    for (auto &i : pop[id].Individual) {
        int n_task = i.task;
        int n_agent = i.agent;
        list_task[n_agent].push_back(n_task);
    }
    return list_task;
}


