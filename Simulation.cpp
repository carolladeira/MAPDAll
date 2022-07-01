//
// Created by carol on 12/17/18.
//

#include "Simulation.h"

#define maxtime1 10000
#define INF 1000000000

#define AG -1
//#define SIMU
//#define TIMESTEP

bool sortbysec(const pair<int,int> &a,
               const pair<int,int> &b)
{
    return (a.second > b.second);
}

void Simulation::run_AS(bool bestInd, int tipo_GA) {
    GA ga;
    int aux = 1;
    Agent *ag;
    int tot = 0;
    vector<vector<int>> task_agents;
    list_agents = ga.run_GA(agents, list_taskset, tasks_total, Dis, bestInd, seed, tipo_GA, 0, 0);
    this->fitness_m = ga.fitness_makespan;
    this->fitness_st = ga.fitness_st;
    bool adc = false;
    this->tempoTotalGA = ga.tempo_GA + this->tempoTotalGA;

    for (timestep = 0; true; timestep++) {
        //     cout << "Timestep " << timestep << " " << tot << endl;

        if (timestep > t_task) {
            bool finish = true;
            for (int i = 0; i < agents.size(); i++)
                if (!list_agents[i].empty()) finish = false;
            if (finish) {
                for (auto &agent : agents) {
                    ag = &agent;
                    tempoTotalPathPlan = ag->timepathPlan + tempoTotalPathPlan;
                    this->num_conflito_PP = ag->num_conflito + this->num_conflito_PP;
                }

                break;
            }
        }


        for (int i = aux; i <= timestep; i++) {
            if (taskset[i].empty())continue;
            if (i >= taskset.size()) break;
            aux = i + 1;
            for (vector<Task>::iterator it = taskset[i].begin(); it != taskset[i].end(); it++) {
                list_taskset.push_back(&(*it));
                adc = true;
            }
        }

        vector<int> need_plan;
        vector<int> plan_order;


        for (auto &agent : agents) agent.loc = agent.path[timestep];

        if (adc) {

            for (auto &agent : agents) {
                if (agent.delivering) {
                    agent.loc = agent.path[agent.finish_time];
                    continue;
                }
                agent.loc = agent.path[timestep];
                agent.finish_time = timestep;
            }

            task_agents = ga.run_GA(agents, list_taskset, tasks_total, Dis, bestInd, seed, tipo_GA, timestep, 0);
            this->fitness_m = ga.fitness_makespan;
            this->fitness_st = ga.fitness_st;
            this->tempoTotalGA = ga.tempo_GA + this->tempoTotalGA;
            adc = false;

            for (int i = 0; i < agents.size(); i++) {

                if (agents[i].delivering) {
                    int id_task = list_agents[i].front();
                    list_agents[agents[i].id].clear();
                    list_agents[agents[i].id].push_back(id_task);
                    for (int j = 0; j < task_agents[i].size(); j++)
                        list_agents[agents[i].id].push_back(task_agents[i][j]);
                } else {
                    //se nao tem tarefas a serem adicionadas para o agente eu limpo a lista dele, e faco ele ficar na posicao que ele esta ate o final;
                    //para nao repetir tarefas
                    if (!task_agents[i].size()) {
                        agents[i].next_ep = NULL;
                        need_plan.push_back(agents[i].id);
                        list_agents[agents[i].id].clear();
                        ag = &agents[i];
                        int finish = ag->planPathPark(agents[i].loc, ag->ep_park_loc, timestep, token);
                        if (finish == -1) {
                            printf("A Star ERROR\n");
                            while (1);
                        }
//                        for (int j = timestep; j < token.path[i].size(); j++) {
//                            token.path[i][j] = agents[i].path[j];
//                        }
                        continue;
                    }
                    int id_task = task_agents[i].front();
                    Task *task = &tasks_total[id_task];
                    if (agents[i].next_ep == NULL) {
                        need_plan.push_back(agents[i].id);
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    } else if (agents[i].next_ep->loc != task->start->loc) {
                        list_agents[agents[i].id].clear();
                        need_plan.push_back(agents[i].id);
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    } else if (agents[i].next_ep->loc == task->start->loc) {
                        list_agents[agents[i].id].clear();
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    }
                }
            }
        }

        if (timestep == 0) {
            for (int i = 0; i < agents.size(); i++)
                need_plan.push_back(i);
        }

        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (agents[id].loc == task->goal->loc && task->delivering == true && timestep == task->ag_arrive_goal) {
                // cout << " Agente " << id << " termina tarefa " << task->id << " no tempo " << task->ag_arrive_goal << " na loc " << agents[id].loc << endl;
                tot++;
                task->agent_id = task->agent->id;
                task->delivering = false;
                task->agent->delivering = false;
                agents[id].finish_time = timestep;
                task->agent->next_ep = NULL;
                task->agent->task = NULL;
                list_agents[i].erase(list_agents[i].begin());
                need_plan.push_back(agents[i].id);
            }
        }

        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            ag = &agents[i];
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (task->delivering == false && timestep >= task->release_time && agents[id].loc == task->start->loc) {
                //  cout << " Agente " << agents[id].id << " da tarefa " << task->id << " terminou coleta na loc "  << agents[id].loc << " no tempo " << timestep << " esta indo para entrega " << task->goal->loc << endl;
                task->ag_arrive_start = timestep;
                agents[id].task = task;
                agents[id].next_ep = task->goal;
                agents[id].delivering = true;
                deleteTask_id(task->id);
                task->agent = &agents[id];
                task->delivering = true;
                plan_order.push_back(id);

            }
        }


        for (int j = 0; j < need_plan.size(); j++) {
            int id = need_plan[j];
            ag = &agents[id];
            if (!list_agents[id].size()) continue;
            int id_task = list_agents[id].front();
            Task *task = &tasks_total[id_task];
            if (agents[id].loc != task->start->loc || timestep < task->release_time) {
                agents[id].next_ep = task->start;
                //     int finish = ag->planPath(agents[id].loc, task->start, timestep, token);
                // cout << " Agente " << agents[id].id << " comeca tarefa " << task->id << " vai da posicao loc " << agents[id].loc << " para coleta loc " << task->start->loc <<" dis: "<<Dis[agents[id].loc][task->start->loc]<<endl;
                plan_order.push_back(id);
            }
        }

        int c = 0;
        int tam = plan_order.size();
        while (c < tam) {
            int maior = 0;
            Agent *ag_final;
            for (int j = 0; j < plan_order.size(); j++) {
                int id = plan_order[j];
                ag = &agents[id];
                if (!list_agents[id].size()) continue;
                int id_task = list_agents[id].front();
                Task *task = &tasks_total[id_task];
                if (Dis[ag->loc][ag->next_ep->loc] > maior) {
                    maior = Dis[ag->loc][ag->next_ep->loc];
                    ag_final = ag;
                }
            }
            int id_task = list_agents[ag_final->id].front();
            Task *task = &tasks_total[id_task];
            int finish = ag_final->planPath(ag_final->loc, ag_final->next_ep, timestep, token);
            if (finish == -1) {
                printf("A Star ERROR\n");
                while (1);
            }
            if (ag_final->next_ep->loc == task->goal->loc) {
                //cout << " Agente " << agents[ag_final->id].id << " da tarefa " << task->id << " terminou coleta na loc "  << agents[ag_final->id].loc << " no tempo " << timestep << " esta indo para entrega " << task->goal->loc << endl;
                agents[ag_final->id].finish_time = finish;
                task->ag_arrive_goal = finish;
            } else {
                // cout << " Agente " << agents[ag_final->id].id << " comeca tarefa " << task->id << " vai da posicao loc " << agents[ag_final->id].loc << " para coleta loc " << task->start->loc <<" no tempo "<<finish<<" dis: "<<Dis[agents[ag_final->id].loc][task->start->loc]<<endl;

            }
            for (int i = 0; i < plan_order.size(); i++) {
                if (plan_order[i] == ag_final->id) {
                    plan_order.erase(plan_order.begin() + i);
                    break;
                }
            }
            c++;
        }

        if (!TestConstraints()) //test correctness
        {
            printf("TestConstraints ERROR\n");
            while (1);
        }
    }

}

void Simulation::run_AS2EP(bool bestInd, int tipo_GA) {
    GA ga;
    int aux = 1;
    Agent *ag;
    int tot = 0;
    vector<vector<int>> task_agents;
    list_agents = ga.run_GA(agents, list_taskset, tasks_total, Dis, bestInd, seed, tipo_GA, 0, 0);
    this->fitness_m = ga.fitness_makespan;
    this->fitness_st = ga.fitness_st;
    bool adc = false;
    this->tempoTotalGA = ga.tempo_GA + this->tempoTotalGA;

    for (timestep = 0; true; timestep++) {
        cout << "Timestep " << timestep << " " << tot << endl;
        if (timestep >= 114) {
            cout << "chegou" << endl;
        }
        if (timestep > t_task) {
            bool finish = true;
            for (int i = 0; i < agents.size(); i++)
                if (!list_agents[i].empty()) finish = false;
            if (finish) {
                for (auto &agent : agents) {
                    ag = &agent;
                    tempoTotalPathPlan = ag->timepathPlan + tempoTotalPathPlan;
                }

                break;
            }
        }


        for (int i = aux; i <= timestep; i++) {
            if (taskset[i].empty())continue;
            if (i >= taskset.size()) break;
            aux = i + 1;
            for (vector<Task>::iterator it = taskset[i].begin(); it != taskset[i].end(); it++) {
                list_taskset.push_back(&(*it));
                adc = true;
            }
        }
        token.taskset = list_taskset;

        vector<int> need_plan;
        vector<int> plan_order;


        for (auto &agent : agents) agent.loc = agent.path[timestep];

        if (adc) {

            for (auto &agent : agents) {
                if (agent.delivering) {
                    agent.loc = agent.path[agent.finish_time];
                    continue;
                }
                agent.loc = agent.path[timestep];
                agent.finish_time = timestep;
            }

            task_agents = ga.run_GA(agents, list_taskset, tasks_total, Dis, bestInd, seed, tipo_GA, timestep, 0);
            this->fitness_m = ga.fitness_makespan;
            this->fitness_st = ga.fitness_st;
            this->tempoTotalGA = ga.tempo_GA + this->tempoTotalGA;
            adc = false;

            for (int i = 0; i < agents.size(); i++) {

                if (agents[i].delivering) {
                    int id_task = list_agents[i].front();
                    list_agents[agents[i].id].clear();
                    list_agents[agents[i].id].push_back(id_task);
                    for (int j = 0; j < task_agents[i].size(); j++)
                        list_agents[agents[i].id].push_back(task_agents[i][j]);
                } else {
                    //se nao tem tarefas a serem adicionadas para o agente eu limpo a lista dele, e faco ele ficar na posicao que ele esta ate o final;
                    //para nao repetir tarefas
                    if (!task_agents[i].size()) {
                        agents[i].next_ep = NULL;
                        need_plan.push_back(agents[i].id);
                        list_agents[agents[i].id].clear();
                        ag = &agents[i];
                        int finish = ag->planPathPark(agents[i].loc, ag->ep_park_loc, timestep, token);
                        if (finish == -1) {
                            printf("A Star ERROR\n");
                            while (1);
                        }
//                        for (int j = timestep; j < token.path[i].size(); j++) {
//                            token.path[i][j] = agents[i].path[j];
//                        }
                        continue;
                    }
                    int id_task = task_agents[i].front();
                    Task *task = &tasks_total[id_task];
                    if (agents[i].next_ep == NULL) {
                        need_plan.push_back(agents[i].id);
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    } else if (agents[i].next_ep->loc != task->start->loc) {
                        list_agents[agents[i].id].clear();
                        need_plan.push_back(agents[i].id);
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    } else if (agents[i].next_ep->loc == task->start->loc) {
                        list_agents[agents[i].id].clear();
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    }
                }
            }
        }

        if (timestep == 0) {
            for (int i = 0; i < agents.size(); i++)
                need_plan.push_back(i);
        }

        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (agents[id].loc == task->goal->loc && task->delivering == true && timestep == task->ag_arrive_goal) {
                // cout << " Agente " << id << " termina tarefa " << task->id << " no tempo " << task->ag_arrive_goal << " na loc " << agents[id].loc << endl;
                tot++;
                task->agent_id = task->agent->id;
                task->delivering = false;
                task->agent->delivering = false;
                agents[id].finish_time = timestep;
                task->agent->next_ep = NULL;
                task->agent->task = NULL;
                list_agents[i].erase(list_agents[i].begin());
                need_plan.push_back(agents[i].id);
            }
        }

        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            ag = &agents[i];
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (task->delivering == false && timestep >= task->release_time && agents[id].loc == task->start->loc) {
                //  cout << " Agente " << agents[id].id << " da tarefa " << task->id << " terminou coleta na loc "  << agents[id].loc << " no tempo " << timestep << " esta indo para entrega " << task->goal->loc << endl;
                task->ag_arrive_start = timestep;
                agents[id].task = task;
                agents[id].next_ep = task->goal;
                agents[id].delivering = true;
                deleteTask_id(task->id);
                task->agent = &agents[id];
                task->delivering = true;
                plan_order.push_back(id);

            }
        }


        for (int j = 0; j < need_plan.size(); j++) {
            int id = need_plan[j];
            ag = &agents[id];
            if (!list_agents[id].size()) continue;
            int id_task = list_agents[id].front();
            Task *task = &tasks_total[id_task];
            if (agents[id].loc != task->start->loc || timestep < task->release_time) {
                agents[id].next_ep = task->start;
                //     int finish = ag->planPath(agents[id].loc, task->start, timestep, token);
                // cout << " Agente " << agents[id].id << " comeca tarefa " << task->id << " vai da posicao loc " << agents[id].loc << " para coleta loc " << task->start->loc <<" dis: "<<Dis[agents[id].loc][task->start->loc]<<endl;
                plan_order.push_back(id);
            }
        }

        int c = 0;
        int tam = plan_order.size();
        while (c < tam) {
            int maior = 0;
            Agent *ag_final;
            for (int j = 0; j < plan_order.size(); j++) {
                int id = plan_order[j];
                ag = &agents[id];
                if (!list_agents[id].size()) continue;
                int id_task = list_agents[id].front();
                Task *task = &tasks_total[id_task];
                if (Dis[ag->loc][ag->next_ep->loc] > maior) {
                    maior = Dis[ag->loc][ag->next_ep->loc];
                    ag_final = ag;
                }
            }
            int id_task = list_agents[ag_final->id].front();
            Task *task = &tasks_total[id_task];
            int finish = ag_final->planPath2EP(ag_final->loc, ag_final->next_ep, timestep, token);
            if (finish == -1) {
                printf("A Star ERROR\n");
                while (1);
            }
            if (ag_final->next_ep->loc == task->goal->loc) {
                //cout << " Agente " << agents[ag_final->id].id << " da tarefa " << task->id << " terminou coleta na loc "  << agents[ag_final->id].loc << " no tempo " << timestep << " esta indo para entrega " << task->goal->loc << endl;
                agents[ag_final->id].finish_time = finish;
                task->ag_arrive_goal = finish;
            } else {
                // cout << " Agente " << agents[ag_final->id].id << " comeca tarefa " << task->id << " vai da posicao loc " << agents[ag_final->id].loc << " para coleta loc " << task->start->loc <<" no tempo "<<finish<<" dis: "<<Dis[agents[ag_final->id].loc][task->start->loc]<<endl;

            }
            for (int i = 0; i < plan_order.size(); i++) {
                if (plan_order[i] == ag_final->id) {
                    plan_order.erase(plan_order.begin() + i);
                    break;
                }
            }
            c++;
        }

        if (!TestConstraints()) //test correctness
        {
            printf("TestConstraints ERROR\n");
            while (1);
        }
    }

}

void Simulation::run_AS_Hold(bool bestInd, int tipo_GA, int objetivo) {
    int aux = 1;
    Agent *ag;
    GA ga;
    int tot = 0;
    vector<vector<int>> task_agents;
    list_agents = ga.run_GA(agents, list_taskset, tasks_total, Dis, bestInd, seed, tipo_GA, 0, objetivo);
    this->fitness_m = ga.fitness_makespan;
    this->fitness_st = ga.fitness_st;
    bool adc = false;
    this->tempoTotalGA = ga.tempo_GA + this->tempoTotalGA;


    for (timestep = 0; true; timestep++) {
#ifdef TIMESTEP
        cout << "Timestep " << timestep << " " << tot << endl;
#endif
        ///Verifica se ja teminou todas as tarefas
        if (timestep > t_task) {
            bool finish = true;
            for (int i = 0; i < agents.size(); i++)
                if (!list_agents[i].empty()) finish = false;
            if (finish) {
                break;
            }
        }
        ///Adiciona Tarefas
        for (int i = aux; i <= timestep; i++) {
            if (taskset[i].empty())continue;
            if (i >= taskset.size()) break;
            aux = i + 1;
            for (vector<Task>::iterator it = taskset[i].begin(); it != taskset[i].end(); it++) {
                list_taskset.push_back(&(*it));
                adc = true;
            }
        }
        token.taskset = list_taskset;

        vector<int> need_plan;
        vector<int> plan_order;

        if (timestep == 0) {
            for (int i = 0; i < agents.size(); i++)
                need_plan.push_back(i);
        }
        for (auto &agent : agents) agent.loc = agent.path_principal[timestep].first;


        if (adc) {
            vector<Agent> copy_agents = agents;

            for (auto &agente_copy : copy_agents) {
                if (agente_copy.delivering) {
                    agente_copy.loc = agente_copy.path[agente_copy.finish_time];
                    continue;
                }
                agente_copy.loc = agente_copy.path[timestep];
                agente_copy.finish_time = timestep;
            }

            task_agents = ga.run_GA(copy_agents, list_taskset, tasks_total, Dis, bestInd, seed, tipo_GA, timestep,
                                    objetivo);
            this->fitness_m = ga.fitness_makespan;
            this->fitness_st = ga.fitness_st;
            this->tempoTotalGA = ga.tempo_GA + this->tempoTotalGA;
            adc = false;

            for (int i = 0; i < agents.size(); i++) {

                if (agents[i].delivering) {
                    int id_task = list_agents[i].front();
                    list_agents[agents[i].id].clear();
                    list_agents[agents[i].id].push_back(id_task);
                    for (int j = 0; j < task_agents[i].size(); j++)
                        list_agents[agents[i].id].push_back(task_agents[i][j]);
                } else {
                    //se nao tem tarefas a serem adicionadas para o agente eu limpo a lista dele, e faco ele ficar na posicao que ele esta ate o final;
                    //para nao repetir tarefas
                    if (!task_agents[i].size()) {
                        agents[i].next_ep = nullptr;
                        agents[i].task = nullptr;
                        need_plan.push_back(agents[i].id);
                        list_agents[agents[i].id].clear();
                        ag = &agents[i];
                        // int finish = ag->planPathPark(agents[i].loc, ag->ep_park_loc, timestep, token);
                        bool finish = pp.Move2EP(&agents[i], timestep, false, token);
                        if (finish == false) {
                            printf("A Star ERROR\n");
                            while (1);
                        }
                        continue;
                    }
                    int id_task = task_agents[i].front();
                    Task *task = &tasks_total[id_task];
                    if (agents[i].next_ep == nullptr || (agents[i].task->id != id_task)) {
                        list_agents[agents[i].id].clear();
                        need_plan.push_back(agents[i].id);
                        list_agents[agents[i].id] = task_agents[i];
                    } else if (agents[i].task->id == id_task) {
                        list_agents[agents[i].id] = task_agents[i];
                    }
                }
            }
        }

        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (agents[id].loc == task->goal->loc && task->delivering == true && timestep == task->ag_arrive_goal) {
#ifdef SIMU
                if(agents[id].id == AG || AG == -1) {
                    cout << " Agente " << id << " termina tarefa " << task->id << " no tempo " << task->ag_arrive_goal
                         << " na loc " << agents[id].loc << endl;
                }
#endif
                tot++;
                task->agent_id = task->agent->id;
                task->delivering = false;
                task->agent->delivering = false;
                agents[id].finish_time = timestep;
                agents[id].list_task_completed.push_back(task->id);
                task->agent->next_ep = nullptr;
                task->agent->task = nullptr;
                list_agents[i].erase(list_agents[i].begin());
                need_plan.push_back(agents[i].id);
            }
        }

        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            ag = &agents[i];
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (task->delivering == false && timestep >= task->release_time && agents[id].loc == task->start->loc) {
#ifdef SIMU
                if(ag->id == AG || AG == -1){
                    cout << " Agente " << agents[id].id << " da tarefa " << task->id << " terminou coleta na loc "  << agents[id].loc << " no tempo " << timestep << " esta indo para entrega " << task->goal->loc << endl;
                }
#endif
                task->ag_arrive_start = timestep;
                agents[id].task = task;
                agents[id].next_ep = task->goal;
                agents[id].delivering = true;
                deleteTask_id(task->id);
                task->agent = &agents[id];
                task->delivering = true;
                plan_order.push_back(id);
            }

        }

        for (int j = 0; j < need_plan.size(); j++) {
            int id = need_plan[j];
            ag = &agents[id];
            if (!list_agents[id].size()) continue;
            int id_task = list_agents[id].front();
            Task *task = &tasks_total[id_task];
            if (agents[id].loc != task->start->loc || timestep < task->release_time) {
                agents[id].next_ep = task->start;
                agents[id].task = task;
#ifdef SIMU
                if(ag->id == AG || AG == -1) {
                    cout << " Agente " << agents[id].id << " comeca tarefa " << task->id << " vai da posicao loc "
                         << agents[id].loc << " para coleta loc " << task->start->loc << " dis: "
                         << Dis[agents[id].loc][task->start->loc] << endl;
                }
#endif
                plan_order.push_back(id);
            }
        }

        vector<pair<int, int> > list_ordered;

        for (int id : plan_order) {
            ag = &agents[id];
            int dist = Dis[ag->loc][ag->next_ep->loc];
            list_ordered.emplace_back(id, dist);
        }
        std::sort(list_ordered.begin(), list_ordered.end(), sortbysec);
        Agent *ag_final;
        while (!list_ordered.empty()) {
            //cout << list_ordered.begin()->first << endl;
            int id_ag = list_ordered.begin()->first;
            ag_final = &agents[id_ag];
            int id_task = list_agents[ag_final->id].front();
            Task *task = &tasks_total[id_task];
         //   cout << endl << " Agent " << ag_final->id <<" task: "<< ag_final->task->id <<" dist: "<<list_ordered.begin()->second<<endl;
            int finish = planPathAStar(ag_final, timestep, ag_final->next_ep);
            if (finish == -1) {
                printf("A Star ERROR\n");
                while (true);
            }
            if (ag_final->next_ep->loc == task->goal->loc) {
                //cout << " Agente " << agents[ag_final->id].id << " da tarefa " << task->id << " terminou coleta na loc "  << agents[ag_final->id].loc << " no tempo " << timestep << " esta indo para entrega " << task->goal->loc << endl;
                agents[ag_final->id].finish_time = finish;
                task->ag_arrive_goal = finish;
            }
            list_ordered.erase(list_ordered.begin());
        }
        if (!TestConstraints()) //test correctness
        {
            printf("TestConstraints ERROR\n");
            while (true);
        }
    }

}

void Simulation::run_AS_Fast(bool bestInd, int tipo_GA) {
    int aux = 1;
    Agent *ag;
    GA ga;
    int tot = 0;
    vector<vector<int>> task_agents;
    list_agents = ga.run_GA(agents, list_taskset, tasks_total, Dis, bestInd, seed, tipo_GA, 0, 0);
    this->fitness_m = ga.fitness_makespan;
    this->fitness_st = ga.fitness_st;
    bool adc = false;
    this->tempoTotalGA = ga.tempo_GA + this->tempoTotalGA;

    for (timestep = 0; true; timestep++) {
#ifdef TIMESTEP
        cout << "Timestep " << timestep << " " << tot << endl;
#endif
        ///Verifica se ja teminou todas as tarefas
        if (timestep > t_task) {
            bool finish = true;
            for (int i = 0; i < agents.size(); i++)
                if (!list_agents[i].empty()) finish = false;
            if (finish) {
                for (auto &agent : agents) {
                    ag = &agent;
                    tempoTotalPathPlan = ag->timepathPlan + tempoTotalPathPlan;
                }

                break;
            }
        }
        ///Adiciona Tarefas
        for (int i = aux; i <= timestep; i++) {
            if (taskset[i].empty())continue;
            if (i >= taskset.size()) break;
            aux = i + 1;
            for (vector<Task>::iterator it = taskset[i].begin(); it != taskset[i].end(); it++) {
                list_taskset.push_back(&(*it));
                adc = true;
            }
        }

        vector<int> need_plan;
        vector<int> plan_order;

        if (timestep == 0) {
            for (int i = 0; i < agents.size(); i++)
                need_plan.push_back(i);
        }
        for (auto &agent : agents) agent.loc = agent.path[timestep];

        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            ag = &agents[i];
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (task->delivering == false && timestep >= task->release_time && agents[id].loc == task->start->loc &&
                task->id == ag->task->id) {
#ifdef SIMU
                if(ag->id == AG || AG == -1){
                    cout << " Agente " << agents[id].id << " da tarefa " << task->id << " terminou coleta na loc "  << agents[id].loc << " no tempo " << timestep << " esta indo para entrega " << task->goal->loc << endl;

                }
#endif
                task->ag_arrive_start = timestep;
                agents[id].delivering = true;
                deleteTask_id(task->id);
                task->agent = &agents[id];
                task->delivering = true;

            }
        }

        if (adc) {
            vector<Agent> copy_agents = agents;

            for (auto &agente_copy : copy_agents) {
                if (agente_copy.delivering) {
                    agente_copy.loc = agente_copy.path[agente_copy.finish_time];
                    continue;
                }
                agente_copy.loc = agente_copy.path[timestep];
                agente_copy.finish_time = timestep;
            }

            task_agents = ga.run_GA(copy_agents, list_taskset, tasks_total, Dis, bestInd, seed, tipo_GA, timestep, 0);
            this->fitness_m = ga.fitness_makespan;
            this->fitness_st = ga.fitness_st;
            this->tempoTotalGA = ga.tempo_GA + this->tempoTotalGA;
            adc = false;

            for (int i = 0; i < agents.size(); i++) {

                if (agents[i].delivering) {
                    int id_task = list_agents[i].front();
                    list_agents[agents[i].id].clear();
                    list_agents[agents[i].id].push_back(id_task);
                    for (int j = 0; j < task_agents[i].size(); j++)
                        list_agents[agents[i].id].push_back(task_agents[i][j]);
                } else {
                    //se nao tem tarefas a serem adicionadas para o agente eu limpo a lista dele, e faco ele ficar na posicao que ele esta ate o final;
                    //para nao repetir tarefas
                    if (!task_agents[i].size()) {
                        agents[i].next_ep = nullptr;
                        need_plan.push_back(agents[i].id);
                        list_agents[agents[i].id].clear();
                        ag = &agents[i];
                        int finish = ag->planPathPark(agents[i].loc, ag->ep_park_loc, timestep, token);
                        if (finish == -1) {
                            printf("A Star ERROR\n");
                            while (1);
                        }
                        continue;
                    }
                    int id_task = task_agents[i].front();
                    Task *task = &tasks_total[id_task];
                    if (agents[i].next_ep == nullptr || (agents[i].task->id != id_task)) {
                        list_agents[agents[i].id].clear();
                        need_plan.push_back(agents[i].id);
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    } else if (agents[i].task->id == id_task) {
                        list_agents[agents[i].id] = task_agents[i];
                    }
//                    else if (agents[i].next_ep->loc != task->start->loc) {
//                        list_agents[agents[i].id].clear();
//                        need_plan.push_back(agents[i].id);
//                        for (int j = 0; j < task_agents[i].size(); j++)
//                            list_agents[agents[i].id].push_back(task_agents[i][j]);
//                    } else if (agents[i].next_ep->loc == task->start->loc) {
//                        list_agents[agents[i].id].clear();
//                        for (int j = 0; j < task_agents[i].size(); j++)
//                            list_agents[agents[i].id].push_back(task_agents[i][j]);
//                    }
                }
            }
        }

        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (agents[id].loc == task->goal->loc && task->delivering == true && timestep == task->ag_arrive_goal) {
#ifdef SIMU
                if(agents[id].id == AG || AG == -1) {

                    cout << " Agente " << id << " termina tarefa " << task->id << " no tempo " << task->ag_arrive_goal
                         << " na loc " << agents[id].loc << endl;
                }
#endif
                tot++;
                task->agent_id = task->agent->id;
                task->delivering = false;
                task->agent->delivering = false;
                agents[id].finish_time = timestep;
                agents[id].list_task_completed.push_back(task->id);
                task->agent->next_ep = nullptr;
                task->agent->task = nullptr;
                list_agents[i].erase(list_agents[i].begin());
                need_plan.push_back(agents[i].id);
            }
        }

        for (int j = 0; j < need_plan.size(); j++) {
            int id = need_plan[j];
            ag = &agents[id];
            if (!list_agents[id].size()) continue;
            int id_task = list_agents[id].front();
            Task *task = &tasks_total[id_task];
            if (agents[id].loc != task->start->loc || timestep < task->release_time || agents[id].delivering == false) {
                agents[id].next_ep = task->start;
                agents[id].task = task;
#ifdef SIMU
                if(ag->id == AG || AG == -1) {
                    cout << " Agente " << agents[id].id << " comeca tarefa " << task->id << " vai da posicao loc "
                         << agents[id].loc << " para coleta loc " << task->start->loc << " dis: "
                         << Dis[agents[id].loc][task->start->loc] << endl;
                }
#endif
                plan_order.push_back(id);
            }
        }

        int c = 0;
        int tam = plan_order.size();
        while (c < tam) {
            int maior = 0;
            Agent *ag_final;
            for (int j = 0; j < plan_order.size(); j++) {
                int id = plan_order[j];
                ag = &agents[id];
                if (!list_agents[id].size()) continue;
                int id_task = list_agents[id].front();
                Task *task = &tasks_total[id_task];
                if (Dis[ag->loc][ag->next_ep->loc] >= maior) {
                    maior = Dis[ag->loc][ag->next_ep->loc];
                    ag_final = ag;
                }
            }
            int id_task = list_agents[ag_final->id].front();
            Task *task = &tasks_total[id_task];
            int finish = ag_final->planFastPath(ag_final->loc, timestep, token);
            if (ag_final->loc == ag_final->task->start->loc) {
                task->ag_arrive_start = timestep;
                ag_final->delivering = true;
                deleteTask_id(task->id);
                task->agent = ag_final;
                task->delivering = true;
            }
            agents[ag_final->id].finish_time = finish;
            task->ag_arrive_goal = finish;
            if (finish == -1) {
                printf("A Star ERROR\n");
                while (1);
            }
            for (int i = 0; i < plan_order.size(); i++) {
                if (plan_order[i] == ag_final->id) {
                    plan_order.erase(plan_order.begin() + i);
                    break;
                }
            }
            c++;
        }

        if (!TestConstraints()) //test correctness
        {
            printf("TestConstraints ERROR\n");
            while (1);
        }
    }

}

void Simulation::run_SCol(bool bestInd, int tipo_GA) {

    GA ga;
    int aux = 1;
    Agent *ag;
    int tot = 0;
    vector<vector<int>> task_agents;
    list_agents = ga.run_GA(agents, list_taskset, tasks_total, Dis, bestInd, seed, tipo_GA, 0, 0);
    this->fitness_m = ga.fitness_makespan;
    this->fitness_st = ga.fitness_st;
    bool adc = false;
    this->tempoTotalGA = ga.tempo_GA + this->tempoTotalGA;

    for (timestep = 0; true; timestep++) {
        //cout << "Timestep " << timestep << " " << tot << endl;

        if (timestep > t_task) {
            bool finish = true;
            for (int i = 0; i < agents.size(); i++)
                if (list_agents[i].size() > 0) finish = false;
            if (finish) {
                for (int i = 0; i < agents.size(); i++) {
                    ag = &agents[i];
                    tempoTotalPathPlan = ag->timepathPlan + tempoTotalPathPlan;
                }
                break;
            }
        }


        for (int i = aux; i <= timestep; i++) {
            if (taskset[i].empty())continue;
            if (i >= taskset.size()) break;
            aux = i + 1;
            for (vector<Task>::iterator it = taskset[i].begin(); it != taskset[i].end(); it++) {
                list_taskset.push_back(&(*it));
                adc = true;
            }
        }

        vector<int> need_plan;

        for (int i = 0; i < agents.size(); i++) agents[i].loc = agents[i].path[timestep];

        if (adc) {

            for (int i = 0; i < agents.size(); i++) {
                if (agents[i].delivering) {
                    agents[i].loc = agents[i].path[agents[i].finish_time];
                    continue;
                }
                agents[i].loc = agents[i].path[timestep];
                agents[i].finish_time = timestep;
            }

            task_agents = ga.run_GA(agents, list_taskset, tasks_total, Dis, bestInd, seed, tipo_GA, timestep, 0);
            this->fitness_m = ga.fitness_makespan;
            this->fitness_st = ga.fitness_st;
            this->tempoTotalGA = ga.tempo_GA + this->tempoTotalGA;

            adc = false;

            for (int i = 0; i < agents.size(); i++) {

                if (agents[i].delivering) {
                    int id_task = list_agents[i].front();
                    list_agents[agents[i].id].clear();
                    list_agents[agents[i].id].push_back(id_task);
                    for (int j = 0; j < task_agents[i].size(); j++)
                        list_agents[agents[i].id].push_back(task_agents[i][j]);
                } else {
                    //se nao tem tarefas a serem adicionadas para o agente eu limpo a lista dele, e faco ele ficar na posicao que ele esta ate o final;
                    //para nao repetir tarefas
                    if (!task_agents[i].size()) {
                        agents[i].next_ep = nullptr;
                        need_plan.push_back(agents[i].id);
                        list_agents[agents[i].id].clear();
                        for (int j = timestep; j < agents[i].path.size(); j++) {
                            agents[i].path[j] = agents[i].path[timestep];
                        }
                        continue;
                    }
                    int id_task = task_agents[i].front();
                    Task *task = &tasks_total[id_task];
                    if (agents[i].next_ep == nullptr) {
                        need_plan.push_back(agents[i].id);
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    } else if (agents[i].next_ep->loc != task->start->loc) {
                        list_agents[agents[i].id].clear();
                        need_plan.push_back(agents[i].id);
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);

                    } else if (agents[i].next_ep->loc == task->start->loc) {
                        list_agents[agents[i].id].clear();
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    }
                }
            }
        }

        if (timestep == 0) {
            for (int i = 0; i < agents.size(); i++)
                need_plan.push_back(i);
        }

        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (agents[id].loc == task->goal->loc && task->delivering == true && timestep == task->ag_arrive_goal) {
                //   cout << " Agente " << id << " termina tarefa " << task->id << " no tempo " << task->ag_arrive_goal << " na loc " << agents[id].loc << endl;
                tot++;
                task->agent_id = task->agent->id;
                task->delivering = false;
                task->agent->delivering = false;
                agents[id].finish_time = timestep;
                task->agent->next_ep = nullptr;
                task->agent->task = nullptr;
                list_agents[i].erase(list_agents[i].begin());
                need_plan.push_back(agents[i].id);
            }
        }

        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            ag = &agents[i];
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (task->delivering == false && timestep >= task->release_time && agents[id].loc == task->start->loc) {
                //   cout << " Agente " << agents[id].id << " da tarefa " << task->id << " terminou coleta na loc "  << agents[id].loc << " no tempo " << timestep << " esta indo para entrega " << task->goal->loc << endl;
                task->ag_arrive_start = timestep;
                agents[id].task = task;
                agents[id].next_ep = task->goal;
                agents[id].delivering = true;
                deleteTask_id(task->id);
                task->agent = &agents[id];
                task->delivering = true;
                int finish = ag->AStar_sCol(agents[id].loc, task->goal, timestep, token, 0, 0);
                if (finish == -1) {
                    printf("A Star ERROR\n");
                    while (1);
                }
                agents[id].finish_time = finish;
                task->ag_arrive_goal = finish;
            }
        }

        for (int j = 0; j < need_plan.size(); j++) {
            int id = need_plan[j];
            ag = &agents[id];
            if (!list_agents[id].size()) continue;
            int id_task = list_agents[id].front();
            Task *task = &tasks_total[id_task];
            if (agents[id].loc != task->start->loc || timestep < task->release_time) {
                agents[id].next_ep = task->start;
                int finish = ag->AStar_sCol(agents[id].loc, task->start, timestep, token, 0, 0);
                //   cout << " Agente " << agents[id].id << " comeca tarefa " << task->id << " vai da posicao loc " << agents[id].loc << " para coleta loc " << task->start->loc <<" no tempo "<<finish<<" dis: "<<Dis[agents[id].loc][task->start->loc]<<endl;
                if (finish == -1) {
                    printf("A Star ERROR\n");
                    while (1);
                }
            }
        }
    }

}

void Simulation::run_CBS(bool bestInd, int tipo_GA) {

    int sumofcost = 0;
    vector<bool> last_vis(agents.size(), false);
    int tot = 0;
    int aux = 1;
    GA ga;
    bool adc = false;
    vector<vector<int>> task_agents;
    list_agents = ga.run_GA(agents, list_taskset, tasks_total, Dis, bestInd, seed, tipo_GA, 0, 0);
    this->fitness_m = ga.fitness_makespan;
    this->fitness_st = ga.fitness_st;
    this->tempoTotalGA = ga.tempo_GA + this->tempoTotalGA;
    int valor = 0;
    for (timestep = 0; true; timestep++) {
        cout << "Timestep " << timestep << " " << tot << endl;

        if (timestep > t_task) {
            bool finish = true;
            for (int i = 0; i < agents.size(); i++)
                if (!list_agents[i].empty()) finish = false;
            if (finish) {
                this->conflito_CSB_final = this->num_conflito_CBS_total - valor;
                //  cout<< this->num_conflito_CBS_total -valor <<endl;
                break;
            }
        }

        for (int i = aux; i <= timestep; i++) {
            if (taskset[i].empty())continue;
            if (i >= taskset.size()) break;
            aux = i + 1;
            for (vector<Task>::iterator it = taskset[i].begin(); it != taskset[i].end(); it++) {
                list_taskset.push_back(&(*it));
                adc = true;
            }
        }
        vector<int> need_plan;


        for (auto &agent : agents) agent.loc = agent.path[timestep];


        if (adc) {
            valor = this->num_conflito_CBS_total;
            if (!list_taskset.empty()) {
                vector<Agent> copy_agents = agents;
                for (int i = 0; i < agents.size(); i++) {
                    if (agents[i].delivering) {
                        int t = timestep;
                        while (agents[i].path[t] != agents[i].next_ep->loc) t++;
                        copy_agents[i].finish_time = t;
                        copy_agents[i].loc = agents[i].path[t];
                        continue;
                    }
                    copy_agents[i].loc = agents[i].path[timestep];
                    copy_agents[i].finish_time = timestep;
                }

                task_agents = ga.run_GA(copy_agents, list_taskset, tasks_total, Dis, bestInd, seed, tipo_GA, timestep,
                                        0);
                this->fitness_m = ga.fitness_makespan;
                this->fitness_st = ga.fitness_st;
                this->tempoTotalGA = ga.tempo_GA + this->tempoTotalGA;
                adc = false;


                for (int i = 0; i < agents.size(); i++) {
                    if (agents[i].delivering) {
                        int id_task = list_agents[i].front();
                        list_agents[agents[i].id].clear();
                        list_agents[agents[i].id].push_back(id_task);
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    } else {
                        //se nao tem tarefas a serem adicionadas para o agente eu limpo a lista dele, e faco ele ficar na posicao que ele esta ate o final;
                        //para nao repetir tarefas
                        if (task_agents[i].empty()) {
                            agents[i].next_ep = nullptr;
                            need_plan.push_back(agents[i].id);
                            list_agents[agents[i].id].clear();
                            continue;
                        }
                        int id_task = task_agents[i].front();
                        Task *task = &tasks_total[id_task];
                        if (agents[i].next_ep == nullptr) {
                            need_plan.push_back(agents[i].id);
                            for (int j = 0; j < task_agents[i].size(); j++)
                                list_agents[agents[i].id].push_back(task_agents[i][j]);
                        } else if (agents[i].next_ep->loc != task->start->loc) {
                            list_agents[agents[i].id].clear();
                            need_plan.push_back(agents[i].id);
                            for (int j = 0; j < task_agents[i].size(); j++)
                                list_agents[agents[i].id].push_back(task_agents[i][j]);

                        } else if (agents[i].next_ep->loc == task->start->loc) {
                            list_agents[agents[i].id].clear();
                            for (int j = 0; j < task_agents[i].size(); j++)
                                list_agents[agents[i].id].push_back(task_agents[i][j]);
                        }
                    }
                }
            }
        }
        if (timestep == 0) {
            for (int i = 0; i < agents.size(); i++) need_plan.push_back(i);
        }

        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (agents[id].loc == task->goal->loc && task->delivering == true && timestep == task->ag_arrive_goal) {
                //cout << " Agente " << id << " termina tarefa " << task->id << " no tempo " << task->ag_arrive_goal << " na loc " << agents[id].loc << endl;
                tot++;
                task->agent_id = task->agent->id;
                task->delivering = false;
                task->agent->delivering = false;
                agents[id].finish_time = timestep;
                task->agent->next_ep = nullptr;
                task->agent->task = nullptr;
                list_agents[i].erase(list_agents[i].begin());
                need_plan.push_back(i);


            }
        }

        vector<Agent *> ag_icbs;
        vector<bool> is_conflict(agents.size(), true);
        for (int j = 0; j < need_plan.size(); j++) {
            int id = need_plan[j];
            if (!list_agents[id].size()) continue;
            int id_task = list_agents[id].front();
            Task *task = &tasks_total[id_task];
            if (agents[id].loc != task->start->loc || timestep < task->release_time) {
                list_agents[id] = list_agents[id];
                // cout << " Agente " << agents[id].id << " comeca tarefa " << task->id << " vai da posicao loc " << agents[id].loc << " para coleta loc " << task->start->loc <<" dis: "<<Dis[agents[id].loc][task->start->loc]<<endl;
                agents[id].next_ep = task->start;
                ag_icbs.push_back(&agents[id]);
                is_conflict[id] = false;
            }
        }


        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (task->delivering == false && timestep >= task->release_time && agents[id].loc == task->start->loc) {
                //   cout << " Agente " << agents[id].id << " da tarefa " << task->id << " terminou coleta na loc " << agents[id].loc << " no tempo " << timestep << " esta indo para entrega " << task->goal->loc << endl;
                task->ag_arrive_start = timestep;
                agents[id].task = task;
                agents[id].next_ep = task->goal;
                agents[id].delivering = true;
                ag_icbs.push_back(&agents[id]);
                deleteTask_id(task->id);
                task->agent = &agents[id];
                task->delivering = true;
                is_conflict[id] = false;
            }
        }

        if (!ag_icbs.empty()) //path finding
        {
            for (int i = 0; i < agents.size(); i++) {
                if (!list_agents[i].size()) continue;
                int id_task = list_agents[i].front();
                Task *task = &tasks_total[id_task];
                int id = i;
                bool flag = true;
                for (int j = 0; j < ag_icbs.size(); j++)
                    if (ag_icbs[j]->id == id)
                        flag = false;
                if (flag) {
                    is_conflict[id] = false;
                    if (agents[id].next_ep == nullptr) {
                        //  cout<<" Agente "<<agents[id].id<<" com next_ep nulo comeca tarefa "<<task->id<<" e vai para posicao coleta "<<task->start->loc<<endl;
                        agents[id].next_ep = task->start;
                    }
                    ag_icbs.push_back(&agents[id]);
                }
            }

            vector<vector<int>> cons_paths;
            for (int i = 0; i < agents.size(); i++)
                if (is_conflict[i])
                    cons_paths.push_back(agents[i].path);
            if (!PathFinding(ag_icbs, cons_paths))
                return;
        }
        if (!TestConstraints()) //test correctness
        {
            printf("TestConstraints ERROR\n");
            while (1);
        }
    }
}

void Simulation::runNSGA() {
//    GA ga;
//    bool adc = false;
//    vector<vector<int>> task_agents;
//    ga.NSGA(agents, list_taskset, token, tasks_total, Dis, 0, 0, 0);
}

void Simulation::run_AS_teste() {

    int sumofcost = 0;
    vector<bool> last_vis(agents.size(), false);
    int tot = 0;
    int aux = 1;
    bool adc = false;
    Agent *ag;

    vector<vector<int>> task_agents;
    list_agents = heuristic(agents, list_taskset);
//    string res_file = "/home/carol/Desktop/MAPD/Resultados/CBS-Heuristic-" +to_string(agents.size()) + "-" +to_string(this->freq)+ "-"+to_string(this->seed)+".csv";
//    ofstream fout(res_file, ios::app);
//    if (!fout) cout << "ERR0";

    for (timestep = 0; true; timestep++) {
#ifdef TIMESTEP

        // cout << "Timestep " << timestep << " " << tot << endl;

#endif

        if (timestep > t_task) {
            bool finish = true;
            for (int i = 0; i < agents.size(); i++)
                if (list_agents[i].size() > 0) finish = false;
            if (finish) {
                // printf("%d\n", tasks_total.size());
                // cout << "MAKESPAN " << timestep - 1 << endl;

                break;
            }
        }

        for (int i = aux; i <= timestep; i++) {
            if (taskset[i].empty())continue;
            if (i >= taskset.size()) break;
            aux = i + 1;
            for (vector<Task>::iterator it = taskset[i].begin(); it != taskset[i].end(); it++) {
                list_taskset.push_back(&(*it));
                adc = true;
            }
        }
        vector<int> need_plan;
        vector<int> plan_order;


        for (int i = 0; i < agents.size(); i++) agents[i].loc = agents[i].path[timestep];


        if (adc) {
            if (!list_taskset.empty()) {
                vector<Agent> copy_agents = agents;
                for (int i = 0; i < agents.size(); i++) {
                    if (agents[i].delivering) {
                        int t = timestep;
                        while (agents[i].path[t] != agents[i].next_ep->loc) t++;
                        copy_agents[i].finish_time = t;
                        copy_agents[i].loc = agents[i].path[t];
                        continue;
                    }
                    copy_agents[i].loc = agents[i].path[timestep];
                    copy_agents[i].finish_time = timestep;
                }

                task_agents = heuristic(copy_agents, list_taskset);
                adc = false;


                for (int i = 0; i < agents.size(); i++) {
                    if (agents[i].delivering) {
                        int id_task = list_agents[i].front();
                        list_agents[agents[i].id].clear();
                        list_agents[agents[i].id].push_back(id_task);
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    } else {
                        //se nao tem tarefas a serem adicionadas para o agente eu limpo a lista dele, e faco ele ficar na posicao que ele esta ate o final;
                        //para nao repetir tarefas
                        if (!task_agents[i].size()) {
                            agents[i].next_ep = nullptr;
                            need_plan.push_back(agents[i].id);
                            list_agents[agents[i].id].clear();
                            continue;
                        }
                        int id_task = task_agents[i].front();
                        Task *task = &tasks_total[id_task];
                        if (agents[i].next_ep == nullptr) {
                            need_plan.push_back(agents[i].id);
                            for (int j = 0; j < task_agents[i].size(); j++)
                                list_agents[agents[i].id].push_back(task_agents[i][j]);
                        } else if (agents[i].next_ep->loc != task->start->loc) {
                            list_agents[agents[i].id].clear();
                            need_plan.push_back(agents[i].id);
                            for (int j = 0; j < task_agents[i].size(); j++)
                                list_agents[agents[i].id].push_back(task_agents[i][j]);

                        } else if (agents[i].next_ep->loc == task->start->loc) {
                            list_agents[agents[i].id].clear();
                            for (int j = 0; j < task_agents[i].size(); j++)
                                list_agents[agents[i].id].push_back(task_agents[i][j]);
                        }
                    }
                }
            }
        }

        if (timestep == 0) {
            for (int i = 0; i < agents.size(); i++)
                need_plan.push_back(i);
        }

        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (agents[id].loc == task->goal->loc && task->delivering == true && timestep == task->ag_arrive_goal) {
                // cout << " Agente " << id << " termina tarefa " << task->id << " no tempo " << task->ag_arrive_goal << " na loc " << agents[id].loc << endl;
                tot++;
                task->agent_id = task->agent->id;
                task->delivering = false;
                task->agent->delivering = false;
                agents[id].finish_time = timestep;
                task->agent->next_ep = nullptr;
                task->agent->task = nullptr;
                list_agents[i].erase(list_agents[i].begin());
                need_plan.push_back(agents[i].id);
            }
        }

        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            ag = &agents[i];
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (task->delivering == false && timestep >= task->release_time && agents[id].loc == task->start->loc) {
                //  cout << " Agente " << agents[id].id << " da tarefa " << task->id << " terminou coleta na loc "  << agents[id].loc << " no tempo " << timestep << " esta indo para entrega " << task->goal->loc << endl;
                task->ag_arrive_start = timestep;
                agents[id].task = task;
                agents[id].next_ep = task->goal;
                agents[id].delivering = true;
                deleteTask_id(task->id);
                task->agent = &agents[id];
                task->delivering = true;
                plan_order.push_back(id);

            }
        }


        for (int j = 0; j < need_plan.size(); j++) {
            int id = need_plan[j];
            ag = &agents[id];
            if (!list_agents[id].size()) continue;
            int id_task = list_agents[id].front();
            Task *task = &tasks_total[id_task];
            if (agents[id].loc != task->start->loc || timestep < task->release_time) {
                agents[id].next_ep = task->start;
                //     int finish = ag->planPath(agents[id].loc, task->start, timestep, token);
                // cout << " Agente " << agents[id].id << " comeca tarefa " << task->id << " vai da posicao loc " << agents[id].loc << " para coleta loc " << task->start->loc <<" dis: "<<Dis[agents[id].loc][task->start->loc]<<endl;
                plan_order.push_back(id);
            }
        }

        int c = 0;
        int tam = plan_order.size();
        while (c < tam) {
            int maior = 0;
            Agent *ag_final;
            for (int j = 0; j < plan_order.size(); j++) {
                int id = plan_order[j];
                ag = &agents[id];
                if (!list_agents[id].size()) continue;
                int id_task = list_agents[id].front();
                Task *task = &tasks_total[id_task];
                if (Dis[ag->loc][ag->next_ep->loc] > maior) {
                    maior = Dis[ag->loc][ag->next_ep->loc];
                    ag_final = ag;
                }
            }
            int id_task = list_agents[ag_final->id].front();
            Task *task = &tasks_total[id_task];
            int finish = SingleFindPath(ag_final, timestep);

            //   int finish = ag_final->planPath(ag_final->loc, ag_final->next_ep, timestep, token);
            // cout<<endl << " Agente " << agents[ag_final->id].id  << " no tempo " << timestep << " esta indo para " << ag_final->next_ep->loc <<" no tempo "<<finish<< endl;

            if (finish == -1) {
                printf("A Star ERROR\n");
                while (1);
            }
            if (ag_final->next_ep->loc == task->goal->loc) {
                //cout << " Agente " << agents[ag_final->id].id << " da tarefa " << task->id << " terminou coleta na loc "  << agents[ag_final->id].loc << " no tempo " << timestep << " esta indo para entrega " << task->goal->loc << endl;
                agents[ag_final->id].finish_time = finish;
                task->ag_arrive_goal = finish;
            } else {
                // cout << " Agente " << agents[ag_final->id].id << " comeca tarefa " << task->id << " vai da posicao loc " << agents[ag_final->id].loc << " para coleta loc " << task->start->loc <<" no tempo "<<finish<<" dis: "<<Dis[agents[ag_final->id].loc][task->start->loc]<<endl;

            }
            for (int i = 0; i < plan_order.size(); i++) {
                if (plan_order[i] == ag_final->id) {
                    plan_order.erase(plan_order.begin() + i);
                    break;
                }
            }
            c++;
        }
        if (!TestConstraints()) //test correctness
        {
            printf("TestConstraints ERROR\n");
            while (1);
        }
        // fout<<timestep <<","<<tot<<","<<fitness_m<<","<< num_conflito_CBS<< ","<< num_conflito_CBS_total <<","<<list_taskset.size()<<","<<atraso_timestep<<endl;
        //  cout<<timestep <<" finish: "<<tot<<" fitness_m: "<<fitness_m<<" n conflitos: "<< num_conflito_CBS<< " Atraso TImestep: "<<atraso_timestep<<endl;
        this->num_conflito_CBS = 0;

    }
}

void Simulation::run_CBS_heuristic() {

    int sumofcost = 0;
    vector<bool> last_vis(agents.size(), false);
    int tot = 0;
    int aux = 1;
    bool adc = false;
    vector<vector<int>> task_agents;
    list_agents = heuristic(agents, list_taskset);
//    string res_file = "/home/carol/Desktop/MAPD/Resultados/CBS-Heuristic-" +to_string(agents.size()) + "-" +to_string(this->freq)+ "-"+to_string(this->seed)+".csv";
//    ofstream fout(res_file, ios::app);
//    if (!fout) cout << "ERR0";

    for (timestep = 0; true; timestep++) {
#ifdef TIMESTEP

        // cout << "Timestep " << timestep << " " << tot << endl;

#endif

        if (timestep > t_task) {
            bool finish = true;
            for (int i = 0; i < agents.size(); i++)
                if (list_agents[i].size() > 0) finish = false;
            if (finish) {
                // printf("%d\n", tasks_total.size());
                // cout << "MAKESPAN " << timestep - 1 << endl;

                break;
            }
        }

        for (int i = aux; i <= timestep; i++) {
            if (taskset[i].empty())continue;
            if (i >= taskset.size()) break;
            aux = i + 1;
            for (vector<Task>::iterator it = taskset[i].begin(); it != taskset[i].end(); it++) {
                list_taskset.push_back(&(*it));
                adc = true;
            }
        }
        vector<int> need_plan;


        for (int i = 0; i < agents.size(); i++) agents[i].loc = agents[i].path[timestep];


        if (adc) {
            if (!list_taskset.empty()) {
                vector<Agent> copy_agents = agents;
                for (int i = 0; i < agents.size(); i++) {
                    if (agents[i].delivering) {
                        int t = timestep;
                        while (agents[i].path[t] != agents[i].next_ep->loc) t++;
                        copy_agents[i].finish_time = t;
                        copy_agents[i].loc = agents[i].path[t];
                        continue;
                    }
                    copy_agents[i].loc = agents[i].path[timestep];
                    copy_agents[i].finish_time = timestep;
                }

                task_agents = heuristic(copy_agents, list_taskset);
                adc = false;


                for (int i = 0; i < agents.size(); i++) {
                    if (agents[i].delivering) {
                        int id_task = list_agents[i].front();
                        list_agents[agents[i].id].clear();
                        list_agents[agents[i].id].push_back(id_task);
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    } else {
                        //se nao tem tarefas a serem adicionadas para o agente eu limpo a lista dele, e faco ele ficar na posicao que ele esta ate o final;
                        //para nao repetir tarefas
                        if (!task_agents[i].size()) {
                            agents[i].next_ep = nullptr;
                            need_plan.push_back(agents[i].id);
                            list_agents[agents[i].id].clear();
                            continue;
                        }
                        int id_task = task_agents[i].front();
                        Task *task = &tasks_total[id_task];
                        if (agents[i].next_ep == nullptr) {
                            need_plan.push_back(agents[i].id);
                            for (int j = 0; j < task_agents[i].size(); j++)
                                list_agents[agents[i].id].push_back(task_agents[i][j]);
                        } else if (agents[i].next_ep->loc != task->start->loc) {
                            list_agents[agents[i].id].clear();
                            need_plan.push_back(agents[i].id);
                            for (int j = 0; j < task_agents[i].size(); j++)
                                list_agents[agents[i].id].push_back(task_agents[i][j]);

                        } else if (agents[i].next_ep->loc == task->start->loc) {
                            list_agents[agents[i].id].clear();
                            for (int j = 0; j < task_agents[i].size(); j++)
                                list_agents[agents[i].id].push_back(task_agents[i][j]);
                        }
                    }
                }
            }
        }
        if (timestep == 0) {
            for (int i = 0; i < agents.size(); i++) need_plan.push_back(i);
        }
        int atraso = 0, atraso_ag = 0, atraso_timestep = 0;
        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (agents[id].loc == task->goal->loc && task->delivering == true && timestep == task->ag_arrive_goal) {
                tot++;
                task->agent_id = task->agent->id;
                task->delivering = false;
                task->agent->delivering = false;
                agents[id].finish_time = timestep;
                task->agent->next_ep = nullptr;
                task->agent->task = nullptr;
                task->atraso_goal = (task->ag_arrive_goal - task->ag_arrive_start) - task->dist;
                list_agents[i].erase(list_agents[i].begin());
#ifdef SIMU
                if (id == AG || AG == -1) {
                      cout << " Agente " << id << " termina tarefa " << task->id << " no tempo " << task->ag_arrive_goal << " na loc " << agents[id].loc <<" Atraso: "<< task->atraso_goal<< endl;

                }
#endif
                atraso = task->atraso_goal;
                atraso_timestep = atraso_timestep + atraso;
                need_plan.push_back(i);
            }
        }

        vector<Agent *> ag_icbs;
        vector<bool> is_conflict(agents.size(), true);
        for (int j = 0; j < need_plan.size(); j++) {
            int id = need_plan[j];
            if (!list_agents[id].size()) continue;
            int id_task = list_agents[id].front();
            Task *task = &tasks_total[id_task];
            agents[id].min_cost = Dis[agents[id].loc][task->start->loc];
            agents[id].start_task = timestep;
            if (agents[id].loc != task->start->loc || timestep < task->release_time) {
                list_agents[id] = list_agents[id];
#ifdef SIMU
                if (id == AG || AG == -1) {
                    cout << " Agente " << agents[id].id << " comeca tarefa " << task->id << " vai da posicao loc " << agents[id].loc << " para coleta loc " << task->start->loc <<" dis: "<<Dis[agents[id].loc][task->start->loc]<<endl;

                }
#endif
                agents[id].min_cost = Dis[agents[id].loc][task->start->loc];
                agents[id].start_task = timestep;
                agents[id].next_ep = task->start;
                ag_icbs.push_back(&agents[id]);
                is_conflict[id] = false;
            }
        }


        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (task->delivering == false && timestep >= task->release_time && agents[id].loc == task->start->loc) {
                atraso_ag = (timestep - agents[id].start_task) - agents[id].min_cost;
                atraso_timestep = atraso_timestep + atraso_ag;
                task->ag_arrive_start = timestep;
                agents[id].task = task;
                agents[id].next_ep = task->goal;
                agents[id].delivering = true;
                ag_icbs.push_back(&agents[id]);
                deleteTask_id(task->id);
                task->agent = &agents[id];
                task->delivering = true;
                is_conflict[id] = false;
#ifdef SIMU
                if (id == AG || AG == -1) {
                    cout << " Agente " << agents[id].id << " da tarefa " << task->id << " terminou coleta na loc " << agents[id].loc << " no tempo " << timestep << " esta indo para entrega " << task->goal->loc <<" Atraso: "<<atraso_ag<< endl;

                }
#endif
            }
        }

        if (!ag_icbs.empty()) //path finding
        {
            for (int i = 0; i < agents.size(); i++) {
                if (!list_agents[i].size()) continue;
                int id_task = list_agents[i].front();
                Task *task = &tasks_total[id_task];
                int id = i;
                bool flag = true;
                for (int j = 0; j < ag_icbs.size(); j++)
                    if (ag_icbs[j]->id == id)
                        flag = false;
                if (flag) {
                    is_conflict[id] = false;
                    if (agents[id].next_ep == nullptr) {
                        cout << " Agente " << agents[id].id << " com next_ep nulo comeca tarefa " << task->id
                             << " e vai para posicao coleta " << task->start->loc << endl;
                        agents[id].start_task = timestep;
                        agents[id].min_cost = Dis[agents[id].loc][task->start->loc];
                        agents[id].next_ep = task->start;
                    }
                    ag_icbs.push_back(&agents[id]);
                }
            }

            vector<vector<int>> cons_paths;
            for (int i = 0; i < agents.size(); i++)
                if (is_conflict[i])
                    cons_paths.push_back(agents[i].path);
            if (!PathFinding(ag_icbs, cons_paths))
                return;
        }
        if (!TestConstraints()) //test correctness
        {
            printf("TestConstraints ERROR\n");
            while (1);
        }
        // fout<<timestep <<","<<tot<<","<<fitness_m<<","<< num_conflito_CBS<< ","<< num_conflito_CBS_total <<","<<list_taskset.size()<<","<<atraso_timestep<<endl;
        //  cout<<timestep <<" finish: "<<tot<<" fitness_m: "<<fitness_m<<" n conflitos: "<< num_conflito_CBS<< " Atraso TImestep: "<<atraso_timestep<<endl;
        this->num_conflito_CBS = 0;

    }
}

void Simulation::run_AS_heuristic(bool bestInd) {
    int aux = 1;
    Agent *ag;
    int tot = 0;
    vector<vector<int>> task_agents;
    list_agents = heuristic(agents, list_taskset);
    bool adc = false;

    for (timestep = 0; true; timestep++) {
#ifdef TIMESTEP
        cout << "Timestep " << timestep << " " << tot << endl;
#endif
        if (timestep > t_task) {
            bool finish = true;
            for (int i = 0; i < agents.size(); i++)
                if (!list_agents[i].empty()) finish = false;
            if (finish) {
                for (auto &agent : agents) {
                    ag = &agent;
                    tempoTotalPathPlan = ag->timepathPlan + tempoTotalPathPlan;
                }

                break;
            }
        }


        for (int i = aux; i <= timestep; i++) {
            if (taskset[i].empty())continue;
            if (i >= taskset.size()) break;
            aux = i + 1;
            for (vector<Task>::iterator it = taskset[i].begin(); it != taskset[i].end(); it++) {
                list_taskset.push_back(&(*it));
                adc = true;
            }
        }

        vector<int> need_plan;
        vector<int> plan_order;

        if (timestep == 0) {
            for (int i = 0; i < agents.size(); i++)
                need_plan.push_back(i);
        }
        for (auto &agent : agents) agent.loc = agent.path[timestep];

        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            ag = &agents[i];
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (task->delivering == false && timestep >= task->release_time && agents[id].loc == task->start->loc &&
                task->id == ag->task->id) {
#ifdef TIMESTEP
                if(ag->id == AG || AG == -1){
            //        cout << " Agente " << agents[id].id << " da tarefa " << task->id << " terminou coleta na loc "  << agents[id].loc << " no tempo " << timestep << " esta indo para entrega " << task->goal->loc << endl;

                }
#endif
                task->ag_arrive_start = timestep;
                agents[id].delivering = true;
                deleteTask_id(task->id);
                task->agent = &agents[id];
                task->delivering = true;

            }
        }

        if (adc) {
            vector<Agent> copy_agents = agents;

            for (auto &agente_copy : copy_agents) {
                if (agente_copy.delivering) {
                    agente_copy.loc = agente_copy.path[agente_copy.finish_time];
                    continue;
                }
                agente_copy.loc = agente_copy.path[timestep];
                agente_copy.finish_time = timestep;
            }

            task_agents = heuristic(agents, list_taskset);
            adc = false;

            for (int i = 0; i < agents.size(); i++) {

                if (agents[i].delivering) {
                    int id_task = list_agents[i].front();
                    list_agents[agents[i].id].clear();
                    list_agents[agents[i].id].push_back(id_task);
                    for (int j = 0; j < task_agents[i].size(); j++)
                        list_agents[agents[i].id].push_back(task_agents[i][j]);
                } else {
                    //se nao tem tarefas a serem adicionadas para o agente eu limpo a lista dele, e faco ele ficar na posicao que ele esta ate o final;
                    //para nao repetir tarefas
                    if (!task_agents[i].size()) {
                        agents[i].next_ep = nullptr;
                        need_plan.push_back(agents[i].id);
                        list_agents[agents[i].id].clear();
                        ag = &agents[i];
                        int finish = ag->planFastPath(ag->loc, timestep, token);
                        if (finish == -1) {
                            printf("A Star ERROR\n");
                            while (1);
                        }
                        for (int j = timestep; j < token.path[i].size(); j++) {
                            token.path[i][j] = agents[i].path[j];
                        }
                        continue;
                    }
                    int id_task = task_agents[i].front();
                    Task *task = &tasks_total[id_task];
                    if (agents[i].next_ep == nullptr) {
                        need_plan.push_back(agents[i].id);
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    } else if (agents[i].next_ep->loc != task->start->loc) {
                        list_agents[agents[i].id].clear();
                        need_plan.push_back(agents[i].id);
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    } else if (agents[i].next_ep->loc == task->start->loc) {
                        list_agents[agents[i].id].clear();
                        for (int j = 0; j < task_agents[i].size(); j++)
                            list_agents[agents[i].id].push_back(task_agents[i][j]);
                    }
                }
            }
        }


        for (int i = 0; i < agents.size(); i++) {
            if (!list_agents[i].size()) continue;
            int id_task = list_agents[i].front();
            Task *task = &tasks_total[id_task];
            int id = i;
            if (agents[id].loc == task->goal->loc && task->delivering == true && timestep == task->ag_arrive_goal) {
#ifdef TIMESTEP
                if(agents[id].id == AG || AG == -1) {

                    cout << " Agente " << id << " termina tarefa " << task->id << " no tempo " << task->ag_arrive_goal
                         << " na loc " << agents[id].loc << endl;
                }
#endif
                tot++;
                task->agent_id = task->agent->id;
                task->delivering = false;
                task->agent->delivering = false;
                agents[id].finish_time = timestep;
                task->agent->next_ep = nullptr;
                task->agent->task = nullptr;
                list_agents[i].erase(list_agents[i].begin());
                need_plan.push_back(agents[i].id);
            }
        }

        for (int j = 0; j < need_plan.size(); j++) {
            int id = need_plan[j];
            ag = &agents[id];
            if (!list_agents[id].size()) continue;
            int id_task = list_agents[id].front();
            Task *task = &tasks_total[id_task];
            if (agents[id].loc != task->start->loc || timestep < task->release_time || agents[id].delivering == false) {
                agents[id].next_ep = task->start;
                agents[id].task = task;
#ifdef TIMESTEP
                if(ag->id == AG || AG == -1) {
                    cout << " Agente " << agents[id].id << " comeca tarefa " << task->id << " vai da posicao loc "
                         << agents[id].loc << " para coleta loc " << task->start->loc << " dis: "
                         << Dis[agents[id].loc][task->start->loc] << endl;
                }
#endif
                plan_order.push_back(id);
            }
        }

        int c = 0;
        int tam = plan_order.size();
        while (c < tam) {
            int maior = 0;
            Agent *ag_final;
            for (int j = 0; j < plan_order.size(); j++) {
                int id = plan_order[j];
                ag = &agents[id];
                if (!list_agents[id].size()) continue;
                int id_task = list_agents[id].front();
                Task *task = &tasks_total[id_task];
                if (Dis[ag->loc][ag->next_ep->loc] >= maior) {
                    maior = Dis[ag->loc][ag->next_ep->loc];
                    ag_final = ag;
                }
            }
            int id_task = list_agents[ag_final->id].front();
            Task *task = &tasks_total[id_task];
            int finish = ag_final->planFastPath(ag_final->loc, timestep, token);
            if (ag_final->loc == ag_final->task->start->loc) {
                task->ag_arrive_start = timestep;
                ag_final->delivering = true;
                deleteTask_id(task->id);
                task->agent = ag_final;
                task->delivering = true;
            }
            agents[ag_final->id].finish_time = finish;
            task->ag_arrive_goal = finish;
            if (finish == -1) {
                printf("A Star ERROR\n");
                while (1);
            }
            for (int i = 0; i < plan_order.size(); i++) {
                if (plan_order[i] == ag_final->id) {
                    plan_order.erase(plan_order.begin() + i);
                    break;
                }
            }
            c++;
        }

        if (!TestConstraints()) //test correctness
        {
            printf("TestConstraints ERROR\n");
            while (1);
        }
    }

}

int Simulation::SingleFindPath(Agent *ag, int t) {
    int *moves_offset; // = new int[5];
    moves_offset = new int[MapLoader::MOVE_COUNT];
    moves_offset[MapLoader::valid_moves_t::WAIT_MOVE] = 0;
    moves_offset[MapLoader::valid_moves_t::NORTH] = -col;
    moves_offset[MapLoader::valid_moves_t::EAST] = 1;
    moves_offset[MapLoader::valid_moves_t::SOUTH] = col;
    moves_offset[MapLoader::valid_moves_t::WEST] = -1;
    bool *mymap = new bool[my_map.size()];
    for (int j = 0; j < my_map.size(); j++) {
        if (my_map[j]) {
            mymap[j] = false;
        } else {
            mymap[j] = true;
        }
    }
    vector<vector<int> > cons_paths;
    for (int i = 0; i < agents.size(); i++)
        if (agents[i].id != ag->id)
            cons_paths.push_back(agents[i].path);
    int start_loc = ag->path[t];
    int goal_loc = ag->next_ep->loc;
    int park_loc = ag->park_loc;
    ComputeHeuristic ch(start_loc, goal_loc, mymap, row, col, moves_offset, nullptr, 1.0, nullptr);
    ComputeHeuristic ch1(goal_loc, park_loc, mymap, row, col, moves_offset, nullptr, 1.0, nullptr);
    SingleAgentICBS singleicbs(start_loc, goal_loc, park_loc, mymap, my_map.size(),
                               moves_offset, col, cons_paths, t, maxtime, ag->release_time - t);
    ch.getHVals(singleicbs.my_heuristic);
    ch1.getHVals(singleicbs.my_heuristic1);
    vector<PathEntry> path;
    int goal_length;
    printf("%d %d\n", start_loc, goal_loc);
    bool find_flag = singleicbs.findPathPrioritized(true, path, goal_length, 1.0, nullptr, nullptr, 0, 0);
    if (find_flag) {
        for (int j = 0; j < path.size(); j++)
            ag->path[t + j] = path.at(j).location;
        //hold endpoint
        for (unsigned int j = path.size() + t; j < maxtime; j++)
            ag->path[j] = ag->park_loc;
        return t + goal_length - 1;
    } else {
        bool find_flag = singleicbs.findPathPrioritized(false, path, goal_length, 1.0, nullptr, NULL, 0, 0);
        if (!find_flag) {
            printf("Error!\n");
            while (1);
        } else {
            for (int j = 0; j < path.size(); j++)
                ag->path[t + j] = path.at(j).location;
            //hold endpoint
            for (unsigned int j = path.size() + t; j < maxtime; j++)
                ag->path[j] = ag->park_loc;
        }
        return t + goal_length - 1;
    }
    //delete singleicbs;
    delete[] moves_offset;
    delete[] mymap;
}

vector<vector<int>> Simulation::heuristic(vector<Agent> copy_agents, vector<Task *> taskset) {
    int menor = INT_MAX;
    int id_agent;
    int id_task;
    int loc_i, loc_f, c = 0;
    vector<Task *> copy_taskset = taskset;
    vector<Gene> genes;
    vector<Agent> agents = copy_agents;
    vector<vector<int>> list_task_agents;
    list_task_agents.resize(num_agents);
    int num_task = copy_taskset.size();

    while (c < num_task) {
        for (int i = 0; i < copy_taskset.size(); i++) {
            for (int j = 0; j < num_agents; j++) {
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
        int id = copy_taskset[id_task]->id;
        list_task_agents[id_agent].push_back(id);
        //  cout<<"Agente "<<id_agent<<" com tarefa "<<id;
        c++;
        agents[id_agent].finish_time = Dis[agents[id_agent].loc][copy_taskset[id_task]->start->loc] +
                                       Dis[copy_taskset[id_task]->start->loc][copy_taskset[id_task]->goal->loc] +
                                       agents[id_agent].finish_time;
        agents[id_agent].loc = copy_taskset[id_task]->goal->loc;
        //  cout<<" tempo final "<<copy_agents[id_agent].finish_time<<endl;
        copy_taskset.erase(copy_taskset.begin() + id_task);
    }


    Task n;
    int start = 0, makespan = 0, soc = 0;
    int st = 0;
    for (int i = 0; i < list_task_agents.size(); i++) {
        // cout<<endl<<"Agente "<<i;
        start = 0;
        int loc = copy_agents[i].loc;
        int time = copy_agents[i].finish_time;
        for (int j = 0; j < list_task_agents[i].size(); j++) {
            n = tasks_total[list_task_agents[i][j]];
            //  cout<<endl<<" pega tarefa "<<n.id <<" start "<<n.start->loc<<" goal "<<n.goal->loc<<endl;
            if (j == 0) {
                //int release_time = n.release_time;
                int dist = n.start->h_val[loc];
                start = time + dist;
                loc = n.start->loc;
                //cout<<" vai pra start "<<loc<<" no tempo "<<start;
                if (j + 1 != list_task_agents[i].size()) continue;
            } else if (j <= list_task_agents[i].size() - 1) {
                Task n1 = tasks_total[list_task_agents[i][j - 1]];
                Task n2 = tasks_total[list_task_agents[i][j]];
                int release_time = n2.release_time;
                int dist_t1 = n1.goal->h_val[loc];
                st += dist_t1 + start - n1.release_time;
                loc = n1.goal->loc;
                // cout<<" vai pra goal "<<loc<<" no tempo "<<start + dist_t1;
                int dist_t2 = n2.start->h_val[loc];
                loc = n2.start->loc;
                start = start + dist_t1 + dist_t2;
                //  cout<<" vai pra start "<<loc<<" no tempo "<<start;
                // start = max(start + dist_t1 + dist_t2, release_time);
                if (j + 1 != list_task_agents[i].size()) continue;

            }
            int dist_t1 = n.goal->h_val[loc];
            start = start + dist_t1;
            st += dist_t1 + start - n.release_time;
            //  cout<<" vai de "<<loc<<" pra goal "<<n.goal->loc<<" no tempo "<<start<<endl;
            // makespan = start;
        }
        //   cout<<endl<<"Agente "<<i<<" st "<<st;
        soc = start + soc;
        if (start > makespan) makespan = start;

    }
    // st = st / qtd_task;
    this->fitness_m = makespan;
    return list_task_agents;
}

Simulation::Simulation(string map, string task, int frequency, int seed, string nome) {
    this->freq = frequency;
    this->seed = seed;
    this->problem_name = nome;
    LoadMap(map);
    BFS();
    LoadTask(task, frequency);

}

void Simulation::LoadMap(string arq) {


    string line;
    ifstream myfile(arq);
    //  cout << arq << endl;
    if (!myfile.is_open()) {
        cout << "Map file not found." << endl;
        system("PAUSE");
        return;
    }

    getline(myfile, line);
    sscanf(line.c_str(), "%d" "%d", &row, &col);

    row = row + 2;
    col = col + 2;
    stringstream ss;
    getline(myfile, line);
    ss << line;
    ss >> num_endpoint; //number of endpoints that may have tasks on. Other endpoints are home endpoints


    ss.clear();
    getline(myfile, line);
    ss << line;
    ss >> num_agents; //agent number

    ss.clear();
    getline(myfile, line);
    ss << line;
    ss >> maxtime; //max timestep


    agents.resize(num_agents);
    token.map.resize(row * col);
    token.agents.resize(num_agents);
    token.path.resize(num_agents);
    token.endpoints.resize(row * col);
    endpoints.resize(num_agents + num_endpoint); //endpoints.resize(workpoint_num + agentes);
    // PathPlanning pp(col);
    this->pp.path_agents.resize(num_agents);
    this->pp.col = col;
    this->pp.seed = this->seed;
    this->pp.problem_name = this->problem_name;

    int ep = 0, ag = 0;
    for (int i = 1; i < row - 1; i++) {
        getline(myfile, line);

        for (int j = 1; j < col - 1; j++) {

            if (line[j - 1] == '@') continue;
            token.map[col * i + j] = true;
            token.endpoints[col * i + j] = (line[j - 1] == 'e') || (line[j - 1] == 'r'); //endpoint
            if (line[j - 1] == 'e') {
                endpoints[ep].loc = i * col + j;
                endpoints[ep].col = j;
                endpoints[ep].row = i;
                ep++;
                //cout << "E[" << j << "," << i << "] "<<i*col+j;

            } else if (line[j - 1] == 'r') { // posicao inicial do robo, endpoint de robo.

                agents[ag].setAgent(i * col + j, col, row, ag);
                token.agents[ag] = &agents[ag];
                token.path[ag].resize(maxtime1);
                this->pp.path_agents[ag].resize(maxtime1);
                endpoints[num_endpoint + ag].loc = i * col + j;
                endpoints[num_endpoint + ag].col = col;
                endpoints[num_endpoint + ag].row = row;
                agents[ag].ep_park_loc = &endpoints[num_endpoint + ag];
                for (int p = 0; p < maxtime1; p++) {
                    token.path[ag][p] = i * col + j;
                    this->pp.path_agents[ag][p].first = i * col + j;
                    this->pp.path_agents[ag][p].second = false;


                }
                ag++;
            }
        }
    }

    myfile.close();


    //coloca falso em volta do mapa, pra nao ser alcancavel
    for (int i = 0; i < row; i++) {
        token.map[i * col] = false;
        token.map[i * col + col - 1] = false;
        token.endpoints[i * col] = false;
        token.endpoints[i * col + col - 1] = false;
    }

    for (int i = 0; i < col - 1; i++) {
        token.map[i] = false;
        token.map[row * col - col + i] = false;
        token.endpoints[i] = false;
        token.endpoints[row * col - col + i] = false;
    }
    my_map = token.map;

    //inicializa matriz com todos os h-values
    for (int e = 0; e < endpoints.size(); e++) {
        endpoints[e].setH_val(token.map, col);
        endpoints[e].id = e;
    }
    pp.map = token.map;
}

void Simulation::LoadTask(string arq, float frequency) {

    string line;
    ifstream myfile(arq);
    if (!myfile.is_open()) {
        cout << "Task file not found." << endl;
        system("PAUSE");
        return;
    }

    stringstream ss;
    getline(myfile, line);
    ss << line;
    ss >> sizeTaskset;

    ss.clear();
    getline(myfile, line);
    ss << line;
    ss >> num_task;

    taskset.resize(num_task);

    for (int i = 0; i < sizeTaskset; i++) {

        int t, s, g, ts, tg;
        ss.clear();
        getline(myfile, line);
        ss << line;
        ss >> t >> s >> g >> ts >> tg;
        int dist = Dis[endpoints[s].loc][endpoints[g].loc];
        Task novo = Task(i, t, &endpoints[s], &endpoints[g], ts, tg, dist);
//        cout<<"TASK START: "<<novo.start->id<<" loc:"<<novo.start->loc<<" row:"<<novo.start->row<<" col:"<<novo.start->col<< endl;
//        cout<<"TASK GOAL: "<<novo.goal->id<<" loc:"<<novo.goal->loc<<" row:"<<novo.goal->row<<" col:"<<novo.goal->col<< endl;
//        cout<<novo.id<<"\t" <<novo.release_time<<"\t"<<novo.start->loc<<"\t"<<novo.goal->loc<< endl;
        t_task = t;
        taskset[t].push_back(novo);
        tasks_total.push_back(Task(i, t_task, &endpoints[s], &endpoints[g], ts, tg, dist));


    }
    myfile.close();

    //  for(int i =0 ; i <= t_task; i++){
    if (!taskset[0].empty()) {
        for (vector<Task>::iterator it = taskset[0].begin(); it != taskset[0].end(); it++) {
            list_taskset.push_back(&(*it));
        }
    }

    //}


//    if (!taskset[0].empty()) {
//        for (vector<Task>::iterator it = taskset[0].begin(); it != taskset[0].end(); it++) {
//            token.taskset.push_back(&(*it));
//        }
//    }

}

void Simulation::run_TP() {

    Agent *ag;
    int t_f = 0;
    ag = &agents[0];
    while (!token.taskset.empty() || token.timestep <= t_task) {
        ag = &agents[0];
        for (auto & agent : agents) {
            if (agent.finish_time == token.timestep) {
                ag = &agent;
                break;
            } else if (agent.finish_time < ag->finish_time) {
                ag = &agent;
            }
        }
        for (int i = token.timestep + 1; i <= ag->finish_time; i++) {
            t_f = i;
            if (i >= taskset.size()) {
                break;
            }
            if (taskset[i].empty()) continue;

            for (auto it = taskset[i].begin(); it != taskset[i].end(); it++) {
                token.taskset.push_back(&(*it));
            }
        }


        token.timestep = ag->finish_time;
        ag->loc = ag->path[token.timestep];

        if (token.taskset.empty())//If no new tasks
        {
            ag->finish_time = ag->finish_time + 1; //agent waits for one timestep
            continue;
        }

        time++;

        clock_t start = clock();
        if (!ag->TP(token)) {
            cerr << "Nao pegou tarefas" << endl;
            system("PAUSE");

        }
        computation_time += clock() - start;
        if (!testCollision()) {
            system("PAUSE");
            return;
        }


    }
}

void Simulation::SavePath() {}

void Simulation::printPaths() {
    cout << endl;
    for (size_t t = 0; t < token.path.size(); t++) {
        cout << "AGENT " << t << " ";
        for (size_t i = 0; i < token.path[t].size(); i++) {
            // cout <<" TEMPO " << i << " Path: ";
            cout << i << ":" << token.path[t][i] << " ";

        }
        cout << endl;
    }
}

void Simulation::printMap() {
    cout << endl;
    for (size_t t = 0; t < row; t++) {
        for (size_t i = 0; i < col; i++) {
            cout << " " << token.map[col * t + i] << " [" << (col * t + i) << "] ";

        }
        cout << endl;
    }
}

void Simulation::printTasks() {
    for (int i = 0; i < sizeTaskset; i++) {
        cout << "ID: " << tasks_total[i].id << " t: " << tasks_total[i].release_time << " " << tasks_total[i].start->loc
             << " -> " << tasks_total[i].goal->loc << endl;
    }
}

void Simulation::SaveTask(string file, string arq) {

    ofstream fout(file, ios::app);
    if (!fout) return;

    fout << "SCORE" << "," << arq << endl;
    for (int i : score) {
        fout << i << ",";

    }
    fout.close();

    string file1 = file + "lista-agent";

    ofstream f(file1, ios::app);
    if (!f) return;

    f << "LISTA AGENTS" << "," << arq << endl;
    for (int i = 0; i < list_agents.size(); i++) {
        f << "Agente " << i << ": ";

        for (int j : list_agents[i]) {
            f << j << ",";

        }
        f << endl;

    }
    f.close();
}

void Simulation::showTask(string arq, float t, int c, string tam) {

    unsigned int WaitingTime = 0;
    float LastFinish = 0;
    for (unsigned int i = 0; i < taskset.size(); i++) {
        if (!taskset[i].empty()) {
            for (auto it = taskset[i].begin(); it != taskset[i].end(); it++) {
                WaitingTime += it->ag_arrive_goal - it->release_time;
                LastFinish = LastFinish > it->ag_arrive_goal ? LastFinish : it->ag_arrive_goal;
            }
        }
    }

    auto comp = computation_time / LastFinish;
    cout << endl << "Finishing Timestep:	" << LastFinish << endl;
    cout << "Sum of Task Waiting Time:	" << WaitingTime << endl;
    cout << "Computation time:	" << comp << endl;
    cout << "Duration:	" << t << endl;


    ofstream fout(arq, ios::app);
    if (!fout) return;

    fout << tam << "," << c << "," << LastFinish << "," << WaitingTime << "," << comp << "," << WaitingTime / num_task
         << "," << t << endl;
    fout.close();


}

bool Simulation::testCollision() {

    for (unsigned int ag = 0; ag < agents.size(); ag++) {
        for (unsigned int i = ag + 1; i < agents.size(); i++) {
            //j=0;
            for (int j = token.timestep + 1; j < maxtime; j++) {
                if (agents[ag].path[j] == agents[i].path[j]) {
                    cerr << "Agent " << ag << " e " << i << " colide no lugar "
                         << agents[ag].path[j] << " no tempo " << j << endl;

                    cout << " " << ag << "\t" << i << endl;
                    for (int t = 0; t < j; t++) {
                        cout << t << " - " << token.path[ag][t] << "\t" << agents[i].path[t] << endl;
                    }
                    return false;
                } else if (token.timestep > 0 && agents[ag].path[j] == agents[i].path[j - 1]
                           && agents[ag].path[j - 1] == agents[i].path[j]) {
                    cerr << "Agent " << ag << " e " << i << " colide no edge "
                         << agents[ag].path[j - 1] << "-" << agents[ag].path[j] << " no tempo " << j << endl;
                    cout << ag << "\t" << i << endl;
                    for (int t = 0; t < j; t++) {
                        cout << t << " - " << token.path[ag][t] << "\t" << agents[i].path[t] << endl;
                    }
                    return false;
                }

            }
        }
    }
    return true;
}

void Simulation::showPathAgents(const string& arq) {

    string file = arq + ".path";
    ofstream fout(file, ios::app);
    if (!fout) return;

    for (int i = 0; i < token.path.size(); i++) {
        fout << "AG " << i << " - Tempo final:" << agents[i].finish_time << endl;
        for (int j = 0; j <= agents[i].finish_time; j++) {
            //  cout << j <<" ";
            fout << token.path[i][j] << "\t";
        }
        fout << endl << endl;
    }
    fout.close();


}

int Simulation::getShortTime() {

    int id = 0;
    int short_time = agents[0].finish_time;
    for (auto & agent : agents) {
        if (agent.finish_time < short_time) {
            short_time = agent.finish_time;
            id = agent.id;
        }
    }
    //  cout<<endl<<"Agent "<<id<<" no menor tempo "<<short_time;
    return short_time;
}

void Simulation::run_HungarianMethod() {

    BFS();
    GA ga;
    int t_f = 1;

    while (!list_taskset.empty() || t_f <= t_task) {
        Agent *ag;
        int ti = getShortTime();
        if (ti >= t_task) {
            ti = t_task;
        }
        for (int i = t_f; i <= ti; i++) {
            if (taskset[i].empty())continue;
            if (i >= taskset.size()) break;
            t_f = i + 1;
            for (vector<Task>::iterator it = taskset[i].begin(); it != taskset[i].end(); it++) list_taskset.push_back(&(*it));
        }
        if (list_taskset.empty()) {
            for (int i = 0; i < agents.size(); i++) {
                if (agents[i].finish_time < ag->finish_time) {
                    ag = &agents[i];
                }
            }
            //   std::cout << endl << " Agent " << ag->id << " espera um timestep  " << ag->finish_time + 1;
            ag->finish_time = ag->finish_time + 1;
            continue;
        }
        cout << "Tamanho " << list_taskset.size() << endl;

        int size = fmax(list_taskset.size(), agents.size());
        int min_size = fmin(list_taskset.size(), agents.size());

        dlib::matrix<int> cost(size, size);

        for (int i = 0; i < size; i++) {
            cout << " " << i;
            for (int j = 0; j < size; j++) {
                if (i < agents.size() && j < list_taskset.size()) {
                    cost(i, j) = -Dis[agents[i].loc][list_taskset[j]->start->loc];
                } else {
                    cost(i, j) = -1000000;
                }
                cout << " [" << cost(i, j) << "]";
            }
            cout << endl;
        }
        vector<long> assignment = max_cost_assignment(cost);

        int c = 0;
        list_agents.resize(agents.size());

        while (c < min_size) {
            int id_agent = agents[c].id;
            int id_task = list_taskset[assignment[c]]->id;
            list_agents[id_agent].push_back(id_task);
            cout << " Agente " << id_agent << " tarefa-> " << id_task << endl;
            c++;
        }

        Task *t;
        int id;
        for (int i = 0; i < list_agents.size(); i++) {
            ag = &agents[i];
            int loc = ag->loc;
            int tempo = ag->finish_time;
            cout << "Agente: " << ag->id << " loc: " << ag->loc << " finish time: " << tempo;
            for (int j = 0; j < list_agents[i].size(); j++) {
                id = list_agents[i][j];
                t = &tasks_total[id];
                cout << endl << "\t pega tarefa " << id << ": ";
                tempo = Dis[loc][t->start->loc] + tempo;
                cout << loc << " -> " << t->start->loc << " [" << tempo << "] " << " -> " << t->goal->loc;
                tempo = Dis[t->start->loc][t->goal->loc] + tempo;
                cout << " [" << tempo << "] ";

                loc = t->goal->loc;
                deleteTask_id(id);
            }
            ag->loc = loc;
            ag->finish_time = tempo;
            token.agents[ag->id]->finish_time = tempo;
            token.agents[ag->id]->loc = loc;
            cout << " Tempo final = " << tempo << endl;

        }
        list_agents.clear();
    }
}

int Simulation::nearestAgent() {

    vector<int> dist;
    int dist_atual = maxtime;
    Agent ag;
    Task *task = nullptr;
    task = token.taskset.front();
    dist = task->start->h_val;
    for (auto & agent : agents) {

        int dist_ag = dist[agent.loc];
        if (agent.finish_time + dist_ag < dist_atual) {
            ag = agent;
            dist_atual = agent.finish_time + dist_ag;
        }

    }
    return ag.id;

}

void Simulation::readTourFile(string tourFile) {

    BFS();

    this->tour_file = tourFile;
    int task_cnt = sizeTaskset;
    int agent_cnt = agents.size();
    int node_cnt = task_cnt + agent_cnt;

    freopen(tour_file.c_str(), "r", stdin);
    char st[20];
    while (strcmp(st, "TOUR_SECTION"))
        scanf("%s\n", st);
    queue<Task *> seq;
    int seq_id = 0, last = 1, t = 0, loc = agents[0].park_loc;
    TSP_agent.push_back(0);
    for (int i = 0; i < node_cnt; i++) {
        int u;
        scanf("%d", &u);
        if (u <= agent_cnt) {
            if (u != 1) {
                TSP_seqs.push_back(seq);
                TSP_len.push_back(t);
                TSP_agent.push_back(u - 1);
                // printf("Makespan: %d Task num: %d\n", t, seq.size());
                while (seq.size() > 0) seq.pop();
                seq_id++;
                t = 0;
                loc = agents[u - 1].park_loc;
            }
        } else {
            Task *task = &tasks_total[u - agent_cnt - 1];
            t += Dis[loc][task->start->loc];
            if (t < task->release_time)
                t = task->release_time;
            t += Dis[task->start->loc][task->goal->loc];
            loc = task->goal->loc;
            tasks_total[u - agent_cnt - 1].seq_id = seq_id;
            seq.push(&tasks_total[u - agent_cnt - 1]);
        }
        last = u;
    }
    //  printf("Makespan: %d Task num: %d\n", t, seq.size());
    TSP_len.push_back(t);
    TSP_seqs.push_back(seq);
    fclose(stdin);

}

void Simulation::BFS() {
    for (int i = 0; i < my_map.size(); i++) {
        vector<int> D;
        D.resize(my_map.size(), INF);
        if (my_map[i]) {
            D[i] = 0;
            queue<int> Q;
            Q.push(i);
            while (!Q.empty()) {
                int u = Q.front();
                Q.pop();
                int offset[4] = {-1, 1, -col, col};
                for (int j : offset) {
                    int v = u + j;
                    if (0 <= v && v < my_map.size() && abs(v % col - u % col) < 2 && my_map[v] && D[v] > D[u] + 1) {
                        D[v] = D[u] + 1;
                        Q.push(v);
                    }
                }
            }
        }
        Dis.push_back(D);
    }
}

int Simulation::mkspn() const {

    return timestep - 1;
}

float Simulation::service_time() {

    // Consequently, the effectiveness of a MAPD algorithm is evaluated by the average number of timesteps, called service time, needed to finish executing each task after it was added to the task set.
    if(this->mkspn() == -1) {
        return -1;
    }
    float WaitingTime = 0;
    int atraso =0;
    for (unsigned int i = 0; i < tasks_total.size(); i++) {
        // cout<<"Task: "<<tasks_total[i].id<<" arive: "<<tasks_total[i].ag_arrive_goal<<endl;
        WaitingTime += tasks_total[i].ag_arrive_goal - tasks_total[i].release_time;
        atraso += tasks_total[i].atraso_goal;
    }
    //   cout<<"WaitingTime: "<<WaitingTime<<" - "<<tasks_total.size()<<endl;
    return WaitingTime/tasks_total.size();
}
void Simulation::deleteTask_id(int id) {

    for (int i = 0; i < list_taskset.size(); i++) {
        if (list_taskset[i]->id == id) {
            list_taskset.erase(list_taskset.begin() + i);
            return;
        }
    }

}

bool Simulation::check_reachable(int id, int loc_finish, int tempo) {

    Agent *ag;
    for (int i = 0; i < num_agents; i++) {
        if (id == i) continue;
        if (loc_finish == agents[i].holding.first) {
            ag = &agents[i];
            //  cout<<"Agente " <<id<<" nao vai conseguir chegar no final na loc "<< loc_finish<<" -- Ag: " <<i<<" TEMPO: "<<agents[i].finish_time<<" DIS: "<<Dis[agents[id].loc][loc_finish] + tempo<<endl;
            pp.Move2EP(ag, agents[i].finish_time, false, token);
            return true;
        }
    }
    return false;
}

int Simulation::planPathAStar1(Agent *ag, int begin_time, Endpoint *goal) {

    clock_t start;
    double duration = 0;
    start = clock();
    bool teste = check_reachable(ag->id, goal->loc, begin_time);
    int arrive_start = pp.Agent_AStar1(ag, ag->loc, goal, begin_time, false, nullptr);
//    cout<<endl<<"Agente: "<<ag->id<<" no tempo "<<begin_time<<" na loc "<<ag->loc<<" indo para coleta "<<ag->task->start->loc<<" arrive_start: "<<arrive_start<<endl;
//    std::cout << "-->" << arrive_start;
//    for (int i = begin_time; i <= arrive_start; i++) {
//        std::cout << "	" << ag->path_principal[i].first;
//    }

    if (arrive_start == -1) {
        cout << "ERRO: Agente "<< ag->id<<" nao encontrou caminho para coleta  - Timestep "<<this->timestep << endl;
        system("PAUSE");
    }
    ag->loc = ag->path[arrive_start];
    bool arrive_hold = pp.Move2EP(ag, arrive_start, true, token);
    if (!arrive_hold) {
        // cout << "ERRO: nao encontrou caminho para coleta" << endl;
        arrive_start = pp.Agent_AStar1(ag, ag->path[begin_time], goal, begin_time, true, nullptr);
        if (arrive_start == 1) {
            cout << "ERROoooooooooooooooo" << endl;
            while (true);
        }
        ag->loc = ag->path[arrive_start];
        arrive_hold = pp.Move2EP(ag, arrive_start, true, token);
        if (!arrive_hold) {
            cout << "EROOOOOOOOOOOOOOOOOOOOOOOOOOO" << endl;
            while (true);
        }
    }

#ifdef TIMESTEP
    if(ag->id == AG || AG == -1){
        // std::cout << endl << " Agent " << ag->id <<" task: "<< ag->task->id <<" begin time: "<<begin_time<< " arrive start: "  << arrive_start<< " arrive hold: "  << arrive_hold<<endl;
    }
#endif
    token.path[ag->id] = ag->path;
    pp.path_agents[ag->id] = ag->path_principal;


    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    this->tempoTotalPathPlan = duration + this->tempoTotalPathPlan;

    return arrive_start;
}

int Simulation::planPathAStar(Agent *ag, int begin_time, Endpoint *goal) {

    clock_t start;
    double duration = 0;
    start = clock();
    bool teste = check_reachable(ag->id, goal->loc, begin_time);
    int arrive_start = pp.Agent_AStar(ag, ag->loc, goal, begin_time, false, nullptr, token);
//    cout<<endl<<"Agente: "<<ag->id<<" no tempo "<<begin_time<<" na loc "<<ag->loc<<" indo para coleta "<<ag->task->start->loc<<" arrive_start: "<<arrive_start<<endl;
//    std::cout << "-->" << arrive_start;
//    for (int i = begin_time; i <= arrive_start; i++) {
//        std::cout << "	" << ag->path_principal[i].first;
//    }

    if (arrive_start == -1) {
        cout << "ERRO: Agente "<< ag->id<<" nao encontrou caminho para coleta  - Timestep "<<this->timestep << endl;
        system("PAUSE");
    }
    ag->loc = ag->path[arrive_start];
//    bool arrive_hold = pp.Move2EP(ag, arrive_start, false, token);
//    if (arrive_hold == false) {
//        // cout << "ERRO: nao encontrou caminho para coleta" << endl;
//        arrive_start = pp.Agent_AStar(ag, ag->path[begin_time], goal, begin_time, true, nullptr);
//        if (arrive_start == 1) {
//            cout << "ERROoooooooooooooooo" << endl;
//            while (1);
//
//        }
//        ag->loc = ag->path[arrive_start];
//        arrive_hold = pp.Move2EP(ag, arrive_start, false, token);
//        if (arrive_hold == false) {
//            cout << "EROOOOOOOOOOOOOOOOOOOOOOOOOOO" << endl;
//            while (1);
//
//        }
//
//    }
    //   cout<<" ficando no goal delivery "<<ag->task->goal->loc<<" arrivi_goal: "<<arrive_hold<<endl;


#ifdef TIMESTEP
    if(ag->id == AG || AG == -1){
      //  std::cout << endl << " Agent " << ag->id <<" task: "<< ag->task->id <<" begin time: "<<begin_time<< " arrive start: "  << arrive_start<< " arrive hold: "  << arrive_hold<<endl;
    }
#endif

    //ag->hold_loc = ag->path[arrive_start+1];
    token.path[ag->id] = ag->path;
    pp.path_agents[ag->id] = ag->path_principal;


    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    this->tempoTotalPathPlan = duration + this->tempoTotalPathPlan;

    return arrive_start;
}

bool Simulation::PathFinding(vector<Agent *> &ags, const vector<vector<int>> &cons_paths) {
    EgraphReader egr;
    constraint_strategy s;
    s = constraint_strategy::ICBS;

    std::clock_t start;
    double duration = 0;
    start = std::clock();
    for (int i = 0; i < ags.size(); i++) {
        ags[i]->goal_loc = ags[i]->next_ep->loc;
        //cout<<" Agente "<<ags[i]->id<<" vai para posicao "<<ags[i]->next_ep->loc<<endl;
        ags[i]->start_time = timestep;
        ags[i]->only_dummy = false;
        ags[i]->cost = CalcCost(ags[i]);
    }
    ICBSSearch icbs(my_map, ags, 1.0, egr, s, col, cons_paths, timestep);
    int res = icbs.runICBSSearch();
    if (res == 1) {
        this->num_conflito_CBS = icbs.HL_num_expanded;
        this->num_conflito_CBS_total = this->num_conflito_CBS + this->num_conflito_CBS_total;
        //update
        for (unsigned int i = 0; i < ags.size(); i++) {
            //update searching path
            for (unsigned int j = 0; j < icbs.paths[i]->size(); j++) {
                ags[i]->path[timestep + j] = icbs.paths[i]->at(j).location;
            }
            //hold endpoint
            for (unsigned int j = icbs.paths[i]->size() + timestep; j < maxtime; j++) {
                ags[i]->path[j] = ags[i]->park_loc;
            }
            //update task
            if (ags[i]->delivering == true) {
                int t = ags[i]->task->ag_arrive_start;
                while (ags[i]->path[t] != ags[i]->next_ep->loc) t++;
                ags[i]->task->ag_arrive_goal = t; //timestep + icbs.paths[i]->size() - 1;
            }
            //agents[i]->task = NULL;
        }
        duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
        tempoTotalPathPlan = duration + tempoTotalPathPlan;
        return true;
    } else if (res == 0) {
        cout << res << endl;
        cout << "CBS fails, timestep: " << timestep << endl;
        timestep = 0;
        return false;
    } else if (res == 3) {
        cout << res << endl;
        cout << "AQUELE ERRO, timestep: " << timestep << endl;
        timestep = -1;
        return false;
    }
}

vector<int> Simulation::CalcCost(Agent *ag) {
    queue<Task *> seq;
    Task *task;
    for (int id_task : list_agents[ag->id]) {
        task = &tasks_total[id_task];
        seq.push(task);


    }
    vector<int> cost;
    for (int t = timestep; t < timestep + 200; t++) {
        queue<Task *> tasks = seq;
        task = tasks.front();
        tasks.pop();
        int loc = task->goal->loc, tt = t;
        while (!tasks.empty()) {
            task = tasks.front();
            tasks.pop();
            tt += Dis[loc][task->start->loc];
            if (tt < task->release_time)
                tt = task->release_time;
            tt += Dis[task->start->loc][task->goal->loc];
            loc = task->goal->loc;
        }
        cost.push_back(tt);
    }
    return cost;

}

void Simulation::printList_taskset() {

    for (int i = 0; i < list_taskset.size(); i++) {
        //  cout<<list_taskset[i]->id<<" "<<list_taskset[i]->start->loc<<" -> "<<list_taskset[i]->goal->loc<<" rt: "<<list_taskset[i]->release_time<<endl;
    }
    for (int i = 0; i < list_agents.size(); i++) {
        Agent *ag = &agents[i];
        cout << endl << "Agente: " << ag->id /*<< " loc: " << ag->loc*/;

        for (int id : list_agents[i]) {
            Task *t = &tasks_total[id];
            cout << endl << "\t pega tarefa " << id << ": ";
        }
    }
    cout << endl;
}

bool Simulation::TestConstraints() {
    int action[5] = {0, 1, -1, col, -col};

    for (int i = 0; i < agents.size(); i++) {
        for (int t = 0; t < maxtime; t++)
            if (!my_map[agents[i].path[t]]) {
                printf("ILLEGAL POS! %d %d\n", i, agents[i].path[t]);
                while (true);
            }
    }
    for (int ag = 0; ag < agents.size(); ag++) {
        for (unsigned int j = timestep + 1; j < maxtime; j++) {
            bool jump = true;
            for (int i = 0; i < 5; i++) {
                if(agents[ag].path[j+1] == agents[ag].path[j]+ action[i]) jump = false;
            }
            if(jump){
                cout<<"Path planning pulando celula"<<endl;
            }
        }
    }
    for (unsigned int ag = 0; ag < agents.size(); ag++) {
        for (unsigned int i = ag + 1; i < agents.size(); i++) {
            for (unsigned int j = timestep + 1; j < maxtime; j++) {
                if (agents[ag].path[j] == agents[i].path[j]) {
                    cout << "Agent " << ag << " and " << i << " collide at location "
                         << agents[ag].path[j] << " at time " << j << endl;
                    for (int k = timestep; k < timestep + 50; k++)
                        printf("%d ", agents[ag].path[k]);
                    printf("\n");
                    for (int k = timestep; k < timestep + 50; k++)
                        printf("%d ", agents[i].path[k]);
                    printf("\n");
                    return false;
                } else if (agents[ag].path[j] == agents[i].path[j - 1]
                           && agents[ag].path[j - 1] == agents[i].path[j]) {
                    cout << "Agent " << ag << " and " << i << " collide at edge "
                         << agents[ag].path[j - 1] << "-" << agents[ag].path[j] << " at time " << j << endl;
                    for (int k = timestep; k < timestep + 50; k++)
                        printf("%d:%d ", k, agents[ag].path[k]);
                    printf("\n");
                    for (int k = timestep; k < timestep + 50; k++)
                        printf("%d:%d ", k, agents[i].path[k]);
                    printf("\n");
                    return false;
                }
            }
        }
    }
    return true;
}
