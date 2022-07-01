//
// Created by carol on 12/17/18.
//

#include "Agent.h"

#define maxtime 10000
#define AG -1

//#define TIMESTEP

bool meujeito(const Node *i, const Node *j) { return i->g_val + i->h_val > j->g_val + j->h_val; }

bool sortLL(const LLNode *i, const LLNode *j) { return i->g_val + i->h_val > j->g_val + j->h_val; }


void Agent::setAgent(int loc, int col, int row, int id) {
    this->park_loc = loc;
    this->loc = loc;
    this->col = col;
    this->row = row;
    this->id = id;
    this->finish_time = 0;
    for (int i = 0; i < maxtime; i++) {
        path.push_back(loc);//stay still all the tiem
        path_principal.push_back(std::make_pair(loc, false));
    }

}

void Agent::updatePath(Node *goal) {
    std::fill(path.begin() + goal->timestep + 1, path.end(), goal->loc);
    Node *curr = goal;
    while (curr != NULL) {
        this->path[curr->timestep] = curr->loc;
        curr = curr->parent;
    }

}

void Agent::updatePath(LLNode *goal) {
    std::fill(path.begin() + goal->timestep + 1, path.end(), goal->loc);
    LLNode *curr = goal;
    while (curr != NULL) {
        this->path[curr->timestep] = curr->loc;
        curr = curr->parent;
    }

}

int Agent::fastEvaluateTP(Token &token) {

//    loc = path[finish_time];
//    vector<int> dist_start;
//    vector<int> dist_goal;
//    Task *task = NULL;
//    task = bestTask(token);
//    if (task == NULL) {
//
//        bool move = false;
//        for (list<Task *>::iterator it = token.taskset.begin(); it != token.taskset.end(); it++) {
//            if ((*it)->goal->loc == loc) {
//                move = true;
//                break;
//            }
//        }
//        if (move) {
//            token.timestep = finish_time;
//            if (Move2EP(token, false)) {
//                int i = 0;
//                for (i = token.timestep; i < token.path[id].size(); i++) //agent move with package or waiting
//                {
//                    token.path[id][i] = path[i];
//                }
//                return 0;
//            } else {
//                debug = 1;
//                //  cout << endl << endl << " Agent  " << id << "  impedindo tarefa no lugar " <<loc<< " no finish time " <<this->finish_time <<endl;
//                bool t = Move2EP(token, true);
//                // cout << endl << endl << "chamo com constraint true, agent  " << id << " finish time " <<this->finish_time <<endl;
//                int i = 0;
//                if (t) {
//                    for (i = token.timestep; i < token.path[id].size(); i++) //agent move with package or waiting
//                    {
//                        token.path[id][i] = path[i];
//                    }
//                } else {
//                    cout << "ERROOOOOOOOOOO na AG: agente nao conseguiu mover pra um endpoint sem restricoes" << endl;
//                }
//                return 0;
//            }
//
//        } else {
//            //   cout << "Agent " << id << " nao pegou tarefa: -1" << endl;
//            return -1;
//        }
//
//    } else {
//        dist_start = task->start->h_val;
//        int finish_time_pickup = dist_start[loc]; //posicao inicial ate pickup
//        dist_goal = task->goal->h_val;
//        int finish_time_delivery = dist_goal[task->start->loc]; //posicao de pickup ate goal
//        int finish_time_final =
//                finish_time + finish_time_pickup + finish_time_delivery; //posicao final do agente + entrega + coleta
//
//
//        path[finish_time_final] = task->goal->loc;
//        path[path.size() - 1] = task->goal->loc;
//        token.path[id][finish_time_pickup + finish_time] = task->start->loc;
//        token.path[id][token.path[id].size() - 1] = task->goal->loc;
//        token.path[id][finish_time_final] = task->goal->loc;
//
////            std::cout << " Agent " << id << " posicao " << loc << " take task " << task->start->loc << " --> "
////                      << task->goal->loc;
////            std::cout << "	Timestep " << finish_time << "-->" << finish_time + finish_time_pickup << " | "
////                      << finish_time + finish_time_pickup << " -->" << finish_time_final << endl;
//
//
//
//        token.timestep = finish_time_final;
//        token.taskset.remove(task);
//        this->finish_time = finish_time_final;
//
//        return finish_time_final;
//    }
    return 0;
}

bool Agent::TP(Token &token) {

//    loc = path[token.timestep];
//
//    Task *task = NULL;
//    task = bestTask(token);
//    if (task == NULL) {
//
//        bool move = false;
//        for (vector<Task *>::iterator it = token.taskset.begin(); it != token.taskset.end(); it++) {
//            if ((*it)->goal->loc == loc) {
//                move = true;
//                break;
//            }
//        }
//        if (move) {
//            if (Move2EP(token, false)) {
//                int i = 0;
//                for (i = token.timestep; i < token.path[id].size(); i++) //agent move with package or waiting
//                {
//                    token.path[id][i] = path[i];
//                }
//                return true;
//            } else {
//                cout << endl << endl << "ERRO " << id << " nao conseguiu se mover pra nennhum endpoint" << endl;
//                debug = 1;
//                bool t = Move2EP(token, false);
//            }
//
//        } else {
//            std::cout << "Agent " << id << " wait at timestep " << token.timestep << endl;
//            this->finish_time = token.timestep + 1;
//            //  this->path[token.timestep + 1] = loc;
//            return true;
//        }
//    } else {
//        std::cout << endl << " Agent " << id << " take task " << task->start->id << " " << task->start->loc << " --> "
//                  << task->goal->id << " " << task->goal->loc;
//
//        int arrive_start = AStar(loc, task->start, token.timestep, token, false, true);
//        if (arrive_start == -1) {
//            cout << "ERRO: nao encontrou caminho para coleta" << endl;
//            system("PAUSE");
//
//        }
//        std::cout << "	Timestep " << token.timestep << "-->" << arrive_start;
//
//        int arrive_goal = AStar(task->start->loc, task->goal, arrive_start, token, false, false);
//
//        if (arrive_goal == -1) {
//            //    cout <<"encontrou caminho para entrego, arrumo chamando coleta e entrega de novo com evaluate true em coleta"<< endl;
//            arrive_start = AStar(loc, task->start, token.timestep, token, true, true);
//            arrive_goal = AStar(task->start->loc, task->goal, arrive_start, token, false, false);
//
//        }
//        std::cout << "-->" << arrive_goal;
//        for (int i = token.timestep; i <= arrive_start; i++) {
//            std::cout << "	" << path[i];
//        }
//        std::cout << " | ";
//
//        for (int i = arrive_start + 1; i <= arrive_goal; i++) {
//            std::cout << "	" << path[i];
//        }
//
//        bool hold = true;
//        for (int i = arrive_goal + 1; i < maxtime; i++) {
//            for (int j = 0; j < token.agents.size(); j++) {
//                if (j != id && task->goal->loc == token.path[j][i]) {
////std::cout << endl << " Agent " << id << " colide no tempo " << i << " com agente " << j;
//                    hold = false;
//                    break;
//                }
//            }
//        }
//        if (!hold) {
//            int t = AStar(task->goal->loc, task->goal, arrive_goal, token, true, false);
//            // std::cout << endl << " Agent " << id << " arruma posicao no tempo  " << t;
//            if (t == -1) {
//                //cout << "Nao conseguiu arrumar a posicao, arrumo chamando entrega de novo com evalueate true" << endl;
//                arrive_goal = AStar(task->start->loc, task->goal, arrive_start, token, true, false);
//                // std::cout << endl << "-->" << arrive_goal;
//
////                for (int i = arrive_start + 1; i <= arrive_goal; i++) {
////                    std::cout << "	" << path[i];
////                }
//            }
////            for (int i = arrive_goal; i <= t; i++) {
////                std::cout << "	" << path[i];
////            }
//        }
//        for (int i = token.timestep; i < token.path[id].size(); i++) {
//            token.path[id][i] = path[i];
//        }
//        this->finish_time = arrive_goal;
//        task->agent = this;
//        task->ag_arrive_start = arrive_start;
//        task->ag_arrive_goal = arrive_goal;
//        token.taskset.erase(task);
//        return true;
//    }
    return false;

}

int Agent::AStar(int start_loc, Endpoint *goal, int begin_time, Token token, bool hold, bool coleta) {

    vector<LLNode *> open;
    int goal_location = goal->loc;
    //  map<unsigned int, Node *> allNodes_table;

    //  Node *start = new Node(start_loc, 0, goal->h_val[start_loc], NULL, begin_time, false);
    LLNode *start = new LLNode(start_loc, 0, goal->h_val[start_loc], NULL, begin_time, 0, false);

    hashtable_t::iterator it;

    hashtable_t allNodes_table;
    LLNode *empty_node;
    LLNode *deleted_node;
    empty_node = new LLNode();
    empty_node->loc = -1;
    deleted_node = new LLNode();
    deleted_node->loc = -2;
    allNodes_table.set_empty_key(empty_node);
    allNodes_table.set_deleted_key(deleted_node);
    open.push_back(start);
    allNodes_table[start] = start;


    start->in_openlist = true;
    // allNodes_table.insert(make_pair(start_loc, start));

    while (!open.empty()) {
        sort(open.begin(), open.end(), sortLL);
        LLNode *curr = open.back();
        open.pop_back();
        curr->in_openlist = false;

        if (curr->loc == goal_location) {
            bool flag = true;

            if (coleta) {
                if (AStar(goal_location, ep_park_loc, curr->timestep, token, false, false) == -1) {
                    flag = false;
                }
            }
            if (hold) {
                for (int i = curr->timestep + 1; i < token.final_timestep; i++) {
                    for (int j = 0; j < token.agents.size(); j++) {
                        if (j != id && curr->loc == token.path[j][i]) {
                            flag = false;
                            break;
                        }
                    }
                }
            }
            if (flag) {
                updatePath(curr);
                int t = curr->timestep;
                releaseClosedListLLNodes(&allNodes_table);
                return t;
            }
        }
        int next_loc;
        int action[5] = {0, 1, -1, col, -col};
        for (int i = 0; i < 5; i++) {
            next_loc = curr->loc + action[i];
            int next_timestep = curr->timestep + 1;
            if (!isConstrained(curr->loc, next_loc, next_timestep, token, 0)) {
                int next_g_val = curr->g_val + 1;
                int next_h_val = goal->h_val[next_loc];

                // Node *next = new Node(next_loc, next_g_val, next_h_val, curr, next_timestep, false);
                LLNode *next = new LLNode(next_loc, next_g_val, next_h_val, curr, next_timestep, 0, false);

                it = allNodes_table.find(next);

                //   it = allNodes_table.find(next->loc + next->g_val * row * col);
                if (it == allNodes_table.end()) {
                    next->in_openlist = true;
                    allNodes_table[next] = next;
                    open.push_back(next);
                } else {
                    delete (next);
                }
            }
        }
    }
    releaseClosedListLLNodes(&allNodes_table);
    return -1;

}

int Agent::AStar_certo(int start_loc, Endpoint *goal, int begin_time, Token token, bool evaluate, bool coleta,
                       Endpoint *next_goal) {

    // vector<Node *> open;
    vector<LLNode *> open;

    int goal_location = goal->loc;
    //  map<unsigned int, Node *> allNodes_table;

    // Node *start = new Node(start_loc, 0, goal->h_val[start_loc], NULL, begin_time, false);
    LLNode *start = new LLNode(start_loc, 0, goal->h_val[start_loc], NULL, begin_time, 0, false);


    //  open.push_back(start);

    //start->in_openList = true;
    //allNodes_table.insert(make_pair(start_loc, start));

    hashtable_t::iterator it;

    hashtable_t allNodes_table;
    LLNode *empty_node;
    LLNode *deleted_node;
    empty_node = new LLNode();
    empty_node->loc = -1;
    deleted_node = new LLNode();
    deleted_node->loc = -2;
    allNodes_table.set_empty_key(empty_node);
    allNodes_table.set_deleted_key(deleted_node);
    open.push_back(start);
    allNodes_table[start] = start;


    start->in_openlist = true;

    while (!open.empty()) {
        // sort(open.begin(), open.end(), meujeito);
        sort(open.begin(), open.end(), sortLL);

        //  Node *curr = open.back();
        LLNode *curr = open.back();

        open.pop_back();
        // curr->in_openList = false;
        curr->in_openlist = false;

        if (curr->loc == goal_location) {
            bool hold = true;


            if (coleta) {
                if (AStar(goal_location, next_goal, curr->timestep, token, false, false) == -1) {
                    hold = false;
                }
            }

            if (evaluate) {


                for (int i = curr->timestep + 1; i < token.final_timestep; i++) {
                    for (int j = 0; j < token.agents.size(); j++) {
                        if (j != id && curr->loc == token.path[j][i]) {
                            hold = false;
                            break;
                        }
                    }
                }

            }

            if (hold) {
                updatePath(curr);
                int t = curr->timestep;
                releaseClosedListLLNodes(&allNodes_table);

                // releaseClosedListNodes(allNodes_table);
                return t;
            }
        }
        int next_loc;
        int action[5] = {0, 1, -1, col, -col};
        for (int i = 0; i < 5; i++) {
            next_loc = curr->loc + action[i];
            int next_timestep = curr->timestep + 1;
            if (!isConstrained(curr->loc, next_loc, next_timestep, token, 0)) {
                int next_g_val = curr->g_val + 1;
                int next_h_val = goal->h_val[next_loc];
                // Node *next = new Node(next_loc, next_g_val, next_h_val, curr, next_timestep, false);
                LLNode *next = new LLNode(next_loc, next_g_val, next_h_val, curr, next_timestep, 0, false);
                it = allNodes_table.find(next);
                if (it == allNodes_table.end()) {
                    next->in_openlist = true;
                    allNodes_table[next] = next;
                    open.push_back(next);
                    //Node *next = new Node(next_loc, next_g_val, next_h_val, curr, next_timestep, false);
                    //map<unsigned int, Node *>::iterator it = allNodes_table.find(next->loc + next->g_val * row * col);
//                LLNode* next = new LLNode(next_loc, next_g_val, next_h_val,	curr, next_timestep, 0, false);
//
//                it = allNodes_table.find(next);
//                if (it == allNodes_table.end()) {
//                    next->in_openList = true;
//                    allNodes_table.insert(pair<unsigned int, Node *>(next->loc + next->g_val * row * col, next));
//                    open.push_back(next);
                } else {
                    delete (next);
                }
            }
        }
    }
//    releaseClosedListNodes(allNodes_table);
    releaseClosedListLLNodes(&allNodes_table);

    return -1;

}

int Agent::planFastPath(int start_loc, int begin_time, Token &token) {

    clock_t start;
    double duration = 0;
    start = clock();

    int arrive_start = AStar_certo(start_loc, this->task->start, begin_time, token, false, false, nullptr);

    if (arrive_start == -1) {
        cout << "ERRO: nao encontrou caminho para coleta" << endl;
        system("PAUSE");
    }

    int arrive_goal = AStar_certo(this->task->start->loc, this->task->goal, arrive_start, token, false, false, nullptr);

    if (arrive_goal == -1) {
        arrive_start = AStar_certo(start_loc, this->task->start, begin_time, token, false, true, this->task->goal);
        if (arrive_start == -1) {
            cout << "ERRO: nao encontrou caminho para coleta" << endl;
            system("PAUSE");

        }

        arrive_goal = AStar_certo(this->task->start->loc, this->task->goal, arrive_start, token, false, false, nullptr);

        if (arrive_goal == -1) {
            cout << "ERRO: nao encontrou caminho para coleta" << endl;
            system("PAUSE");

        }
    }
    int arrive_park = AStar_certo(this->task->goal->loc, this->ep_park_loc, arrive_goal, token, true, false, nullptr);

    if (arrive_park == -1) {
        arrive_goal = AStar_certo(this->task->start->loc, this->task->goal, arrive_start, token, false, true,
                                  this->ep_park_loc);

        if (arrive_goal == -1) {
            cout << " 2 ERRO!! Ag nao conseguiu chegar no park location " << endl;
            system("PAUSE");
        }

        arrive_park = AStar_certo(this->task->goal->loc, this->ep_park_loc, arrive_goal, token, true, false, nullptr);
    }
    if (token.final_timestep < arrive_park) {
        token.final_timestep = arrive_park;
    }

#ifdef TIMESTEP
    if(this->id == AG || AG == -1)std::cout << endl << " Agent " << id <<" task: "<< this->task->id <<" begin time: "<<begin_time<< " arrive start: "  << arrive_start << " arrive goal: "  << arrive_goal<< " arrive park: "  << arrive_park<<endl;
#endif

//    for (int i = begin_time; i < token.path[id].size(); i++) {
//        token.path[id][i] = path[i];
//    }
    std::cout << "-->" << arrive_start;
    for (int i = begin_time; i <= arrive_start; i++) {
        std::cout << "	" << path[i];
    }

    token.path[id] = path;

    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    this->timepathPlan = duration + this->timepathPlan;

    return arrive_goal;
}

int Agent::planPathPark(int start_loc, Endpoint *goal, int begin_time, Token &token) {
#ifdef TIMESTEP
    if(this->id == AG || AG == -1)std::cout << endl << " Agent " << id <<" na loc "<<start_loc << " vai para a posicao de park location: "<<goal->loc <<" " ;
#endif
    clock_t start;
    double duration = 0;
    start = clock();

    int arrive_park = AStar(start_loc, goal, begin_time, token, true, false);
    if (arrive_park == -1) {
        cout << "ERRO: nao encontrou caminho para coleta" << endl;
        system("PAUSE");
    }
    if (token.final_timestep < arrive_park) {
        token.final_timestep = arrive_park;
    }
    token.path[id] = path;

    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    this->timepathPlan = duration + this->timepathPlan;
    return arrive_park;
}

int Agent::planPath(int start_loc, Endpoint *goal, int begin_time, Token &token) {

    clock_t start;
    double duration = 0;
    start = clock();

    int arrive_start = AStar(start_loc, goal, begin_time, token, false, false);
    if (arrive_start == -1) {
        cout << "ERRO: nao encontrou caminho para coleta" << endl;
        system("PAUSE");

    }
    token.timestep = arrive_start + 1;
    this->loc = goal->loc;
    int arrive_park = AStar(goal->loc, this->ep_park_loc, arrive_start, token, true, false);


    if (arrive_park == -1) {

        arrive_start = AStar(start_loc, goal, begin_time, token, false, true);
        if (arrive_start == -1) {
            cout << "ERRO: nao encontrou caminho para coleta" << endl;
            system("PAUSE");

        }
        arrive_park = AStar(goal->loc, this->ep_park_loc, arrive_start, token, true, false);
        if (arrive_park == -1) {
            cout << "ERRO: nao encontrou caminho para coleta" << endl;
            system("PAUSE");

        }

    }
    if (token.final_timestep < arrive_park) {
        token.final_timestep = arrive_park;
    }
#ifdef TIMESTEP
    if(this->id == AG || AG == -1)std::cout << endl << " Agent " << id  <<" begin time: "<<begin_time<< " arrive location: "  << arrive_start << " arrive park: "  << arrive_park<<endl;
#endif

    token.path[id] = path;

    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    this->timepathPlan = duration + this->timepathPlan;

    return arrive_start;


}

int Agent::planPath2EP(int start_loc, Endpoint *goal, int begin_time, Token &token) {

    clock_t start;
    double duration = 0;
    start = clock();
#ifdef TIMESTEP
    if(this->id == AG || AG == -1)std::cout << endl << " Agent " << id << endl;
#endif
    int arrive_start = AStar(start_loc, goal, begin_time, token, false, false);
    if (arrive_start == -1) {
        cout << "ERRO: nao encontrou caminho para coleta" << endl;
        system("PAUSE");

    }
    token.timestep = arrive_start + 1;
    this->loc = goal->loc;
    //int arrive_park = AStar(goal->loc, this->ep_park_loc, arrive_start, token, true, false);
    int arrive_park = Move2EP(token, false);


    if (arrive_park == -1) {

        arrive_start = AStar(start_loc, goal, begin_time, token, false, true);
        if (arrive_start == -1) {
            cout << "ERRO: nao encontrou caminho para coleta" << endl;
            system("PAUSE");

        }
        arrive_park = AStar(goal->loc, this->ep_park_loc, arrive_start, token, true, false);
        if (arrive_park == -1) {
            cout << "ERRO: nao encontrou caminho para coleta" << endl;
            system("PAUSE");

        }

    }
    if (token.final_timestep < arrive_park) {
        token.final_timestep = arrive_park;
    }
#ifdef TIMESTEP
    if(this->id == AG || AG == -1)std::cout << endl << " Agent " << id  <<" begin time: "<<begin_time<< " arrive location: "  << arrive_start << " arrive park: "  << arrive_park<<endl;
#endif

    token.path[id] = path;

    duration = (clock() - start) / (double) CLOCKS_PER_SEC;
    this->timepathPlan = duration + this->timepathPlan;

    return arrive_start;


}

int Agent::AStar_sCol(int start_loc, Endpoint *goal, int begin_time, Token token, bool evaluate, bool coleta) {

    clock_t inicio;
    double duration = 0;
    inicio = clock();

    vector<Node *> open;
    int goal_location = goal->loc;
    map<unsigned int, Node *> allNodes_table;


    Node *start = new Node(start_loc, 0, goal->h_val[start_loc], NULL, begin_time, false);

    open.push_back(start);
    start->in_openList = true;
    allNodes_table.insert(make_pair(start_loc, start));

    while (!open.empty()) {
        sort(open.begin(), open.end(), meujeito);
        Node *curr = open.back();
        open.pop_back();
        curr->in_openList = false;

        if (curr->loc == goal_location) {
            updatePath(curr);
            int t = curr->timestep;
            releaseClosedListNodes(allNodes_table);
            duration = (clock() - inicio) / (double) CLOCKS_PER_SEC;
            this->timepathPlan = duration + this->timepathPlan;
            return t;

        }
        int next_loc;
        int action[5] = {0, 1, -1, col, -col};
        for (int i = 0; i < 5; i++) {
            next_loc = curr->loc + action[i];
            int next_timestep = curr->timestep + 1;
            if (token.map[next_loc]) {
                int next_g_val = curr->g_val + 1;
                int next_h_val = goal->h_val[next_loc];

                Node *next = new Node(next_loc, next_g_val, next_h_val, curr, next_timestep, false);

                map<unsigned int, Node *>::iterator it = allNodes_table.find(next->loc + next->g_val * row * col);
                if (it == allNodes_table.end()) {
                    next->in_openList = true;
                    allNodes_table.insert(pair<unsigned int, Node *>(next->loc + next->g_val * row * col, next));
                    open.push_back(next);
                } else {
                    delete (next);
                }
            }
        }
    }
    releaseClosedListNodes(allNodes_table);
    return -1;

}

Task *Agent::bestTask(Token token) {
    vector<bool> hold(col * row, false);
    for (int i = 0; i < token.path.size(); i++) {
        if (i != id) {
            int loc_f = token.path[i][token.path[i].size() - 1];
            hold[loc_f] = true;
        }
    }

    Task *task = NULL;
    list<Task *>::iterator n;
    for (vector<Task *>::iterator it = token.taskset.begin(); it != token.taskset.end(); it++) {
        bool p = hold[(*it)->start->loc];
        if (hold[(*it)->start->loc] == true || hold[(*it)->goal->loc] == true) continue;
        if (task == NULL) task = *it;
        unsigned int t = task->start->h_val[loc];
        unsigned int m = (*it)->start->h_val[loc];
        if ((*it)->start->h_val[loc] < task->start->h_val[loc]) {
            task = *it;
        }
    }
    return task;
}

bool Agent::isConstrained(int curr_loc, int next_loc, int next_timestep, Token token, int ag_hide) {

    if (!token.map[next_loc]) return true;

    for (int i = 0; i < token.path.size(); i++) {
        if (i == id) continue;
        else if (token.path[i][next_timestep] == next_loc) {
            this->num_conflito++;
            if (this->debug)
                cout << " VERTEX agente " << i << " no tempo " << next_timestep << " na posicao " << next_loc << endl;
            return true;
        } else if (token.path[i][next_timestep] == curr_loc && token.path[i][next_timestep - 1] == next_loc) {
            this->num_conflito++;

            if (this->debug)
                cout << " EDGE agente " << i << " no tempo " << next_timestep << " na posicao " << curr_loc << endl;
            return true;
        }
    }

    return false;
}

bool Agent::Move2EP(Token &token, bool constraint) {

    //BFS algorithm, choose the first empty endpoint to go to
    queue<Node *> Q;
    map<unsigned int, Node *> allNodes_table; //key = g_val * map_size + loc
    int action[5] = {0, 1, -1, col, -col};
    Node *start = new Node(loc, 0, NULL, token.timestep);
    allNodes_table.insert(make_pair(loc, start)); //g_val = 0 --> key = loc
    Q.push(start);
    while (!Q.empty()) {
        Node *v = Q.front();
        Q.pop();
        if (v->timestep >= maxtime - 1) continue; // time limit
        if (token.endpoints[v->loc]) // if v->loc is an endpoint
        {
            bool occupied = false;
            // check whether v->loc can be held (no collision with other agents)
            for (unsigned int t = v->timestep; t < maxtime && !occupied; t++) {
                for (unsigned int ag = 0; ag < token.agents.size() && !occupied; ag++) {
                    if (ag != id && token.path[ag][t] == v->loc) occupied = true;
                }
            }
            // check whether it is a goal of a task
            for (vector<Task *>::iterator it = token.taskset.begin(); it != token.taskset.end() && !occupied; it++) {
                if ((*it)->goal->loc == v->loc) occupied = true;
            }
            if (!occupied) {
                updatePath(v);
                finish_time = v->timestep;
                //   if(this->debug) cout << "Agent " << id << " moves to endpoint " << v->loc << " no tempo " << finish_time << endl;
                releaseClosedListNodes(allNodes_table);
                return true;
            }
        }
        for (int i = 0; i < 5; i++) // search its neighbor
        {
            if (constraint == true) {
                map<unsigned int, Node *>::iterator it = allNodes_table.find(
                        v->loc + action[i] + (v->g_val + 1) * row * col);
                if (it == allNodes_table.end()) //undiscover
                {  // add the newly generated node to hash table
                    Node *u = new Node(v->loc + action[i], v->g_val + 1, v, v->timestep + 1);
                    allNodes_table.insert(pair<unsigned int, Node *>(u->loc + u->g_val * row * col, u));
                    Q.push(u);
                }
            } else {
                if (!isConstrained(v->loc, v->loc + action[i], v->timestep + 1, token, id)) {
                    //try to retrieve it from the hash table
                    map<unsigned int, Node *>::iterator it = allNodes_table.find(
                            v->loc + action[i] + (v->g_val + 1) * row * col);
                    if (it == allNodes_table.end()) //undiscover
                    {  // add the newly generated node to hash table
                        Node *u = new Node(v->loc + action[i], v->g_val + 1, v, v->timestep + 1);
                        allNodes_table.insert(pair<unsigned int, Node *>(u->loc + u->g_val * row * col, u));
                        Q.push(u);
                    }
                }
            }

        }
    }
    return false;
}

void Agent::releaseClosedListNodes(map<unsigned int, Node *> &allNodes_table) {
    map<unsigned int, Node *>::iterator it;
    for (it = allNodes_table.begin(); it != allNodes_table.end(); it++) {
        delete ((*it).second);
    }
    allNodes_table.clear();
}

void Agent::releaseClosedListLLNodes(hashtable_t *allNodes_table) {
    hashtable_t::iterator it;
    for (it = allNodes_table->begin(); it != allNodes_table->end(); it++) {
        delete ((*it).second);  // Node* s = (*it).first; delete (s);
    }
}

int
PathPlanning::Agent_AStar(Agent *ag, int start_loc, Endpoint *goal, int begin_time, bool hold, Endpoint *next_goal,
                           Token &token) {

    vector<LLNode *> open;
    int goal_location = goal->loc;
    LLNode *start = new LLNode(start_loc, 0, goal->h_val[start_loc], NULL, begin_time, 0, false);

    hashtable_t::iterator it;
    hashtable_t allNodes_table;
    LLNode *empty_node;
    LLNode *deleted_node;
    empty_node = new LLNode();
    empty_node->loc = -1;
    deleted_node = new LLNode();
    deleted_node->loc = -2;
    allNodes_table.set_empty_key(empty_node);
    allNodes_table.set_deleted_key(deleted_node);
    open.push_back(start);
    allNodes_table[start] = start;

    start->in_openlist = true;

    while (!open.empty()) {
        sort(open.begin(), open.end(), sortLL);

        LLNode *curr = open.back();

        open.pop_back();
        curr->in_openlist = false;

        if (curr->loc == goal_location) {
            bool flag = true;
            if (flag) {
                updatePathAgent(curr, ag);
                int t = curr->timestep;
                // cout << "1 - Agent " << ag->id << " moves to endpoint " << curr->loc << " no tempo " << t << endl;
//                if (hold) {
//                    for (int tt = begin_time; tt < t; tt++) {
//                        if (ag->path[tt] == goal_location) {
//                            t = tt;
//                            break;
//                        }
//                    }
//                }
                bool work = this->Move2EP(ag, t, true, token);
                if (work == false) {
//                    string res_file =
//                            "/home/carol/Desktop/MAPD/endpoint/EP-"+this->problem_name +"-1000-" + to_string(token.agents.size()) + "-" + to_string(this->seed) + ".csv";
//                    ofstream fout(res_file, ios::app);
//                    if (!fout) cout << "ERR0 - salvar arquivo endpoint"<<endl;
//                    fout <<ag->id<<","<< begin_time << endl;
//                    fout.close();
                   // cout << "ERRO MOVE2EP - Timestep: "<<begin_time << endl;
                    continue;
                }
                releaseClosedLLNodes(&allNodes_table);
                delete (empty_node);
                delete (deleted_node);
                return t;
            }
        }
        int next_loc;
        int action[5] = {0, 1, -1, col, -col};
        for (int i = 0; i < 5; i++) {
            next_loc = curr->loc + action[i];
            int next_timestep = curr->timestep + 1;
            if (!isConstrained(curr->loc, next_loc, next_timestep, ag->id)) {
                int next_g_val = curr->g_val + 1;
                int next_h_val = goal->h_val[next_loc];
                LLNode *next = new LLNode(next_loc, next_g_val, next_h_val, curr, next_timestep, 0, false);
                it = allNodes_table.find(next);
                if (it == allNodes_table.end()) {
                    next->in_openlist = true;
                    allNodes_table[next] = next;
                    open.push_back(next);
                } else {
                    delete (next);
                }
            }
        }
    }
    releaseClosedLLNodes(&allNodes_table);
    delete (empty_node);
    delete (deleted_node);
    return -1;

}

int
PathPlanning::Agent_AStar1(Agent *ag, int start_loc, Endpoint *goal, int begin_time, bool hold, Endpoint *next_goal) {

    vector<LLNode *> open;
    int goal_location = goal->loc;
    LLNode *start = new LLNode(start_loc, 0, goal->h_val[start_loc], NULL, begin_time, 0, false);

    hashtable_t::iterator it;
    hashtable_t allNodes_table;
    LLNode *empty_node;
    LLNode *deleted_node;
    empty_node = new LLNode();
    empty_node->loc = -1;
    deleted_node = new LLNode();
    deleted_node->loc = -2;
    allNodes_table.set_empty_key(empty_node);
    allNodes_table.set_deleted_key(deleted_node);
    open.push_back(start);
    allNodes_table[start] = start;

    start->in_openlist = true;

    while (!open.empty()) {
        sort(open.begin(), open.end(), sortLL);
        LLNode *curr = open.back();

        open.pop_back();
        curr->in_openlist = false;

        if (curr->loc == goal_location) {
            bool flag = true;

            if (hold) {
                for (int i = curr->timestep + 1; i < maxtime; i++) {
                    for (int j = 0; j < this->path_agents.size(); j++) {
                        if (j != ag->id && curr->loc == this->path_agents[j][i].first) {
                            flag = false;
                            break;
                        }
                    }
                }
            }
            if (flag) {
                updatePathAgent(curr, ag);
                int t = curr->timestep;
                releaseClosedLLNodes(&allNodes_table);
                if (hold) {
                    for (int tt = begin_time; tt < t; tt++) {
                        if (ag->path[tt] == goal_location) {
                            t = tt;
                            break;
                        }
                    }
                }
                return t;
            }
        }
        int next_loc;
        int action[5] = {0, 1, -1, col, -col};
        for (int i = 0; i < 5; i++) {
            next_loc = curr->loc + action[i];
            int next_timestep = curr->timestep + 1;
            if (!isConstrained(curr->loc, next_loc, next_timestep, ag->id)) {
                int next_g_val = curr->g_val + 1;
                int next_h_val = goal->h_val[next_loc];
                LLNode *next = new LLNode(next_loc, next_g_val, next_h_val, curr, next_timestep, 0, false);
                it = allNodes_table.find(next);
                if (it == allNodes_table.end()) {
                    next->in_openlist = true;
                    allNodes_table[next] = next;
                    open.push_back(next);
                } else {
                    delete (next);
                }
            }
        }
    }
    releaseClosedLLNodes(&allNodes_table);
    return -1;

}

bool PathPlanning::isConstrained(int curr_loc, int next_loc, int next_timestep, int id) {

    if (!this->map[next_loc]) return true;

    for (int i = 0; i < this->path_agents.size(); i++) {
        if (i == id) continue;
        else if (path_agents[i][next_timestep].first == next_loc) {
            this->num_collision++;
//            if (path_agents[i][next_timestep].second == true) {
//                cout << "HOLD - Agente: " << id << " colisao com VERTEX agente " << i << " no tempo " << next_timestep
//                     << " na posicao " << next_loc << endl;
//            } else {
//                cout << "Agente: " << id << " colisao com VERTEX agente " << i << " no tempo " << next_timestep
//                     << " na posicao " << next_loc << endl;
//            }
            return true;
        } else if (path_agents[i][next_timestep].first == curr_loc &&
                   path_agents[i][next_timestep - 1].first == next_loc) {
            this->num_collision++;
//            cout << "Agente: " << id << " colisao com EDGE agente " << i << " no tempo " << next_timestep
//                 << " na posicao " << curr_loc << endl;
//
//            if (path_agents[i][next_timestep].second == true) {
//                //   cout <<"HOLD - Agente: "<<id<<" colisao com EDGE agente "<<i<<" no tempo "<<next_timestep<<" na posicao "<< curr_loc << endl;
//            } else {
//                //  cout <<"Agente: "<<id<<" colisao com EDGE agente "<<i<<" no tempo "<<next_timestep<<" na posicao "<< curr_loc << endl;
//            }
            return true;
        }
    }

    return false;
}

void PathPlanning::releaseClosedLLNodes(hashtable_t *allNodes_table) {
    hashtable_t::iterator it;
    for (it = allNodes_table->begin(); it != allNodes_table->end(); it++) {
        LLNode* s = (*it).first;
        LLNode* g = (*it).second;
        delete ((*it).second);  // Node* s = (*it).first; delete (s);

    }
}

void PathPlanning::releaseClosedListNodes(std::map<unsigned int, Node *> &allNodes_table) {
    std::map<unsigned int, Node *>::iterator it;
    for (it = allNodes_table.begin(); it != allNodes_table.end(); it++) {
        delete ((*it).second);
    }
    allNodes_table.clear();
}

void PathPlanning::updatePathAgent(LLNode *goal, Agent *ag) {
//    for (int i = goal->timestep + 1; i < ag->path_principal.size(); i++) {
//        ag->path_principal[i].first = goal->loc;
//        ag->path_principal[i].second = true;
//        ag->path[i] = goal->loc;
//
//
//    }
    std::fill(ag->path.begin() + goal->timestep + 1, ag->path.end(), goal->loc);
    std::fill(ag->path_principal.begin() + goal->timestep + 1, ag->path_principal.end(),
              std::make_pair(goal->loc, true));

    ag->holding.first = goal->loc;
    ag->holding.second = goal->timestep + 1;
    LLNode *curr = goal;
    while (curr != nullptr) {
        ag->path_principal[curr->timestep].first = curr->loc;
        ag->path_principal[curr->timestep].second = false;
        ag->path[curr->timestep] = curr->loc;

        curr = curr->parent;

    }
    this->path_agents[ag->id] = ag->path_principal;

}

void PathPlanning::updatePathAgent(Node *goal, Agent *ag) {
//    for (int i = goal->timestep + 1; i < ag->path_principal.size(); i++) {
//        ag->path_principal[i].first = goal->loc;
//        ag->path_principal[i].second = true;
//        ag->path[i] = goal->loc;
//    }
    std::fill(ag->path.begin() + goal->timestep + 1, ag->path.end(), goal->loc);
    std::fill(ag->path_principal.begin() + goal->timestep + 1, ag->path_principal.end(),
              std::make_pair(goal->loc, true));
    ag->holding.first = goal->loc;
    ag->holding.second = goal->timestep + 1;

    Node *curr = goal;
    while (curr != nullptr) {
        ag->path_principal[curr->timestep].first = curr->loc;
        ag->path_principal[curr->timestep].second = false;
        ag->path[curr->timestep] = curr->loc;

        curr = curr->parent;

    }
    this->path_agents[ag->id] = ag->path_principal;
}

bool PathPlanning::Move2EP(Agent *agent, int begin_time, bool constraint, Token &token) {

    //BFS algorithm, choose the first empty endpoint to go to
    queue<Node *> Q;
    std::map<unsigned int, Node *> allNodes_table; //key = g_val * map_size + loc
    int action[5] = {0, 1, -1, col, -col};
    Node *start = new Node(agent->path[begin_time], 0, NULL, begin_time);
    allNodes_table.insert(make_pair(agent->loc, start)); //g_val = 0 --> key = loc
    Q.push(start);
    int id_task = -1;
    if (agent->task != NULL) {
        id_task = agent->task->id;
    }

    while (!Q.empty()) {
        Node *v = Q.front();
        Q.pop();
        if (v->timestep >= maxtime - 1) continue; // time limit
        if (token.endpoints[v->loc]) // if v->loc is an endpoint
        {
            bool occupied = false;
            for (vector<Task *>::iterator it = token.taskset.begin(); it != token.taskset.end() && !occupied; it++) {
                if (constraint == true && (*it)->id == id_task) continue;
                if ((*it)->goal->loc == v->loc || (*it)->start->loc == v->loc) {
                    //  cout << (*it)->id << " s: " << (*it)->start->loc << " g: " << (*it)->goal->loc;
                    occupied = true;
                }
            }
            // check whether v->loc can be held (no collision with other agents)
            for (unsigned int ag = 0; ag < token.agents.size() && !occupied; ag++) {
                for (unsigned int t = v->timestep; t < token.agents[ag]->holding.second && !occupied; t++) {
                    if (ag != agent->id && token.path[ag][t] == v->loc) {
                        //  cout << ag << " " << t;
                        occupied = true;
                    }
                }
            }

            if (!occupied) {
                updatePathAgent(v, agent);
                agent->finish_time = v->timestep;
                token.path[agent->id] = agent->path;
//                cout << "2 - Agent " << agent->id << " moves to endpoint " << v->loc << " no tempo "
//                     << agent->finish_time << endl;
                releaseClosedListNodes(allNodes_table);
                return true;
            }
        }
        for (int i = 0; i < 5; i++) // search its neighbor
        {
//            if (constraint == true) {
//                std::map<unsigned int, Node *>::iterator it = allNodes_table.find(
//                        v->loc + action[i] + (v->g_val + 1) * agent->row * col);
//                if (it == allNodes_table.end()) //undiscover
//                {  // add the newly generated node to hash table
//                    Node *u = new Node(v->loc + action[i], v->g_val + 1, v, v->timestep + 1);
//                    allNodes_table.insert(pair<unsigned int, Node *>(u->loc + u->g_val * agent->row * col, u));
//                    Q.push(u);
//                }
//            } else {
            if (!isConstrained(v->loc, v->loc + action[i], v->timestep + 1, agent->id)) {
                //try to retrieve it from the hash table
                std::map<unsigned int, Node *>::iterator it = allNodes_table.find(
                        v->loc + action[i] + (v->g_val + 1) * agent->row * col);
                if (it == allNodes_table.end()) //undiscover
                {  // add the newly generated node to hash table
                    Node *u = new Node(v->loc + action[i], v->g_val + 1, v, v->timestep + 1);
                    allNodes_table.insert(pair<unsigned int, Node *>(u->loc + u->g_val * agent->row * col, u));
                    Q.push(u);
                }
            }
        }
    }
    return false;
}

PathPlanning::~PathPlanning() {

}


//bool PathPlanning::Move2EP(Agent *agent, int begin_time, bool constraint, Token &token) {
//
//    //BFS algorithm, choose the first empty endpoint to go to
//    queue<Node *> Q;
//    std::map<unsigned int, Node *> allNodes_table; //key = g_val * map_size + loc
//    int action[5] = {0, 1, -1, col, -col};
//    Node *start = new Node(agent->path[begin_time], 0, NULL, begin_time);
//    allNodes_table.insert(make_pair(agent->loc, start)); //g_val = 0 --> key = loc
//    Q.push(start);
//    int id_task = -1;
//    if (agent->task != NULL) {
//        id_task = agent->task->id;
//    }
//
//    while (!Q.empty()) {
//        Node *v = Q.front();
//        Q.pop();
//        if (v->timestep >= maxtime - 1) continue; // time limit
//        if (token.endpoints[v->loc]) // if v->loc is an endpoint
//        {
//            bool occupied = false;
//            for (vector<Task *>::iterator it = token.taskset.begin(); it != token.taskset.end() && !occupied; it++) {
//                if ((*it)->id == id_task) continue;
//                if ((*it)->goal->loc == v->loc || (*it)->start->loc == v->loc) {
//                  //  cout << (*it)->id << " s: " << (*it)->start->loc << " g: " << (*it)->goal->loc;
//                    occupied = true;
//                }
//            }
//            // check whether v->loc can be held (no collision with other agents)
//            for (unsigned int ag = 0; ag < token.agents.size() && !occupied; ag++) {
//                for (unsigned int t = v->timestep; t < token.agents[ag]->holding.second && !occupied; t++) {
//                    if (ag != agent->id && token.path[ag][t] == v->loc) {
//                      //  cout << ag << " " << t;
//                        occupied = true;
//                    }
//                }
//            }
//
//            if (!occupied) {
//                updatePathAgent(v, agent);
//                agent->finish_time = v->timestep;
//                token.path[agent->id] = agent->path;
//                //   if(this->debug) cout << "Agent " << id << " moves to endpoint " << v->loc << " no tempo " << finish_time << endl;
//                releaseClosedListNodes(allNodes_table);
//                return true;
//            }
//        }
//        for (int i = 0; i < 5; i++) // search its neighbor
//        {
//            if (constraint == true) {
//                std::map<unsigned int, Node *>::iterator it = allNodes_table.find(
//                        v->loc + action[i] + (v->g_val + 1) * agent->row * col);
//                if (it == allNodes_table.end()) //undiscover
//                {  // add the newly generated node to hash table
//                    Node *u = new Node(v->loc + action[i], v->g_val + 1, v, v->timestep + 1);
//                    allNodes_table.insert(pair<unsigned int, Node *>(u->loc + u->g_val * agent->row * col, u));
//                    Q.push(u);
//                }
//            } else {
//                if (!isConstrained(v->loc, v->loc + action[i], v->timestep + 1, agent->id)) {
//                    //try to retrieve it from the hash table
//                    std::map<unsigned int, Node *>::iterator it = allNodes_table.find(
//                            v->loc + action[i] + (v->g_val + 1) * agent->row * col);
//                    if (it == allNodes_table.end()) //undiscover
//                    {  // add the newly generated node to hash table
//                        Node *u = new Node(v->loc + action[i], v->g_val + 1, v, v->timestep + 1);
//                        allNodes_table.insert(pair<unsigned int, Node *>(u->loc + u->g_val * agent->row * col, u));
//                        Q.push(u);
//                    }
//                }
//            }
//
//        }
//    }
//    return false;
//}
