#include <iostream>
#include <time.h>
#include <stdlib.h>     /* srand, rand */
#include <stdio.h>      /* printf, NULL */
#include <iomanip>
#include <ostream>
#include <fstream>
#include <string>
#include "include/Timer.hpp"
#include "Simulation.h"

using namespace std;


int run(const string& ag, string fr,  int seed, string problem, int bestInd, int tipo_GA, int objetivo) {


    Simulation *simu;
    srand(seed);
    string ind ="sI";
    string ga ="GA";
    string obj = "";

    if(bestInd == 1) ind = "cI";
    if (tipo_GA == 0) {
        ga = "GA_TA";
    }else if (tipo_GA == 1) {
        ga = "Elitista";
    }else if (tipo_GA == 2) {
        ga = "SteadyState";
    }else if (tipo_GA == 3) {
        ga = "GA_E";
    }else if (tipo_GA == 4) {
        ga = "NSGA";
        if (objetivo == 0) {
            obj = "_M";
        }else if (objetivo == 1) {
            obj = "_ST";
        }
        else if (objetivo == 2) {
            obj = "_O";
        }
    }
    cout <<"=== SEED: "<<seed<< " | " << problem <<" - "<<ga <<obj<<" | "<< ind << " === AG: " << ag << " - F: " << fr << " === ";

    string map_file = " ";
    string task_file = " ";
    string res_file = " ";
    string p = "/Users/anacarolina/Desktop/newMapd/newMapd";
    if(stoi(ag)> 50){
      cout << "LARGE ===" << endl;
      map_file = p + "/Instances/large/kiva-" + ag + "-1000-50.map";     //LARGE
      task_file = p+"/Instances/large/kiva-1000-50.task";              //LARGE
      res_file = p+"/Resultados/large-" + ind + "-" + problem + "-" + ga + obj+".csv"; //LARGE
    }else{
      cout << "SMALL ===" << endl;
      map_file = p+"/Instances/small/kiva-" + ag + "-500-5.map";       //SMALL
      task_file = p+"/Instances/small/kiva-" + fr + ".task";           //SMALL
      res_file = p+"/Resultados/small-" + ind + "-" + problem + "-" + ga + obj+ ".csv"; //SMALL
    }
    string nome = ga+obj;
    simu = new Simulation(map_file, task_file, stoi(fr), seed, nome);
    std::clock_t c_start = std::clock();


//    simu->run_HungarianMethod();

    if(problem == "SCOL") simu->run_SCol(bestInd, tipo_GA);
    else if(problem == "CBS") simu->run_CBS(bestInd, tipo_GA);
    else if(problem == "AS") simu->run_AS(bestInd, tipo_GA);
    else if(problem == "AS_2EP") simu->run_AS2EP(bestInd, tipo_GA);
    else if(problem == "AS_FAST") simu->run_AS_Fast(bestInd, tipo_GA);
    else if(problem == "AS_HOLD") simu->run_AS_Hold(bestInd, tipo_GA, objetivo);
    else if(problem == "NSGA") simu->runNSGA();
    else if(problem == "CBS_HEUR") simu->run_CBS_heuristic();
    else if(problem == "AS_HEUR") simu->run_AS_heuristic(bestInd);

    std::clock_t c_end = std::clock();

    cout<<endl;
    int mksn = simu->mkspn();
    cout<<"Mksn: "<<mksn<<" ";
    float service_time = simu->service_time();
    cout<<"Service time: "<<service_time<<endl;

    cout<<"Fitness m: "<<simu->fitness_m<<" ";
    cout<<"Fitness st: "<<simu->fitness_st<<endl;
//    cout<<"CBS - Colisoes: "<<simu->num_conflito_CBS_total<<endl;
//    cout<<"CBS - Colisoes 49: "<<simu->conflito_CSB_final<<endl;
    cout<<"A STAR - Colisoes ASTAR: "<<simu->pp.num_collision<<endl;
    //cout<<"A STAR - Colisoes ASTAR: "<<simu->num_conflito_PP<<endl;

    double tempoTotalGA = simu->tempoGA();
    cout<<"Tempo GA: "<<tempoTotalGA<<" ";
    double tempoTotalPathPlan = simu->tempoPathPlan();
    cout<<"Tempo Path: "<<tempoTotalPathPlan<<" ";
    std::cout << std::fixed << std::setprecision(2) << "CPU time used: " << (c_end - c_start) / CLOCKS_PER_SEC <<endl;

   ofstream fout(res_file, ios::app);
   if (!fout) cout << "ERR0 - salvar arquivo";
   if(problem == "CBS" | problem == "CBS_HEUR"){
       // Frequencia, Agente, Semente, Makespan, Service Time, Fitness Makespan, Fitness Service time, Num colisoes total, Num colisoes dps do ult execucao GA, Tempo total, Tempo total GA, tempo total PP
       fout << fr << "," << ag << "," << seed << "," << mksn << "," <<service_time<<","<<simu->fitness_m<<","<<simu->fitness_st<<","<<simu->num_conflito_CBS_total<<","<<simu->conflito_CSB_final<<","<< (c_end - c_start) / CLOCKS_PER_SEC << ","<<tempoTotalGA <<","<<tempoTotalPathPlan<< endl;
   }
   else{
       // Frequencia, Agente, Semente, Makespan, Service Time, Fitness Makespan, Fitness Service time, Num colisoes AS HOLD, Num colisoes AS, Tempo total, Tempo total GA, tempo total PP
       fout << fr << "," << ag << "," << seed << "," << mksn << "," <<service_time<<","<<simu->fitness_m<<","<<simu->fitness_st<<","<<simu->pp.num_collision<<","<<simu->num_conflito_PP<<","<< (c_end - c_start) / CLOCKS_PER_SEC << ","<<tempoTotalGA <<","<<tempoTotalPathPlan<< endl;
   }
   fout.close();

    delete simu;
    return 0;
}

int main(int argc, char *argv[]) {
//
// 0 -> GA_TA
// 1 -> Elitista
// 2 -> SteadyState
// 3 -> GA_E
// 4 -> NSGA
//        -> 0 makespan
//        -> 1 st
//        -> 2 to origem

  run("100", "1000", 1, "AS_HOLD", 1, 4, 2);

  // agente frequencia semente, problema, melhorInd tipo_GA objetivo
   /// run( argv[1], argv[2], atoi(argv[3]), argv[4], atoi(argv[5]), atoi(argv[6]), atoi(argv[7]));

   return 0;
}
