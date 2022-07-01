#!/bin/bash
#freq = ["0.2","0.5","1","2","5","10"]
#agents = ["10","20","30","40","50"]
#seed = [1..30]
#bestInd = 0 (nao cria um melhor individuo) 
#          1 (cria melhor individuo)
# 0 -> GA_TA
# 1 -> Elitista
# 2 -> SteadyState
# 3 -> GA_E
# 4 -> NSGA
#        -> 0 makespan
#        -> 1 st
#        -> 2 to origem

# ./exec_mapd agents freq seed problem bestInd tipo_GA objetivo

for seed in {1..30};do
	for ag in "100" "200" "300" "400" "500"; do
	    ./exec_mapd $ag 1000 $seed AS_HOLD 1 4 0
	done
done
