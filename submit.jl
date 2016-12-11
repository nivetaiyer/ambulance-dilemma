#!/usr/bin/bash

#$ -N ambulance_test
#$ -o job.out
#$ -e job.error
#$ -cwd

julia test_script.jl
