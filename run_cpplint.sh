#!bin/bash
cpplint $( find . -name *.cpp ) $( find . -name *.hpp )> results/cpplint_result.txt
echo "Results are stored in results/cpplint_result.txt"