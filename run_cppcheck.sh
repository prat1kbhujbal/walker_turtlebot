#!bin/bash
cppcheck --language=c++ --std=c++11 -I include --suppress=missingIncludeSystem  $( find . -name \*.hpp -or -name \*.cpp) > results/cppcheck_result.txt
echo "Results are stored in results/cppcheck_result.txt"