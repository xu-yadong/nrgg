# nrgg-1.0
===
This is an implementation of the published work:
Y. Xu, W. Cai, D. Eckhoff, S. Nair, A. Knoll, "A Graph Partitioning Algorithm
for Parallel Agent-Based Road Traffic Simulation", In ACM SIGSIM PADS'17,
May 24-26, 2017, Singapore, DOI: http://dx.doi.org/10.1145/3064911.3064914

===

This program does not need external libraries except standard C++ libraries.
To compile the source code, make sure you have g++ in your PATH, just run the script

compile.sh 

file under in compile folder. A static library will be generated in the same folder.

To use NRGG in you program, include the header file "nrgg.h" in your program and link the
static library generated to your program.

For how to use the API, please refer to the NRGG_user_guide.pdf file in the same folder.

For any feedback or bug report, please email to: xuya0006@e.ntu.edu.sg
