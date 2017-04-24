/******************************************************************************
 * UtilFunctions.h
 *
 * Source of NRGG -- Neighbour-restricting Graph-Grow partitioning algorithm
 *
 * This is an implementation of the published work:
 * Y. Xu, W. Cai, D. Eckhoff, S. Nair, A. Knoll, "A Graph Partitioning Algorithm
 * for Parallel Agent-Based Road Traffic Simulation", In ACM SIGSIM PADS'17,
 * May 24-26, 2017, Singapore, DOI: http://dx.doi.org/10.1145/3064911.3064914
 *
 ******************************************************************************
 * Copyright (C) 2016 - 2017 Yadong Xu <xuya0006@e.ntu.edu.sg>
 *
 * This file is part of NRGG.

 * NRGG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * NRGG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with NRGG.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#ifndef UTILITY_H
#define UTILITY_H

class NRGraph;
class NRVertex;

typedef enum {
    NRGG_MULTI_LEVEL = 0,
    NRGG_SINGLE_LEVEL = 1
} part_type;

typedef enum {
    X_DIR = 0,
    Y_DIR = 1
} grow_direction_type;

double imb_low_thres = 0.9; // Input from user
double imb_high_thres = 1.02; // Input from user
int refinement_iter = 8; // Input from user
bool with_cout = true; // Input from user

int coarsest_min_vertex_per_part = 100; // Input from user
part_type part_t = NRGG_SINGLE_LEVEL; // Input from user

NRGraph* partGraph = NULL;  // internally used
NRVertex* leftMostVertex = NULL;  // internally used
NRVertex* bottomVertex = NULL;  // internally used
grow_direction_type grow_dir = X_DIR;   // internally used, will be modified according to the bounding box of vertices

/**
 * @brief The VTuple class a tuple (i, v), where i is the partition id, v is the vertex, used as a priority queue element for graph growing
 */
class VTuple {
public:
    int partitionId;
    NRVertex* vertex;

    VTuple() {}
    VTuple(int pId, NRVertex* v)
        : partitionId(pId),
          vertex(v) {}
};

/**
 * @brief graphGrowLeftRight Grow from the left side, make sure that no jumping of neighbours
 * @param graph
 * @param partitionCount
 */
void graphGrowLeftRight(NRGraph *graph, int partitionCount);

/**
 * @brief graphGrowBottomTop Grow from the bottom to the top (larger YCoord), make sure that no jumping of neighbours
 * @param graph
 * @param partitionCount
 */
void graphGrowBottomTop(NRGraph *graph, int partitionCount);

/**
 * @brief KLRefineGR Greedy Refinement in METIS
 * @param graph
 */
void KLRefineGR(NRGraph* graph);

/**
 * @brief HEMcoarsen same with METIS, heavy edge random matching.
 * @param finerGraph
 * @return the pointer to the coarsened graph
 */
NRGraph* HEMcoarsen(NRGraph* finerGraph);

/**
 * @brief uncoarsen uncoarsen the layers of coarsened graphs stored in the array provided
 * @param layers
 */
void uncoarsen(std::vector<NRGraph*>& layers);

/**
 * @brief constructPartGraph contruct a graph data structure using the data provided, and set parameters
 * @param nvtxs
 * @param vwgt
 * @param vcrdnt
 * @param xadj
 * @param adjncy
 * @param adjwgt
 */
void constructPartGraph(int *nvtxs, int *vwgt, double *vcrdnt, int *xadj, int *adjncy, int* adjwgt);

/**
 * @brief partitionGraph partition the graph into partitionCount partitions. The graph should be preprocessed to be partitionable.
 * @param partitionCount
 */
void partitionGraph(int partitionCount);

/**
 * @brief outputPartResult put the partitioning result into the provided int array
 * @param part_out array of partitions in the order of {vId, pId,}
 * @param edge_out total weight of edge cut
 * @return whether the partitioning was successful
 */
int outputPartResult(int* part_out, int *edge_out);

#endif // UTILITY_H
