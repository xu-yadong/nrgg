/******************************************************************************
 * NRGG.h
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

#ifndef NRGG_H
#define NRGG_H

/**
 * @brief NRGG_part_graph partition a spatial graph using the NRGG algorithm
 * @param nvtxs number of vertices
 * @param vwgt weights of vertices
 * @param vcrdnt coordinates of vertices
 * @param xadj indices of the adjacency of vertices
 * @param adjncy adjacency of vertices
 * @param adjwgt weight of the adjacencies
 * @param nparts number of partitions to be partitioned
 * @param part_out the partitioning result, ordered with vertex ids
 * @param edge_out total weight of the edge cut
 * @param wcout whether std output is enabled
 * @param imb_thres two thresholds of imbalance allowance used for refinement
 * @param rf_itr maximum number of iterations used for refinement
 * @param multilvl whether multilevel partitioning should be used
 * @param cstvtx maximum number of vertices in the coarsest graph if multilevel partitioning is used
 * @return whether the partioning is successful
 */
int NRGG_part_graph(int* nvtxs, int* vwgt, double* vcrdnt, int* xadj, int* adjncy, int* adjwgt, int* nparts, int* part_out, int* edge_out, bool wcout = true, double* imb_thres = NULL, int* rf_itr = NULL, bool multilvl = false, int* cstvtx = NULL);


#endif // NRGG_H
