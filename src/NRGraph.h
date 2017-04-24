/******************************************************************************
 * NRGraph.h
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

#ifndef GRAPH_H
#define GRAPH_H

#include "NREdge.h"
#include "NRVertex.h"
#include <unordered_map>

class NRGraph {

public:

    typedef std::unordered_map<int, int> PartitionMap;

private:

    typedef std::unordered_map<int, NREdge*> EdgeMap;
    typedef std::unordered_map<int, NRVertex*> VertexMap;
    typedef std::unordered_map<int, int> PartitionWeightMap;   // <pId, weight>

    EdgeMap edges;
    VertexMap vertices;
    PartitionWeightMap partitionWeights;
    int partitionCount;
    int totalWeight;
    int emptyVertexCount;


public:

    PartitionMap partitions;

    typedef std::unordered_map<int, NREdge*>::iterator EdgeIterator;
    typedef std::unordered_map<int, NRVertex*>::iterator VertexIterator;

    NRGraph();

    int getEdgeCount() const;
    int getVertexCount() const;
    NREdge* getEdge(int id) const;
    NRVertex* getVertex(int id) const;
    void addVertex(NRVertex *vertex);
    void addEdge(NREdge* edge);

    void getNeighbourVertices(int vertexId, std::vector<NRVertex*>& out);
    void getConnectingEdges(int vertexId, std::vector<NREdge *> &out);

    Vertices getVertices(int edgeId);

    void getAllVertices(std::vector<NRVertex*> &out);

    void getAllEdges(std::vector<NREdge*>& out);

    bool isEdgeExist(int vtx1, int vtx2);

    NREdge *getEdgeByVertices(const int vtx1Id, const int vtx2Id);

    void setVertexPartition(int vId, int pId);

    void setPartitionWeight(int pId, int weight);

    void increasePartitionWeight(int pId, int weight);
    void decreasePartitionWeight(int pId, int weight);

    int getPartitionWeight(int pId);

    void resetPartitionWeight();

    bool hasEmptyPartition();

    int getVertexPartition(int vId);
    int getPartitionCount() const;
    void setPartitionCount(int value);
    int getTotalWeight() const;
    void setTotalWeight(int value);
    int getEmptyVertexCount() const;
    void setEmptyVertexCount(int value);
    void increaseEmptyVertexCount();

    int getNonEmptyVertexCount();

    EdgeIterator edge_begin();

    EdgeIterator edge_end();

    VertexIterator vertex_begin();

    VertexIterator vertex_end();

    ~NRGraph();
};

#endif // GRAPH_H
