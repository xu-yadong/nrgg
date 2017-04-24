/******************************************************************************
 * NRGraph.cpp
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

#include "NRGraph.h"
#include <iostream>
#include <algorithm>

using namespace std;

NRGraph::NRGraph()
    : partitionCount(0), totalWeight(0), emptyVertexCount(0) {
}

int NRGraph::getEmptyVertexCount() const {
    return emptyVertexCount;
}

void NRGraph::setEmptyVertexCount(int value) {
    emptyVertexCount = value;
}

void NRGraph::increaseEmptyVertexCount() {
    emptyVertexCount++;
}

int NRGraph::getNonEmptyVertexCount() {
    return getVertexCount() - emptyVertexCount;
}

int NRGraph::getEdgeCount() const {
    return edges.size();
}

int NRGraph::getVertexCount() const {
    return vertices.size();
}

NREdge *NRGraph::getEdge(int id) const {
    EdgeMap::const_iterator found = edges.find(id);
    if(found != edges.end()) {
        return found->second;
    } else {
        cout << "[Error] edge " << id << " is not found" << endl;
        return NULL;
    }
}

NRVertex *NRGraph::getVertex(int id) const {
    VertexMap::const_iterator found = vertices.find(id);
    if(found != vertices.end()) {
        return found->second;
    } else {
        cout << "[Error] vertex " << id << " is not found" << endl;
        return NULL;
    }
}

void NRGraph::addVertex(NRVertex *vertex) {
    if(!vertex) {
        cout << "[Error] Adding NULL vertex to the graph." << endl;
    }
    vertices.insert(VertexMap::value_type(vertex->getId(), vertex));
}

void NRGraph::addEdge(NREdge * edge) {
    if(!edge) {
        cout << "[Error] Adding NULL edge to the graph." << endl;
    }
    edges.insert(EdgeMap::value_type(edge->getId(), edge));
}

void NRGraph::getNeighbourVertices(int vertexId, std::vector<NRVertex *> &out) {
    VertexMap::iterator found = vertices.find(vertexId);
    if(found != vertices.end()) {
        NRVertex* curV = found->second;
        const vector<NREdge*>& connectingEdges = curV->getEdges();
        for(unsigned int i = 0; i < connectingEdges.size(); i++) {
            NREdge* curE = connectingEdges.at(i);
            out.push_back(curE->getAnotherVertex(curV));
        }
    } else {
        cout << "[Error] vertex " << vertexId << " is not found to get neighbours" << endl;
    }
}

void NRGraph::getConnectingEdges(int vertexId, std::vector<NREdge *> & out) {
    VertexMap::iterator found = vertices.find(vertexId);
    if(found != vertices.end()) {
        NRVertex* curV = found->second;
        out = curV->getEdges();
    } else {
        cout << "[Error] vertex " << vertexId << " is not found to get connecting edges" << endl;
    }
}

Vertices NRGraph::getVertices(int edgeId) {
    EdgeMap::iterator found = edges.find(edgeId);
    if(found != edges.end()) {
        NREdge* curE = found->second;
        return curE->getVertices();
    } else {
        cout << "[Error] edge " << edgeId << " is not found for getting vertices" << endl;
        Vertices ver;
        return ver;
    }
}

void NRGraph::getAllVertices(std::vector<NRVertex *> &out) {
    for(VertexMap::iterator iter = vertices.begin(); iter != vertices.end(); iter++) {
        out.push_back(iter->second);
    }
}

void NRGraph::getAllEdges(std::vector<NREdge *> &out) {
    for(EdgeMap::iterator iter = edges.begin(); iter != edges.end(); iter++) {
        out.push_back(iter->second);
    }
}

bool NRGraph::isEdgeExist(int vtx1, int vtx2) {
    VertexMap::iterator found1 = vertices.find(vtx1);
    if(found1 == vertices.end()) {
        return false;
    } else if(vertices.find(vtx2) == vertices.end()) {
        return false;
    } else {
        NRVertex* curV = found1->second;
        if(curV->hasConnWithVertex(vtx2)) {
            return true;
        } else {
            return false;
        }
    }
}

NREdge *NRGraph::getEdgeByVertices(const int vtx1Id, const int vtx2Id) {
    VertexMap::iterator found1 = vertices.find(vtx1Id);
    if(found1 == vertices.end()) {
        return NULL;
    } else if(vertices.find(vtx2Id) == vertices.end()) {
        return NULL;
    } else {
        NRVertex* curV = found1->second;
        return curV->getEdgeForVertex(vtx2Id);
    }
}

void NRGraph::setVertexPartition(int vId, int pId) {
    partitions[vId] = pId;
}

void NRGraph::setPartitionWeight(int pId, int weight) {
    partitionWeights[pId] = weight;
}

void NRGraph::increasePartitionWeight(int pId, int weight) {
    PartitionWeightMap::iterator found = partitionWeights.find(pId);
    if(found != partitionWeights.end()) {
        found->second += weight;
    } else {
        partitionWeights[pId] = weight;
    }
}

void NRGraph::decreasePartitionWeight(int pId, int weight) {
    PartitionWeightMap::iterator found = partitionWeights.find(pId);
    if(found != partitionWeights.end()) {
        found->second -= weight;
    }
}

int NRGraph::getPartitionWeight(int pId) {
    PartitionWeightMap::iterator found = partitionWeights.find(pId);
    if(found != partitionWeights.end()) {
        return found->second;
    } else {
        return 0;
    }
}

void NRGraph::resetPartitionWeight() {
    partitionWeights.clear();
}

bool NRGraph::hasEmptyPartition() {
    if(partitionCount > (int)partitionWeights.size()) {
        return true;
    }
    for(PartitionWeightMap::iterator iter = partitionWeights.begin(); iter != partitionWeights.end(); iter++) {
        if(iter->second == 0) {
            return true;
        }
    }
    return false;
}

int NRGraph::getVertexPartition(int vId) {
    PartitionMap::iterator found = partitions.find(vId);
    if(found != partitions.end()) {
        return found->second;
    } else {
        return -1;
    }
}

int NRGraph::getPartitionCount() const {
    return partitionCount;
}

void NRGraph::setPartitionCount(int value) {
    partitionCount = value;
}

int NRGraph::getTotalWeight() const {
    return totalWeight;
}

void NRGraph::setTotalWeight(int value) {
    totalWeight = value;
}

NRGraph::EdgeIterator NRGraph::edge_begin() {
    return edges.begin();
}

NRGraph::EdgeIterator NRGraph::edge_end() {
    return edges.end();
}

NRGraph::VertexIterator NRGraph::vertex_begin() {
    return vertices.begin();
}

NRGraph::VertexIterator NRGraph::vertex_end() {
    return vertices.end();
}

NRGraph::~NRGraph() {
    for(EdgeMap::iterator iter = edges.begin(); iter != edges.end(); iter++) {
        delete iter->second;
    }
    for(VertexMap::iterator iter = vertices.begin(); iter != vertices.end(); iter++) {
        delete iter->second;
    }
}
