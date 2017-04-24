/******************************************************************************
 * NREdge.cpp
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

#include "NREdge.h"
#include <iostream>

using namespace std;

NREdge::NREdge(int id_)
    : id(id_), weight(0), weight_2nd(0), length(-1.0) {
}

NREdge::NREdge(int id_, NRVertex *vtx1_, NRVertex *vtx2_)
    : id(id_), weight(0), weight_2nd(0), length(-1.0) {
    vertices.vtx1 = vtx1_;
    vertices.vtx2 = vtx2_;
}

int NREdge::getId() const {
    return id;
}

void NREdge::setId(int value) {
    id = value;
}

int NREdge::getWeight() const {
    return weight;
}

void NREdge::setWeight(int value) {
    weight = value;
}

Vertices NREdge::getVertices() const {
    return vertices;
}

void NREdge::setVertices(const Vertices &value) {
    vertices = value;
}

NRVertex *NREdge::getAnotherVertex(NRVertex *oneV) {
    if(oneV == vertices.vtx1) {
        return vertices.vtx2;
    } else if(oneV == vertices.vtx2) {
        return vertices.vtx1;
    } else {
        cout << "[Warning] Accessing a non existing vertex." << endl;
        return NULL;
    }
}

double NREdge::getLength() const {
    return length;
}

void NREdge::setLength(double value) {
    length = value;
}

void NREdge::addEIdUpperLayer(int eId) {
    eIdsUpperLayer.push_back(eId);
}

void NREdge::getEIdsUpperLayer(std::vector<int> &out) {
    out = eIdsUpperLayer;
}

int NREdge::getEIdsUpperLayerCount() {
    return eIdsUpperLayer.size();
}

int NREdge::getWeight_2nd() const {
    return weight_2nd;
}

void NREdge::setWeight_2nd(int value) {
    weight_2nd = value;
}
