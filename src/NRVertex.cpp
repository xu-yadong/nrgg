/******************************************************************************
 * NRVertex.cpp
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

#include "NRVertex.h"
#include "NREdge.h"
#include <iostream>

using namespace std;


NRVertex::NRVertex(int id_)
    : id(id_), weight(0), xCoor(-1.0), yCoor(-1.0) {
}

NRVertex::NRVertex(int id_, std::vector<NREdge*> &edges_)
    : id(id_), weight(0), xCoor(-1.0), yCoor(-1.0) {
    edges = edges_;
}

void NRVertex::addEdge(NREdge *edge) {
    if(!edge) {
        cout << "[Error]: adding NULL edge to DLBVertex." << endl;
    }
    edges.push_back(edge);
}

void NRVertex::addIdUpperLayer(int idtoadd) {
    vIdsUpperLayer.push_back(idtoadd);
}

int NRVertex::getId() const {
    return id;
}

void NRVertex::setId(int value) {
    id = value;
}

int NRVertex::getWeight() const {
    return weight;
}

void NRVertex::setWeight(int value) {
    weight = value;
}

void NRVertex::setCoords(double x, double y) {
    xCoor = x;
    yCoor = y;
}

std::pair<double, double> NRVertex::getCoords() {
    return pair<double, double>(xCoor, yCoor);
}

std::vector<int> &NRVertex::getVIdsUpperLayer() {
    return vIdsUpperLayer;
}

void NRVertex::setVIdsUpperLayer(const std::vector<int> &value) {
    vIdsUpperLayer = value;
}

void NRVertex::addVidUpperLayer(int vId) {
    vIdsUpperLayer.push_back(vId);
}

std::vector<NREdge *> NRVertex::getEdges() {
    return edges;
}

void NRVertex::setEdges(const std::vector<NREdge *> &value) {
    edges = value;
}

bool NRVertex::hasConnWithVertex(int vtxId) {
    for(unsigned int i = 0, n = edges.size(); i < n; i++) {
        NREdge* curE = edges.at(i);
        if(curE->getAnotherVertex(this)->getId() == vtxId) {
            return true;
        }
    }
    return false;
}

NREdge *NRVertex::getEdgeForVertex(int vtxId) {
    for(unsigned int i = 0, n = edges.size(); i < n; i++) {
        NREdge* curE = edges.at(i);
        if(curE->getAnotherVertex(this)->getId() == vtxId) {
            return curE;
        }
    }
    return NULL;
}

double NRVertex::getXCoor() const {
    return xCoor;
}

void NRVertex::setXCoor(double value) {
    xCoor = value;
}

double NRVertex::getYCoor() const {
    return yCoor;
}

void NRVertex::setYCoor(double value) {
    yCoor = value;
}
