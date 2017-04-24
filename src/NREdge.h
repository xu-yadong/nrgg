/******************************************************************************
 * NREdge.h
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

#ifndef EDGE_H
#define EDGE_H

#include <vector>

class NRVertex;

struct Vertices {

    NRVertex* vtx1;
    NRVertex* vtx2;

};

class NREdge {

private:

    int id;

    int weight;

    int weight_2nd;

    double length;

    Vertices vertices;

    std::vector<int> eIdsUpperLayer;

public:

    NREdge(int id_);

    NREdge(int id_, NRVertex* vtx1_, NRVertex* vtx2_);

    int getId() const;
    void setId(int value);
    int getWeight() const;
    void setWeight(int value);
    Vertices getVertices() const;
    void setVertices(const Vertices &value);

    NRVertex* getAnotherVertex(NRVertex* oneV);
    double getLength() const;
    void setLength(double value);

    void addEIdUpperLayer(int eId);
    void getEIdsUpperLayer(std::vector<int>& out);
    int getEIdsUpperLayerCount();
    int getWeight_2nd() const;
    void setWeight_2nd(int value);
};

#endif // EDGE_H
