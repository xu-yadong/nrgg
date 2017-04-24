/******************************************************************************
 * NRVertex.h
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

#ifndef VERTEX_H
#define VERTEX_H

#include <vector>

class NREdge;

class NRVertex {

private:

    int id;

    int weight;

    double xCoor;   /**< coordinate on the x-axis */

    double yCoor;   /**< coordinate on the y-axis */

    std::vector<NREdge*> edges; /**< edges connecting to this vertex */

    std::vector<int> vIdsUpperLayer;    /**< the corresponding ids of the vertices at a less coarser level */

public:

    NRVertex(int id_);

    NRVertex(int id_, std::vector<NREdge*>& edges_);

    void addEdge(NREdge* edge);

    void addIdUpperLayer(int idtoadd);

    int getId() const;
    void setId(int value);
    int getWeight() const;
    void setWeight(int value);
    void setCoords(double x, double y);
    std::pair<double, double> getCoords();
    std::vector<int>& getVIdsUpperLayer();
    void setVIdsUpperLayer(const std::vector<int> &value);
    void addVidUpperLayer(int vId);
    std::vector<NREdge *> getEdges();
    void setEdges(const std::vector<NREdge *> &value);

    bool hasConnWithVertex(int vtxId);
    NREdge* getEdgeForVertex(int vtxId);
    double getXCoor() const;
    void setXCoor(double value);
    double getYCoor() const;
    void setYCoor(double value);
};

#endif // VERTEX_H
