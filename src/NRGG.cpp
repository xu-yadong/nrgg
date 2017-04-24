/******************************************************************************
 * NRGG.cpp
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


#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <iostream>
#include <chrono>
#include <limits>
#include "NRGraph.h"
#include "NRGG.h"
#include "Utility.h"

using namespace std;

struct VTupleXGreaterComparator {
    // greater comparator results in ascending order of the queue.
    bool operator()(const VTuple* vt1, const VTuple* vt2) {
        // if vt1->partitionId > vt2->partitionId, v1 will appear more to the back than vt2.
        // smaller first. so it is the reverse.
        if(vt1->partitionId > vt2->partitionId) {
            return true;
        } else if (vt1->partitionId < vt2->partitionId) {
            return false;
        } else { // it is unlikely for two float values to be equal, so just use '>' to compare
            return vt1->vertex->getXCoor() > vt2->vertex->getXCoor();
        }
    }
};

struct VTupleYGreaterComparator {
    // greater comparator results in ascending order of the queue.
    bool operator()(const VTuple* vt1, const VTuple* vt2) {
        // if vt1->partitionId > vt2->partitionId, v1 will appear more to the back than vt2.
        // smaller first. so it is the reverse.
        if(vt1->partitionId > vt2->partitionId) {
            return true;
        } else if (vt1->partitionId < vt2->partitionId) {
            return false;
        } else { // it is unlikely for two float values to be equal, so just use '>' to compare
            return vt1->vertex->getYCoor() > vt2->vertex->getYCoor();
        }
    }
};

void graphGrowLeftRight(NRGraph* graph, int partitionCount) {
    // get the total weight, and then partition by iterating through
    int totalWeight = graph->getTotalWeight();
    int singleWeight = totalWeight / partitionCount;
    int accuWeight = 0;
    int curPartition = 0;
    graph->setPartitionCount(partitionCount);
    if(with_cout) {
        cout << "[Info] Graph-grow phase started: Total vertex weight = " << totalWeight << ", weight per partition=" << singleWeight << endl;
    }
    auto ggStart = chrono::steady_clock::now();

    priority_queue<VTuple*, vector<VTuple*>, VTupleXGreaterComparator> queue;
    std::unordered_set<NRVertex*> enqueued;

    queue.push(new VTuple(curPartition, leftMostVertex));   // start from the left-most vertex
    enqueued.insert(leftMostVertex);

    while(!queue.empty()) {
        VTuple* curTuple = queue.top();
        NRVertex* curV = curTuple->vertex;
        queue.pop();
        delete curTuple;

        int curWeight = curV->getWeight();
        double rdm = (double) rand()/(RAND_MAX);

        if((accuWeight > singleWeight || (accuWeight + curWeight > singleWeight && rdm < 0.5) )
                && curPartition < partitionCount - 1) {
            curPartition++;
            accuWeight = curWeight;
            graph->setPartitionWeight(curPartition, curWeight);
        } else {
            accuWeight += curWeight;
            graph->increasePartitionWeight(curPartition, curWeight);
        }

        graph->setVertexPartition(curV->getId(), curPartition);

        vector<NRVertex*> preVNeighbours;  // put the neighbours of the current vertex first.
        graph->getNeighbourVertices(curV->getId(), preVNeighbours);
        for(unsigned int preNI = 0; preNI < preVNeighbours.size(); preNI++) {
            NRVertex* curNV = preVNeighbours.at(preNI);
            if(enqueued.find(curNV) == enqueued.end()) {
                queue.push(new VTuple(curPartition, curNV));
                enqueued.insert(curNV);
            }
        }
    }

    auto ggEnd = chrono::steady_clock::now();
    if(with_cout) {
        cout << "[Info] Graph-grow phase ended. It took " << ((float)(chrono::duration <double, milli> (ggEnd - ggStart).count()))/1000.0 << " seconds." << endl;
    }
}

void graphGrowBottomTop(NRGraph *graph, int partitionCount) {
    // get the total weight, and then partition by iterating through
    int totalWeight = graph->getTotalWeight();
    int singleWeight = totalWeight / partitionCount;
    int accuWeight = 0;
    int curPartition = 0;
    graph->setPartitionCount(partitionCount);
    if(with_cout) {
        cout << "[Info] NRGG partitioning: Total vertex weight = " << totalWeight << ", weight per partition=" << singleWeight << endl;
    }

    priority_queue<VTuple*, vector<VTuple*>, VTupleYGreaterComparator> queue;
    std::unordered_set<NRVertex*> enqueued;

    queue.push(new VTuple(curPartition, bottomVertex));   // start from the left-most vertex
    enqueued.insert(bottomVertex);

    while(!queue.empty()) {
        VTuple* curTuple = queue.top();
        NRVertex* curV = curTuple->vertex;
        queue.pop();
        delete curTuple;

        int curWeight = curV->getWeight();
        double rdm = (double) rand()/(RAND_MAX);

        if((accuWeight > singleWeight || (accuWeight + curWeight > singleWeight && rdm < 0.5) )
                && curPartition < partitionCount - 1) {
            curPartition++;
            accuWeight = curWeight;
            graph->setPartitionWeight(curPartition, curWeight);
        } else {
            accuWeight += curWeight;
            graph->increasePartitionWeight(curPartition, curWeight);
        }

        graph->setVertexPartition(curV->getId(), curPartition);

        vector<NRVertex*> preVNeighbours;  // put the neighbours of the current vertex first.
        graph->getNeighbourVertices(curV->getId(), preVNeighbours);
        for(unsigned int preNI = 0; preNI < preVNeighbours.size(); preNI++) {
            NRVertex* curNV = preVNeighbours.at(preNI);
            if(enqueued.find(curNV) == enqueued.end()) {
                queue.push(new VTuple(curPartition, curNV));
                enqueued.insert(curNV);
            }
        }
    }
}

NRGraph *HEMcoarsen(NRGraph *finerGraph) {
    NRGraph* coarserGraph = new NRGraph();

    std::unordered_map<int, int> fineToCoarseMatch;

    vector<NRVertex*> allVertices;
    finerGraph->getAllVertices(allVertices);

    while(!allVertices.empty()) {  // loop until all vertices are matched
        int index = std::rand() % allVertices.size();
        NRVertex* curV = allVertices[index];
        if(fineToCoarseMatch.find(curV->getId()) == fineToCoarseMatch.end()) {
            // match it if it is not matched yet
            // create a coarse vertex first
            NRVertex* newCoarseV = new NRVertex(curV->getId());    // use the same id as the first vertex

            const vector<NREdge*>& allEdges = curV->getEdges();
            int heaviestEdgeIndex = -1;
            int maxWeight = -1;
            for(unsigned int i = 0; i < allEdges.size(); i++) { // O(|E|)
                NREdge* curE = allEdges.at(i);
                if(curE->getWeight() > maxWeight && fineToCoarseMatch.find(curE->getAnotherVertex(curV)->getId()) == fineToCoarseMatch.end()) {
                    maxWeight = curE->getWeight();
                    heaviestEdgeIndex = i;
                }
            }
            if(heaviestEdgeIndex > -1) { // matched with another vertex
                NREdge* matchedEdge = allEdges.at(heaviestEdgeIndex);
                NRVertex* matchedVertex = matchedEdge->getAnotherVertex(curV);
                newCoarseV->setWeight(curV->getWeight() + matchedVertex->getWeight());
                newCoarseV->setXCoor((curV->getXCoor() + matchedVertex->getXCoor()) / 2);
                newCoarseV->setYCoor((curV->getYCoor() + matchedVertex->getYCoor()) / 2);
                newCoarseV->addVidUpperLayer(matchedVertex->getId());

                fineToCoarseMatch.insert(pair<int,int>(matchedVertex->getId(), newCoarseV->getId()));

            if(leftMostVertex == matchedVertex) {
                leftMostVertex = newCoarseV;
            }
            if(bottomVertex == matchedVertex) {
                bottomVertex = newCoarseV;
            }

            } else { // not matched with any vertex

                newCoarseV->setWeight(curV->getWeight());
                newCoarseV->setXCoor(curV->getXCoor());
                newCoarseV->setYCoor(curV->getYCoor());
            }

            newCoarseV->addVidUpperLayer(curV->getId());
            coarserGraph->addVertex(newCoarseV);
            fineToCoarseMatch.insert(pair<int,int>(curV->getId(), curV->getId()));

            // Update lefmost and bottom vertices
            if(leftMostVertex == curV) {
                leftMostVertex = newCoarseV;
            }
            if(bottomVertex == curV) {
                bottomVertex = newCoarseV;
            }
        }
        allVertices.erase(allVertices.begin() + index);
    }

    // match edges
    for(NRGraph::EdgeIterator iter = finerGraph->edge_begin(), iter_end = finerGraph->edge_end(); iter != iter_end; iter++) {

        NREdge* curE = iter->second;

        Vertices vs = curE->getVertices();
        int fine1Id = vs.vtx1->getId();
        int fine2Id = vs.vtx2->getId();

        int coarse1Id = fineToCoarseMatch[fine1Id];
        int coarse2Id = fineToCoarseMatch[fine2Id];
        if(coarse1Id != coarse2Id) {
            NREdge* coarseEdge = coarserGraph->getEdgeByVertices(coarse1Id, coarse2Id);
            if(!coarseEdge) {
                NRVertex* v1 = coarserGraph->getVertex(coarse1Id);
                NRVertex* v2 = coarserGraph->getVertex(coarse2Id);
                coarseEdge = new NREdge(iter->first, v1, v2);
                coarseEdge->setWeight(curE->getWeight());
                coarserGraph->addEdge(coarseEdge);
                v1->addEdge(coarseEdge);
                v2->addEdge(coarseEdge);
            } else {
                coarseEdge->setWeight(coarseEdge->getWeight() + curE->getWeight());
            }
            coarseEdge->addEIdUpperLayer(iter->first);
        }
    }
    coarserGraph->setTotalWeight(finerGraph->getTotalWeight());

    return coarserGraph;
}

void uncoarsen(std::vector<NRGraph *> &layers) {
    // map back the partitions
    cout << "Uncoarsening..." << endl;
    for(unsigned int i = layers.size() - 1; i > 0; i--) {
        NRGraph* coarserGraph = layers.at(i);
        NRGraph* finerGraph = layers.at(i - 1);
        int count = 0;
        if(with_cout)
            cout << endl << "Layer " << i << "(" << coarserGraph->getVertexCount() << " vertices) " << " to " << (i-1) << "(" << finerGraph->getVertexCount() << " vertices)" << endl;
        for(NRGraph::VertexIterator iter = coarserGraph->vertex_begin(), iter_end = coarserGraph->vertex_end(); iter != iter_end; iter++) {
            int vId = iter->first;
            NRVertex* curV = iter->second;
            int partId = coarserGraph->getVertexPartition(vId);

            vector<int>& finerIds = curV->getVIdsUpperLayer();
            for(unsigned int fii = 0; fii < finerIds.size(); fii++) {
                finerGraph->setVertexPartition(finerIds.at(fii), partId);
                count++;
            }
        }
        if(with_cout)
            cout << count << " vertices in finer graph has its partition" << endl;
        for(int pi = 0, pn = coarserGraph->getPartitionCount(); pi < pn; pi++) {
            finerGraph->setPartitionWeight(pi, coarserGraph->getPartitionWeight(pi));
        }
        finerGraph->setPartitionCount(coarserGraph->getPartitionCount());

        // KL refinement
        KLRefineGR(finerGraph);
    }
}

void KLRefineGR(NRGraph *graph) {
    if(with_cout) {
        cout << endl << "[Info] Refinement phases started: imb_low_thres=" << imb_low_thres << ", imb_high_thres=" << imb_high_thres << ", ";
    }
    auto refineStart = chrono::steady_clock::now();
    int Wmin = imb_low_thres * graph->getTotalWeight() / graph->getPartitionCount();  // min weight a partition can have
    int Wmax = (Wmin / imb_low_thres) * imb_high_thres;    // max weight a partition can have
    if(with_cout) {
        cout << "Wmin=" << Wmin << ", Wmax=" << Wmax << endl;
    }

    // 1. Caculate all boundary vertices and interval and external weights
    std::unordered_map<NRVertex*, pair<int,int> > boundaryVertices;  // <vPtr, <targetlp,gain>> to track the boundary vertices
    map<int, std::unordered_set<NRVertex*> > buckets;   // group boundary vertices into buckets according to "gain"
    // *****Scan all Vertices to determine boundary, put them in bV and buck
    for(NRGraph::VertexIterator vIter = graph->vertex_begin(), vIter_end = graph->vertex_end(); vIter != vIter_end; vIter++) {
        NRVertex* curV = vIter->second;
        int curVId = vIter->first;
        vector<NREdge*> neighbourEs = curV->getEdges();
        int curVParId = graph->getVertexPartition(curVId);
        std::unordered_set<int> neighbourLPs;
        int curID = 0;
        int curED = 0;
        for(unsigned int eC = 0; eC < neighbourEs.size(); eC++) {
            NREdge* curNeighbourE = neighbourEs.at(eC);
            NRVertex* curNeighbourV = curNeighbourE->getAnotherVertex(curV);
            int curNeighbourVId = curNeighbourV->getId();
            int curNeighbourVParId = graph->getVertexPartition(curNeighbourVId);
            if(curVParId == curNeighbourVParId) {
                curID += curNeighbourE->getWeight();
            } else {
                neighbourLPs.insert(curNeighbourVParId);
                curED += curNeighbourE->getWeight();
            }
        }
        // put into ID and ED, and boundary, and buckets
        if(neighbourLPs.size() == 1) {  // if only next to one LP, put is as a boundary vertex
            pair<int,int> newPair(*neighbourLPs.begin(), curED-curID);
            boundaryVertices.insert(pair<NRVertex*,pair<int,int> >(curV, newPair));
            map<int, std::unordered_set<NRVertex*> >::iterator foundBucket = buckets.find(curED - curID);
            if(foundBucket != buckets.end()) {
                foundBucket->second.insert(curV);
            } else {
                std::unordered_set<NRVertex*> newBucket;
                newBucket.insert(curV);
                buckets[curED-curID] = newBucket;
            }
        }
    }

    std::unordered_set<NRVertex*> potentialBoundaryVertices;  // <vId> to track the potential boundary vertices

    int movedVInCurIter = 0;
    int iterCount = 0;
    vector<int> movedVCount;
    do {
        // Print out  partitioning information for the first iteration
        if(with_cout && iterCount == 0) {
            cout << endl << "[Info] Pre refinement Weights=";
            for(int ran = 0; ran < graph->getPartitionCount(); ran++) {
                int curWeight = graph->getPartitionWeight(ran);
                cout << curWeight << " ";
            }
            cout << endl;
            std::unordered_map<int, std::unordered_map<int,int> > boundaryEdgeWeights;    // <pId, <pId, boundaryEdgeWeight> >
            for(NRGraph::EdgeIterator iter = graph->edge_begin(), iter_end = graph->edge_end(); iter != iter_end; iter++) {
                NREdge* curE = iter->second;
                Vertices vs = curE->getVertices();
                int v1Id = vs.vtx1->getId();
                int v2Id = vs.vtx2->getId();
                int pV1 = graph->getVertexPartition(v1Id);
                int pV2 = graph->getVertexPartition(v2Id);

                if(pV1 != pV2) {
                    std::unordered_map<int, std::unordered_map<int,int> >::iterator foundP1 = boundaryEdgeWeights.find(pV1);
                    if(foundP1 != boundaryEdgeWeights.end()) {
                        std::unordered_map<int,int>::iterator foundP1P2 = foundP1->second.find(pV2);
                        if(foundP1P2 != foundP1->second.end()) {
                            foundP1P2->second += curE->getWeight();
                        } else {
                            foundP1->second[pV2] = curE->getWeight();
                        }
                    } else {
                        std::unordered_map<int,int> newB;
                        newB.insert(pair<int,int>(pV2, curE->getWeight()));
                        boundaryEdgeWeights[pV1] = newB;
                    }
                    std::unordered_map<int, std::unordered_map<int,int> >::iterator foundP2 = boundaryEdgeWeights.find(pV2);
                    if(foundP2 != boundaryEdgeWeights.end()) {
                        std::unordered_map<int,int>::iterator foundP2P1 = foundP2->second.find(pV1);
                        if(foundP2P1 != foundP2->second.end()) {
                            foundP2P1->second += curE->getWeight();
                        } else {
                            foundP2->second[pV1] = curE->getWeight();
                        }
                    } else {
                        std::unordered_map<int,int> newB;
                        newB.insert(pair<int,int>(pV1, curE->getWeight()));
                        boundaryEdgeWeights[pV2] = newB;
                    }
                }
            }
            cout << "[Info] Pre refinement edge cut statistics" << endl;
            for(std::unordered_map<int, std::unordered_map<int,int> >::iterator p1Iter = boundaryEdgeWeights.begin(); p1Iter != boundaryEdgeWeights.end(); p1Iter++) {
                cout << "Partition " << p1Iter->first << ": ";
                for(std::unordered_map<int,int>::iterator p2Iter = p1Iter->second.begin(); p2Iter != p1Iter->second.end(); p2Iter++) {
                    cout << "neighbour " << p2Iter->first << "(" << p2Iter->second << ") ";
                }
                cout << endl;
            }
        }

        // 1. Update boundary vertices according potential boundary vertices
        for(std::unordered_set<NRVertex*>::iterator pBIter = potentialBoundaryVertices.begin(), pBIter_end = potentialBoundaryVertices.end(); pBIter != pBIter_end; pBIter++) {
            NRVertex* curV = *pBIter;
            int curVId = curV->getId();
            vector<NREdge*> neighbourEs = curV->getEdges();
            int curVParId = graph->getVertexPartition(curVId);
            std::unordered_set<int> neighbourLPs;
            int curID = 0;
            int curED = 0;
            for(unsigned int vC = 0; vC < neighbourEs.size(); vC++) {
                NREdge* curNeighbourE = neighbourEs.at(vC);
                NRVertex* curNeighbourV = curNeighbourE->getAnotherVertex(curV);
                int curNeighbourVId = curNeighbourV->getId();
                int curNeighbourVParId = graph->getVertexPartition(curNeighbourVId);
                if(curVParId == curNeighbourVParId) {
                    curID += curNeighbourE->getWeight();
                } else {
                    neighbourLPs.insert(curNeighbourVParId);
                    curED += curNeighbourE->getWeight();
                }
            }
            // put into boundary, and buckets
            if(neighbourLPs.size() == 1) {  // if only next to one LP (to avoid creating new neighboring LPs by moving vertices), put is as a boundary vertex
                std::unordered_map<NRVertex*, pair<int,int> >::iterator foundInBoundary = boundaryVertices.find(curV);
                if(foundInBoundary != boundaryVertices.end()) {
                    map<int, std::unordered_set<NRVertex*> >::iterator foundOldBucket = buckets.find(foundInBoundary->second.second);
                    foundOldBucket->second.erase(curV);  // remove it from the original buckets
                }
                boundaryVertices[curV] = pair<int,int>(*neighbourLPs.begin(),curED-curID);
                map<int, std::unordered_set<NRVertex*> >::iterator foundNewBucket = buckets.find(curED - curID);
                if(foundNewBucket != buckets.end()) {
                    foundNewBucket->second.insert(curV);
                } else {
                    std::unordered_set<NRVertex*> newBucket;
                    newBucket.insert(curV);
                    buckets[curED-curID] = newBucket;
                }
            } else {
                std::unordered_map<NRVertex*, pair<int,int> >::iterator foundInBoundary = boundaryVertices.find(curV);
                if(foundInBoundary != boundaryVertices.end()) {
                    map<int, std::unordered_set<NRVertex*> >::iterator oldIter = buckets.find(foundInBoundary->second.second);
                    oldIter->second.erase(curV);  // remove it from buckets
                    if(oldIter->second.empty()) {
                        buckets.erase(oldIter);
                    }
                    boundaryVertices.erase(foundInBoundary);   // remove it in case it is in the boundary collection
                }

            }
        }
        if(with_cout) {
            cout << "Current number of boundary vertices is " << boundaryVertices.size() << endl;
        }

        potentialBoundaryVertices.clear();

        // 2. Move vertices. Following is the multi k-way in METIS (improved version)
        movedVInCurIter = 0;
        std::unordered_set<NRVertex*>::iterator curIter;
        while(!buckets.empty()) {
            // get the best gain vertex
            map<int, std::unordered_set<NRVertex*> >::reverse_iterator bestGainIter = buckets.rbegin();// last bucket has the best gain
            if(bestGainIter->second.empty()) {
                cerr << "[Error] The best gain bucket is empty." << endl;
            }
            curIter = bestGainIter->second.begin();
            NRVertex* curV = *curIter;
            int curVId = curV->getId();

            std::unordered_map<NRVertex*, pair<int,int> >::iterator foundInBoundary = boundaryVertices.find(curV);
            int gain = bestGainIter->first;
            int targetPId = foundInBoundary->second.first;
            int curPId = graph->getVertexPartition(curVId);
            int curVWeight = curV->getWeight();

            // put to potential boundary vertices
            potentialBoundaryVertices.insert(curV);
            // remove current element from the buckets
            boundaryVertices.erase(foundInBoundary);
            bestGainIter->second.erase(curV);
            if(bestGainIter->second.empty()) {
                buckets.erase(gain);
            }

            // see if the balance criteria will be satisfied, and gain not less than 0
            bool shouldMove = false;
            int myPWeight = graph->getPartitionWeight(curPId);
            int targetPWeight = graph->getPartitionWeight(targetPId);
            if((gain > 0 && myPWeight - curVWeight > Wmin && targetPWeight + curVWeight < Wmax)
                    || (gain <= 0 && curVWeight > 0 && myPWeight > Wmax && myPWeight - targetPWeight > 2*curVWeight)) {
                shouldMove = true;
            }
            if(shouldMove) {
                if(with_cout) {
                    cout << "gain=" << gain << ": moving vertex " << curVId << " with weight " << curVWeight << " from " << curPId << " to " << targetPId << " weight1=" << myPWeight << ", weight2=" << targetPWeight << endl;
                }
                graph->setVertexPartition(curVId, targetPId);
                graph->increasePartitionWeight(targetPId, curVWeight);
                graph->decreasePartitionWeight(curPId, curVWeight);
                movedVInCurIter++;

                // update all neighbouring nodes ID and ED to reflect the change
                const vector<NREdge*>& neighbourEs = curV->getEdges();
                for(unsigned int ei = 0; ei < neighbourEs.size(); ei++) {
                    NREdge* curNE = neighbourEs.at(ei);
                    NRVertex* curNeighbourV = curNE->getAnotherVertex(curV);
                    int curNeighbourVId = curNeighbourV->getId();
                    int curEWeight = curNE->getWeight();
                    // update the neighbour's id and ed

                    std::unordered_map<NRVertex*, pair<int,int> >::iterator foundNInBoundary = boundaryVertices.find(curNeighbourV);
                    if(curEWeight > 0 && foundNInBoundary != boundaryVertices.end()) {
                        int nPId = graph->getVertexPartition(curNeighbourVId);
                        if(nPId == curPId) {
                            int oldGain = foundNInBoundary->second.second;
                            map<int, std::unordered_set<NRVertex*> >::iterator oldGainIter = buckets.find(oldGain);
                            if(oldGainIter ==buckets.end()) {
                                cout << "[Error] old gain bucket " << oldGain << " not found when nPiD=curPid" << endl;
                            }
                            oldGainIter->second.erase(curNeighbourV);
                            if(oldGainIter->second.empty()) {
                                buckets.erase(oldGainIter);
                            }

                            if(foundNInBoundary->second.first == targetPId) { // if they are facing the same neighbour LP
                                // deduce ID, increase ED
                                int newGain = oldGain + 2 * curEWeight;
                                map<int, std::unordered_set<NRVertex*> >::iterator newGainIter = buckets.find(newGain);

                                if(newGainIter != buckets.end()) {
                                    newGainIter->second.insert(curNeighbourV);
                                } else {
                                    std::unordered_set<NRVertex*> newBucket;
                                    newBucket.insert(curNeighbourV);
                                    buckets[newGain] = newBucket;
                                }
                                // update the entry in boundaryVertices also
                                foundNInBoundary->second.second = newGain;
                            } else {
                                boundaryVertices.erase(curNeighbourV);
                            }

                        } else if(nPId == targetPId) {
                            // deduce ED, increase ID

                            int oldGain = foundNInBoundary->second.second;
                            map<int, std::unordered_set<NRVertex*> >::iterator oldGainIter = buckets.find(oldGain);
                            // remove from previous bucket first
                            if(oldGainIter ==buckets.end()) {
                                cout << "[Error] old gain bucket " << oldGain << " not found when nPiD=targetPId" << endl;
                            }
                            oldGainIter->second.erase(curNeighbourV);
                            if(oldGainIter->second.empty()) {
                                buckets.erase(oldGainIter);
                            }
                            // see if it is still qualified as a boundary
                            vector<NRVertex*> neiOfNeighs;
                            bool isBoundary = false;
                            graph->getNeighbourVertices(curNeighbourVId, neiOfNeighs);
                            for(unsigned int nni = 0; nni < neiOfNeighs.size(); nni++) {
                                if(graph->getVertexPartition(neiOfNeighs.at(nni)->getId()) != nPId) {
                                    isBoundary = true;
                                    break;
                                }
                            }
                            if(isBoundary) {
                                int newGain = oldGain - 2 * curEWeight;
                                map<int, std::unordered_set<NRVertex*> >::iterator newGainIter = buckets.find(newGain);
                                if(newGainIter != buckets.end()) {
                                    newGainIter->second.insert(curNeighbourV);
                                } else {
                                    std::unordered_set<NRVertex*> newBucket;
                                    newBucket.insert(curNeighbourV);
                                    buckets[newGain] = newBucket;
                                }
                                // update the entry in boundaryVertices also
                                foundNInBoundary->second.second = newGain;
                            } else {
                                boundaryVertices.erase(foundNInBoundary);
                            }

                        } else {
                            cout << "[Error] a vertex that has more than two neighbours LPs are moved." << endl;
                        }
                    } else if(curEWeight == 0 && foundNInBoundary != boundaryVertices.end()) {
                        int nPId = graph->getVertexPartition(curNeighbourVId);
                        if(nPId == curPId) {
                            // remove it from the boundary if now it is facing two neighbours.
                            if(foundNInBoundary->second.first != targetPId) {
                                int oldGain = foundNInBoundary->second.second;
                                map<int, std::unordered_set<NRVertex*> >::iterator oldGainIter = buckets.find(oldGain);
                                if(oldGainIter ==buckets.end()) {
                                    cout << "[Error] old gain bucket " << oldGain << " not found when nPiD=curPid" << endl;
                                }
                                oldGainIter->second.erase(curNeighbourV);
                                if(oldGainIter->second.empty()) {
                                    buckets.erase(oldGainIter);
                                }
                                boundaryVertices.erase(curNeighbourV);
                            }
                        } else if(nPId == targetPId) {
                            // see if it is still qualified as a boundary
                            vector<NRVertex*> neiOfNeighs;
                            bool isBoundary = false;
                            graph->getNeighbourVertices(curNeighbourVId, neiOfNeighs);
                            for(unsigned int nni = 0; nni < neiOfNeighs.size(); nni++) {
                                if(graph->getVertexPartition(neiOfNeighs.at(nni)->getId()) != nPId) {
                                    isBoundary = true;
                                    break;
                                }
                            }
                            if(!isBoundary) {
                                int oldGain = foundNInBoundary->second.second;
                                map<int, std::unordered_set<NRVertex*> >::iterator oldGainIter = buckets.find(oldGain);
                                // remove from previous bucket first
                                if(oldGainIter ==buckets.end()) {
                                    cout << "[Error] old gain bucket " << oldGain << " not found when nPiD=targetPId" << endl;
                                }
                                oldGainIter->second.erase(curNeighbourV);
                                if(oldGainIter->second.empty()) {
                                    buckets.erase(oldGainIter);
                                }
                                boundaryVertices.erase(foundNInBoundary);
                            }
                        }
                    }

                    // put possible neighbours as the potential boundary nodes
                    potentialBoundaryVertices.insert(curNeighbourV);
                }
            }
        }

        buckets.clear();
        boundaryVertices.clear();

        ++iterCount;
        if(with_cout) {
            cout << "Iteration " << iterCount << ": " << movedVInCurIter << " vertices are moved." << endl;
            if(movedVInCurIter == 0 || iterCount == 8) {
                std::unordered_map<int, std::unordered_map<int,int> > boundaryEdgeWeights;    // <pId, <pId, boundaryEdgeWeight> >
                for(NRGraph::EdgeIterator iter = graph->edge_begin(), iter_end = graph->edge_end(); iter != iter_end; iter++) {
                    NREdge* curE = iter->second;
                    Vertices vs = curE->getVertices();
                    int v1Id = vs.vtx1->getId();
                    int v2Id = vs.vtx2->getId();
                    int pV1 = graph->getVertexPartition(v1Id);
                    int pV2 = graph->getVertexPartition(v2Id);

                    if(pV1 != pV2) {
                        std::unordered_map<int, std::unordered_map<int,int> >::iterator foundP1 = boundaryEdgeWeights.find(pV1);
                        if(foundP1 != boundaryEdgeWeights.end()) {
                            std::unordered_map<int,int>::iterator foundP1P2 = foundP1->second.find(pV2);
                            if(foundP1P2 != foundP1->second.end()) {
                                foundP1P2->second += curE->getWeight();
                            } else {
                                foundP1->second[pV2] = curE->getWeight();
                            }
                        } else {
                            std::unordered_map<int,int> newB;
                            newB.insert(pair<int,int>(pV2, curE->getWeight()));
                            boundaryEdgeWeights[pV1] = newB;
                        }
                        std::unordered_map<int, std::unordered_map<int,int> >::iterator foundP2 = boundaryEdgeWeights.find(pV2);
                        if(foundP2 != boundaryEdgeWeights.end()) {
                            std::unordered_map<int,int>::iterator foundP2P1 = foundP2->second.find(pV1);
                            if(foundP2P1 != foundP2->second.end()) {
                                foundP2P1->second += curE->getWeight();
                            } else {
                                foundP2->second[pV1] = curE->getWeight();
                            }
                        } else {
                            std::unordered_map<int,int> newB;
                            newB.insert(pair<int,int>(pV1, curE->getWeight()));
                            boundaryEdgeWeights[pV2] = newB;
                        }
                    }
                }
                for(std::unordered_map<int, std::unordered_map<int,int> >::iterator p1Iter = boundaryEdgeWeights.begin(); p1Iter != boundaryEdgeWeights.end(); p1Iter++) {
                    cout << "Partition " << p1Iter->first << ": ";
                    for(std::unordered_map<int,int>::iterator p2Iter = p1Iter->second.begin(); p2Iter != p1Iter->second.end(); p2Iter++) {
                        cout << p2Iter->first << "(" << p2Iter->second << ") ";
                    }
                    cout << endl;
                }
            }
            cout << "Post refinement Weights=";
            int maxWeight = -1;
            for(int ran = 0; ran < graph->getPartitionCount(); ran++) {
                int curWeight = graph->getPartitionWeight(ran);
                cout << curWeight << " ";
                if(curWeight > maxWeight) {
                    maxWeight = curWeight;
                }
            }
            cout << endl << "Imbalance factor=" << ((double)maxWeight)/graph->getTotalWeight()*graph->getPartitionCount() << endl << endl;
        }
        movedVCount.push_back(movedVInCurIter);
        if(movedVCount.size() > 3 && movedVCount[movedVCount.size()-1] == movedVCount[movedVCount.size()-2] && movedVCount[movedVCount.size()-1] == movedVCount[movedVCount.size()-3]) {
            break;  // break when the number of moved vertices are the same for 3 iterations, there's a swaying.
        }
    } while(movedVInCurIter > 0 && iterCount < refinement_iter);

    auto refineEnd  = chrono::steady_clock::now();
    if(with_cout) {
        cout << "[Info] Refinement phase ended. It took " << ((float)(chrono::duration <double, milli> (refineEnd - refineStart).count()))/1000.0 << " seconds" << endl;
    }
}

int NRGG_part_graph(int* nvtxs, int* vwgt, double* vcrdnt, int* xadj, int* adjncy, int* adjwgt, int* nparts, int* part_out, int* edge_out, bool wcout, double* imb_thres, int* rf_itr, bool multilvl, int* cstvtx) {
    auto partStart = chrono::steady_clock::now();
    if(imb_thres != NULL) {
        imb_low_thres = imb_thres[0];
        imb_high_thres = imb_thres[1];
    }
    if(rf_itr != NULL) {
        refinement_iter = *rf_itr;
    }
    with_cout = wcout;
    if(part_out == NULL) {
        cout << "[Error] Output array is not provided. Partitioning algorithm halted." << endl;
        return 0;
    }
    if(multilvl == true) {
        part_t = NRGG_MULTI_LEVEL;
        cout << "[Info] Using NRGG Multi-level partitioning, applying partitioning on the coarsest graph..." << endl;
        if(cstvtx != NULL) {
            coarsest_min_vertex_per_part = *cstvtx;
        }
    } else {
        cout << "[Info] Using NRGG single-level partitioning, applying partitioning directly on the graph..." << endl;
    }

    // 1. Convert the input data stream into NRGraph structure, and set option variables
    constructPartGraph(nvtxs, vwgt, vcrdnt, xadj, adjncy, adjwgt);

    // 2. Call the proper partitioning routine
    partitionGraph(*nparts);

    // 3. Put the partitionin result in part_out
    int result = outputPartResult(part_out, edge_out);

    auto partEnd  = chrono::steady_clock::now();
    cout << "[Info] NRGG partitioning took " << ((float)(chrono::duration <double, milli> (partEnd - partStart).count()))/1000.0 << " seconds" << endl;

    return result;
}

void constructPartGraph(int *nvtxs, int *vwgt, double *vcrdnt, int *xadj, int *adjncy, int *adjwgt) {
    if(nvtxs == NULL) return;
    partGraph = new NRGraph();
    int totalVWeight = 0;

    // coordinates for computing the bounding box, to determine graph-grow direction
    double minX = std::numeric_limits<double>::max();
    double maxX = -minX;
    double minY = std::numeric_limits<double>::max();
    double maxY = -minY;

    // 1. create vertices
    for(int vId = 0; vId < *nvtxs; ++vId) {
        NRVertex* newV = new NRVertex(vId);
        newV->setWeight(vwgt[vId]);
        newV->setCoords(vcrdnt[vId*2], vcrdnt[vId*2 + 1]);
        partGraph->addVertex(newV);
        totalVWeight += vwgt[vId];

        if(vcrdnt[vId*2] < minX) {
            minX = vcrdnt[vId*2];
            leftMostVertex = newV;  // used for graph-growing later
        }
        if(vcrdnt[vId*2] > maxX) {
            maxX = vcrdnt[vId*2];
        }
        if(vcrdnt[vId*2 + 1] < minY) {
            minY = vcrdnt[vId*2 + 1];
            bottomVertex = newV;  // used for graph-growing later
        }
        if(vcrdnt[vId*2 + 1] > maxY) {
            maxY = vcrdnt[vId*2 + 1];
        }
    }
    partGraph->setTotalWeight(totalVWeight); // set vertex total weight
    // determine graph-grow direction, along the longer direction
    if(maxY - minY > maxX - minX) {
        grow_dir = Y_DIR;
    } // else do nothing, since X_DIR is the default

    // 2. create edges and connect vertices and edges
    int edgeId = 0;
    for(int vId = 0; vId < *nvtxs; ++vId) {
        int edgeStartIndex = xadj[vId];   // inclusive
        int edgeEndIndex = xadj[vId + 1]; // exclusive
        for(int edgeIndex = edgeStartIndex; edgeIndex < edgeEndIndex; ++edgeIndex) {
            int curAdjnVId = adjncy[edgeIndex];
            if(vId < curAdjnVId) { // only create new edge if current vId < ajacent id
                NREdge* newE = new NREdge(edgeId, partGraph->getVertex(vId), partGraph->getVertex(curAdjnVId));
                newE->setWeight(adjwgt[edgeIndex]);
                partGraph->getVertex(vId)->addEdge(newE);
                partGraph->getVertex(curAdjnVId)->addEdge(newE);
                partGraph->addEdge(newE);
                ++edgeId;
            }
        }
    }
}

void partitionGraph(int partitionCount) {
    if(partGraph == NULL || partitionCount < 2)   {
        cout << "Partioning graph cannot be constructed or partition count less than 2, returning without partitioning." << endl;
        return;
    }

    if (part_t == NRGG_SINGLE_LEVEL) {
        if(grow_dir == X_DIR) {
            graphGrowLeftRight(partGraph, partitionCount);
        } else {
            graphGrowBottomTop(partGraph, partitionCount);
        }
        KLRefineGR(partGraph);
    } else {
        // 1. build coarsend graphs
        vector<NRGraph*> layers;   // contains all layers (i+1)th layer is coarsened by ith layer
        layers.push_back(partGraph);
        NRGraph* newLayer = layers.at(layers.size() - 1);

        while (newLayer->getVertexCount() > coarsest_min_vertex_per_part * partitionCount) {
            newLayer = HEMcoarsen(newLayer);
            layers.push_back(newLayer);
            cout << "new layer v count = " << newLayer->getVertexCount() << endl;
        }

        // 2. Partition on the coarsest graph
        if(grow_dir == X_DIR) {
            graphGrowLeftRight(layers.at(layers.size() - 1), partitionCount);
        } else {
            graphGrowBottomTop(layers.at(layers.size() - 1), partitionCount);
        }
        KLRefineGR(layers.at(layers.size() - 1));

        // 3. the coarsest graph is uncoarsend, in each layer, the partitions are refined.
        uncoarsen(layers);
        for(unsigned int i = 1, n = layers.size(); i< n; i++) {
            delete layers.at(i);    // keep the finest layer
        }
    }
}

int outputPartResult(int* part_out, int* edge_out) {
    if(partGraph == NULL) return 0;

    try {
        int vCount = partGraph->getVertexCount();

        for(int vId = 0; vId < vCount; ++vId) {
            part_out[vId] = partGraph->getVertexPartition(vId);
        }

        if(edge_out != NULL) {
            *edge_out = 0;
            for(NRGraph::EdgeIterator iter = partGraph->edge_begin(), iter_end = partGraph->edge_end(); iter != iter_end; ++iter) {
                NREdge* curEdge = iter->second;
                *edge_out += curEdge->getWeight();
            }
            cout << endl << "[Info] Finished NRGG partitioning, total edge cut weight = " << *edge_out << endl;
        }

        delete partGraph;
    } catch (exception& e) {
        cout << e.what() << endl;
        return 0;
    }

    return 1;
}
