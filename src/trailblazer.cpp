
// CS106B Trailblazer assignment 12/8/16
// Author: Tommy Pawelski
// Note: picture of Westeros was taken from https://jseo.com/case-studies/game-of-thrones-westeros-hits-google-maps/
// Citations: Lecture slides and tutorial hours

#include "trailblazer.h"
#include "queue.h"
#include "set.h"
#include "pqueue.h"
#include "point.h"
#include "RoadGraph.h"
#include "basicgraph.h"
#include "point.h"

using namespace std;

// this makes Path an alias for the type Vector<Vertex*>
typedef Vector<Vertex*> Path;
double findDifference(Path currPath, Path bestPath);
Path aStarTwo(RoadGraph& graph, Vertex* start, Vertex* end, Edge* removedEdge);

// the minimum difference for an alternate route
const double SUFFICIENT_DIFFERENCE = 0.2;
Path emptyPath;


//This implements the BFS search algorithm and returns the first path
//from start to end that is found
Path breadthFirstSearch(RoadGraph& graph, Vertex* start, Vertex* end) {
    // set the start vertex's color to green
    start->setColor(GREEN);
    Path startPath;
    //the start node is added to the start path
    startPath.add(start);
    //This is a queue that will store all paths
    Queue<Path> pathQueue;
    // This set will store all visited veritces
    Set<Vertex*> visitedSet;
    visitedSet.add(start);
    pathQueue.enqueue(startPath);
    Path currPath;
    // While the path queue is not empty and the end node has not been visited,
    // the current path is dequeued from the path queue, and the last vertex in
    //the path queue is colored green as it is the current node.
    while(!pathQueue.isEmpty() && !visitedSet.contains(end)){
        currPath = pathQueue.dequeue();
        //last element in visited
        Vertex* v = currPath.get(currPath.size()-1);
        v->setColor(GREEN);
        visitedSet.add(v);
        //for the current node, all neighbor nodes are added to a set
        Set<Vertex*> neighborSet=graph.getNeighbors(v);
        //unvisited neighbor set is the nodes in neighbor set that have not been visited
        Set<Vertex*> unvisitedSet= neighborSet - visitedSet;
        //for every neighbor in the unvisited set, add the neighbor to the current path
        // and enqueue the new path and set the last node in the new path (the neighbor) to yellow
        if(v==end){
            v->setColor(RED);
        }
        for(Vertex* neighbor:unvisitedSet){
            Path newPath = currPath;
            newPath.add(neighbor);
            pathQueue.enqueue(newPath);
            Vertex* newVertex = newPath.get(newPath.size()-1);
            newPath.add(newVertex);
            newVertex->setColor(YELLOW);
        }
    }
    //if the end node is visited, the current path is returned, otherwise an empty payh is returned
    if (visitedSet.contains(end)){
        return currPath;
    } else {
        Path emptyPath;
        return emptyPath;
    }
}

// This implements dijkstras algorith and returns the lowest cost path
// from start to end using a priority queue
Path dijkstrasAlgorithm(RoadGraph& graph, Vertex* start, Vertex* end) {
    //creates a to-do list priority queue
    PriorityQueue<Path> toDo;
    //the start node is set to green, added to the path, and enqued into to do list
    start->setColor(GREEN);
    Path startPath;
    startPath.add(start);
    toDo.enqueue(startPath, 1);
    Path currPath;
    //this creates a set that will contain all nodes that have been seen
    Set<Vertex*> seen;
    //while the to-do list is not empty and the end vertex has not yet been seen,
    //the current is dequeued from the to-do list and the last node of the current path
    // is set to green as it is the current node.
    while(!toDo.isEmpty() && !seen.contains(end)){
        double currPathCost = toDo.peekPriority();
        currPath = toDo.dequeue();
        Vertex* currNode = currPath.get(currPath.size()-1);
        currNode->setColor(GREEN);
        // if the current node is the end node, it is set to red and the
        // current path is returned
        if(currNode==end){
            currNode->setColor(RED);
            return currPath;
        }
        // if the current node has not yet been seen, it is added to the seen set,
        //and its neighbors are found. for each one of its neighbors, the edge cost
        //from the current node to its neighbor is found. the neighbor is then added to the
        //path, and the edge cost is added to the current cost of the path. the resulting
        // path enqueued into the todo list along with the total path cost.
        if(!seen.contains(currNode)){
            seen.add(currNode);
            Set<Vertex*> neighborSet=graph.getNeighbors(currNode);
            for(Vertex* neighbor:neighborSet){
                Edge* nextEdge = graph.getEdge(currNode, neighbor);
                double edgeCost = nextEdge->cost;
                Path tempPath = currPath;
                tempPath.add(neighbor);
                toDo.enqueue(tempPath, currPathCost + edgeCost);
                //if the neighbor has not been visited it is set to yellow
                if(!seen.contains(neighbor)){
                    neighbor->setColor(YELLOW);
                }
            }
        }
    }
    //if no path has been found, an empty path is returned
    Path emptyPath;
    return emptyPath;
}

// This implements the aStar search algorith which usues a hueristic to
// return the lowest weight path from start to end
Path aStar(RoadGraph& graph, Vertex* start, Vertex* end) {
    //creates a priority queue of paths where the weight of each path is the
    //current cost plus the future cost denoted by the heuristic
    PriorityQueue<Path> toDo;
    start->setColor(GREEN);
    Path startPath;
    startPath.add(start);
    toDo.enqueue(startPath, 1);
    Path currPath;
    Set<Vertex*> seen;
    //while the to-do list is not empty and the end vertex has not yet been seen,
    //the current is dequeued from the to-do list and the last node of the current path
    // is set to green as it is the current node.
    while(!toDo.isEmpty() && !seen.contains(end)){
        double currPathCost = toDo.peekPriority();
        currPath = toDo.dequeue();
        Vertex* currNode = currPath.get(currPath.size()-1);
        currNode->setColor(GREEN);
        if(currNode==end){
            currNode->setColor(RED);
            return currPath;
        }
        // if the current node has not yet been seen, it is added to the seen set,
        //and its neighbors are found. for each one of its neighbors, the edge cost
        //from the current node to its neighbor is found. the neighbor is then added to the
        //path, and the edge cost is added to the current cost of the path. the resulting
        // path enqueued into the todo list along with the total path cost.
        if(!seen.contains(currNode)){
            seen.add(currNode);
            Set<Vertex*> neighborSet=graph.getNeighbors(currNode);
            //the edge from the current node to its neighbor is calculated
            for(Vertex* neighbor:neighborSet){
                Edge* nextEdge = graph.getEdge(currNode, neighbor);
                double edgeCost = nextEdge->cost;
                //the "travel time" from the current node to the end node is calculated
                //as the heuristic by findig the crow fly distance an dividing it
                //by the maximum row speed, making it an underestimate
                double oldAStarCost = graph.getCrowFlyDistance(currNode, end) / graph.getMaxRoadSpeed();
                double nextAStarCost = graph.getCrowFlyDistance(neighbor, end) / graph.getMaxRoadSpeed();
                Path tempPath = currPath;
                //the neighbor is added to the new path and the new path is enqueued the aforementioned heuristic
                //as the priority and the old heuristic is subtracted
                tempPath.add(neighbor);
                toDo.enqueue(tempPath, currPathCost + edgeCost + nextAStarCost - oldAStarCost);
                if(!seen.contains(neighbor)){
                    neighbor->setColor(YELLOW);
                }
            }
        }
    }
    //if no path is found, an empty node is returned
    return emptyPath;
}

// this function finds an alternate path with atleast 20% nodes different than the optimal aStar path
Path alternativeRoute(RoadGraph& graph, Vertex* start, Vertex* end) {
    Path emptyPath;
    //the best path is the path found by the aStar algorithm
    Path bestPath = aStar(graph, start, end);
    //this is a priority queue of all  alternative paths
    PriorityQueue<Path> possibleAlternatives;
    //gets every edge in the best path
    for (int i =0; i < bestPath.size() -1; i++){
        Edge* blockedEdge = graph.getEdge(bestPath.get(i), bestPath.get(i+1));
        //finds all possible trial paths by passing in every edge from the best path into aStart two
        Path trialPath = aStarTwo(graph, start, end, blockedEdge);
        if (trialPath != emptyPath){
            //each possible path is enqueued into the priority queue of alternitives, with the difference as the
            // priority, if trial path is not empty
            possibleAlternatives.enqueue(trialPath, findDifference(trialPath, bestPath));
        }
    }
    // returns the first path from the priority queue that satisfies the sufficient difference
    while(!possibleAlternatives.isEmpty()){
        double diff = possibleAlternatives.peekPriority();
        if (diff >= SUFFICIENT_DIFFERENCE){
            return possibleAlternatives.dequeue();
        } else {
             possibleAlternatives.dequeue();
        }
    }
    //returns an empty path if no alteritive is found
    return emptyPath;
}

//this function is the same as aStar, but it takes in an edge as well
Path aStarTwo(RoadGraph& graph, Vertex* start, Vertex* end, Edge* removedEdge) {
    PriorityQueue<Path> toDo;
    start->setColor(GREEN);
    Path startPath;
    startPath.add(start);
    toDo.enqueue(startPath, 1);
    Path currPath;
    Set<Vertex*> seen;
    while(!toDo.isEmpty() && !seen.contains(end)){
        double currPathCost = toDo.peekPriority();
        currPath = toDo.dequeue();
        Vertex* currNode = currPath.get(currPath.size()-1);
        currNode->setColor(GREEN);
        if(currNode==end){
            currNode->setColor(RED);
            return currPath;
        }
        if(!seen.contains(currNode)){
            seen.add(currNode);
            Set<Vertex*> neighborSet=graph.getNeighbors(currNode);
            for(Vertex* neighbor:neighborSet){
                Edge* nextEdge = graph.getEdge(currNode, neighbor);
                double edgeCost = nextEdge->cost;
                double oldAStarCost= graph.getCrowFlyDistance(currNode, end) / graph.getMaxRoadSpeed();
                double nextAStarCost = graph.getCrowFlyDistance(neighbor, end) / graph.getMaxRoadSpeed();
                Path tempPath = currPath;
                tempPath.add(neighbor);
                //if the path is not the edge that is removed from the optimal path, it is enqueued into the
                //priority queue with the cost of the path plus the heuristic as the priority
                if (nextEdge != removedEdge){
                    toDo.enqueue(tempPath, currPathCost + edgeCost + nextAStarCost - oldAStarCost);
                }
                if(!seen.contains(neighbor)){
                    neighbor->setColor(YELLOW);
                }
            }
        }
    }
//    Path emptyPath;
    return emptyPath;
}

//this helper function finds the number of nodes in the alternate path that
//are not in the best path
double findDifference(Path currPath, Path bestPath){
    Set<Vertex*> bestSet;
    Set<Vertex*> currentSet;
    double sizeBestPath = bestPath.size();
    for (Vertex* node: bestPath){
        bestSet.add(node);
    }
    for (Vertex* node2: currPath){
        currentSet.add(node2);
    }
    Set<Vertex*> diffSet = bestSet-currentSet;
    double numDiffNodes = diffSet.size();
    return numDiffNodes/sizeBestPath;
}
