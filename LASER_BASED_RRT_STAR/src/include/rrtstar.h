#ifndef RRTSTAR_H
#define RRTSTAR_H

//#include "obstacles.h"
#include "dubins.h"
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "mtrand.h"
using namespace std;
using namespace Eigen;

#define BOT_TURN_RADIUS     2
#define END_DIST_THRESHOLD     1.5

struct Node {
    vector<Node *> children;
    Node *parent;
    Vector2f position;
    float orientation;
    double cost;
    DubinsPath path;
};

class RRTSTAR
{
public:


    RRTSTAR();
    void initialize();
    Node* getRandomNode();
    Node* nearest(Vector2f point);
    void near(Vector2f point, float radius, vector<Node *>& out_nodes);
    double distance(Vector2f &p, Vector2f &q);
    double Cost(Node *q);
    double PathCost(Node *qFrom, Node *qTo);
    Vector3f newConfig(Node *q, Node *qNearest);
    void add(Node *qNearest, Node *qNew);
    bool reached();
    void setStepSize(int step);
    void setMaxIterations(int iter);
    void setSTART(float x,float y);
    void setEND(float x,float y);
    void deleteNodes(Node *root);
   // Obstacles *obstacles;
    vector<Node *> nodes;
    vector<Node *> path;
    Node *root, *lastNode;
    Vector2f startPos, endPos;
    int max_iter;
    int step_size;
    MTRand drand;

};

#endif // RRTSTAR_H
