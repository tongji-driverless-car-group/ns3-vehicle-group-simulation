#ifndef GROUP_INITIALER_H
#define GROUP_INITIALER_H

#include "ns3/core-module.h"
#include "ns3/node-container.h"
#include "EvolutionApplication.h"
#include "VGTreeHelper.h"
using namespace std;

class GroupInitializer{
private:
    vector<VGTree*> groups;
    map<int,set<int> >graph;
    void ConstructVehicleGroup(VGTree* root, VGTree* t,VGTree* parent, NodeContainer& nodes, uint32_t task_id, uint8_t level);
    void ConstructLinkBetweenGroups(NodeContainer& nodes);
    bool isLeader(int id);
public:
    //将一颗VGTree添加到Groupnitialer
    void AddGroup(VGTree* root);
    
    //将group的leader连接,a,b均为节点编号
    void AddLink(int a,int b);
    
    //根据Group信息和Link信息
    void Construct(NodeContainer& nodes);
    
    //打印所有group的树状结构
    void PrintGroupStructures();
};
#endif
