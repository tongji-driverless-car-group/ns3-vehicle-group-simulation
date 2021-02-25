#include "ns3/log.h"
#include "GroupInitializer.h"

void GroupInitializer::ConstructVehicleGroup(VGTree* root, VGTree* t,VGTree* parent, NodeContainer& nodes, uint32_t task_id, uint8_t level){
    if(!t){
        return;
    }
    
    for(int8_t i=0;i<t->c_num;i++){
        ConstructVehicleGroup(root, t->child[i], t, nodes,task_id,level+1);
    }
    
    Ptr<Node> node = nodes.Get(t->node_id);
    Ptr<EvolutionApplication> node_app = DynamicCast<EvolutionApplication>(node->GetApplication(0));
    NeighborInformation leader_info;
    node_app->m_level = level;
    //分配任务编号
    node_app->m_task_id = task_id;
    if(t==root){
        node_app->m_state = LEADER_STATE;
    }
    else{
        node_app->m_state = MEMBER_STATE;
        
        NeighborInformation parent_info;
        Ptr<Node> parent_node = nodes.Get(parent->node_id);
        Ptr<NetDevice> parent_dev = parent_node->GetDevice(0);
        Ptr<EvolutionApplication> parent_app = DynamicCast<EvolutionApplication>(parent_node->GetApplication(0));
        parent_info.mac = parent_dev->GetAddress();
        parent_info.last_beacon = Now();
        //parent_info.pos = parent_app->GetNode()->GetObject<MobilityModel>()->GetPosition();
        node_app->m_parent = parent_info;
        
        Ptr<Node> leader_node = nodes.Get(root->node_id);
        Ptr<NetDevice> leader_dev = leader_node->GetDevice(0);
        Ptr<EvolutionApplication> leader_app = DynamicCast<EvolutionApplication>(leader_node->GetApplication(0));
        leader_info.mac = leader_dev->GetAddress();
        leader_info.last_beacon = Now();
        //leader_info.pos = leader_node->GetObject<MobilityModel>()->GetPosition();
        node_app->m_leader = leader_info;

        // 在路由表里添加：<leader, parent>和<parent, parent>
        // node_app->m_router[leader_info.mac] = parent_info.mac;
        // node_app->m_router[parent_info.mac] = parent_info.mac;
    }
    
    for(int8_t i=0;i<t->c_num;i++){
        //子结点信息
        Ptr<Node> child_node = nodes.Get(t->child[i]->node_id);
        Ptr<NetDevice> child_dev = child_node->GetDevice(0);
        Ptr<EvolutionApplication> child_app = DynamicCast<EvolutionApplication>(child_node->GetApplication(0));
        NeighborInformation child_info;
        child_info.mac = child_dev->GetAddress();
        child_info.last_beacon = Now();
        //child_info.pos = child_node->GetObject<MobilityModel>()->GetPosition();
        node_app->m_next.push_back(child_info); 
        
        //路由信息
        for(map<Address,Address>::iterator iter=child_app->m_router.begin(); iter!=child_app->m_router.end();iter++){
            // if (iter->first == child_dev->GetAddress()) { // <parent, parent>
            //     continue;
            // }
            // if (iter->first == leader_info.mac) {
            //     continue;
            // }
            node_app->m_router[iter->first] = child_dev->GetAddress();
        }
        node_app->m_router[child_dev->GetAddress()] = child_dev->GetAddress();
        
    }

    // debug
    std::cout << t->node_id+1 << ":" << std::endl;
    node_app->PrintRouter();
}

void GroupInitializer::ConstructLinkBetweenGroups(NodeContainer& nodes){
    for(map<int,set<int> >::iterator iter = graph.begin(); iter!=graph.end(); iter++){
        Ptr<Node> node = nodes.Get(iter->first);
        if(!isLeader(iter->first)){
            NS_FATAL_ERROR ("GroupInitializer::ConstructLinkBetweenGroup()节点不是leader");
        }
        Ptr<EvolutionApplication> node_app = DynamicCast<EvolutionApplication>(node->GetApplication(0));
        set<int>& other_leader_ids = iter->second;
        for(set<int>::iterator siter=other_leader_ids.begin();siter!=other_leader_ids.end();siter++){
            Ptr<Node> other_leader = nodes.Get(*siter);
            Ptr<NetDevice> other_leader_dev = other_leader->GetDevice(0);
            if(!isLeader(*siter)){
                NS_FATAL_ERROR ("GroupInitializer::ConstructLinkBetweenGroup()节点不是leader");
            }
            NeighborInformation other_leader_info;
            
            other_leader_info.mac = other_leader_dev->GetAddress();
            other_leader_info.last_beacon = Now();
            //other_leader_info.pos = other_leader->GetObject<MobilityModel>()->GetPosition();
            node_app->m_neighbor_leaders.push_back(other_leader_info);
            
            //添加路由
            node_app->m_router[other_leader_info.mac] = other_leader_info.mac;
        }
        
    }
}

bool GroupInitializer::isLeader(int id){
    for(long unsigned int i=0;i<groups.size();i++){
        if(groups[i]->node_id==id){
            return true;
        }
    }
    return false;
}

void GroupInitializer::AddGroup(VGTree* root){
    groups.push_back(root);
}

void GroupInitializer::AddLink(int a, int b){
    graph[a].insert(b);
    graph[b].insert(a);
}

void GroupInitializer::Construct(NodeContainer& nodes){
    uint32_t task_id = 0;
    for(vector<VGTree*>::iterator iter=groups.begin();iter!=groups.end();iter++){
        ConstructVehicleGroup(*iter,*iter,NULL,nodes,task_id++,1);
    }
    ConstructLinkBetweenGroups(nodes);
}

void GroupInitializer::PrintGroupStructures(){
    for(vector<VGTree*>::iterator iter=groups.begin();iter!=groups.end();iter++){
        VGTreeHelper::PrintTree(*iter);
    }
}
