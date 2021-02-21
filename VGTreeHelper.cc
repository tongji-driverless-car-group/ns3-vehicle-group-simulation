#include "VGTreeHelper.h"
#include "ns3/log.h"
#include "ns3/fatal-error.h"
VGTreeHelper::VGTreeHelper(){
    root = NULL;
}
    
VGTreeHelper::VGTreeHelper(VGTree* t){
    root = t;
}

VGTreeHelper::~VGTreeHelper(){

}

void VGTreeHelper::Destroy(VGTree* t){
    if(!t){
        return ;
    }
    for(int i=0; i<t->c_num;i++){
        Destroy(t->child[i]);
    }
    delete t;
}

void VGTreeHelper::Destroy(){
    VGTreeHelper::Destroy(root);
    root = NULL;
}

void VGTreeHelper::AddLeader(int id){
    root = new VGTree();
    root->node_id = id;
    root->c_num = 0;
    id2Tree[id] = root;
}

void VGTreeHelper::AddSubNodesFor(vector<int>sub_nodes, int node_id){
    VGTree* t = id2Tree[node_id];
    
    if(t->c_num+sub_nodes.size()>MAX_CHILD_NUN){
        NS_FATAL_ERROR ("VGTreeHelper::AddSubNodesFor 超出最大子结点数");
    }
    
    for(int i=t->c_num;i<t->c_num+(int)sub_nodes.size();i++){
        t->child[i] = new VGTree();
        t->child[i]->node_id = sub_nodes[i-t->c_num];
        t->child[i]->c_num = 0;
        id2Tree[t->child[i]->node_id] = t->child[i];
    }
    t->c_num += sub_nodes.size();
}

void VGTreeHelper::PrintTree(VGTree* T, int space_num){
    if(T==NULL){
        return ;
    }
    cout << T->node_id+1 <<endl;
    if(T->c_num==0){
        return ;
    }
    else if(T->c_num ==1){
        for(int i=0;i<space_num;i++){
            cout << "│   ";
        }
        cout << "└── " ;
        PrintTree(T->child[0],space_num+1);
    }
    else{
        for(int i=0;i<T->c_num;i++){
            for(int i=0;i<space_num;i++){
                cout << "│   ";
            }
            if(i==T->c_num-1){
                cout << "└── ";
            }
            else{
                cout<<"├── ";
            }
            PrintTree(T->child[i],space_num+1);
        }
    }
}

void VGTreeHelper::PrintTree(){
    VGTreeHelper::PrintTree(root,0);
}

void VGTreeHelper::PrintTree(VGTree* T){
    VGTreeHelper::PrintTree(T,0);
}

VGTree* VGTreeHelper::GetTree(){
    return root;
}
