#include <iostream>
#include <vector>
#include <map>
using namespace std;
#define MAX_CHILD_NUN 10//最大子节点个数

typedef struct VGTree{
    VGTree* child[MAX_CHILD_NUN];
    int node_id;
    int c_num;
}VGTree;

class VGTreeHelper{
private:
    VGTree* root;//
    map<int, VGTree*> id2Tree;//
    
    //帮助PrintTree，递归实现
    static void PrintTree(VGTree* T, int space_num);
    
public:
    //初始化一颗空的子树
    VGTreeHelper();
    
    //用一颗现有的子树初始化
    VGTreeHelper(VGTree* t);

    //释放分配的空间
    ~VGTreeHelper();
    
    //释放t的空间
    static void Destroy(VGTree* t);
    
    //打印T
    static void PrintTree(VGTree* T);
    
    //销毁当前数，即释放空间
    void Destroy();
    
    //为当前的树增添一个leader
    void AddLeader(int id);
    
    //为一个节点增加子结点
    void AddSubNodesFor(vector<int>sub_nodes, int node_id);
    
    //打印当前树
    void PrintTree();
    
    //获取root
    VGTree* GetTree();
};
