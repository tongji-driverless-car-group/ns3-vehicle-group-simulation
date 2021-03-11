#ifndef EVOLUTION_APPLICATION_H
#define EVOLUTION_APPLICATION_H
#include "ns3/application.h"
#include "ns3/wave-net-device.h"
#include "ns3/wifi-phy.h"
#include "ns3/vector.h"
#include <vector>
#include <map>
// #include <thread>
// #include <mutex>
// #include <unistd.h>

using namespace ns3;
using namespace std;

typedef struct 
{
    Address mac;
    Time last_beacon;
    Vector pos;
} NeighborInformation;

typedef struct{
    Vector pos;
} HelloInformation;

typedef struct{
    
} HelloRInformation;

typedef struct{
    Vector pos;
    uint32_t task_id;
} ConstructInformation;

typedef struct {
    Vector pos;
} ConstructReplyInformation;

typedef struct{
    uint8_t accept;// 0为不接受，1为接受
    uint8_t level;//子结点的层数
    NeighborInformation leader;
    NeighborInformation parent;
    
}ConstructConfirmInformation;

typedef struct{
    NeighborInformation leader; // 失联节点的leader，便于其它节点查找
    Vector pos; // 上一次心跳的位置 目前没什么用 但可以方便查找
    Time last_beacon; // 失联时间
}SearchInformation;

// typedef struct {
//     Address addr; 
// } NewLeaderInformation;

typedef uint16_t NodeState;

const double HELLO_INTERVAL = 0.5; //心跳包发送间隔 单位s
const double CHECK_MISSING_INTERVAL = 1.0; //检查丢失节点的周期 单位s
const double TIME_LIMIT = 1.0; //确认节点丢失的通信时间限制 单位s
const char WIFI_MODE[] = "OfdmRate6MbpsBW10MHz"; //wifi 通信模式，具体见文档
const double WAIT_CONSTRUCT_TIME = 5;//等待车群建立消息的时间
const double CONSTRUCT_INTERVAL = 2;//发送车群建立消息的时间间隔
const uint8_t MAX_LEVEL = 8;
const uint8_t MAX_SUBNODES = 5;

//节点基本状态标记
const NodeState INITIAL_STATE = 0x0001;
const NodeState WAIT_CONSTRUCT_STATE = 0x0002;
const NodeState WAIT_CONSTRUCT_CONFIRM_STATE = 0x0100;
const NodeState MEMBER_STATE = 0x0004;
const NodeState LEADER_STATE = 0x0008;

//节点附加状态标记
const NodeState MISSING_STATE = 0x0010;
const NodeState SEARCH_STATE = 0x0020;
const NodeState ADJUST_STATE = 0x0040;
const NodeState ERROR_STATE = 0x0080;


class EvolutionApplication : public ns3::Application
{
public: 
    
    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const;
    EvolutionApplication();
    ~EvolutionApplication();
    
    //判断自己是否是leader
    bool isLeader();
    
    //判断自己是否是member
    bool isMember();

    // 获取自己的mac地址，debug用
    Address GetAddress();
    
    // 获取自己的位置
    Vector GetLocation();
        
    //向整个车群节点广播
    void BroadcastInformation(Ptr<Packet> packet);
    
    //向指定节点单播
    void SendInformation(Ptr<Packet> packet, Address addr);
    
    // 只有leader能调用，给自己的子车群发消息
    void SendGroupInformation(Ptr<Packet> packet);
    
    //向车群leader发送消息
    void SendToLeader(Ptr<Packet> packet);
    
    //向其他车群的leader发送消息
    void SendToLeader(Ptr<Packet> packet, Address addr);

    //向其它所有车群的leader发送消息
    void SendToAllOtherLeaders(Ptr<Packet> packet);
    
    // 向兄弟节点发送消息
    void SendToBrothers(Ptr<Packet> packet);

    //收到数据包后的回调
    bool ReceivePacket (Ptr<NetDevice> device,Ptr<const Packet> packet,uint16_t protocol, const Address &sender);
    
    //分配任务编号
    void AssignTask(uint32_t task_id);
    
    //在time时间为校车分配任务
    void AssignTaskAtTime(uint32_t task_id, Time t);
    
    //分配编号后，一段时间没有收到建立消息，自己成为Leader
    void ConvertFromWaitConstructToLeader();

    //收到数据包后更新邻居节点
    void UpdateNeighbor (Address addr);
    
    //移除长时间未通信节点
    void RemoveOldNeighbors ();
    
    //
    void SetWifiMode (WifiMode mode);
    
    //发送心跳包
    void SendHello();
    
    //处理HELLO消息
    void HandleHelloMessage(uint8_t *buffer, const Address &sender, Time timestamp);
    
    //发送心跳包的回复
    void SendHelloR(const Address &addr);
    
    //处理HELLO_R消息
    void HandleHelloRMessage(uint8_t *buffer, const Address &sender, Time timestamp);
    
    //发送建立消息
    void SendConstructMessage();
    
    //处理建立消息
    void HandleConstructMessage(uint8_t *buffer, const Address &sender);
    
    //发送建立回复消息
    void SendConstructReplyMessage(const Address &addr);
    
    //处理建立回复消息
    void HandleConstructReplyMessage(uint8_t* buffer, const Address &sender, Time timestamp);
    
    //发送建立确认消息
    void SendConstructConfirmMessage(const ConstructConfirmInformation& cci, const Address &addr);
    
    //处理建立确认消息
    void HandleConstructConfirmMessage(uint8_t* buffer);
    
    // 查看附近是否有障碍物
    bool CheckObstacle();

    // 查看是否有节点失联
    bool CheckMissing();

    // 检查是否在搜索状态
    bool IsSearching();

    // 人为设定一个时间点，令某节点故障 / 失联
    bool IsMissing();

    // 处理MISSING_MESSAGE
    void HandleMissingMessage(uint8_t *buffer);

    // 检查是否leader失联
    // 该函数与CheckMissing区别在于，后者只能查找子节点是否missing
    // 而leader失联是根据所有二级节点都找不到leader为准
    bool CheckLeaderMissing();

    // 判断自己是否是二级节点
    bool IsSecondLevelNode();

    // 处理CheckLeaderMessage
    void HandleCheckLeaderMessage(const Address &addr);

    // 处理CheckLeaderReplyMessage
    void HandleCheckLeaderReplyMessage(const Address &sender);

    void HandleNewLeaderMessage(uint8_t *buffer);

    // leader更替
    void ChangeLeader();

    // 更新leader后刷新车群
    void UpdateGroupAfterChangeLeader(NeighborInformation *leader, bool isChangeLevel);

    // for debug
    void PrintRouter();

    void PrintBrothers();
private:
    //StartApplication函数是应用启动后第一个调用的函数
    void StartApplication();
   
public:
    //初始化固定的参数
    uint8_t m_max_level;//车群最大级数
    uint8_t m_max_subnodes;//最大子结点数
    NodeState m_state;//当前车辆的状态
    Ptr<WifiNetDevice> m_wifiDevice; //车辆的WAVE设备
    Time m_time_limit; //移除超过m_time_limit未通信的节点
    Time m_check_missing_interval;//检查丢失节点的周期
    Time m_hello_interval; //发送心跳包的间隔
    WifiMode m_mode; //wifi的模式
    
    uint8_t m_level;//节点的级数，leader节点为1
    uint32_t m_task_id;//车群的任务ID
    std::vector <NeighborInformation> m_next; //节点的子节点列表
    NeighborInformation m_parent; //节点的父节点
    NeighborInformation m_leader; //车群的leader信息
    std::vector<NeighborInformation> m_neighbor_leaders;//其它邻近leader信息，只有车群的leader维护这个表
    std::map<Address,Address> m_router; //路由信息router[mac]即为发送到mac消息下一跳要发送的节点
    std::vector<NeighborInformation> m_brothers; // 兄弟节点信息
    
    //心跳包相关
    bool m_debug_hello;
    
    // ----------- 车群建立相关 -------------
    Time m_wait_construct_time;//等待车群建立消息的时间
    Time m_construct_interval;//发送车群建立消息的时间间隔
    bool m_debug_construct;
    
    // ------------ 避障相关 --------------
    bool m_is_simulate_avoid_outer_obstacle; // 是否仿真避障
    Vector m_obstacle; // 障碍物信息，单障碍物，初始化时设置
    int m_safe_avoid_obstacle_distance; // 单位：m, 与障碍物之间的安全距离，超过则发避障消息
    Time m_check_obstacle_interval; // 检查丢失节点的周期

    // ------------ 节点失联相关 -------------
    bool m_is_simulate_node_missing; // 是否仿真节点失联
    Time m_max_hello_interval; // 超过该时间长度认为失联
    Time m_confirm_missing_interval; // 搜索失败，leader向上汇报节点失联
    Time m_missing_time; // 失联时间
    bool m_is_missing; // 是否是失联的内部节点

    bool m_is_found; // 是找到失联节点的节点
    Time m_found_time; // 找到失联节点的时间
    
    // ------------ change leader相关 --------------
    bool m_is_simulate_change_leader;
    bool m_is_next_leader; // 是二级节点中引领度最高的节点
    std::map<Address, bool> m_check_other_second_level_nodes; // 确认其它二级节点也发现leader丢失 
};

#endif
