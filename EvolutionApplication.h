#ifndef EVOLUTION_APPLICATION_H
#define EVOLUTION_APPLICATION_H
#include "ns3/application.h"
#include "ns3/wave-net-device.h"
#include "ns3/wifi-phy.h"
#include "ns3/vector.h"
#include <vector>
#include <map>

using namespace ns3;

typedef struct 
{
    Address mac;
    Time last_beacon;
    Vector pos;
} NeighborInformation;

typedef uint16_t NodeState;

const double HELLO_INTERVAL = 0.5; //心跳包发送间隔 单位s
const double CHECK_MISSING_INTERVAL = 1.0; //检查丢失节点的周期 单位s
const double TIME_LIMIT = 1.0; //确认节点丢失的通信时间限制 单位s
const char WIFI_MODE[] = "OfdmRate6MbpsBW10MHz"; //wifi 通信模式，具体见文档
const NodeState INITIAL_STATE = 0x0001;
const NodeState MEMBER_STATE = 0x0002;
const NodeState LEADER_STATE = 0x0004;
const NodeState MISSING_STATE = 0x0008;
const NodeState SEARCH_STATE = 0x0010;
const NodeState ADJUST_STATE = 0x0020;
const NodeState ERROR_STATE = 0x0040;


class EvolutionApplication : public ns3::Application
{
public: 
    
    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const;
    EvolutionApplication();
    ~EvolutionApplication();
    
    //判断自己是否是leader
    bool isLeader();
        
    //向整个车群节点广播
    void BroadcastInformation(Ptr<Packet> packet);
    void TBroadcastInformation();
    
    //向指定节点单播
    void SendInformation(Ptr<Packet> packet, Address addr);
    
    //向指定子车群组播 哪里需要用到暂时不明 没有实现
    void SendGroupInformation(Ptr<Packet> packet, Address addr);
    
    //向车群leader发送消息
    void SendToLeader(Ptr<Packet> packet);
    
    //向其他车群的leader发送消息
    void SendToLeader(Ptr<Packet> packet, Address addr);
    
    //收到数据包后的回调
    bool ReceivePacket (Ptr<NetDevice> device,Ptr<const Packet> packet,uint16_t protocol, const Address &sender);

    //收到数据包后更新邻居节点
    void UpdateNeighbor (Address addr);
    
    //移除长时间未通信节点
    void RemoveOldNeighbors ();
    
    //
    void SetWifiMode (WifiMode mode);
    
    //发送心跳包
    void SendHello();
private:
    //StartApplication函数是应用启动后第一个调用的函数
    void StartApplication();
   
public: 
    NodeState m_state;//当前车辆的状态
    Ptr<WifiNetDevice> m_wifiDevice; //车辆的WAVE设备  
    std::vector <NeighborInformation> m_next; //节点的子节点列表
    NeighborInformation m_parent; //节点的父节点
    NeighborInformation m_leader; //车群的leader信息
    std::vector<NeighborInformation> m_neighbor_leaders;//其它邻近leader信息，只有车群的leader维护这个表
    std::map<Address,Address> m_router; //路由信息router[mac]即为发送到mac消息下一跳要发送的节点
    Time m_time_limit; //移除超过m_time_limit未通信的节点
    Time m_check_missing_interval;//检查丢失节点的周期
    Time m_hello_interval; //发送心跳包的间隔
    WifiMode m_mode; //wifi的模式
};

#endif
