#include "ns3/mobility-model.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "EvolutionApplication.h"
#include "MessageHeader.h"

NS_LOG_COMPONENT_DEFINE("EvolutionApplication");
NS_OBJECT_ENSURE_REGISTERED(EvolutionApplication);

TypeId EvolutionApplication::GetTypeId()
{
    static TypeId tid = TypeId("ns3::EvolutionApplication")
                .SetParent <Application> ()
                .AddConstructor<EvolutionApplication> ()
                .AddAttribute ("Interval", "Broadcast Interval",
                      TimeValue (MilliSeconds(500)),
                      MakeTimeAccessor (&EvolutionApplication::m_hello_interval),
                      MakeTimeChecker()
                      )
                      ;
    return tid;
}

TypeId EvolutionApplication::GetInstanceTypeId() const
{
    return EvolutionApplication::GetTypeId();
}

EvolutionApplication::EvolutionApplication()
{
    m_hello_interval = Seconds(HELLO_INTERVAL);
    m_check_missing_interval = Seconds(CHECK_MISSING_INTERVAL);
    m_time_limit = Seconds (TIME_LIMIT);
    m_mode = WifiMode(WIFI_MODE);

    // ! 如果需要开启哪些功能的仿真，就需要把开关打开

    // ---------- 避障相关 -------------
    m_is_simulate_avoid_obstacle = true;
    m_check_obstacle_interval = Seconds(1);
    m_obstacle = Vector(60, -4.8, 0);
    m_safe_avoid_obstacle_distance = 21;

    // ---------- 节点失联相关 ----------
    m_is_simulate_node_missing = false;
}

EvolutionApplication::~EvolutionApplication()
{

}

bool EvolutionApplication::isLeader(){
    return m_state & LEADER_STATE;
}

void EvolutionApplication::StartApplication()
{
    Ptr<Node> n = GetNode ();
    
    for (uint32_t i = 0; i < n->GetNDevices (); i++)
    {
        Ptr<NetDevice> dev = n->GetDevice (i);
        if (dev->GetInstanceTypeId () == WifiNetDevice::GetTypeId())
        {
            //获取节点的wifi设备
            m_wifiDevice = DynamicCast <WifiNetDevice> (dev);
            //接收数据包的回调
            dev->SetReceiveCallback (MakeCallback (&EvolutionApplication::ReceivePacket, this));
            break;
        } 
    }
    if (m_wifiDevice)
    {
        Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable> ();
        Time random_offset = MicroSeconds (rand->GetValue(50,200));
        //每m_hello_interval秒发送心跳包
        //Simulator::Schedule (m_hello_interval+random_offset, &EvolutionApplication::SendHello, this);
    }
    else
    {
        NS_FATAL_ERROR ("There's no WifiNetDevice in your node");
    }
    //周期性检查邻居节点，并移除长时间未通信的节点
    Simulator::Schedule (Seconds (1), &EvolutionApplication::RemoveOldNeighbors, this);
    
    // 周期性检查周围是否有障碍物
    if (m_is_simulate_avoid_obstacle) {
        Simulator::Schedule(m_check_obstacle_interval, &EvolutionApplication::CheckObstacle, this);
    }
}

void EvolutionApplication::TBroadcastInformation()
{
    Ptr<Packet> packet = Create <Packet> (100);
    
    //将数据包以 WSMP (0x88dc)格式广播出去
    m_wifiDevice->Send (packet, Mac48Address::GetBroadcast(), 0x88dc);

    //每m_broadcast_time广播一次
    Simulator::Schedule (m_hello_interval, &EvolutionApplication::TBroadcastInformation, this);
}

void EvolutionApplication::BroadcastInformation(Ptr<Packet> packet)
{
    //将数据包以 WSMP (0x88dc)格式广播出去
    m_wifiDevice->Send (packet, Mac48Address::GetBroadcast(), 0x88dc);
    
}

void EvolutionApplication::SendInformation(Ptr<Packet> packet, Address addr){
    m_wifiDevice->Send (packet, m_router[addr], 0x88dc);
}

void EvolutionApplication::SendGroupInformation(Ptr<Packet> packet, Address addr){
    
}

void EvolutionApplication::SendToLeader(Ptr<Packet> packet){
    if(isLeader()){
        NS_FATAL_ERROR ("leader 不能发送消息给自己");
    }
    NS_LOG_DEBUG(m_leader.mac);
    SendInformation(packet, m_leader.mac);
}

void EvolutionApplication::SendToLeader(Ptr<Packet> packet, Address addr){
    if(!isLeader()){
        NS_FATAL_ERROR ("成员不能与其它车群的leader交互");
    }
    SendInformation(packet, addr);
}

bool EvolutionApplication::ReceivePacket (Ptr<NetDevice> device, Ptr<const Packet> packet,uint16_t protocol, const Address &sender)
{   
    std::cout<<"receive"<<std::endl;
    MessageHeader tag;
    if (packet->PeekPacketTag (tag))
    {
        //取得载荷
        uint32_t payloadSize = tag.GetPayloadSize();
        uint8_t* buffer = new uint8_t[payloadSize];
        packet->CopyData(buffer,payloadSize);
        uint8_t type = tag.GetType();
        Vector apos;
        switch(type){
            case HELLO:
                memcpy(&apos, buffer, sizeof(apos));
                std::cout<<apos.x<<' '<<apos.y<<' '<<apos.z<<std::endl;
                break;
            case HELLO_R:
                break;
            case OBSTACLE_MESSAGE:
                // 如果是leader接到，则发给它的子节点避障命令
                // 如果是普通节点，则执行避障命令
                if (isLeader()) {
                    // todo SendGroupInformation(packet, addr);
                    std::cout << "Leader向子节点下达避障命令" << std::endl;
                } else {
                    // 模拟执行避障动作
                    std::cout << "正在避障" << std::endl;
                }
                break;
            case MISSING_MESSAGE:
                // 如果是leader，switchLeader
                // 如果是普通节点，则向其leader发送查找节点命令
                break;
            case SEARCH_MESSAGE:
                // 如果是leader收到搜寻消息，则让子节点也去帮忙找
                // 如果是子节点收到消息，则帮忙找
                if (isLeader()) {
                    // todo SendGroupInformation(packet, addr);
                    std::cout << "Leader向子节点下达搜寻命令(?自己也开始搜寻?)" << std::endl;
                } else {
                    // 模拟执行搜寻动作
                    // todo 如果sumo设计得好，后面真找到了，需要再写一个schedule用的定时搜寻
                    std::cout << "正在搜寻节点" << std::endl;
                }
            default:
                break;
        }
        delete buffer;
    }

    UpdateNeighbor (sender);
    return true;
}

void EvolutionApplication::UpdateNeighbor (Address addr)
{

}

void EvolutionApplication::RemoveOldNeighbors ()
{

}

void EvolutionApplication::SetWifiMode (WifiMode mode)
{
    m_mode = mode;
}

bool EvolutionApplication::CheckObstacle()
{
    //取得节点位置
    Vector curPos = GetNode()->GetObject<MobilityModel>()->GetPosition();

    // 取整曼哈顿距离近似计算；不使用浮点数，加快运算；基本上不涉及z轴，不计算
    int dist = abs((int)m_obstacle.x - (int)curPos.x) + abs((int)m_obstacle.y - (int)curPos.y);
    if (dist < m_safe_avoid_obstacle_distance) {
        return false; // 没遇到障碍，或者太远不需要避障
    }
    // std::cout << "distance: " << dist << std::endl;

    // 将障碍物位置信息放在payload里
    Vector pos = Vector(m_obstacle.x, m_obstacle.y, m_obstacle.z); // todo check valid
    int payloadSize = sizeof(Vector);
    uint8_t *buffer = new uint8_t[payloadSize];
    uint16_t index = 0;
    memcpy(buffer + index, &pos, sizeof(pos));
    index += sizeof(pos);
    Ptr<Packet> packet = Create<Packet>(buffer, payloadSize);

    //心跳包消息头
    MessageHeader tag;
    tag.SetType(OBSTACLE_MESSAGE);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(payloadSize);
    tag.SetSrcAddr(m_wifiDevice->GetAddress());

    // 遇到障碍，如果是leader，通知子车群避障；如果是普通子节点，通知leader
    if (isLeader()) {
        tag.SetDesAddr(Mac48Address::GetBroadcast());
        packet->AddPacketTag(tag);
        std::set<NeighborInformation>::iterator it;
        for(it = m_neighbor_leaders.begin(); it != m_neighbor_leaders.end(); it++) {
            SendToLeader(packet, it->mac);
        }
    } else {
        tag.SetDesAddr(Mac48Address::GetBroadcast());
        packet->AddPacketTag(tag);
        SendToLeader(packet);
    }
    
    return true;
}

bool EvolutionApplication::CheckMissing()
{
    return false;
}

void switchLeader()
{
    
}

void EvolutionApplication::SendHello (){
    //心跳包载荷
    Vector pos = GetNode()->GetObject<MobilityModel>()->GetPosition();//取得节点位置
    int payloadSize = sizeof(Vector);
    uint8_t * buffer = new uint8_t[payloadSize];
    uint16_t index = 0;
    
    memcpy(buffer+index, &pos, sizeof(pos));
    index += sizeof(pos);
    
    Ptr<Packet> packet = Create <Packet> (buffer,payloadSize);
    
    delete buffer;
    
    //心跳包消息头
    MessageHeader tag;
    tag.SetType(HELLO);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(payloadSize);
    tag.SetDesAddr(Mac48Address::GetBroadcast());
    tag.SetSrcAddr(m_wifiDevice->GetAddress());
    packet->AddPacketTag (tag);
    
    //广播心跳包
    BroadcastInformation(packet);
    Simulator::Schedule (m_hello_interval, &EvolutionApplication::SendHello, this);
}

