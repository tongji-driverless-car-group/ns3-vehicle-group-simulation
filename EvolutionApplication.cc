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
    m_max_level = MAX_LEVEL;
    m_max_subnodes = MAX_SUBNODES;
    m_hello_interval = Seconds(HELLO_INTERVAL);
    m_mode = WifiMode(WIFI_MODE);
    m_state = INITIAL_STATE;
    m_wait_construct_time = Seconds(WAIT_CONSTRUCT_TIME);
    m_construct_interval = Seconds(CONSTRUCT_INTERVAL);

    // 各个参数的默认值，默认关闭，具体设置在Test.cc每一个testCase的函数里
    
    //
    m_debug_hello = false;
    m_debug_construct = false;
    
    // ---------- 避障相关 -------------
    m_is_simulate_avoid_outer_obstacle = false;
    m_check_obstacle_interval = Seconds(1);
    m_obstacle = Vector(60, -4.8, 0);
    m_safe_avoid_obstacle_distance = 21;

    // ---------- 节点失联相关 ----------
    m_is_simulate_node_missing = false;
    m_check_missing_interval = Seconds(CHECK_MISSING_INTERVAL);
    m_max_hello_interval = Seconds(MAX_HELLO_INTERVAL); // 超过该时间长度认为失联
    m_search_time = Seconds(SEARCH_TIME); //搜寻失联节点的时间
    m_stay_missing_time = Seconds(STAY_MISSING_TIME);//失联节点保持失联状态的时间
    // m_max_hello_interval = Seconds(1.2 * HELLO_INTERVAL);
    // m_confirm_missing_interval = Seconds(3 * HELLO_INTERVAL);
    m_is_missing = false;
    m_missing_time = Seconds(2.5); 

    // ---------- change leader相关 -----------
    m_is_simulate_change_leader = false;
}

EvolutionApplication::~EvolutionApplication()
{

}

bool EvolutionApplication::isLeader(){
    return m_state & LEADER_STATE;
}

bool EvolutionApplication::isMember(){
    return m_state & MEMBER_STATE;
}

Address EvolutionApplication::GetAddress()
{
    return m_wifiDevice->GetAddress();
}


Vector EvolutionApplication::GetLocation()
{
    return GetNode()->GetObject<MobilityModel>()->GetPosition();
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
        // 每m_hello_interval秒发送心跳包

        if (m_is_simulate_node_missing || m_is_simulate_change_leader) {
            Simulator::Schedule (m_hello_interval+random_offset, &EvolutionApplication::SendHello, this);
        }

        if (m_is_simulate_node_missing) {
            Simulator::Schedule (m_check_missing_interval, &EvolutionApplication::CheckMissing, this);
        }
        
        if (m_is_simulate_change_leader) {
            Simulator::Schedule (m_hello_interval+random_offset+Seconds(2), &EvolutionApplication::CheckLeaderMissing, this);
        }
    }
    else
    {
        NS_FATAL_ERROR ("There's no WifiNetDevice in your node");
    }
    //周期性检查邻居节点，并移除长时间未通信的节点
    Simulator::Schedule (Seconds (1), &EvolutionApplication::RemoveOldNeighbors, this);
    
    // 周期性检查周围是否有障碍物
    if (m_is_simulate_avoid_outer_obstacle) {
        Simulator::Schedule(m_check_obstacle_interval, &EvolutionApplication::CheckObstacle, this);
    }
}

void EvolutionApplication::BroadcastInformation(Ptr<Packet> packet)
{
    //将数据包以 WSMP (0x88dc)格式广播出去
    m_wifiDevice->Send (packet, Mac48Address::GetBroadcast(), 0x88dc);
    
}

void EvolutionApplication::SendInformation(Ptr<Packet> packet, Address addr){
    
    // std::cout << m_router[addr] << std::endl;
    Address dest_addr; // 目的地址
    if (addr == m_leader.mac) {
        dest_addr = m_leader.mac;
    } else if (addr == m_parent.mac) {
        dest_addr = m_parent.mac;
    } else {
	    //router不存在则丢弃
	    if(m_router.find(addr) == m_router.end()){
            std::cout << GetAddress() << " 出现错误路由 " << addr << std::endl;
            return;
        }
        dest_addr = m_router[addr];
    }
    m_wifiDevice->Send (packet, dest_addr, 0x88dc);
}

void EvolutionApplication::SendGroupInformation(Ptr<Packet> packet){
    if (!isLeader()) {
        NS_LOG_ERROR("只有Leader可以向子车群发送消息");
    }

    // 只向二级子节点发送消息
    for(std::vector<NeighborInformation>::iterator iter = m_next.begin();
        iter != m_next.end(); iter++){
        // std::cout << GetAddress() << " 有这些子节点" << std::endl;
        SendInformation(packet, iter->mac);
    }
}

void EvolutionApplication::SendToLeader(Ptr<Packet> packet){
    if(isLeader()){
        NS_FATAL_ERROR ("leader 不能发送消息给自己");
    }
    // std::cout << m_leader.mac << std::endl;
    SendInformation(packet, m_leader.mac);
}

void EvolutionApplication::SendToLeader(Ptr<Packet> packet, Address addr){
    if(!isLeader()){
        NS_FATAL_ERROR ("成员不能与其它车群的leader交互");
    }
    SendInformation(packet, addr);
}

void EvolutionApplication::SendToAllOtherLeaders(Ptr<Packet> packet) {
    if (!isLeader()) {
        NS_FATAL_ERROR ("成员不能与其它车群的leader交互");
    }
    for (std::vector<NeighborInformation>::iterator iter = m_neighbor_leaders.begin();
        iter != m_neighbor_leaders.end(); iter++) {
        // std::cout << "Leader " << GetAddress() << " sending to Leader " << iter->mac << std::endl;
        m_wifiDevice->Send (packet, iter->mac, 0x88dc);
    }
}

void EvolutionApplication::ConvertFromWaitConstructToLeader(){
    //若过了一段时间还是WAIT_CONSTRUCT_STATE则主动成为leader
    if(m_state == WAIT_CONSTRUCT_STATE){
        m_state = LEADER_STATE;
        m_level = 1;
        m_leader.mac = GetAddress();
        m_leader.pos = GetLocation();
        m_leader.last_beacon = Now();
        SendConstructMessage();
    }
    else if(m_state == WAIT_CONSTRUCT_CONFIRM_STATE){//如果在等待确认阶段则过一段时间再看
        Simulator::Schedule(m_wait_construct_time, &EvolutionApplication::ConvertFromWaitConstructToLeader, this);
    }
}

void EvolutionApplication::AssignTask(uint32_t task_id){
    m_state = WAIT_CONSTRUCT_STATE;
    m_task_id = task_id;
    Simulator::Schedule(m_wait_construct_time, &EvolutionApplication::ConvertFromWaitConstructToLeader, this);
}

void EvolutionApplication::AssignTaskAtTime(uint32_t task_id, Time t){
    Simulator::Schedule(t, &EvolutionApplication::AssignTask, this, task_id);
}

bool EvolutionApplication::ReceivePacket (Ptr<NetDevice> device, Ptr<const Packet> packet,uint16_t protocol, const Address &sender)
{   
    if (IsMissing()) { // 如果是仿真预先设定的失联节点，则不接收任何消息
        return false;
    }

    MessageHeader tag;
    if (packet->PeekPacketTag (tag))
    {
        //取得载荷
        uint32_t payloadSize = tag.GetPayloadSize();
        uint8_t* buffer = new uint8_t[payloadSize];
        packet->CopyData(buffer,payloadSize);
        uint8_t type = tag.GetType();
        bool isGroup = type & GROUP_MESSAGE;
        type &= ~(GROUP_MESSAGE);

        // ReceivePacket要求packet参数指向一个const，但是我们其它的SendInformation类的函数不要求const，所以复制一个
        Ptr<Packet> copy_packet = new Packet(*packet);
        // std::cout << (int)tag.GetType() << " isGroup: " << isGroup << ", " << type << std::endl;
        
        switch(type){
            case HELLO:
                HandleHelloMessage(buffer, sender, tag.GetTimestamp());
                break;
            case HELLO_R:
                HandleHelloRMessage(buffer, sender, tag.GetTimestamp());
                break;
            case CONSTRUCT_MESSAGE:
                if(m_debug_construct){
                    HandleConstructMessage(buffer, sender);
                }
                break;
            case CONSTRUCT_REPLY_MESSAGE:
                if(m_debug_construct){
                    cout<<Now()<<" "<<GetAddress()<<" receive construct reply message from "<<sender<<endl;
                }
                HandleConstructReplyMessage(buffer, sender, tag.GetTimestamp());
                break;
            case CONSTRUCT_CONFIRM_MESSAGE:
                if(m_debug_construct){
                    cout<<Now()<<" "<<GetAddress()<<" receive construct confirm message from "<<sender<<endl;
                }
                HandleConstructConfirmMessage(buffer);
                break;
            case OBSTACLE_MESSAGE:
                // 如果是leader接到，则发给它的子节点避障命令
                // 如果是普通节点，则执行避障命令
                if (isLeader()) {
                    SendGroupInformation(copy_packet);
                    std::cout << "Leader " << GetAddress() << " 向子节点下达避障命令，自己也避障" << std::endl;
                } else {
                    // 模拟执行避障动作
                    std::cout << GetAddress() << " 正在避障" << std::endl;
                }
                break;
            // case AVOID_MESSAGE:
            //     std::cout << GetAddress() << " is avoiding obstacle" << std::endl;
            //     break;
            case MISSING_MESSAGE:
                HandleMissingMessage(buffer);
                break;
            case SEARCH_MESSAGE:
                HandleSearchMessage(buffer);
                break;
            case RETURN_MESSAGE:
                HandleReturnMessage(buffer, sender);
                break;
            case RECEIVE_MESSAGE:
                HandleReceiveMessage(buffer);
                break;
            case RECEIVE_REPLY_MESSAGE:
                HandleReceiveReplyMessage(buffer);
                break;
            case CHECK_LEADER_MESSAGE:
                HandleCheckLeaderMessage(sender);
                break;
            case CHECK_LEADER_REPLY_MESSAGE:
                HandleCheckLeaderReplyMessage(sender);
                break;
            case NEW_LEADER_MESSAGE:
                HandleNewLeaderMessage(buffer);
                break;
            case NEW_LEADER_REPLY_MESSAGE:
                break;
            default:
                NS_LOG_ERROR("unknown message type");
                break;
        }
        
        // 如果是组播 还需要继续转发消息，目前只有一个车群的组播，子节点的信息都在router里
        if (isGroup) {
            for (std::map<Address,Address>::iterator iter = m_router.begin();
                iter != m_router.end(); iter++) {
                SendInformation(copy_packet, iter->first);
                std::cout << GetAddress() << " 向 " << iter->second << "发送消息" << std::endl;
            }
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
    // std::cout << "distance: " << dist << std::endl;
    if (dist > m_safe_avoid_obstacle_distance) {
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

    // 遇到障碍，如果是leader，通知子车群和其它车群leader避障；如果是普通子节点，通知leader
    if (isLeader()) {
        // 通知其它车群避障
        std::cout << "Leader " << GetAddress() << " 检测到障碍，通知其子节点和其它车群Leader避障" << std::endl;
        tag.SetDesAddr(Mac48Address::GetBroadcast());
        packet->AddPacketTag(tag);
        std::vector<NeighborInformation>::iterator it;
        for(it = m_neighbor_leaders.begin(); it != m_neighbor_leaders.end(); it++) {
            // std::cout << "hello1" << std::endl;
            SendToLeader(packet, it->mac);
        }

        // 通知子车群避障
        Ptr<Packet> packetForSon = Create<Packet>(buffer, payloadSize);
        tag.SetType(OBSTACLE_MESSAGE | GROUP_MESSAGE); // 组播
        packetForSon->AddPacketTag(tag);
        SendGroupInformation(packetForSon);
        // std::cout << "==========================" << std::endl;
    } else {
        tag.SetDesAddr(Mac48Address::GetBroadcast());
        packet->AddPacketTag(tag);
        // std::cout << "hello2" << std::endl;
        // PrintRouter();
        SendToLeader(packet);
    }
    
    return true;
}

bool EvolutionApplication::CheckMissing()
{
    for(vector<NeighborInformation>::iterator iter = m_next.begin(); iter != m_next.end(); iter++){
        // 查看是否很久没收到子节点的心跳包了，如果是则发出搜索消息
        Time curTime = Now();
        // std::cout << GetAddress() << ":" << iter->mac << " last_beacon: " << iter->last_beacon << " " << curTime - iter->last_beacon << " " << m_max_hello_interval << std::endl;
        if (curTime - iter->last_beacon > m_max_hello_interval) {
            if(isLeader()){
                SendMissingMessage(iter->mac, iter->last_beacon, iter->pos);
            }
            else{
                SendSearchMessage(iter->mac);
                m_search_nodes[iter->mac] = Now()+m_search_time;
            }
            cout<<Now()<<" "<<GetAddress()<<" check missing "<<iter->mac<<endl;
            m_next.erase(iter); // 直接退出循环 不会有问题
            return true;
        }
    }

    return false;
}

void EvolutionApplication::PrintRouter() {
    std::cout << "======= begin print router =======" << std::endl;
    for (std::map<Address,Address>::iterator iter = m_router.begin();
        iter != m_router.end(); iter++) {
        std::cout << iter->first << " : " << iter->second << std::endl;
    }
    std::cout << "======= end print router =======" << std::endl;
}

void EvolutionApplication::SendHello(){
    if (IsMissing()) {
        // std::cout << GetAddress() << " is missing, would not send hello" << std::endl;
        return;
    }

    //如果是leader，则不用发送心跳包
    if(!isMember()){
        return ;
    }
    // std::cout << GetAddress() << " sending hello" << std::endl;

    //心跳包载荷
    HelloInformation hi;
    hi.pos = GetLocation();
    Ptr<Packet> packet = Create <Packet> ((uint8_t*)&hi, sizeof(hi));
    
    //心跳包消息头 
    MessageHeader tag;
    tag.SetType(HELLO);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(sizeof(hi));
    tag.SetDesAddr(m_parent.mac);
    tag.SetSrcAddr(m_wifiDevice->GetAddress());
    packet->AddPacketTag (tag);
    
    //广播心跳包
    SendInformation(packet,m_parent.mac);
    Simulator::Schedule (m_hello_interval, &EvolutionApplication::SendHello, this);

}

void EvolutionApplication::HandleHelloMessage(uint8_t *buffer, const Address &sender, Time timestamp){
    bool find = false;//是否在m_next中找到sender
    //更新m_next的节点信息
    HelloInformation* hi = (HelloInformation*)buffer;
    for(vector<NeighborInformation>::iterator iter=m_next.begin();iter!=m_next.end();iter++){
        if(iter->mac == sender){
            iter->last_beacon = timestamp;
            iter->pos = hi->pos;
            find = true;
            break;
        }
    }
    if(!find){
        NS_FATAL_ERROR ("收到非子结点的HELLOMessage");
        return ;
    }
    
    SendHelloR(sender);
}

void EvolutionApplication::SendHelloR(const Address &addr){
    //TODO 计算节点引领度
    
    
    //构造载荷
    int payloadSize = 0;
    uint8_t * buffer = new uint8_t[payloadSize];
    //uint16_t index = 0;
    
    Ptr<Packet> packet = Create <Packet> (buffer,payloadSize);
    
    delete buffer;
    
    //心跳回复包消息头 
    MessageHeader tag;
    tag.SetType(HELLO_R);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(payloadSize);
    tag.SetDesAddr(addr);
    tag.SetSrcAddr(m_wifiDevice->GetAddress());
    packet->AddPacketTag (tag);
    
    SendInformation(packet, addr);
}

void EvolutionApplication::HandleHelloRMessage(uint8_t *buffer, const Address &sender, Time timestamp){
    if(sender!=m_parent.mac){
        NS_FATAL_ERROR ("收到非父结点的HELLORMessage");
        return ;
    }
    //TODO 更新节点引领度
    
    //更新parent信息
    m_parent.last_beacon = timestamp;
}

void EvolutionApplication::SendConstructMessage(){
    //如果自己没有车群，则不发送建立消息
    if(!isLeader() && !isMember()){
        if(m_debug_construct){
            cout<<Now()<<" "<<GetAddress()<<" stop broadcast construct message for isnt leader or member"<<endl;
        }
        return;
    }
    
    //如果子结点达到最大子结点数的2/3，就不再广播建立消息
    if(3*m_next.size()>=2*m_max_subnodes){
        if(m_debug_construct){
            cout<<Now()<<" "<<GetAddress()<<" stop broadcast construct message for too many subnodes "<<endl;
        }
        return ;
    }
    
    //建立消息载荷
    ConstructInformation ci;
    ci.pos = GetNode()->GetObject<MobilityModel>()->GetPosition();//取得节点位置
    ci.task_id = m_task_id;
    
    Ptr<Packet> packet = Create <Packet> ((uint8_t*)&ci,sizeof(ci));
    
    //建立消息消息头 
    MessageHeader tag;
    tag.SetType(CONSTRUCT_MESSAGE);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(sizeof(ci));
    tag.SetDesAddr(Mac48Address::GetBroadcast());
    tag.SetSrcAddr(m_wifiDevice->GetAddress());
    packet->AddPacketTag (tag);
    
    //广播建立消息
    if(m_debug_construct){
        cout<<Now()<<" "<<GetAddress()<<" broadcast construct message"<<endl;
    }
    BroadcastInformation(packet);
    
    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable> ();
    Time random_offset = Seconds (rand->GetValue(0,m_construct_interval.GetSeconds()/4));
    Simulator::Schedule (m_construct_interval, &EvolutionApplication::SendConstructMessage, this);
    
}

void EvolutionApplication::HandleConstructMessage(uint8_t *buffer, const Address &sender){
    //只有处于等待建立状态才接收建立消息
    if(m_state!=WAIT_CONSTRUCT_STATE){
        return ;
    }
    
    ConstructInformation* ci = (ConstructInformation*)buffer;
    
    
    //和发送建立消息的车任务不同，则忽视其他的建立消息
    if(ci->task_id != m_task_id){
            return ;
    }
    //回复建立消息
    SendConstructReplyMessage(sender);
    m_state = WAIT_CONSTRUCT_CONFIRM_STATE;
}

void EvolutionApplication::SendConstructReplyMessage(const Address &addr){
    if(m_debug_construct){
        cout<<Now()<<" "<<GetAddress()<<" send construct reply message to "<<addr<<endl;
    }
    //建立回复消息载荷
    ConstructReplyInformation cri;
    cri.pos = GetNode()->GetObject<MobilityModel>()->GetPosition();//取得节点位置
    Ptr<Packet> packet = Create <Packet> ((uint8_t*)&cri,sizeof(cri));
    
    //建立回复消息消息头 
    MessageHeader tag;
    tag.SetType(CONSTRUCT_REPLY_MESSAGE);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(sizeof(cri));
    tag.SetDesAddr(addr);
    tag.SetSrcAddr(m_wifiDevice->GetAddress());
    packet->AddPacketTag (tag);
    
    m_wifiDevice->Send (packet, addr, 0x88dc);
}

void EvolutionApplication::HandleConstructReplyMessage(uint8_t* buffer, const Address &sender, Time timestamp){
    ConstructReplyInformation* cri = (ConstructReplyInformation*)buffer;       
       
    //建立确认消息载荷
    ConstructConfirmInformation cci;
    if(3*m_next.size()<2*m_max_subnodes){
        cci.accept = 1;
        cci.level = m_level + 1;
        cci.leader = m_leader;
        cci.parent.mac = GetAddress();
        cci.parent.last_beacon = Now();
        m_router[sender] = sender;
        //添加子节点信息
        NeighborInformation ni;
        ni.mac = sender;
        ni.last_beacon = timestamp;
        ni.pos = cri->pos;
        m_next.push_back(ni);
    }
    else{
        cci.accept = 0;
    }
    
    SendConstructConfirmMessage(cci,sender);
}

void EvolutionApplication::SendConstructConfirmMessage(const ConstructConfirmInformation& cci, const Address &addr){
    Ptr<Packet> packet = Create <Packet> ((uint8_t*)&cci,sizeof(cci));
    
    //建立确认消息消息头 
    MessageHeader tag;
    tag.SetType(CONSTRUCT_CONFIRM_MESSAGE);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(sizeof(cci));
    tag.SetDesAddr(addr);
    tag.SetSrcAddr(m_wifiDevice->GetAddress());
    packet->AddPacketTag (tag);
    if(m_debug_construct){
        cout<<Now()<<" "<<GetAddress()<<" send construct confirm message to "<<addr<<endl;
    }
    m_wifiDevice->Send (packet, addr, 0x88dc);
}

void EvolutionApplication::HandleConstructConfirmMessage(uint8_t* buffer){
    ConstructConfirmInformation* cci = (ConstructConfirmInformation*)buffer;
    if(cci->accept == 1){
        m_state = MEMBER_STATE;
        m_parent = cci->parent;
        m_leader = cci->leader;
        m_level = cci->level;
        SendConstructMessage();
        if(m_debug_construct){
            cout<<Now()<<" "<<GetAddress()<<" get constructed level= "<<(int)m_level<<" parent= "<<m_parent.mac<<" leader= "<<m_leader.mac<<endl;
        }
    }
    else{
        m_state = WAIT_CONSTRUCT_STATE;
        if(m_debug_construct){
            cout<<Now()<<" "<<GetAddress()<<" get rejected "<<endl;
        }
    }
}

bool EvolutionApplication::IsMissing() {
    return m_is_missing && Now() > m_missing_time;
}

bool EvolutionApplication::CheckLeaderMissing() {
    // 所有二级节点发现leader无了

    // 只由二级节点候选人判断
    if (!m_is_next_leader) {
        return false;
    }

    if (Now() - m_leader.last_beacon < m_max_hello_interval) {
        return false;
    }
    
    for (auto iter = m_brothers.begin(); iter != m_brothers.end(); iter++) {
        m_check_other_second_level_nodes[iter->mac] = false;
    }

    std::cout << GetAddress() << " 发现Leader失联，开始向兄弟节点确认" << std::endl;

    Ptr<Packet> packet = Create<Packet>(); // 暂不需要payload
    //建立确认消息消息头
    MessageHeader tag;
    tag.SetType(CHECK_LEADER_MESSAGE);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(0);
    tag.SetDesAddr(Mac48Address::GetBroadcast()); // meaningless
    tag.SetSrcAddr(GetAddress());
    packet->AddPacketTag(tag);

    // 向兄弟节点发消息，确认是leader没了
    SendToBrothers(packet);

    return true;
}

void EvolutionApplication::SendMissingMessage(Address miss_mac, Time last_beacon, Vector pos){
    MissingInformation mi;
    mi.mac = miss_mac;
    mi.last_beacon = last_beacon;
    mi.pos = pos;
    Ptr<Packet> packet = Create<Packet>((uint8_t*)&mi, sizeof(mi));

    //丢失消息消息头
    MessageHeader tag;
    tag.SetType(MISSING_MESSAGE);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(sizeof(mi));
    tag.SetDesAddr(m_leader.mac);
    tag.SetSrcAddr(m_wifiDevice->GetAddress());
    packet->AddPacketTag (tag);

    cout<<Now()<<" "<<GetAddress()<<" send missing message to "<<m_leader.mac<<endl;
    SendToLeader(packet);
}

void EvolutionApplication::HandleMissingMessage(uint8_t *buffer) {
    // 如果是leader收到搜寻消息，则让子节点和其它车群帮忙找
    // 如果是子节点收到消息，则帮忙找
    if (isLeader()) {
        MessageHeader tag;
        SearchInformation *si = (SearchInformation *) buffer;
        tag.SetType(SEARCH_MESSAGE);
        tag.SetTimestamp(Now());
        tag.SetPayloadSize(sizeof(*si));
        tag.SetSrcAddr(GetAddress());
        tag.SetDesAddr(Mac48Address::GetBroadcast());

        Ptr<Packet> packet = Create<Packet>((uint8_t*)si, sizeof(*si));
        packet->AddPacketTag(tag);

        SendGroupInformation(packet);
        std::cout << "Leader " << GetAddress() << " 向子节点下达搜寻命令" << std::endl;
        SendToAllOtherLeaders(packet);
        std::cout << "Leader " << GetAddress() << " 向其它车群发送协助搜寻命令" << std::endl;
    } else {
        // 模拟执行搜寻动作
        // todo 如果sumo设计得好，后面真找到了，需要再写一个schedule用的定时搜寻
        std::cout << GetAddress() << " 正在搜寻节点" << std::endl;
    }
}

void EvolutionApplication::SendSearchMessage(Address miss_mac){
    SearchInformation si;
    si.mac = miss_mac;
    si.stopTime = Now() + m_search_time;
    Ptr<Packet> packet = Create<Packet>((uint8_t*)&si, sizeof(si));

    //搜寻消息消息头
    MessageHeader tag;
    tag.SetType(SEARCH_MESSAGE);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(sizeof(si));
    tag.SetDesAddr(Mac48Address::GetBroadcast());
    tag.SetSrcAddr(m_wifiDevice->GetAddress());
    packet->AddPacketTag (tag);

    cout<<Now()<<" "<<GetAddress()<<" send search message "<<endl;
    SendGroupInformation(packet);
}

void EvolutionApplication::HandleSearchMessage(uint8_t *buffer){
    cout<<Now()<<" "<<GetAddress()<<" receive search message "<<endl;
    SearchInformation* si = (SearchInformation*)buffer;
    m_search_nodes[si->mac] = si->stopTime;
}

void EvolutionApplication::BroadcastReturnMessage(){
    ReturnInformation ri;
    Ptr<Packet> packet = Create<Packet>((uint8_t*)&ri, sizeof(ri));

    //搜寻消息消息头
    MessageHeader tag;
    tag.SetType(RETURN_MESSAGE);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(sizeof(ri));
    tag.SetDesAddr(Mac48Address::GetBroadcast());
    tag.SetSrcAddr(m_wifiDevice->GetAddress());
    packet->AddPacketTag (tag);
    cout<<Now()<<" "<<GetAddress()<<" broadcast return message "<<endl;
    BroadcastInformation(packet);
}

void EvolutionApplication::HandleReturnMessage(uint8_t *buffer, const Address addr){
    //达到最大子节点数则不处理消息
    if(m_next.size() >= m_max_subnodes){
        return;
    }
    for(map<Address,Time>::iterator iter = m_search_nodes.begin(); iter != m_search_nodes.end(); ){
        //搜寻消息已经过期，则将搜寻信息移除
        if(iter->second < Now()){
            m_search_nodes.erase(iter);
            continue;
        }

        //如果找到了丢失节点
        if(iter->first == addr){
            SendReceiveMessage(addr);
            break;
        }
        iter++;
    }
}

void EvolutionApplication::SendReceiveMessage(const Address addr){
    ReceiveInformation ri;
    ri.task_id = m_task_id;
    ri.leader = m_leader;
    ri.parent.mac = GetAddress();
    ri.parent.pos = GetLocation();
    ri.parent.last_beacon = Now();
    ri.level = m_level + 1;
    if(isLeader()){
        for(vector<NeighborInformation>::iterator iter=m_next.begin();iter!=m_next.end();iter++){
            ri.brothers.push_back(*iter);
        }
    }

    Ptr<Packet> packet = Create<Packet>((uint8_t*)&ri, sizeof(ri));

    //搜寻消息消息头
    MessageHeader tag;
    tag.SetType(RECEIVE_MESSAGE);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(sizeof(ri));
    tag.SetDesAddr(addr);
    tag.SetSrcAddr(GetAddress());
    packet->AddPacketTag (tag);

    m_wifiDevice->Send (packet, addr, 0x88dc);
}

void EvolutionApplication::HandleReceiveMessage(uint8_t *buffer){
    if(!IsMissing()){
        return ;
    }
    ReceiveInformation* ri = (ReceiveInformation*)buffer;
    m_leader = ri->leader;
    m_parent = ri->parent;
    m_level = ri->level;
    m_brothers = ri->brothers;
    SendReceiveReplyMessage(m_parent.mac);
}

void EvolutionApplication::SendReceiveReplyMessage(const Address addr){
    ReceiveReplyInformation rri;
    
    Ptr<Packet> packet = Create<Packet>((uint8_t*)&rri, sizeof(rri));

    //搜寻消息消息头
    MessageHeader tag;
    tag.SetType(RECEIVE_REPLY_MESSAGE);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(sizeof(rri));
    tag.SetDesAddr(addr);
    tag.SetSrcAddr(GetAddress());
    packet->AddPacketTag (tag);

    m_wifiDevice->Send (packet, addr, 0x88dc);
}

void EvolutionApplication::HandleReceiveReplyMessage(uint8_t *buffer){

    //ReceiveReplyInformation* rri = (ReceiveReplyInformation*)buffer;
    SendReportMessage();

}

void EvolutionApplication::SendReportMessage(){
    Ptr<Packet> packet = Create<Packet>();
    SendToLeader(packet);
}

void EvolutionApplication::HandleCheckLeaderMessage(const Address &sender) {
    NeighborInformation ni;
    Ptr<Packet> packet = Create<Packet>((uint8_t *)&ni, sizeof(ni)); // 暂不需要payload，但如果要算引领度就要了
    //建立确认消息消息头
    MessageHeader tag;
    tag.SetType(CHECK_LEADER_REPLY_MESSAGE);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(sizeof(ni));
    tag.SetDesAddr(sender);
    tag.SetSrcAddr(GetAddress());
    packet->AddPacketTag(tag);
    m_wifiDevice->Send(packet, sender, 0x88dc);
    std::cout << GetAddress() << " 向候选人 " << sender  << " 反馈" << std::endl;
}

void EvolutionApplication::HandleCheckLeaderReplyMessage(const Address &sender) {
    m_check_other_second_level_nodes[sender] = true;
    std::cout << GetAddress() << " 已收到其它二级节点 " << sender << " 反馈消息" << std::endl;
    for (auto iter = m_check_other_second_level_nodes.begin();
        iter != m_check_other_second_level_nodes.end(); iter++) {
        if (!iter->second) {
            return;
        }
    }
    
    std::cout << GetAddress() << " 已确定Leader失联，开始执行Leader更替" << std::endl;
    ChangeLeader();
}

void EvolutionApplication::ChangeLeader() {
    // 推举引领度最高的节点
    // 这里暂时默认是推举调用这个方法的节点为新leader
    // 换言之，引领度最高节点的计算是其中一个二级节点负责，但是计算的节点不一定是leader
    // 它只负责使用NewLeaderMessage推举新leader
    NeighborInformation ni;
    ni.mac = GetAddress();
    ni.pos = GetLocation();
    ni.last_beacon = Now();

    Ptr<Packet> packet = Create<Packet>((uint8_t *)&ni, sizeof(ni)); // 不需要payload
    //建立确认消息消息头
    MessageHeader tag;
    tag.SetType(NEW_LEADER_MESSAGE);
    tag.SetTimestamp(Now());
    tag.SetPayloadSize(sizeof(ni));
    tag.SetDesAddr(Mac48Address::GetBroadcast()); // meaningless
    tag.SetSrcAddr(GetAddress());
    packet->AddPacketTag(tag);

    // 通知brothers新的leader是谁
    SendToBrothers(packet);

    // todo 通知其它车群Leader，新Leader是谁，但是需要二级节点备份Leader信息
    // SendToAllOtherLeaders(packet);
}

void EvolutionApplication::HandleNewLeaderMessage(uint8_t *buffer) {
    // NeighborInformation *ni = (NeighborInformation *) buffer;
    std::cout << "二级节点 " << GetAddress() << " 更新Leader信息，并通知子节点更新Leader信息" << std::endl;
    // m_leader.mac = ni->mac;
    // m_leader.last_beacon = ni->last_beacon;
    // m_leader.pos = ni->pos;
    
    // 如果自己是新leader，level都要-1
    // bool isChangeLevel = ni->mac == GetAddress();
    // UpdateGroupAfterChangeLeader(ni, isChangeLevel);
}

void EvolutionApplication::UpdateGroupAfterChangeLeader(NeighborInformation *leader, bool isChangeLevel) {
    // todo 在更替leader后递归更新数据 不做了，因为不影响仿真结果 
}

void EvolutionApplication::SendToBrothers(Ptr<Packet> packet) {
    for (std::vector<NeighborInformation>::iterator iter = m_brothers.begin();
        iter != m_brothers.end(); iter++) {
        m_wifiDevice->Send(packet, iter->mac, 0x88dc); // 这种写法不好，但是SendInformation不行，无兄弟节点路由信息
    }
} 

bool EvolutionApplication::IsSecondLevelNode() {
    return m_level == 2;
}

void EvolutionApplication::PrintBrothers() {
    for (std::vector<NeighborInformation>::iterator iter = m_brothers.begin();
        iter != m_brothers.end(); iter++) {
        std::cout << iter->mac << std::endl;
    }
}