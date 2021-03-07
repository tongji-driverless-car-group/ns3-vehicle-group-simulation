#include "ns3/wave-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/core-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/netanim-module.h"
#include "EvolutionApplication.h"
#include "GroupInitializer.h"
#include "Test.h"

void TestVGTreeHelper(){
    VGTreeHelper vh;
    vh.AddLeader(0);
    vh.AddSubNodesFor(vector<int>({1,2,3}), 0);
    vh.AddSubNodesFor(vector<int>({4,5,6}), 2);
    vh.PrintTree();
    vh.Destroy();
    
}

void TestGroupInitialer(){
    uint32_t nNodes = 9;//节点数目
    double simTime = 10; //仿真时间

    NodeContainer nodes;
    nodes.Create(nNodes);
  
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    
    VGTreeHelper vh;
    GroupInitializer gi;
    //group1
    positionAlloc->Add (Vector (100, 100, 0));
    positionAlloc->Add (Vector (80, 80, 0));
    positionAlloc->Add (Vector (80, 100, 0));
    positionAlloc->Add (Vector (60, 100, 0));
    positionAlloc->Add (Vector (60, 80, 0));
    
    vh.AddLeader(0);
    vh.AddSubNodesFor(vector<int>({1,2}), 0);
    vh.AddSubNodesFor(vector<int>({3,4}), 2);
    gi.AddGroup(vh.GetTree());
    
    //group2
    positionAlloc->Add (Vector (120, 100, 0));
    positionAlloc->Add (Vector (120, 120, 0));
    positionAlloc->Add (Vector (120, 80, 0));
    
    vh.AddLeader(5);
    vh.AddSubNodesFor(vector<int>({6,7}), 5);
    gi.AddGroup(vh.GetTree());
    
    //group3
    positionAlloc->Add (Vector (90, 120, 0));
    vh.AddLeader(8);
    
    mobility.SetPositionAllocator (positionAlloc);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (nodes);
    gi.AddGroup(vh.GetTree());
    
    //add link between groups
    gi.AddLink(0,5);
    gi.AddLink(5,8);
    gi.AddLink(0,8);
    
    gi.PrintGroupStructures();
    
    // The below set of helpers will help us to put together the wifi NICs we want
    YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    Ptr<YansWifiChannel> channel = wifiChannel.Create ();
    wifiPhy.SetChannel (channel);
  
    //可以调节通信距离
//    wifiPhy.Set ("TxPowerStart", DoubleValue (20) );
//    wifiPhy.Set ("TxPowerEnd", DoubleValue (40) );

  // ns-3 supports generate a pcap trace
    wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
    NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
    Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

    wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue ("OfdmRate6MbpsBW10MHz"),
                                      "ControlMode",StringValue ("OfdmRate6MbpsBW10MHz"));
    NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes);

    //为节点添加应用
    for (uint32_t i=0; i<nodes.GetN(); i++)
    {
        Ptr<EvolutionApplication> app_i = CreateObject<EvolutionApplication>();
        app_i->SetStartTime (Seconds (0));
        app_i->SetStopTime (Seconds (simTime));
        nodes.Get(i)->AddApplication (app_i);
    }

    Simulator::Stop(Seconds(simTime));
    
    //初始化车群
    gi.Construct(nodes);
    //netAnim可视化
//    AnimationInterface anim("EvolutionApplication.xml");
//    anim.SetMobilityPollInterval (Seconds (1));
  
    Simulator::Run();

    Simulator::Destroy();
}

void TestConstructGroup(){
    uint32_t nNodes = 7;//节点数目
    double simTime = 60; //仿真时间

    NodeContainer nodes;
    nodes.Create(nNodes);
  
    LogComponentEnable ("EvolutionApplication", LOG_LEVEL_FUNCTION);
    
    //使用NS3的移动模型，可以修改为SUMO的FCD输出
    // 基点是waf所在目录
    Ns2MobilityHelper mobility("./scratch/ns3-vehicle-group-simulation/sumofiles/vehicleGroupConstruct/vehicleGroupConstruct.tcl");
    mobility.Install(nodes.Begin(), nodes.End());
//    MobilityHelper mobility;
//    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
//    positionAlloc->Add (Vector (0, 0, 0));
//    positionAlloc->Add (Vector (0, 0, 1));
//    positionAlloc->Add (Vector (0, 0, 2));
//    positionAlloc->Add (Vector (0, 1, 3));
//    positionAlloc->Add (Vector (0, 0, 4));
//    positionAlloc->Add (Vector (0, 1, 5));
//    positionAlloc->Add (Vector (0, 0, 6));
//    mobility.SetPositionAllocator (positionAlloc);
//    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
//    mobility.Install (nodes);
    
    // The below set of helpers will help us to put together the wifi NICs we want
    YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    Ptr<YansWifiChannel> channel = wifiChannel.Create ();
    wifiPhy.SetChannel (channel);
  
    //可以调节通信距离
    wifiPhy.Set ("TxPowerStart", DoubleValue (40) );
    wifiPhy.Set ("TxPowerEnd", DoubleValue (40) );

    // ns-3 supports generate a pcap trace
    wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
    NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
    Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

    wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue ("OfdmRate6MbpsBW10MHz"),
                                      "ControlMode",StringValue ("OfdmRate6MbpsBW10MHz"));
    NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes);

    //为节点添加应用
    Ptr<EvolutionApplication> app0 = CreateObject<EvolutionApplication>();
    app0->SetStartTime (Seconds (0));
    app0->SetStopTime (Seconds (simTime));
    app0->AssignTaskAtTime(1,Seconds(0));
    app0->m_debug_construct = true;
    nodes.Get(0)->AddApplication (app0);
    
    Ptr<EvolutionApplication> app1 = CreateObject<EvolutionApplication>();
    app1->SetStartTime (Seconds (0));
    app1->SetStopTime (Seconds (simTime));
    app1->AssignTaskAtTime(1,Seconds(10));
    app1->m_debug_construct = true;
    nodes.Get(1)->AddApplication (app1);
    
    Ptr<EvolutionApplication> app2 = CreateObject<EvolutionApplication>();
    app2->SetStartTime (Seconds (0));
    app2->SetStopTime (Seconds (simTime));
    app2->AssignTaskAtTime(1,Seconds(10));
    app2->m_debug_construct = true;
    nodes.Get(2)->AddApplication (app2);
    
    Ptr<EvolutionApplication> app3 = CreateObject<EvolutionApplication>();
    app3->SetStartTime (Seconds (0));
    app3->SetStopTime (Seconds (simTime));
    app3->AssignTaskAtTime(1,Seconds(20));
    app3->m_debug_construct = true;
    nodes.Get(3)->AddApplication (app3);
    
    Ptr<EvolutionApplication> app4 = CreateObject<EvolutionApplication>();
    app4->SetStartTime (Seconds (0));
    app4->SetStopTime (Seconds (simTime));
    app4->AssignTaskAtTime(1,Seconds(20));
    app4->m_debug_construct = true;
    nodes.Get(4)->AddApplication (app4);
    
    Ptr<EvolutionApplication> app5 = CreateObject<EvolutionApplication>();
    app5->SetStartTime (Seconds (0));
    app5->SetStopTime (Seconds (simTime));
    app5->AssignTaskAtTime(1,Seconds(20));
    app5->m_debug_construct = true;
    nodes.Get(5)->AddApplication (app5);
    Simulator::Stop(Seconds(simTime));
    
    Ptr<EvolutionApplication> app6 = CreateObject<EvolutionApplication>();
    app6->SetStartTime (Seconds (0));
    app6->SetStopTime (Seconds (simTime));
    app6->AssignTaskAtTime(1,Seconds(20));
    app6->m_debug_construct = true;
    nodes.Get(6)->AddApplication (app6);
    Simulator::Stop(Seconds(simTime));
    //netAnim可视化
//    AnimationInterface anim("EvolutionApplication.xml");
//    anim.SetMobilityPollInterval (Seconds (1));
  
    Simulator::Run();

    Simulator::Destroy();
}

void TestAvoidObstable() {
    uint32_t nNodes = 6;//节点数目
    double simTime = 1.2; //仿真时间

    NodeContainer nodes;
    nodes.Create(nNodes);
  
    LogComponentEnable ("EvolutionApplication", LOG_LEVEL_FUNCTION);
    
    //使用NS3的移动模型，可以修改为SUMO的FCD输出
    // 基点是waf所在目录
    Ns2MobilityHelper mobility("./scratch/ns3-vehicle-group-simulation/sumofiles/avoidObstacle/mobility.tcl");
    mobility.Install(nodes.Begin(), nodes.End());

    VGTreeHelper vh;
    GroupInitializer gi;
    
    // group1, avoidObstacle.rou.xml上的黄色车群
    vh.AddLeader(2);
    vh.AddSubNodesFor(vector<int>({4}), 2);
    vh.AddSubNodesFor(vector<int>({0}), 4);
    gi.AddGroup(vh.GetTree());
    
    // group2, avoidObstacle.rou.xml上的蓝色车群
    vh.AddLeader(5);
    vh.AddSubNodesFor(vector<int>({1, 3}), 5);
    gi.AddGroup(vh.GetTree());

    //add link between groups
    gi.AddLink(2, 5);
    
    gi.PrintGroupStructures();
    
    // The below set of helpers will help us to put together the wifi NICs we want
    YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    Ptr<YansWifiChannel> channel = wifiChannel.Create ();
    wifiPhy.SetChannel (channel);
  
    //可以调节通信距离
//    wifiPhy.Set ("TxPowerStart", DoubleValue (20) );
//    wifiPhy.Set ("TxPowerEnd", DoubleValue (40) );

    // ns-3 supports generate a pcap trace
    wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
    NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
    Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

    wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue ("OfdmRate6MbpsBW10MHz"),
                                      "ControlMode",StringValue ("OfdmRate6MbpsBW10MHz"));
    NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes);

    //为节点添加应用
    for (uint32_t i=0; i<nodes.GetN(); i++)
    {
        Ptr<EvolutionApplication> app_i = CreateObject<EvolutionApplication>();
        // ======================= application 的一些参数的初始化 begin =============================
        // 默认值见EvolutionApplication的构造函数
        app_i->SetStartTime (Seconds (0));
        app_i->SetStopTime (Seconds (simTime));

        // ---------- 避障相关 -------------
        app_i->m_is_simulate_avoid_obstacle = true;
        app_i->m_check_obstacle_interval = Seconds(1);
        app_i->m_obstacle = Vector(60, -4.8, 0);
        app_i->m_safe_avoid_obstacle_distance = 21;

        nodes.Get(i)->AddApplication (app_i);
        // ======================= application 的一些参数的初始化 end ===============================
        
    }

    Simulator::Stop(Seconds(simTime));
    
    //初始化车群
    gi.Construct(nodes);
  
    //netAnim可视化
//    AnimationInterface anim("EvolutionApplication.xml");
//    anim.SetMobilityPollInterval (Seconds (1));
  
    Simulator::Run();

    Simulator::Destroy();
}
