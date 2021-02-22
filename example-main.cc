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
#include "string"
using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("EvolutionpplicationExample");

int main (int argc, char *argv[])
{
    string testCase = "test";
    string tclFilePath = "scratch/ns3-vehicle-group-simulation/sumofiles/test.tcl";
    
    CommandLine cmd;
    cmd.AddValue("testCase", "通过指定testCase对main函数进行个性化修改", testCase);
    cmd.AddValue("tclFilePath", "要加载的tcl文件位置", tclFilePath);
    cmd.Parse (argc, argv);
    
    uint32_t nNodes = 0;//节点数目
    double simTime = 0; //仿真时间
    
    if(testCase=="test"){
        nNodes = 4;
        simTime = 60;
        tclFilePath = "scratch/ns3-vehicle-group-simulation/sumofiles/test.tcl";
    }
    
    cout<<"testCase: "<< testCase <<endl;
    cout<<"tclFilePath: "<< tclFilePath <<endl;
    cout<<endl;
    

    NodeContainer nodes;
    nodes.Create(nNodes);

    LogComponentEnable ("EvolutionApplication", LOG_LEVEL_FUNCTION);
    
    //使用NS3的移动模型，可以修改为SUMO的FCD输出
    Ns2MobilityHelper mobility(tclFilePath);
    mobility.Install(nodes.Begin(),nodes.End());

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
  
    //netAnim可视化
//    AnimationInterface anim("EvolutionApplication.xml");
//    anim.SetMobilityPollInterval (Seconds (1));
  
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}
