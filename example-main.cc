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

    cout<<"testCase: "<< testCase <<endl;
    cout<<"tclFilePath: "<< tclFilePath <<endl;
    cout<<endl;
    
   
    if(testCase == "constructGroup"){
        TestConstructGroup();
    } else if (testCase == "avoidInnerObstacle") {
        TestAvoidObstable(true);
    } else if (testCase == "avoidOuterObstacle") {
        TestAvoidObstable(false);
    } else {
        TestGroupInitialer();
    }
    
    return 0;
}
