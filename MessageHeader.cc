#include "MessageHeader.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
namespace ns3 {

NS_LOG_COMPONENT_DEFINE("MessageHeader");
NS_OBJECT_ENSURE_REGISTERED (MessageHeader);

MessageHeader::MessageHeader() {
	m_timestamp = Simulator::Now();
	m_payloadSize = 0;
}

MessageHeader::MessageHeader(uint8_t type, Address des, Address src) {
	m_type = type;
	m_timestamp = Simulator::Now();
	m_payloadSize = 0;
	m_des = des;
	m_src = src;
}

MessageHeader::~MessageHeader() {
}

TypeId MessageHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MessageHeader")
    .SetParent<Tag> ()
    .AddConstructor<MessageHeader> ();
  return tid;
}

TypeId MessageHeader::GetInstanceTypeId (void) const
{
  return MessageHeader::GetTypeId ();
}


uint32_t MessageHeader::GetSerializedSize (void) const
{
	//      保留字段+消息类型           时间戳            载荷大小         源地址+目的地址
	return 4 * sizeof(uint8_t) + sizeof (ns3::Time) + sizeof(uint32_t) + 2 * sizeof(Mac48Address);
}

//注意Serialize中的顺序要和Deserialize中的一致
void MessageHeader::Serialize (TagBuffer i) const
{
    //保留字段
    i.WriteU8(m_reserve[0]);
    i.WriteU8(m_reserve[1]);
    i.WriteU8(m_reserve[2]);
    
    //消息类型
    i.WriteU8(m_type);
    
	//发送数据包的时间
	i.WriteDouble(m_timestamp.GetDouble());

	//载荷大小
	i.WriteU32(m_payloadSize);
	
	//目的地址和源地址
	uint8_t buffer[20];
	m_des.CopyTo(buffer);
	for(int j = 0; j < 6; j++){
	    i.WriteU8(buffer[j]);
	}
	
	m_src.CopyTo(buffer);
	for(int j = 0; j < 6; j++){
	    i.WriteU8(buffer[j]);
	}
}


void MessageHeader::Deserialize (TagBuffer i)
{
    //保留字段
    m_reserve[0] = i.ReadU8();
    m_reserve[1] = i.ReadU8();
    m_reserve[2] = i.ReadU8();
    
    //消息类型
    m_type = i.ReadU8();
    
	//发送数据包的时间
	m_timestamp =  Time::FromDouble (i.ReadDouble(), Time::NS);

	//载荷大小
	m_payloadSize = i.ReadU32();
	
	//目的地址和源地址
	uint8_t buffer[6];
	for(int j = 0; j < 6; j++){
	    buffer[j] = i.ReadU8();
	}
	m_des.CopyFrom(buffer,sizeof(buffer));
	
	for(int j = 0; j < 6; j++){
	    buffer[j] = i.ReadU8();
	}
	m_src.CopyFrom(buffer,sizeof(buffer));

}

uint8_t MessageHeader::GetType(){
    return m_type;
}

Time MessageHeader::GetTimestamp (){
    return m_timestamp;
}

uint32_t MessageHeader::GetPayloadSize(){
    return m_payloadSize;
}

Address MessageHeader::GetDesAddr(){
    return m_des;
}
    
Address MessageHeader::GetSrcAddr(){
    return m_src;
}

void MessageHeader::SetType (uint8_t type){
    m_type = type;
}

void MessageHeader::SetTimestamp (Time time){
    m_timestamp = time;
}

void MessageHeader::SetPayloadSize(uint32_t payloadSize){
    m_payloadSize = payloadSize;
}

void MessageHeader::SetDesAddr(Address des){
    m_des = des; 
}

void MessageHeader::SetSrcAddr(Address src){
    m_src = src;
}

void MessageHeader::Print (std::ostream &os) const
{
}

}

