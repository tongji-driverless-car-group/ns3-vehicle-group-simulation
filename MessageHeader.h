#ifndef MESSAGE_HEADER_H
#define MESSAGE_HEADER_H

#include "ns3/tag.h"
#include "ns3/vector.h"
#include "ns3/nstime.h"
#include "ns3/mac48-address.h"
//message type
const uint8_t HELLO = 0; 
const uint8_t HELLO_R = 1; 
const uint8_t CONSTRUCT_MESSAGE = 2; 
const uint8_t REPLY_MESSAGE = 3; 
const uint8_t COMFIRM_MESSAGE = 4; 
const uint8_t ERROR_MESSAGE = 5; 
const uint8_t RETURN_MESSAGE = 6; 
const uint8_t RECEIVE_MESSAGE = 7; 
const uint8_t MISSING_MESSAGE = 8; 
const uint8_t SEARCH_MESSAGE = 9; 
const uint8_t TRANSFER_MESSAGE = 10; 
const uint8_t OBSTACLE_MESSAGE = 11; 
const uint8_t ADJUST_MESSAGE = 12; 
const uint8_t AVOID_MESSAGE = 13;
const uint8_t GROUP_MESSAGE = 0x80;

namespace ns3
{
class MessageHeader : public Tag {
public:

	//以下六个函数是Tag中的，必须重写
	static TypeId GetTypeId(void);
	virtual TypeId GetInstanceTypeId(void) const;
	virtual uint32_t GetSerializedSize(void) const;
	virtual void Serialize (TagBuffer i) const;
	virtual void Deserialize (TagBuffer i);
	virtual void Print (std::ostream & os) const;

	//tag变量的set和get函数
	uint8_t GetType();
	Time GetTimestamp ();
	uint32_t GetPayloadSize();
	Address GetDesAddr();
	Address GetSrcAddr();

	void SetType (uint8_t type);
	void SetTimestamp (Time t);
	void SetPayloadSize(uint32_t payloadSize);
	void SetDesAddr(Address des);
	void SetSrcAddr(Address src);

	MessageHeader();
	MessageHeader(uint8_t type, Address des, Address src);
	virtual ~MessageHeader();
private:

	uint8_t m_reserve[3];//保留字段
    uint8_t m_type;//消息类型
    Time m_timestamp;//时间戳
    uint32_t m_payloadSize;//载荷大小
    Address m_des;//目的mac地址
    Address m_src;//源mac地址
};
}

#endif
