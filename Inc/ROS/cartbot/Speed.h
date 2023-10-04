#ifndef _ROS_cartbot_Speed_h
#define _ROS_cartbot_Speed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cartbot
{

class Speed : public ros::Msg
{
public:
	typedef int32_t _tar_rpm_l_type;
	_tar_rpm_l_type tar_rpm_l;
	typedef int32_t _tar_rpm_r_type;
	_tar_rpm_r_type tar_rpm_r;

	Speed():
		tar_rpm_l(0),
		tar_rpm_r(0)
	{
	}

	virtual int serialize(unsigned char *outbuffer) const override
	{
		int offset = 0;
		union {
			int32_t real;
			uint32_t base;
		} u_tar_rpm_l;
		u_tar_rpm_l.real = this->tar_rpm_l;
		*(outbuffer + offset + 0) = (u_tar_rpm_l.base >> (8 * 0)) & 0xFF;
		*(outbuffer + offset + 1) = (u_tar_rpm_l.base >> (8 * 1)) & 0xFF;
		*(outbuffer + offset + 2) = (u_tar_rpm_l.base >> (8 * 2)) & 0xFF;
		*(outbuffer + offset + 3) = (u_tar_rpm_l.base >> (8 * 3)) & 0xFF;
		offset += sizeof(this->tar_rpm_l);
		union {
			int32_t real;
			uint32_t base;
		} u_tar_rpm_r;
		u_tar_rpm_r.real = this->tar_rpm_r;
		*(outbuffer + offset + 0) = (u_tar_rpm_r.base >> (8 * 0)) & 0xFF;
		*(outbuffer + offset + 1) = (u_tar_rpm_r.base >> (8 * 1)) & 0xFF;
		*(outbuffer + offset + 2) = (u_tar_rpm_r.base >> (8 * 2)) & 0xFF;
		*(outbuffer + offset + 3) = (u_tar_rpm_r.base >> (8 * 3)) & 0xFF;
		offset += sizeof(this->tar_rpm_r);
		return offset;
	}

	virtual int deserialize(unsigned char *inbuffer) override
	{
		int offset = 0;
		union {
			int32_t real;
			uint32_t base;
		} u_tar_rpm_l;
		u_tar_rpm_l.base = 0;
		u_tar_rpm_l.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
		u_tar_rpm_l.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
		u_tar_rpm_l.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
		u_tar_rpm_l.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
		this->tar_rpm_l = u_tar_rpm_l.real;
		offset += sizeof(this->tar_rpm_l);
		union {
			int32_t real;
			uint32_t base;
		} u_tar_rpm_r;
		u_tar_rpm_r.base = 0;
		u_tar_rpm_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
		u_tar_rpm_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
		u_tar_rpm_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
		u_tar_rpm_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
		this->tar_rpm_r = u_tar_rpm_r.real;
		offset += sizeof(this->tar_rpm_r);
		return offset;
	}

	virtual const char * getType() override { return "cartbot/Speed"; };
	virtual const char * getMD5() override { return "f9cfffacaa72d4427fadc8174db2c579"; };

};

}
#endif
