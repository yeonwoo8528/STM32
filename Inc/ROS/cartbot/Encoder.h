#ifndef _ROS_cartbot_Encoder_h
#define _ROS_cartbot_Encoder_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cartbot
{

  class Encoder : public ros::Msg
  {
    public:
      typedef uint16_t _enc_cnt_l_type;
      _enc_cnt_l_type enc_cnt_l;
      typedef uint16_t _enc_cnt_r_type;
      _enc_cnt_r_type enc_cnt_r;

    Encoder():
      enc_cnt_l(0),
      enc_cnt_r(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->enc_cnt_l >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->enc_cnt_l >> (8 * 1)) & 0xFF;
      offset += sizeof(this->enc_cnt_l);
      *(outbuffer + offset + 0) = (this->enc_cnt_r >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->enc_cnt_r >> (8 * 1)) & 0xFF;
      offset += sizeof(this->enc_cnt_r);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->enc_cnt_l =  ((uint16_t) (*(inbuffer + offset)));
      this->enc_cnt_l |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->enc_cnt_l);
      this->enc_cnt_r =  ((uint16_t) (*(inbuffer + offset)));
      this->enc_cnt_r |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->enc_cnt_r);
     return offset;
    }

    virtual const char * getType() override { return "cartbot/Encoder"; };
    virtual const char * getMD5() override { return "40436c2be102fd0207dc1c1d05c2ef2c"; };

  };

}
#endif
