#ifndef _ROS_cartbot_Current_h
#define _ROS_cartbot_Current_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "cartbot/Cluster.h"

namespace cartbot
{

  class Current : public ros::Msg
  {
    public:
      typedef std_msgs::Header _Header_type;
      _Header_type Header;
      typedef double _x_type;
      _x_type x;
      typedef double _y_type;
      _y_type y;
      typedef double _vx_type;
      _vx_type vx;
      typedef double _vy_type;
      _vy_type vy;
      uint32_t objects_length;
      typedef cartbot::Cluster _objects_type;
      _objects_type st_objects;
      _objects_type * objects;
      typedef uint8_t _state_type;
      _state_type state;
      typedef int32_t _lost_cnt_type;
      _lost_cnt_type lost_cnt;
      enum { STOP = 0 };
      enum { LOST = 1 };
      enum { COUNT = 2 };
      enum { INIT = 3 };
      enum { TRACKING = 4 };

    Current():
      Header(),
      x(0),
      y(0),
      vx(0),
      vy(0),
      objects_length(0), st_objects(), objects(nullptr),
      state(0),
      lost_cnt(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->Header.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->x);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->y);
      union {
        double real;
        uint64_t base;
      } u_vx;
      u_vx.real = this->vx;
      *(outbuffer + offset + 0) = (u_vx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vx.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vx.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vx.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vx.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vx.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vx);
      union {
        double real;
        uint64_t base;
      } u_vy;
      u_vy.real = this->vy;
      *(outbuffer + offset + 0) = (u_vy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vy);
      *(outbuffer + offset + 0) = (this->objects_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->objects_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->objects_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->objects_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->objects_length);
      for( uint32_t i = 0; i < objects_length; i++){
      offset += this->objects[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      union {
        int32_t real;
        uint32_t base;
      } u_lost_cnt;
      u_lost_cnt.real = this->lost_cnt;
      *(outbuffer + offset + 0) = (u_lost_cnt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lost_cnt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lost_cnt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lost_cnt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lost_cnt);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->Header.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        double real;
        uint64_t base;
      } u_vx;
      u_vx.base = 0;
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vx = u_vx.real;
      offset += sizeof(this->vx);
      union {
        double real;
        uint64_t base;
      } u_vy;
      u_vy.base = 0;
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vy = u_vy.real;
      offset += sizeof(this->vy);
      uint32_t objects_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      objects_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      objects_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      objects_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->objects_length);
      if(objects_lengthT > objects_length)
        this->objects = (cartbot::Cluster*)realloc(this->objects, objects_lengthT * sizeof(cartbot::Cluster));
      objects_length = objects_lengthT;
      for( uint32_t i = 0; i < objects_length; i++){
      offset += this->st_objects.deserialize(inbuffer + offset);
        memcpy( &(this->objects[i]), &(this->st_objects), sizeof(cartbot::Cluster));
      }
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
      union {
        int32_t real;
        uint32_t base;
      } u_lost_cnt;
      u_lost_cnt.base = 0;
      u_lost_cnt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lost_cnt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lost_cnt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lost_cnt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lost_cnt = u_lost_cnt.real;
      offset += sizeof(this->lost_cnt);
     return offset;
    }

    virtual const char * getType() override { return "cartbot/Current"; };
    virtual const char * getMD5() override { return "4d01d6a47dea24abe185d87c51f91fc5"; };

  };

}
#endif
