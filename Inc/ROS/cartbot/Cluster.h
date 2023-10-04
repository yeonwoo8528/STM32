#ifndef _ROS_cartbot_Cluster_h
#define _ROS_cartbot_Cluster_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace cartbot
{

  class Cluster : public ros::Msg
  {
    public:
      typedef std_msgs::Header _Header_type;
      _Header_type Header;
      typedef int32_t _id_type;
      _id_type id;
      typedef double _mid_x_type;
      _mid_x_type mid_x;
      typedef double _mid_y_type;
      _mid_y_type mid_y;
      typedef double _dist_type;
      _dist_type dist;
      uint32_t points_length;
      typedef geometry_msgs::Point _points_type;
      _points_type st_points;
      _points_type * points;

    Cluster():
      Header(),
      id(0),
      mid_x(0),
      mid_y(0),
      dist(0),
      points_length(0), st_points(), points(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->Header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      union {
        double real;
        uint64_t base;
      } u_mid_x;
      u_mid_x.real = this->mid_x;
      *(outbuffer + offset + 0) = (u_mid_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mid_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mid_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mid_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mid_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mid_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mid_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mid_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mid_x);
      union {
        double real;
        uint64_t base;
      } u_mid_y;
      u_mid_y.real = this->mid_y;
      *(outbuffer + offset + 0) = (u_mid_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mid_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mid_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mid_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mid_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mid_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mid_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mid_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mid_y);
      union {
        double real;
        uint64_t base;
      } u_dist;
      u_dist.real = this->dist;
      *(outbuffer + offset + 0) = (u_dist.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dist.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dist.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dist.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_dist.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_dist.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_dist.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_dist.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->dist);
      *(outbuffer + offset + 0) = (this->points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->points_length);
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->Header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        double real;
        uint64_t base;
      } u_mid_x;
      u_mid_x.base = 0;
      u_mid_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mid_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mid_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mid_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mid_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mid_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mid_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mid_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mid_x = u_mid_x.real;
      offset += sizeof(this->mid_x);
      union {
        double real;
        uint64_t base;
      } u_mid_y;
      u_mid_y.base = 0;
      u_mid_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mid_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mid_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mid_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mid_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mid_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mid_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mid_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mid_y = u_mid_y.real;
      offset += sizeof(this->mid_y);
      union {
        double real;
        uint64_t base;
      } u_dist;
      u_dist.base = 0;
      u_dist.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dist.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dist.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dist.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_dist.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_dist.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_dist.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_dist.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->dist = u_dist.real;
      offset += sizeof(this->dist);
      uint32_t points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->points_length);
      if(points_lengthT > points_length)
        this->points = (geometry_msgs::Point*)realloc(this->points, points_lengthT * sizeof(geometry_msgs::Point));
      points_length = points_lengthT;
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(geometry_msgs::Point));
      }
     return offset;
    }

    virtual const char * getType() override { return "cartbot/Cluster"; };
    virtual const char * getMD5() override { return "5483b1a607c3b90a8b87bd87fef9ba45"; };

  };

}
#endif
