#ifndef _ROS_cartbot_ClusterArray_h
#define _ROS_cartbot_ClusterArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cartbot/Cluster.h"

namespace cartbot
{

  class ClusterArray : public ros::Msg
  {
    public:
      uint32_t Clusters_length;
      typedef cartbot::Cluster _Clusters_type;
      _Clusters_type st_Clusters;
      _Clusters_type * Clusters;

    ClusterArray():
      Clusters_length(0), st_Clusters(), Clusters(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->Clusters_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Clusters_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Clusters_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Clusters_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Clusters_length);
      for( uint32_t i = 0; i < Clusters_length; i++){
      offset += this->Clusters[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t Clusters_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      Clusters_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      Clusters_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      Clusters_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->Clusters_length);
      if(Clusters_lengthT > Clusters_length)
        this->Clusters = (cartbot::Cluster*)realloc(this->Clusters, Clusters_lengthT * sizeof(cartbot::Cluster));
      Clusters_length = Clusters_lengthT;
      for( uint32_t i = 0; i < Clusters_length; i++){
      offset += this->st_Clusters.deserialize(inbuffer + offset);
        memcpy( &(this->Clusters[i]), &(this->st_Clusters), sizeof(cartbot::Cluster));
      }
     return offset;
    }

    virtual const char * getType() override { return "cartbot/ClusterArray"; };
    virtual const char * getMD5() override { return "087be88d97f8feab35832e96802395f5"; };

  };

}
#endif
