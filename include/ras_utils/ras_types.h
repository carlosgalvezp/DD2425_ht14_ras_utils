#ifndef RAS_TYPES_H
#define RAS_TYPES_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

namespace RAS_Types
{
typedef message_filters::sync_policies::
    ApproximateTime<sensor_msgs::Image,
                    sensor_msgs::Image>                         RGBD_Sync_Policy;

typedef message_filters::Synchronizer<RGBD_Sync_Policy>         RGBD_Sync;



}

#endif // RAS_TYPES_H
