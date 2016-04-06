/*
 * Desc: Gazebo link attacher plugin.
 * Author: Sammy Pfeiffer (sam.pfeiffer@pal-robotics.com)
 * Date: 05/04/2016
 */

#ifndef GAZEBO_ROS_LINK_ATTACHER_HH
#define GAZEBO_ROS_LINK_ATTACHER_HH

#include <ros/ros.h>

#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>
#include "gazebo/physics/PhysicsTypes.hh"
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/msgs/light.pb.h"
#include "gazebo/transport/transport.hh"

#include <gazebo_ros_link_attacher/LightState.h>

namespace gazebo
{

   class GazeboRosLinkAttacher : public WorldPlugin
   {
      public:
        /// \brief Constructor
        GazeboRosLinkAttacher();

        /// \brief Destructor
        virtual ~GazeboRosLinkAttacher();

        /// \brief Load the controller
        void Load( physics::WorldPtr _world, sdf::ElementPtr _sdf );

        /// \brief Attach with a fixed joint link1 to link2
        bool attach(std::string link1, std::string link2);

        /// \brief Detach link1 from link2
        bool detach(std::string link1, std::string link2);



      protected:
        /// \brief Update the controller
        //virtual void UpdateChild();
      
      private:
        //transport::NodePtr light_node;
        msgs::Light lightMsg;
        transport::PublisherPtr lightsPub;
        ros::NodeHandle nh_;
        ros::Subscriber set_light_st_subscriber_;
        void set_light_st_callback(const gazebo_ros_link_attacher::LightStateConstPtr& msg);

        bool attached;
        physics::JointPtr fixedJoint;

        physics::LinkPtr link1;
        physics::LinkPtr link2;

        /// \brief Model that contains this
        physics::ModelPtr model;

        /// \brief The physics engine.
        physics::PhysicsEnginePtr physics;

        /// \brief Pointer to the world.
        physics::WorldPtr world;

   };

}

#endif

