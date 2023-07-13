#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class ModelJointControler : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelJointControler::OnUpdate, this));
      
      this->old_secs =ros::Time::now().toSec();



      if (_sdf->HasElement("prop_kp"))
          this->prop_kp = _sdf->Get<double>("prop_kp");
      if (_sdf->HasElement("prop_ki"))
          this->prop_ki = _sdf->Get<double>("prop_ki");
      if (_sdf->HasElement("prop_kd"))
          this->prop_kd = _sdf->Get<double>("prop_kd");
      if (_sdf->HasElement("namespace_model"))
          this->namespace_model = _sdf->Get<std::string>("namespace_model");
      if (_sdf->HasElement("activate_pid_control"))
          this->activate_pid_control = (_sdf->Get<std::string>("activate_pid_control") == "yes");


      // Creando los Tópicos
      std::string prop1_speed = "/"+this->namespace_model + "/prop1_speed";
      std::string prop2_speed = "/"+this->namespace_model + "/prop2_speed";
      std::string prop3_speed = "/"+this->namespace_model + "/prop3_speed";
      std::string prop4_speed = "/"+this->namespace_model + "/prop4_speed";


      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "set_propSpeed_rosnode",
            ros::init_options::NoSigintHandler);
      }
         
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("earthquake_rosnode"));
      

      if(this->activate_pid_control)
      {
          // Activated PID Speed Control
	  const auto &jointController = this->model->GetJointController();
          jointController->Reset();

          jointController->AddJoint(model->GetJoint("prop1_joint"));
          jointController->AddJoint(model->GetJoint("prop3_joint"));
          jointController->AddJoint(model->GetJoint("prop2_joint"));
          jointController->AddJoint(model->GetJoint("prop4_joint"));


          this->prop1_name = model->GetJoint("prop1_joint")->GetScopedName();
          this->prop2_name = model->GetJoint("prop2_joint")->GetScopedName();
          this->prop3_name = model->GetJoint("prop3_joint")->GetScopedName();
          this->prop4_name = model->GetJoint("prop4_joint")->GetScopedName();
          
          jointController->SetVelocityPID(this->prop1_name, common::PID(this->prop_kp, this->prop_ki, this->prop_kd));
          jointController->SetVelocityPID(this->prop2_name, common::PID(this->prop_kp, this->prop_ki, this->prop_kd));
          jointController->SetVelocityPID(this->prop3_name, common::PID(this->prop_kp, this->prop_ki, this->prop_kd));
          jointController->SetVelocityPID(this->prop4_name, common::PID(this->prop_kp, this->prop_ki, this->prop_kd));
          
          jointController->SetVelocityTarget(this->prop1_name, 0.0);
          jointController->SetVelocityTarget(this->prop2_name, 0.0);
          jointController->SetVelocityTarget(this->prop3_name, 0.0);
          jointController->SetVelocityTarget(this->prop4_name, 0.0);
      }

      // Freq
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            prop1_speed,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_prop1_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
      
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&ModelJointControler::QueueThread, this));
        
        
      // Magnitude
      ros::SubscribeOptions so2 =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            prop2_speed,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_prop2_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue2);
      this->rosSub2 = this->rosNode->subscribe(so2);
      
      // Spin up the queue helper thread.
      this->rosQueueThread2 =
        std::thread(std::bind(&ModelJointControler::QueueThread2, this));
         
      // Freq
      ros::SubscribeOptions so3 =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            prop3_speed,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_prop3_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue3);
      this->rosSub3 = this->rosNode->subscribe(so3);
      
      // Spin up the queue helper thread.
      this->rosQueueThread3 =
        std::thread(std::bind(&ModelJointControler::QueueThread3, this));
        
        
      // Magnitude
      ros::SubscribeOptions so4 =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            prop4_speed,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_prop4_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue4);
      this->rosSub4 = this->rosNode->subscribe(so4);
      
      // Spin up the queue helper thread.
      this->rosQueueThread4 =
        std::thread(std::bind(&ModelJointControler::QueueThread4, this));
           
      ROS_WARN("Loaded Plugin with parent...%s", this->model->GetName().c_str());
      
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      double new_secs =ros::Time::now().toSec();
      double delta = new_secs - this->old_secs;
      
      double max_delta = 0.0;
      
      if (this->freq_update != 0.0)
      {
        max_delta = 1.0 / this->freq_update;
      }
      
      if (delta > max_delta && delta != 0.0)
      {
        this->old_secs = new_secs;

	if(this->activate_pid_control)
        {
          ROS_DEBUG("Update propeller Speed PID...");
          const auto &jointController = this->model->GetJointController();
          jointController->SetVelocityTarget(this->prop1_name, this->prop1_speed_magn);
          jointController->SetVelocityTarget(this->prop2_name, this->prop2_speed_magn);
          jointController->SetVelocityTarget(this->prop3_name, this->prop3_speed_magn);
          jointController->SetVelocityTarget(this->prop4_name, this->prop4_speed_magn);
          jointController->Update();
        }else
        {
            // Apply a small linear velocity to the model.
          ROS_DEBUG("Update propeller Speed BASIC...");
          this->model->GetJoint("prop1_joint")->SetVelocity(0, this->prop1_speed_magn);
          this->model->GetJoint("prop2_joint")->SetVelocity(0, this->prop2_speed_magn);
          this->model->GetJoint("prop3_joint")->SetVelocity(0, this->prop3_speed_magn);
          this->model->GetJoint("prop4_joint")->SetVelocity(0, this->prop4_speed_magn);
        }

      }

    }
        

    public: void SetProp1Speed(const double &_freq)
    {
      this->prop1_speed_magn = _freq;
//      ROS_WARN("prop1_speed_magn >> %f", this->prop1_speed_magn);
    }
    
    public: void SetProp2Speed(const double &_magn)
    {
      this->prop2_speed_magn = _magn;
//      ROS_WARN("prop2_speed_magn >> %f", this->prop2_speed_magn);
    }
    
    public: void SetProp3Speed(const double &_freq)
    {
      this->prop3_speed_magn = _freq;
//      ROS_WARN("prop3_speed_magn >> %f", this->prop3_speed_magn);
    }
    
    public: void SetProp4Speed(const double &_magn)
    {
      this->prop4_speed_magn = _magn;
//      ROS_WARN("prop4_speed_magn >> %f", this->prop4_speed_magn);
    }
    
    
    public: void OnRosMsg_prop1_speed(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetProp1Speed(_msg->data);
    }
    
    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    
    public: void OnRosMsg_prop2_speed(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetProp2Speed(_msg->data);
    }
    
    /// \brief ROS helper function that processes messages
    private: void QueueThread2()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue2.callAvailable(ros::WallDuration(timeout));
      }
    }
    
    
    public: void OnRosMsg_prop3_speed(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetProp3Speed(_msg->data);
    }
    
    /// \brief ROS helper function that processes messages
    private: void QueueThread3()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue3.callAvailable(ros::WallDuration(timeout));
      }
    }
    
    public: void OnRosMsg_prop4_speed(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetProp4Speed(_msg->data);
    }
    
    /// \brief ROS helper function that processes messages
    private: void QueueThread4()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue4.callAvailable(ros::WallDuration(timeout));
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // Time Memory
    double old_secs;
    
    // Frequency of earthquake
    double freq_update = 10.0;

    double prop1_speed_magn = 0.0;
    // Magnitude of the Oscilations
    double prop2_speed_magn = 0.0;
    double prop3_speed_magn = 0.0;
    // Magnitude of the Oscilations
    double prop4_speed_magn = 0.0;

    
    
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub2;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue2;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread2;
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub3;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue3;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread3;
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub4;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue4;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread4;
    
    std::string prop1_name;
    std::string prop2_name;
    std::string prop3_name;
    std::string prop4_name;


    std::string namespace_model = "";
    bool activate_pid_control;

    double prop_kp = 0.1;
    double prop_ki = 0.0;
    double prop_kd = 0.0;

    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelJointControler)
}
