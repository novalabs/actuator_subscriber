/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <Core/MW/Subscriber.hpp>
#include <Core/MW/CoreNode.hpp>
#include <Core/MW/CoreActuator.hpp>

#include <actuator_subscriber/SpeedConfiguration.hpp>
#include <actuator_subscriber/PID.hpp>
#include <actuator_msgs/Setpoint_f32.hpp>

#include <Configuration.hpp>
#include <Module.hpp>

namespace actuator_subscriber {
   template <typename _DATATYPE, class _MESSAGETYPE = _DATATYPE>
   class Speed:
      public Core::MW::CoreNode
   {
public:
      using DataType    = _DATATYPE;
      using MessageType = _MESSAGETYPE;

public:
      Speed(
         const char*                       name,
         Core::MW::CoreActuator<DataType>& actuator,
         Core::MW::Thread::Priority        priority = Core::MW::Thread::PriorityEnum::NORMAL
      ) :
         CoreNode::CoreNode(name, priority),
         _actuator(actuator)
      {
         _workingAreaSize = 256;
         _pid.set(0.0);
      }

      virtual
      ~Speed()
      {
         teardown();
      }

public:
      SpeedConfiguration configuration;

private:
      Core::MW::Subscriber<MessageType, Configuration::SUBSCRIBER_QUEUE_LENGTH> _setpoint_subscriber;
      Core::MW::Subscriber<sensor_msgs::Delta_f32, Configuration::SUBSCRIBER_QUEUE_LENGTH> _encoder_subscriber;
      Core::MW::CoreActuator<DataType>& _actuator;
      PID _pid;
      Core::MW::Time _setpoint_timestamp;

private:
      bool
      onConfigure()
      {
         _pid.config(configuration.kp, configuration.ti, configuration.td, configuration.ts, configuration.min, configuration.max);

         return true;
      }

      bool
      onPrepareMW()
      {
         _setpoint_subscriber.set_callback(Speed::setpoint_callback);
         this->subscribe(_setpoint_subscriber, configuration.setpoint_topic);

         _encoder_subscriber.set_callback(Speed::encoder_callback);
         this->subscribe(_encoder_subscriber, configuration.encoder_topic);

         return true;
      }

      bool
      onPrepareHW()
      {
         _actuator.start();

         return true;
      }

      bool
      onLoop()
      {
         if (!this->spin(Configuration::SUBSCRIBER_SPIN_TIME)) {
            Module::led.toggle();
         }

         if (Core::MW::Time::now() > (this->_setpoint_timestamp + Core::MW::Time::ms(configuration.timeout))) {
//				Core::MW::log(???)
//				_actuator.stop();
            _pid.set(configuration.idle);
         }

         return true;
      }

      static bool
      setpoint_callback(
         const actuator_msgs::Setpoint_f32& msg,
         Core::MW::Node* node
      )
      {
         Speed<_DATATYPE, _MESSAGETYPE>* _this = static_cast<Speed<_DATATYPE, _MESSAGETYPE>*>(node);
         _this->_setpoint_timestamp = Core::MW::Time::now();
         _this->_pid.set(msg.value);

         return true;
      }

      static bool
      encoder_callback(
         const sensor_msgs::Delta_f32& msg,
         Core::MW::Node* node
      )
      {
         Speed<_DATATYPE, _MESSAGETYPE>* _this = static_cast<Speed<_DATATYPE, _MESSAGETYPE>*>(node);
         _this->_actuator.set(_this->_pid.update(msg.value));

         return true;
      }
   };
}
