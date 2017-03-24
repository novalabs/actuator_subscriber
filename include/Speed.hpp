/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/utils/BasicActuator.hpp>

#include <core/actuator_subscriber/SpeedConfiguration.hpp>
#include <core/actuator_subscriber/PID.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>

#include <ModuleConfiguration.hpp>
#include <Module.hpp>

namespace core {
namespace actuator_subscriber {
template <typename _DATATYPE, class _MESSAGETYPE = _DATATYPE>
class Speed:
   public core::mw::CoreNode,
   public core::mw::CoreConfigurable<SpeedConfiguration>
{
public:
   using DataType    = _DATATYPE;
   using MessageType = _MESSAGETYPE;

public:
   Speed(
      const char*                           name,
      core::utils::BasicActuator<DataType>& actuator,
      core::os::Thread::Priority            priority = core::os::Thread::PriorityEnum::NORMAL
   ) :
      CoreNode::CoreNode(name, priority),
      CoreConfigurable<core::actuator_subscriber::SpeedConfiguration>::CoreConfigurable(name),
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

private:
   core::mw::Subscriber<MessageType, ModuleConfiguration::SUBSCRIBER_QUEUE_LENGTH> _setpoint_subscriber;
   core::mw::Subscriber<core::sensor_msgs::Delta_f32, ModuleConfiguration::SUBSCRIBER_QUEUE_LENGTH> _encoder_subscriber;
   core::utils::BasicActuator<DataType>& _actuator;
   PID _pid;
   core::os::Time _setpoint_timestamp;

private:
   bool
   onConfigure()
   {
      _pid.config(configuration().kp, configuration().ti, configuration().td, configuration().ts, configuration().min, configuration().max);

      return true;
   }

   bool
   onPrepareMW()
   {
      _setpoint_subscriber.set_callback(Speed::setpoint_callback);
      this->subscribe(_setpoint_subscriber, configuration().setpoint_topic);

      _encoder_subscriber.set_callback(Speed::encoder_callback);
      this->subscribe(_encoder_subscriber, configuration().encoder_topic);

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
      if (!this->spin(ModuleConfiguration::SUBSCRIBER_SPIN_TIME)) {
         Module::led.toggle();
      }

      if (core::os::Time::now() > (this->_setpoint_timestamp + core::os::Time::ms(configuration().timeout))) {
//				core::mw::log(???)
//				_actuator.stop();
         _pid.set(configuration().idle);
      }

      return true;
   }

   static bool
   setpoint_callback(
      const core::actuator_msgs::Setpoint_f32& msg,
      void*                                    context
   )
   {
      Speed<_DATATYPE, _MESSAGETYPE>* _this = static_cast<Speed<_DATATYPE, _MESSAGETYPE>*>(context);
      _this->_setpoint_timestamp = core::os::Time::now();
      _this->_pid.set(msg.value);

      return true;
   }

   static bool
   encoder_callback(
      const core::sensor_msgs::Delta_f32& msg,
      void*                               context
   )
   {
      Speed<_DATATYPE, _MESSAGETYPE>* _this = static_cast<Speed<_DATATYPE, _MESSAGETYPE>*>(context);
      _this->_actuator.set(_this->_pid.update(msg.value));

      return true;
   }
};
}
}
