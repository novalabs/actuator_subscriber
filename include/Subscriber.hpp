/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/utils/BasicActuator.hpp>

#include <core/actuator_subscriber/Configuration.hpp>

#include <ModuleConfiguration.hpp>

namespace core {
namespace actuator_subscriber {
template <class _DATATYPE, class _MESSAGETYPE>
struct ValueOf {
   static inline _DATATYPE
   _(
      const _MESSAGETYPE& from
   )
   {
      return from.value;
   }
};

template <typename _DATATYPE, class _MESSAGETYPE = _DATATYPE, class _CONVERTER = ValueOf<_DATATYPE, _MESSAGETYPE> >
class Subscriber:
   public core::mw::CoreNode,
   public core::mw::CoreConfigurable<core::actuator_subscriber::Configuration>
{
public:
   using DataType    = _DATATYPE;
   using MessageType = _MESSAGETYPE;
   using Converter   = _CONVERTER;

public:
   Subscriber(
      const char*                           name,
      core::utils::BasicActuator<DataType>& actuator,
      core::os::Thread::Priority            priority = core::os::Thread::PriorityEnum::NORMAL
   ) :
      CoreNode::CoreNode(name, priority),
      CoreConfigurable<core::actuator_subscriber::Configuration>::CoreConfigurable(name),
      _actuator(actuator)
   {
      _workingAreaSize = 256;
   }

   virtual
   ~Subscriber()
   {
      teardown();
   }

private:
   core::mw::Subscriber<MessageType, ModuleConfiguration::SUBSCRIBER_QUEUE_LENGTH> _subscriber;
   core::utils::BasicActuator<DataType>& _actuator;

private:
   bool
   onPrepareMW()
   {
      _subscriber.set_callback(Subscriber::callback);
      this->subscribe(_subscriber, configuration().topic);

      return true;
   }

   bool
   onConfigure()
   {
      return isConfigured();
   }

   bool
   onPrepareHW()
   {
      return _actuator.init();
   }

   bool
   onStart()
   {
      return _actuator.start();
   }

   bool
   onLoop()
   {
      if (!this->spin(ModuleConfiguration::SUBSCRIBER_SPIN_TIME)) {
//				core::mw::log(???)
//				_actuator.stop();
         float x = 0.0;       // TODO: isn't it better to use something like NaN???
         _actuator.set(x);
      }

      return true;
   }

   bool
   onStop()
   {
      return _actuator.stop();
   }

   static bool
   callback(
      const core::actuator_msgs::Setpoint_f32& msg,
      void*                                    context
   )
   {
      Subscriber<_DATATYPE, _MESSAGETYPE, _CONVERTER>* _this = static_cast<Subscriber<_DATATYPE, _MESSAGETYPE, _CONVERTER>*>(context);
      float x = Converter::_(msg);
      _this->_actuator.set(x);

      return true;
   }
};
}
}
