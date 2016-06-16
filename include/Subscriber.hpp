/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <Core/MW/Subscriber.hpp>
#include <Core/MW/CoreNode.hpp>
#include <Core/MW/CoreActuator.hpp>

#include <actuator_subscriber/SubscriberConfiguration.hpp>
#include <actuator_msgs/Setpoint_f32.hpp>

#include <Configuration.hpp>
#include <Module.hpp>

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
      public Core::MW::CoreNode
   {
public:
      using DataType    = _DATATYPE;
      using MessageType = _MESSAGETYPE;
      using Converter   = _CONVERTER;

public:
      Subscriber(
         const char*                       name,
         Core::MW::CoreActuator<DataType>& actuator,
         Core::MW::Thread::Priority        priority = Core::MW::Thread::PriorityEnum::NORMAL
      ) :
         CoreNode::CoreNode(name, priority),
         _actuator(actuator)
      {
         _workingAreaSize = 256;
      }

      virtual
      ~Subscriber()
      {
         teardown();
      }

public:
      SubscriberConfiguration configuration;

private:
      Core::MW::Subscriber<MessageType, Configuration::SUBSCRIBER_QUEUE_LENGTH> _subscriber;
      Core::MW::CoreActuator<DataType>& _actuator;

private:
      bool
      onPrepareMW()
      {
         _subscriber.set_callback(Subscriber::callback);
         this->subscribe(_subscriber, configuration.topic);

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
//				Core::MW::log(???)
//				_actuator.stop();
            float x = 0.0;
            _actuator.set(x);
         }

         return true;
      }

      static bool
      callback(
         const actuator_msgs::Setpoint_f32& msg,
         Core::MW::Node*                    node
      )
      {
         Subscriber<_DATATYPE, _MESSAGETYPE, _CONVERTER>* _this = static_cast<Subscriber<_DATATYPE, _MESSAGETYPE, _CONVERTER>*>(node);
         float x = Converter::_(msg);
         _this->_actuator.set(x);

         return true;
      }
   };
}
