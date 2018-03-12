/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/utils/BasicActuator.hpp>

#include <core/actuator_subscriber/SpeedConfiguration.hpp>
#include <core/pid_ie/pid_ie.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>

#include <ModuleConfiguration.hpp>
#include <Module.hpp>

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

template <typename _DATATYPE, class _MESSAGETYPE = _DATATYPE, class _ENCODER_MESSAGETYPE = _DATATYPE, class _CONVERTER = ValueOf<_DATATYPE, _MESSAGETYPE>, class _ENCODER_CONVERTER = ValueOf<_DATATYPE, _ENCODER_MESSAGETYPE> >
class Speed:
    public core::mw::CoreNode,
    public core::mw::CoreConfigurable<SpeedConfiguration>
{
public:
    using DataType    = _DATATYPE;
    using MessageType = _MESSAGETYPE;
    using Converter   = _CONVERTER;
    using EncoderMessageType = _ENCODER_MESSAGETYPE;
    using EncoderConverter   = _ENCODER_CONVERTER;

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
    core::mw::Subscriber<MessageType, ModuleConfiguration::SUBSCRIBER_QUEUE_LENGTH>        _setpoint_subscriber;
    core::mw::Subscriber<EncoderMessageType, ModuleConfiguration::SUBSCRIBER_QUEUE_LENGTH> _encoder_subscriber;
    core::utils::BasicActuator<DataType>& _actuator;
    core::pid_ie::PID_IE _pid;
    core::os::Time       _setpoint_timestamp;

private:
    bool
    onConfigure()
    {
        _pid.config(configuration().kp, configuration().ti, configuration().td, configuration().ts, 100, configuration().min, configuration().max);

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
            _pid.reset();
            _pid.set(configuration().idle);
        }

        return true;
    }

    static bool
    setpoint_callback(
        const MessageType& msg,
        void*              context
    )
    {
        Speed<_DATATYPE, _MESSAGETYPE, _ENCODER_MESSAGETYPE>* _this = static_cast<Speed<_DATATYPE, _MESSAGETYPE, _ENCODER_MESSAGETYPE>*>(context);

        _this->_setpoint_timestamp = core::os::Time::now();

        DataType setpoint = Converter::_(msg);
        _this->_pid.set(setpoint);

        return true;
    }

    static bool
    encoder_callback(
        const EncoderMessageType& msg,
        void*                     context
    )
    {
        Speed<_DATATYPE, _MESSAGETYPE, _ENCODER_MESSAGETYPE>* _this = static_cast<Speed<_DATATYPE, _MESSAGETYPE, _ENCODER_MESSAGETYPE>*>(context);

        DataType measurement = EncoderConverter::_(msg);

        _this->_actuator.set(_this->_pid.update(measurement));

        return true;
    }
};

template <class _ACTUATOR, class _ENCODER>
class Speed_:
    public Speed<typename _ACTUATOR::Converter::TO, typename _ACTUATOR::Converter::FROM, typename _ENCODER::Converter::TO>
{
public:
    Speed_(
        const char*                                                    name,
        core::utils::BasicActuator<typename _ACTUATOR::Converter::TO>& sensor,
        core::os::Thread::Priority                                     priority = core::os::Thread::PriorityEnum::NORMAL
    ) : Speed<typename _ACTUATOR::Converter::TO, typename _ACTUATOR::Converter::FROM, typename _ENCODER::Converter::TO>(name, sensor, priority) {}
};
}
}
