// Generated by gencpp from file AUTMAV_msgs/LandingMsg.msg
// DO NOT EDIT!


#ifndef AUTMAV_MSGS_MESSAGE_LANDINGMSG_H
#define AUTMAV_MSGS_MESSAGE_LANDINGMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace AUTMAV_msgs
{
template <class ContainerAllocator>
struct LandingMsg_
{
  typedef LandingMsg_<ContainerAllocator> Type;

  LandingMsg_()
    : speed(0.0)  {
    }
  LandingMsg_(const ContainerAllocator& _alloc)
    : speed(0.0)  {
  (void)_alloc;
    }



   typedef float _speed_type;
  _speed_type speed;




  typedef boost::shared_ptr< ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> const> ConstPtr;

}; // struct LandingMsg_

typedef ::AUTMAV_msgs::LandingMsg_<std::allocator<void> > LandingMsg;

typedef boost::shared_ptr< ::AUTMAV_msgs::LandingMsg > LandingMsgPtr;
typedef boost::shared_ptr< ::AUTMAV_msgs::LandingMsg const> LandingMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace AUTMAV_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'AUTMAV_msgs': ['/home/hojat/GS_WS/src/AUTMAV_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ca65bba734a79b4a6707341d829f4d5c";
  }

  static const char* value(const ::AUTMAV_msgs::LandingMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xca65bba734a79b4aULL;
  static const uint64_t static_value2 = 0x6707341d829f4d5cULL;
};

template<class ContainerAllocator>
struct DataType< ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "AUTMAV_msgs/LandingMsg";
  }

  static const char* value(const ::AUTMAV_msgs::LandingMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 speed\n\
";
  }

  static const char* value(const ::AUTMAV_msgs::LandingMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.speed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LandingMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::AUTMAV_msgs::LandingMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::AUTMAV_msgs::LandingMsg_<ContainerAllocator>& v)
  {
    s << indent << "speed: ";
    Printer<float>::stream(s, indent + "  ", v.speed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTMAV_MSGS_MESSAGE_LANDINGMSG_H
