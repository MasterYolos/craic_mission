// Generated by gencpp from file craic_mission/Px4sp_msgs.msg
// DO NOT EDIT!


#ifndef CRAIC_MISSION_MESSAGE_PX4SP_MSGS_H
#define CRAIC_MISSION_MESSAGE_PX4SP_MSGS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace craic_mission
{
template <class ContainerAllocator>
struct Px4sp_msgs_
{
  typedef Px4sp_msgs_<ContainerAllocator> Type;

  Px4sp_msgs_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , yaw(0.0)  {
    }
  Px4sp_msgs_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , yaw(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;

   typedef float _yaw_type;
  _yaw_type yaw;





  typedef boost::shared_ptr< ::craic_mission::Px4sp_msgs_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::craic_mission::Px4sp_msgs_<ContainerAllocator> const> ConstPtr;

}; // struct Px4sp_msgs_

typedef ::craic_mission::Px4sp_msgs_<std::allocator<void> > Px4sp_msgs;

typedef boost::shared_ptr< ::craic_mission::Px4sp_msgs > Px4sp_msgsPtr;
typedef boost::shared_ptr< ::craic_mission::Px4sp_msgs const> Px4sp_msgsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::craic_mission::Px4sp_msgs_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::craic_mission::Px4sp_msgs_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::craic_mission::Px4sp_msgs_<ContainerAllocator1> & lhs, const ::craic_mission::Px4sp_msgs_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.yaw == rhs.yaw;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::craic_mission::Px4sp_msgs_<ContainerAllocator1> & lhs, const ::craic_mission::Px4sp_msgs_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace craic_mission

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::craic_mission::Px4sp_msgs_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::craic_mission::Px4sp_msgs_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::craic_mission::Px4sp_msgs_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::craic_mission::Px4sp_msgs_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::craic_mission::Px4sp_msgs_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::craic_mission::Px4sp_msgs_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::craic_mission::Px4sp_msgs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b81376c2774ff4f956e5110e01c9db26";
  }

  static const char* value(const ::craic_mission::Px4sp_msgs_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb81376c2774ff4f9ULL;
  static const uint64_t static_value2 = 0x56e5110e01c9db26ULL;
};

template<class ContainerAllocator>
struct DataType< ::craic_mission::Px4sp_msgs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "craic_mission/Px4sp_msgs";
  }

  static const char* value(const ::craic_mission::Px4sp_msgs_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::craic_mission::Px4sp_msgs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
"float32 z\n"
"float32 yaw\n"
;
  }

  static const char* value(const ::craic_mission::Px4sp_msgs_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::craic_mission::Px4sp_msgs_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.yaw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Px4sp_msgs_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::craic_mission::Px4sp_msgs_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::craic_mission::Px4sp_msgs_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRAIC_MISSION_MESSAGE_PX4SP_MSGS_H
