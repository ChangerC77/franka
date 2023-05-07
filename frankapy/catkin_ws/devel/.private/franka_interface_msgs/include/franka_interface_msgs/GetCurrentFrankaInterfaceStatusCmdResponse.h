// Generated by gencpp from file franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmdResponse.msg
// DO NOT EDIT!


#ifndef FRANKA_INTERFACE_MSGS_MESSAGE_GETCURRENTFRANKAINTERFACESTATUSCMDRESPONSE_H
#define FRANKA_INTERFACE_MSGS_MESSAGE_GETCURRENTFRANKAINTERFACESTATUSCMDRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <franka_interface_msgs/FrankaInterfaceStatus.h>

namespace franka_interface_msgs
{
template <class ContainerAllocator>
struct GetCurrentFrankaInterfaceStatusCmdResponse_
{
  typedef GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> Type;

  GetCurrentFrankaInterfaceStatusCmdResponse_()
    : franka_interface_status()  {
    }
  GetCurrentFrankaInterfaceStatusCmdResponse_(const ContainerAllocator& _alloc)
    : franka_interface_status(_alloc)  {
  (void)_alloc;
    }



   typedef  ::franka_interface_msgs::FrankaInterfaceStatus_<ContainerAllocator>  _franka_interface_status_type;
  _franka_interface_status_type franka_interface_status;





  typedef boost::shared_ptr< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetCurrentFrankaInterfaceStatusCmdResponse_

typedef ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<std::allocator<void> > GetCurrentFrankaInterfaceStatusCmdResponse;

typedef boost::shared_ptr< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse > GetCurrentFrankaInterfaceStatusCmdResponsePtr;
typedef boost::shared_ptr< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse const> GetCurrentFrankaInterfaceStatusCmdResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator1> & lhs, const ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator2> & rhs)
{
  return lhs.franka_interface_status == rhs.franka_interface_status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator1> & lhs, const ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace franka_interface_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1d3fe55393993751ebdd77c7f87c1d1e";
  }

  static const char* value(const ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1d3fe55393993751ULL;
  static const uint64_t static_value2 = 0xebdd77c7f87c1d1eULL;
};

template<class ContainerAllocator>
struct DataType< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmdResponse";
  }

  static const char* value(const ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "FrankaInterfaceStatus franka_interface_status\n"
"\n"
"================================================================================\n"
"MSG: franka_interface_msgs/FrankaInterfaceStatus\n"
"# Franka robot state\n"
"std_msgs/Header header\n"
"bool is_ready\n"
"string error_description\n"
"bool is_fresh\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.franka_interface_status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetCurrentFrankaInterfaceStatusCmdResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse_<ContainerAllocator>& v)
  {
    s << indent << "franka_interface_status: ";
    s << std::endl;
    Printer< ::franka_interface_msgs::FrankaInterfaceStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.franka_interface_status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FRANKA_INTERFACE_MSGS_MESSAGE_GETCURRENTFRANKAINTERFACESTATUSCMDRESPONSE_H
