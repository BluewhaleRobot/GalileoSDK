// Generated by gencpp from file galileo_serial_server/GalileoNativeCmds.msg
// DO NOT EDIT!


#ifndef GALILEO_SERIAL_SERVER_MESSAGE_GALILEONATIVECMDS_H
#define GALILEO_SERIAL_SERVER_MESSAGE_GALILEONATIVECMDS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace galileo_serial_server
{
template <class ContainerAllocator>
struct GalileoNativeCmds_
{
  typedef GalileoNativeCmds_<ContainerAllocator> Type;

  GalileoNativeCmds_()
    : header()
    , length(0)
    , data()  {
    }
  GalileoNativeCmds_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , length(0)
    , data(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint32_t _length_type;
  _length_type length;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> const> ConstPtr;

}; // struct GalileoNativeCmds_

typedef ::galileo_serial_server::GalileoNativeCmds_<std::allocator<void> > GalileoNativeCmds;

typedef boost::shared_ptr< ::galileo_serial_server::GalileoNativeCmds > GalileoNativeCmdsPtr;
typedef boost::shared_ptr< ::galileo_serial_server::GalileoNativeCmds const> GalileoNativeCmdsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace galileo_serial_server

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'galileo_serial_server': ['/home/xiaoqiang/Documents/ros/src/galileo_serial_server/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3274695c621e6cd7fc9fc51e9b36c67f";
  }

  static const char* value(const ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3274695c621e6cd7ULL;
  static const uint64_t static_value2 = 0xfc9fc51e9b36c67fULL;
};

template<class ContainerAllocator>
struct DataType< ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> >
{
  static const char* value()
  {
    return "galileo_serial_server/GalileoNativeCmds";
  }

  static const char* value(const ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n\
uint32  length\n\
uint8[] data\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.length);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GalileoNativeCmds_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::galileo_serial_server::GalileoNativeCmds_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "length: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.length);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // GALILEO_SERIAL_SERVER_MESSAGE_GALILEONATIVECMDS_H
