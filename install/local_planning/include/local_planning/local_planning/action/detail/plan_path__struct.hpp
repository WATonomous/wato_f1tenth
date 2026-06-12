// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from local_planning:action/PlanPath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "local_planning/action/plan_path.hpp"


#ifndef LOCAL_PLANNING__ACTION__DETAIL__PLAN_PATH__STRUCT_HPP_
#define LOCAL_PLANNING__ACTION__DETAIL__PLAN_PATH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__local_planning__action__PlanPath_Goal __attribute__((deprecated))
#else
# define DEPRECATED__local_planning__action__PlanPath_Goal __declspec(deprecated)
#endif

namespace local_planning
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_Goal_
{
  using Type = PlanPath_Goal_<ContainerAllocator>;

  explicit PlanPath_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->intent = 0;
    }
  }

  explicit PlanPath_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->intent = 0;
    }
  }

  // field types and members
  using _intent_type =
    uint8_t;
  _intent_type intent;

  // setters for named parameter idiom
  Type & set__intent(
    const uint8_t & _arg)
  {
    this->intent = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FOLLOW_RACING_LINE =
    0u;
  static constexpr uint8_t OVERTAKE =
    1u;
  static constexpr uint8_t MERGE =
    2u;

  // pointer types
  using RawPtr =
    local_planning::action::PlanPath_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const local_planning::action::PlanPath_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__local_planning__action__PlanPath_Goal
    std::shared_ptr<local_planning::action::PlanPath_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__local_planning__action__PlanPath_Goal
    std::shared_ptr<local_planning::action::PlanPath_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_Goal_ & other) const
  {
    if (this->intent != other.intent) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_Goal_

// alias to use template instance with default allocator
using PlanPath_Goal =
  local_planning::action::PlanPath_Goal_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PlanPath_Goal_<ContainerAllocator>::FOLLOW_RACING_LINE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PlanPath_Goal_<ContainerAllocator>::OVERTAKE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PlanPath_Goal_<ContainerAllocator>::MERGE;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace local_planning


// Include directives for member types
// Member 'path'
#include "nav_msgs/msg/detail/path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__local_planning__action__PlanPath_Result __attribute__((deprecated))
#else
# define DEPRECATED__local_planning__action__PlanPath_Result __declspec(deprecated)
#endif

namespace local_planning
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_Result_
{
  using Type = PlanPath_Result_<ContainerAllocator>;

  explicit PlanPath_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : path(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit PlanPath_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : path(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _path_type =
    nav_msgs::msg::Path_<ContainerAllocator>;
  _path_type path;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__path(
    const nav_msgs::msg::Path_<ContainerAllocator> & _arg)
  {
    this->path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    local_planning::action::PlanPath_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const local_planning::action::PlanPath_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__local_planning__action__PlanPath_Result
    std::shared_ptr<local_planning::action::PlanPath_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__local_planning__action__PlanPath_Result
    std::shared_ptr<local_planning::action::PlanPath_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->path != other.path) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_Result_

// alias to use template instance with default allocator
using PlanPath_Result =
  local_planning::action::PlanPath_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace local_planning


#ifndef _WIN32
# define DEPRECATED__local_planning__action__PlanPath_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__local_planning__action__PlanPath_Feedback __declspec(deprecated)
#endif

namespace local_planning
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_Feedback_
{
  using Type = PlanPath_Feedback_<ContainerAllocator>;

  explicit PlanPath_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit PlanPath_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    local_planning::action::PlanPath_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const local_planning::action::PlanPath_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__local_planning__action__PlanPath_Feedback
    std::shared_ptr<local_planning::action::PlanPath_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__local_planning__action__PlanPath_Feedback
    std::shared_ptr<local_planning::action::PlanPath_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_Feedback_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_Feedback_

// alias to use template instance with default allocator
using PlanPath_Feedback =
  local_planning::action::PlanPath_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace local_planning


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "local_planning/action/detail/plan_path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__local_planning__action__PlanPath_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__local_planning__action__PlanPath_SendGoal_Request __declspec(deprecated)
#endif

namespace local_planning
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_SendGoal_Request_
{
  using Type = PlanPath_SendGoal_Request_<ContainerAllocator>;

  explicit PlanPath_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit PlanPath_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    local_planning::action::PlanPath_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const local_planning::action::PlanPath_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__local_planning__action__PlanPath_SendGoal_Request
    std::shared_ptr<local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__local_planning__action__PlanPath_SendGoal_Request
    std::shared_ptr<local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_SendGoal_Request_

// alias to use template instance with default allocator
using PlanPath_SendGoal_Request =
  local_planning::action::PlanPath_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace local_planning


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__local_planning__action__PlanPath_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__local_planning__action__PlanPath_SendGoal_Response __declspec(deprecated)
#endif

namespace local_planning
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_SendGoal_Response_
{
  using Type = PlanPath_SendGoal_Response_<ContainerAllocator>;

  explicit PlanPath_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit PlanPath_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__local_planning__action__PlanPath_SendGoal_Response
    std::shared_ptr<local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__local_planning__action__PlanPath_SendGoal_Response
    std::shared_ptr<local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_SendGoal_Response_

// alias to use template instance with default allocator
using PlanPath_SendGoal_Response =
  local_planning::action::PlanPath_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace local_planning


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__local_planning__action__PlanPath_SendGoal_Event __attribute__((deprecated))
#else
# define DEPRECATED__local_planning__action__PlanPath_SendGoal_Event __declspec(deprecated)
#endif

namespace local_planning
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_SendGoal_Event_
{
  using Type = PlanPath_SendGoal_Event_<ContainerAllocator>;

  explicit PlanPath_SendGoal_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit PlanPath_SendGoal_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<local_planning::action::PlanPath_SendGoal_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<local_planning::action::PlanPath_SendGoal_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    local_planning::action::PlanPath_SendGoal_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const local_planning::action::PlanPath_SendGoal_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_SendGoal_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_SendGoal_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_SendGoal_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_SendGoal_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_SendGoal_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_SendGoal_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_SendGoal_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_SendGoal_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__local_planning__action__PlanPath_SendGoal_Event
    std::shared_ptr<local_planning::action::PlanPath_SendGoal_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__local_planning__action__PlanPath_SendGoal_Event
    std::shared_ptr<local_planning::action::PlanPath_SendGoal_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_SendGoal_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_SendGoal_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_SendGoal_Event_

// alias to use template instance with default allocator
using PlanPath_SendGoal_Event =
  local_planning::action::PlanPath_SendGoal_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace local_planning

namespace local_planning
{

namespace action
{

struct PlanPath_SendGoal
{
  using Request = local_planning::action::PlanPath_SendGoal_Request;
  using Response = local_planning::action::PlanPath_SendGoal_Response;
  using Event = local_planning::action::PlanPath_SendGoal_Event;
};

}  // namespace action

}  // namespace local_planning


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__local_planning__action__PlanPath_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__local_planning__action__PlanPath_GetResult_Request __declspec(deprecated)
#endif

namespace local_planning
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_GetResult_Request_
{
  using Type = PlanPath_GetResult_Request_<ContainerAllocator>;

  explicit PlanPath_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit PlanPath_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__local_planning__action__PlanPath_GetResult_Request
    std::shared_ptr<local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__local_planning__action__PlanPath_GetResult_Request
    std::shared_ptr<local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_GetResult_Request_

// alias to use template instance with default allocator
using PlanPath_GetResult_Request =
  local_planning::action::PlanPath_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace local_planning


// Include directives for member types
// Member 'result'
// already included above
// #include "local_planning/action/detail/plan_path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__local_planning__action__PlanPath_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__local_planning__action__PlanPath_GetResult_Response __declspec(deprecated)
#endif

namespace local_planning
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_GetResult_Response_
{
  using Type = PlanPath_GetResult_Response_<ContainerAllocator>;

  explicit PlanPath_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit PlanPath_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    local_planning::action::PlanPath_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const local_planning::action::PlanPath_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__local_planning__action__PlanPath_GetResult_Response
    std::shared_ptr<local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__local_planning__action__PlanPath_GetResult_Response
    std::shared_ptr<local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_GetResult_Response_

// alias to use template instance with default allocator
using PlanPath_GetResult_Response =
  local_planning::action::PlanPath_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace local_planning


// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__local_planning__action__PlanPath_GetResult_Event __attribute__((deprecated))
#else
# define DEPRECATED__local_planning__action__PlanPath_GetResult_Event __declspec(deprecated)
#endif

namespace local_planning
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_GetResult_Event_
{
  using Type = PlanPath_GetResult_Event_<ContainerAllocator>;

  explicit PlanPath_GetResult_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit PlanPath_GetResult_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<local_planning::action::PlanPath_GetResult_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<local_planning::action::PlanPath_GetResult_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    local_planning::action::PlanPath_GetResult_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const local_planning::action::PlanPath_GetResult_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_GetResult_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_GetResult_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_GetResult_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_GetResult_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_GetResult_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_GetResult_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_GetResult_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_GetResult_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__local_planning__action__PlanPath_GetResult_Event
    std::shared_ptr<local_planning::action::PlanPath_GetResult_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__local_planning__action__PlanPath_GetResult_Event
    std::shared_ptr<local_planning::action::PlanPath_GetResult_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_GetResult_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_GetResult_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_GetResult_Event_

// alias to use template instance with default allocator
using PlanPath_GetResult_Event =
  local_planning::action::PlanPath_GetResult_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace local_planning

namespace local_planning
{

namespace action
{

struct PlanPath_GetResult
{
  using Request = local_planning::action::PlanPath_GetResult_Request;
  using Response = local_planning::action::PlanPath_GetResult_Response;
  using Event = local_planning::action::PlanPath_GetResult_Event;
};

}  // namespace action

}  // namespace local_planning


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "local_planning/action/detail/plan_path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__local_planning__action__PlanPath_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__local_planning__action__PlanPath_FeedbackMessage __declspec(deprecated)
#endif

namespace local_planning
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanPath_FeedbackMessage_
{
  using Type = PlanPath_FeedbackMessage_<ContainerAllocator>;

  explicit PlanPath_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit PlanPath_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    local_planning::action::PlanPath_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const local_planning::action::PlanPath_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    local_planning::action::PlanPath_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const local_planning::action::PlanPath_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<local_planning::action::PlanPath_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      local_planning::action::PlanPath_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<local_planning::action::PlanPath_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<local_planning::action::PlanPath_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__local_planning__action__PlanPath_FeedbackMessage
    std::shared_ptr<local_planning::action::PlanPath_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__local_planning__action__PlanPath_FeedbackMessage
    std::shared_ptr<local_planning::action::PlanPath_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanPath_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanPath_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanPath_FeedbackMessage_

// alias to use template instance with default allocator
using PlanPath_FeedbackMessage =
  local_planning::action::PlanPath_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace local_planning

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace local_planning
{

namespace action
{

struct PlanPath
{
  /// The goal message defined in the action definition.
  using Goal = local_planning::action::PlanPath_Goal;
  /// The result message defined in the action definition.
  using Result = local_planning::action::PlanPath_Result;
  /// The feedback message defined in the action definition.
  using Feedback = local_planning::action::PlanPath_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = local_planning::action::PlanPath_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = local_planning::action::PlanPath_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = local_planning::action::PlanPath_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct PlanPath PlanPath;

}  // namespace action

}  // namespace local_planning

#endif  // LOCAL_PLANNING__ACTION__DETAIL__PLAN_PATH__STRUCT_HPP_
