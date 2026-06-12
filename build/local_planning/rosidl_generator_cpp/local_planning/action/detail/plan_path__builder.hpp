// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from local_planning:action/PlanPath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "local_planning/action/plan_path.hpp"


#ifndef LOCAL_PLANNING__ACTION__DETAIL__PLAN_PATH__BUILDER_HPP_
#define LOCAL_PLANNING__ACTION__DETAIL__PLAN_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "local_planning/action/detail/plan_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace local_planning
{

namespace action
{

namespace builder
{

class Init_PlanPath_Goal_intent
{
public:
  Init_PlanPath_Goal_intent()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::local_planning::action::PlanPath_Goal intent(::local_planning::action::PlanPath_Goal::_intent_type arg)
  {
    msg_.intent = std::move(arg);
    return std::move(msg_);
  }

private:
  ::local_planning::action::PlanPath_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::local_planning::action::PlanPath_Goal>()
{
  return local_planning::action::builder::Init_PlanPath_Goal_intent();
}

}  // namespace local_planning


namespace local_planning
{

namespace action
{

namespace builder
{

class Init_PlanPath_Result_path
{
public:
  explicit Init_PlanPath_Result_path(::local_planning::action::PlanPath_Result & msg)
  : msg_(msg)
  {}
  ::local_planning::action::PlanPath_Result path(::local_planning::action::PlanPath_Result::_path_type arg)
  {
    msg_.path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::local_planning::action::PlanPath_Result msg_;
};

class Init_PlanPath_Result_success
{
public:
  Init_PlanPath_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPath_Result_path success(::local_planning::action::PlanPath_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PlanPath_Result_path(msg_);
  }

private:
  ::local_planning::action::PlanPath_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::local_planning::action::PlanPath_Result>()
{
  return local_planning::action::builder::Init_PlanPath_Result_success();
}

}  // namespace local_planning


namespace local_planning
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::local_planning::action::PlanPath_Feedback>()
{
  return ::local_planning::action::PlanPath_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace local_planning


namespace local_planning
{

namespace action
{

namespace builder
{

class Init_PlanPath_SendGoal_Request_goal
{
public:
  explicit Init_PlanPath_SendGoal_Request_goal(::local_planning::action::PlanPath_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::local_planning::action::PlanPath_SendGoal_Request goal(::local_planning::action::PlanPath_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::local_planning::action::PlanPath_SendGoal_Request msg_;
};

class Init_PlanPath_SendGoal_Request_goal_id
{
public:
  Init_PlanPath_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPath_SendGoal_Request_goal goal_id(::local_planning::action::PlanPath_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PlanPath_SendGoal_Request_goal(msg_);
  }

private:
  ::local_planning::action::PlanPath_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::local_planning::action::PlanPath_SendGoal_Request>()
{
  return local_planning::action::builder::Init_PlanPath_SendGoal_Request_goal_id();
}

}  // namespace local_planning


namespace local_planning
{

namespace action
{

namespace builder
{

class Init_PlanPath_SendGoal_Response_stamp
{
public:
  explicit Init_PlanPath_SendGoal_Response_stamp(::local_planning::action::PlanPath_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::local_planning::action::PlanPath_SendGoal_Response stamp(::local_planning::action::PlanPath_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::local_planning::action::PlanPath_SendGoal_Response msg_;
};

class Init_PlanPath_SendGoal_Response_accepted
{
public:
  Init_PlanPath_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPath_SendGoal_Response_stamp accepted(::local_planning::action::PlanPath_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_PlanPath_SendGoal_Response_stamp(msg_);
  }

private:
  ::local_planning::action::PlanPath_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::local_planning::action::PlanPath_SendGoal_Response>()
{
  return local_planning::action::builder::Init_PlanPath_SendGoal_Response_accepted();
}

}  // namespace local_planning


namespace local_planning
{

namespace action
{

namespace builder
{

class Init_PlanPath_SendGoal_Event_response
{
public:
  explicit Init_PlanPath_SendGoal_Event_response(::local_planning::action::PlanPath_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::local_planning::action::PlanPath_SendGoal_Event response(::local_planning::action::PlanPath_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::local_planning::action::PlanPath_SendGoal_Event msg_;
};

class Init_PlanPath_SendGoal_Event_request
{
public:
  explicit Init_PlanPath_SendGoal_Event_request(::local_planning::action::PlanPath_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_PlanPath_SendGoal_Event_response request(::local_planning::action::PlanPath_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_PlanPath_SendGoal_Event_response(msg_);
  }

private:
  ::local_planning::action::PlanPath_SendGoal_Event msg_;
};

class Init_PlanPath_SendGoal_Event_info
{
public:
  Init_PlanPath_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPath_SendGoal_Event_request info(::local_planning::action::PlanPath_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_PlanPath_SendGoal_Event_request(msg_);
  }

private:
  ::local_planning::action::PlanPath_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::local_planning::action::PlanPath_SendGoal_Event>()
{
  return local_planning::action::builder::Init_PlanPath_SendGoal_Event_info();
}

}  // namespace local_planning


namespace local_planning
{

namespace action
{

namespace builder
{

class Init_PlanPath_GetResult_Request_goal_id
{
public:
  Init_PlanPath_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::local_planning::action::PlanPath_GetResult_Request goal_id(::local_planning::action::PlanPath_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::local_planning::action::PlanPath_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::local_planning::action::PlanPath_GetResult_Request>()
{
  return local_planning::action::builder::Init_PlanPath_GetResult_Request_goal_id();
}

}  // namespace local_planning


namespace local_planning
{

namespace action
{

namespace builder
{

class Init_PlanPath_GetResult_Response_result
{
public:
  explicit Init_PlanPath_GetResult_Response_result(::local_planning::action::PlanPath_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::local_planning::action::PlanPath_GetResult_Response result(::local_planning::action::PlanPath_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::local_planning::action::PlanPath_GetResult_Response msg_;
};

class Init_PlanPath_GetResult_Response_status
{
public:
  Init_PlanPath_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPath_GetResult_Response_result status(::local_planning::action::PlanPath_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PlanPath_GetResult_Response_result(msg_);
  }

private:
  ::local_planning::action::PlanPath_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::local_planning::action::PlanPath_GetResult_Response>()
{
  return local_planning::action::builder::Init_PlanPath_GetResult_Response_status();
}

}  // namespace local_planning


namespace local_planning
{

namespace action
{

namespace builder
{

class Init_PlanPath_GetResult_Event_response
{
public:
  explicit Init_PlanPath_GetResult_Event_response(::local_planning::action::PlanPath_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::local_planning::action::PlanPath_GetResult_Event response(::local_planning::action::PlanPath_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::local_planning::action::PlanPath_GetResult_Event msg_;
};

class Init_PlanPath_GetResult_Event_request
{
public:
  explicit Init_PlanPath_GetResult_Event_request(::local_planning::action::PlanPath_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_PlanPath_GetResult_Event_response request(::local_planning::action::PlanPath_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_PlanPath_GetResult_Event_response(msg_);
  }

private:
  ::local_planning::action::PlanPath_GetResult_Event msg_;
};

class Init_PlanPath_GetResult_Event_info
{
public:
  Init_PlanPath_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPath_GetResult_Event_request info(::local_planning::action::PlanPath_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_PlanPath_GetResult_Event_request(msg_);
  }

private:
  ::local_planning::action::PlanPath_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::local_planning::action::PlanPath_GetResult_Event>()
{
  return local_planning::action::builder::Init_PlanPath_GetResult_Event_info();
}

}  // namespace local_planning


namespace local_planning
{

namespace action
{

namespace builder
{

class Init_PlanPath_FeedbackMessage_feedback
{
public:
  explicit Init_PlanPath_FeedbackMessage_feedback(::local_planning::action::PlanPath_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::local_planning::action::PlanPath_FeedbackMessage feedback(::local_planning::action::PlanPath_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::local_planning::action::PlanPath_FeedbackMessage msg_;
};

class Init_PlanPath_FeedbackMessage_goal_id
{
public:
  Init_PlanPath_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPath_FeedbackMessage_feedback goal_id(::local_planning::action::PlanPath_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PlanPath_FeedbackMessage_feedback(msg_);
  }

private:
  ::local_planning::action::PlanPath_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::local_planning::action::PlanPath_FeedbackMessage>()
{
  return local_planning::action::builder::Init_PlanPath_FeedbackMessage_goal_id();
}

}  // namespace local_planning

#endif  // LOCAL_PLANNING__ACTION__DETAIL__PLAN_PATH__BUILDER_HPP_
