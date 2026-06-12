// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from local_planning:action/PlanPath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "local_planning/action/plan_path.h"


#ifndef LOCAL_PLANNING__ACTION__DETAIL__PLAN_PATH__STRUCT_H_
#define LOCAL_PLANNING__ACTION__DETAIL__PLAN_PATH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FOLLOW_RACING_LINE'.
enum
{
  local_planning__action__PlanPath_Goal__FOLLOW_RACING_LINE = 0
};

/// Constant 'OVERTAKE'.
enum
{
  local_planning__action__PlanPath_Goal__OVERTAKE = 1
};

/// Constant 'MERGE'.
enum
{
  local_planning__action__PlanPath_Goal__MERGE = 2
};

/// Struct defined in action/PlanPath in the package local_planning.
typedef struct local_planning__action__PlanPath_Goal
{
  uint8_t intent;
} local_planning__action__PlanPath_Goal;

// Struct for a sequence of local_planning__action__PlanPath_Goal.
typedef struct local_planning__action__PlanPath_Goal__Sequence
{
  local_planning__action__PlanPath_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} local_planning__action__PlanPath_Goal__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'path'
#include "nav_msgs/msg/detail/path__struct.h"

/// Struct defined in action/PlanPath in the package local_planning.
typedef struct local_planning__action__PlanPath_Result
{
  bool success;
  nav_msgs__msg__Path path;
} local_planning__action__PlanPath_Result;

// Struct for a sequence of local_planning__action__PlanPath_Result.
typedef struct local_planning__action__PlanPath_Result__Sequence
{
  local_planning__action__PlanPath_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} local_planning__action__PlanPath_Result__Sequence;

// Constants defined in the message

/// Struct defined in action/PlanPath in the package local_planning.
typedef struct local_planning__action__PlanPath_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} local_planning__action__PlanPath_Feedback;

// Struct for a sequence of local_planning__action__PlanPath_Feedback.
typedef struct local_planning__action__PlanPath_Feedback__Sequence
{
  local_planning__action__PlanPath_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} local_planning__action__PlanPath_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "local_planning/action/detail/plan_path__struct.h"

/// Struct defined in action/PlanPath in the package local_planning.
typedef struct local_planning__action__PlanPath_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  local_planning__action__PlanPath_Goal goal;
} local_planning__action__PlanPath_SendGoal_Request;

// Struct for a sequence of local_planning__action__PlanPath_SendGoal_Request.
typedef struct local_planning__action__PlanPath_SendGoal_Request__Sequence
{
  local_planning__action__PlanPath_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} local_planning__action__PlanPath_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/PlanPath in the package local_planning.
typedef struct local_planning__action__PlanPath_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} local_planning__action__PlanPath_SendGoal_Response;

// Struct for a sequence of local_planning__action__PlanPath_SendGoal_Response.
typedef struct local_planning__action__PlanPath_SendGoal_Response__Sequence
{
  local_planning__action__PlanPath_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} local_planning__action__PlanPath_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  local_planning__action__PlanPath_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  local_planning__action__PlanPath_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/PlanPath in the package local_planning.
typedef struct local_planning__action__PlanPath_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  local_planning__action__PlanPath_SendGoal_Request__Sequence request;
  local_planning__action__PlanPath_SendGoal_Response__Sequence response;
} local_planning__action__PlanPath_SendGoal_Event;

// Struct for a sequence of local_planning__action__PlanPath_SendGoal_Event.
typedef struct local_planning__action__PlanPath_SendGoal_Event__Sequence
{
  local_planning__action__PlanPath_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} local_planning__action__PlanPath_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/PlanPath in the package local_planning.
typedef struct local_planning__action__PlanPath_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} local_planning__action__PlanPath_GetResult_Request;

// Struct for a sequence of local_planning__action__PlanPath_GetResult_Request.
typedef struct local_planning__action__PlanPath_GetResult_Request__Sequence
{
  local_planning__action__PlanPath_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} local_planning__action__PlanPath_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "local_planning/action/detail/plan_path__struct.h"

/// Struct defined in action/PlanPath in the package local_planning.
typedef struct local_planning__action__PlanPath_GetResult_Response
{
  int8_t status;
  local_planning__action__PlanPath_Result result;
} local_planning__action__PlanPath_GetResult_Response;

// Struct for a sequence of local_planning__action__PlanPath_GetResult_Response.
typedef struct local_planning__action__PlanPath_GetResult_Response__Sequence
{
  local_planning__action__PlanPath_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} local_planning__action__PlanPath_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  local_planning__action__PlanPath_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  local_planning__action__PlanPath_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/PlanPath in the package local_planning.
typedef struct local_planning__action__PlanPath_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  local_planning__action__PlanPath_GetResult_Request__Sequence request;
  local_planning__action__PlanPath_GetResult_Response__Sequence response;
} local_planning__action__PlanPath_GetResult_Event;

// Struct for a sequence of local_planning__action__PlanPath_GetResult_Event.
typedef struct local_planning__action__PlanPath_GetResult_Event__Sequence
{
  local_planning__action__PlanPath_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} local_planning__action__PlanPath_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "local_planning/action/detail/plan_path__struct.h"

/// Struct defined in action/PlanPath in the package local_planning.
typedef struct local_planning__action__PlanPath_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  local_planning__action__PlanPath_Feedback feedback;
} local_planning__action__PlanPath_FeedbackMessage;

// Struct for a sequence of local_planning__action__PlanPath_FeedbackMessage.
typedef struct local_planning__action__PlanPath_FeedbackMessage__Sequence
{
  local_planning__action__PlanPath_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} local_planning__action__PlanPath_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LOCAL_PLANNING__ACTION__DETAIL__PLAN_PATH__STRUCT_H_
