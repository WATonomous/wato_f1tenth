// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from local_planning:action/PlanPath.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "local_planning/action/detail/plan_path__functions.h"
#include "local_planning/action/detail/plan_path__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace local_planning
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_Goal_type_support_ids_t;

static const _PlanPath_Goal_type_support_ids_t _PlanPath_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanPath_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_Goal_type_support_symbol_names_t _PlanPath_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, local_planning, action, PlanPath_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, local_planning, action, PlanPath_Goal)),
  }
};

typedef struct _PlanPath_Goal_type_support_data_t
{
  void * data[2];
} _PlanPath_Goal_type_support_data_t;

static _PlanPath_Goal_type_support_data_t _PlanPath_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_Goal_message_typesupport_map = {
  2,
  "local_planning",
  &_PlanPath_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &local_planning__action__PlanPath_Goal__get_type_hash,
  &local_planning__action__PlanPath_Goal__get_type_description,
  &local_planning__action__PlanPath_Goal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace local_planning

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<local_planning::action::PlanPath_Goal>()
{
  return &::local_planning::action::rosidl_typesupport_cpp::PlanPath_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, local_planning, action, PlanPath_Goal)() {
  return get_message_type_support_handle<local_planning::action::PlanPath_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "local_planning/action/detail/plan_path__functions.h"
// already included above
// #include "local_planning/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace local_planning
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_Result_type_support_ids_t;

static const _PlanPath_Result_type_support_ids_t _PlanPath_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanPath_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_Result_type_support_symbol_names_t _PlanPath_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, local_planning, action, PlanPath_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, local_planning, action, PlanPath_Result)),
  }
};

typedef struct _PlanPath_Result_type_support_data_t
{
  void * data[2];
} _PlanPath_Result_type_support_data_t;

static _PlanPath_Result_type_support_data_t _PlanPath_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_Result_message_typesupport_map = {
  2,
  "local_planning",
  &_PlanPath_Result_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_Result_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &local_planning__action__PlanPath_Result__get_type_hash,
  &local_planning__action__PlanPath_Result__get_type_description,
  &local_planning__action__PlanPath_Result__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace local_planning

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<local_planning::action::PlanPath_Result>()
{
  return &::local_planning::action::rosidl_typesupport_cpp::PlanPath_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, local_planning, action, PlanPath_Result)() {
  return get_message_type_support_handle<local_planning::action::PlanPath_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "local_planning/action/detail/plan_path__functions.h"
// already included above
// #include "local_planning/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace local_planning
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_Feedback_type_support_ids_t;

static const _PlanPath_Feedback_type_support_ids_t _PlanPath_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanPath_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_Feedback_type_support_symbol_names_t _PlanPath_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, local_planning, action, PlanPath_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, local_planning, action, PlanPath_Feedback)),
  }
};

typedef struct _PlanPath_Feedback_type_support_data_t
{
  void * data[2];
} _PlanPath_Feedback_type_support_data_t;

static _PlanPath_Feedback_type_support_data_t _PlanPath_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_Feedback_message_typesupport_map = {
  2,
  "local_planning",
  &_PlanPath_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &local_planning__action__PlanPath_Feedback__get_type_hash,
  &local_planning__action__PlanPath_Feedback__get_type_description,
  &local_planning__action__PlanPath_Feedback__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace local_planning

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<local_planning::action::PlanPath_Feedback>()
{
  return &::local_planning::action::rosidl_typesupport_cpp::PlanPath_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, local_planning, action, PlanPath_Feedback)() {
  return get_message_type_support_handle<local_planning::action::PlanPath_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "local_planning/action/detail/plan_path__functions.h"
// already included above
// #include "local_planning/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace local_planning
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_SendGoal_Request_type_support_ids_t;

static const _PlanPath_SendGoal_Request_type_support_ids_t _PlanPath_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanPath_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_SendGoal_Request_type_support_symbol_names_t _PlanPath_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, local_planning, action, PlanPath_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, local_planning, action, PlanPath_SendGoal_Request)),
  }
};

typedef struct _PlanPath_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _PlanPath_SendGoal_Request_type_support_data_t;

static _PlanPath_SendGoal_Request_type_support_data_t _PlanPath_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_SendGoal_Request_message_typesupport_map = {
  2,
  "local_planning",
  &_PlanPath_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &local_planning__action__PlanPath_SendGoal_Request__get_type_hash,
  &local_planning__action__PlanPath_SendGoal_Request__get_type_description,
  &local_planning__action__PlanPath_SendGoal_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace local_planning

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<local_planning::action::PlanPath_SendGoal_Request>()
{
  return &::local_planning::action::rosidl_typesupport_cpp::PlanPath_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, local_planning, action, PlanPath_SendGoal_Request)() {
  return get_message_type_support_handle<local_planning::action::PlanPath_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "local_planning/action/detail/plan_path__functions.h"
// already included above
// #include "local_planning/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace local_planning
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_SendGoal_Response_type_support_ids_t;

static const _PlanPath_SendGoal_Response_type_support_ids_t _PlanPath_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanPath_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_SendGoal_Response_type_support_symbol_names_t _PlanPath_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, local_planning, action, PlanPath_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, local_planning, action, PlanPath_SendGoal_Response)),
  }
};

typedef struct _PlanPath_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _PlanPath_SendGoal_Response_type_support_data_t;

static _PlanPath_SendGoal_Response_type_support_data_t _PlanPath_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_SendGoal_Response_message_typesupport_map = {
  2,
  "local_planning",
  &_PlanPath_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &local_planning__action__PlanPath_SendGoal_Response__get_type_hash,
  &local_planning__action__PlanPath_SendGoal_Response__get_type_description,
  &local_planning__action__PlanPath_SendGoal_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace local_planning

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<local_planning::action::PlanPath_SendGoal_Response>()
{
  return &::local_planning::action::rosidl_typesupport_cpp::PlanPath_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, local_planning, action, PlanPath_SendGoal_Response)() {
  return get_message_type_support_handle<local_planning::action::PlanPath_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "local_planning/action/detail/plan_path__functions.h"
// already included above
// #include "local_planning/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace local_planning
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_SendGoal_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_SendGoal_Event_type_support_ids_t;

static const _PlanPath_SendGoal_Event_type_support_ids_t _PlanPath_SendGoal_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanPath_SendGoal_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_SendGoal_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_SendGoal_Event_type_support_symbol_names_t _PlanPath_SendGoal_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, local_planning, action, PlanPath_SendGoal_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, local_planning, action, PlanPath_SendGoal_Event)),
  }
};

typedef struct _PlanPath_SendGoal_Event_type_support_data_t
{
  void * data[2];
} _PlanPath_SendGoal_Event_type_support_data_t;

static _PlanPath_SendGoal_Event_type_support_data_t _PlanPath_SendGoal_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_SendGoal_Event_message_typesupport_map = {
  2,
  "local_planning",
  &_PlanPath_SendGoal_Event_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_SendGoal_Event_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_SendGoal_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_SendGoal_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_SendGoal_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &local_planning__action__PlanPath_SendGoal_Event__get_type_hash,
  &local_planning__action__PlanPath_SendGoal_Event__get_type_description,
  &local_planning__action__PlanPath_SendGoal_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace local_planning

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<local_planning::action::PlanPath_SendGoal_Event>()
{
  return &::local_planning::action::rosidl_typesupport_cpp::PlanPath_SendGoal_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, local_planning, action, PlanPath_SendGoal_Event)() {
  return get_message_type_support_handle<local_planning::action::PlanPath_SendGoal_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "local_planning/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace local_planning
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_SendGoal_type_support_ids_t;

static const _PlanPath_SendGoal_type_support_ids_t _PlanPath_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanPath_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_SendGoal_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_SendGoal_type_support_symbol_names_t _PlanPath_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, local_planning, action, PlanPath_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, local_planning, action, PlanPath_SendGoal)),
  }
};

typedef struct _PlanPath_SendGoal_type_support_data_t
{
  void * data[2];
} _PlanPath_SendGoal_type_support_data_t;

static _PlanPath_SendGoal_type_support_data_t _PlanPath_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_SendGoal_service_typesupport_map = {
  2,
  "local_planning",
  &_PlanPath_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t PlanPath_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<local_planning::action::PlanPath_SendGoal_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<local_planning::action::PlanPath_SendGoal_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<local_planning::action::PlanPath_SendGoal_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<local_planning::action::PlanPath_SendGoal>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<local_planning::action::PlanPath_SendGoal>,
  &local_planning__action__PlanPath_SendGoal__get_type_hash,
  &local_planning__action__PlanPath_SendGoal__get_type_description,
  &local_planning__action__PlanPath_SendGoal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace local_planning

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<local_planning::action::PlanPath_SendGoal>()
{
  return &::local_planning::action::rosidl_typesupport_cpp::PlanPath_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, local_planning, action, PlanPath_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<local_planning::action::PlanPath_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "local_planning/action/detail/plan_path__functions.h"
// already included above
// #include "local_planning/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace local_planning
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_GetResult_Request_type_support_ids_t;

static const _PlanPath_GetResult_Request_type_support_ids_t _PlanPath_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanPath_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_GetResult_Request_type_support_symbol_names_t _PlanPath_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, local_planning, action, PlanPath_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, local_planning, action, PlanPath_GetResult_Request)),
  }
};

typedef struct _PlanPath_GetResult_Request_type_support_data_t
{
  void * data[2];
} _PlanPath_GetResult_Request_type_support_data_t;

static _PlanPath_GetResult_Request_type_support_data_t _PlanPath_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_GetResult_Request_message_typesupport_map = {
  2,
  "local_planning",
  &_PlanPath_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &local_planning__action__PlanPath_GetResult_Request__get_type_hash,
  &local_planning__action__PlanPath_GetResult_Request__get_type_description,
  &local_planning__action__PlanPath_GetResult_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace local_planning

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<local_planning::action::PlanPath_GetResult_Request>()
{
  return &::local_planning::action::rosidl_typesupport_cpp::PlanPath_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, local_planning, action, PlanPath_GetResult_Request)() {
  return get_message_type_support_handle<local_planning::action::PlanPath_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "local_planning/action/detail/plan_path__functions.h"
// already included above
// #include "local_planning/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace local_planning
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_GetResult_Response_type_support_ids_t;

static const _PlanPath_GetResult_Response_type_support_ids_t _PlanPath_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanPath_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_GetResult_Response_type_support_symbol_names_t _PlanPath_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, local_planning, action, PlanPath_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, local_planning, action, PlanPath_GetResult_Response)),
  }
};

typedef struct _PlanPath_GetResult_Response_type_support_data_t
{
  void * data[2];
} _PlanPath_GetResult_Response_type_support_data_t;

static _PlanPath_GetResult_Response_type_support_data_t _PlanPath_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_GetResult_Response_message_typesupport_map = {
  2,
  "local_planning",
  &_PlanPath_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &local_planning__action__PlanPath_GetResult_Response__get_type_hash,
  &local_planning__action__PlanPath_GetResult_Response__get_type_description,
  &local_planning__action__PlanPath_GetResult_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace local_planning

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<local_planning::action::PlanPath_GetResult_Response>()
{
  return &::local_planning::action::rosidl_typesupport_cpp::PlanPath_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, local_planning, action, PlanPath_GetResult_Response)() {
  return get_message_type_support_handle<local_planning::action::PlanPath_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "local_planning/action/detail/plan_path__functions.h"
// already included above
// #include "local_planning/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace local_planning
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_GetResult_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_GetResult_Event_type_support_ids_t;

static const _PlanPath_GetResult_Event_type_support_ids_t _PlanPath_GetResult_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanPath_GetResult_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_GetResult_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_GetResult_Event_type_support_symbol_names_t _PlanPath_GetResult_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, local_planning, action, PlanPath_GetResult_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, local_planning, action, PlanPath_GetResult_Event)),
  }
};

typedef struct _PlanPath_GetResult_Event_type_support_data_t
{
  void * data[2];
} _PlanPath_GetResult_Event_type_support_data_t;

static _PlanPath_GetResult_Event_type_support_data_t _PlanPath_GetResult_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_GetResult_Event_message_typesupport_map = {
  2,
  "local_planning",
  &_PlanPath_GetResult_Event_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_GetResult_Event_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_GetResult_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_GetResult_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_GetResult_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &local_planning__action__PlanPath_GetResult_Event__get_type_hash,
  &local_planning__action__PlanPath_GetResult_Event__get_type_description,
  &local_planning__action__PlanPath_GetResult_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace local_planning

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<local_planning::action::PlanPath_GetResult_Event>()
{
  return &::local_planning::action::rosidl_typesupport_cpp::PlanPath_GetResult_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, local_planning, action, PlanPath_GetResult_Event)() {
  return get_message_type_support_handle<local_planning::action::PlanPath_GetResult_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "local_planning/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace local_planning
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_GetResult_type_support_ids_t;

static const _PlanPath_GetResult_type_support_ids_t _PlanPath_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanPath_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_GetResult_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_GetResult_type_support_symbol_names_t _PlanPath_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, local_planning, action, PlanPath_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, local_planning, action, PlanPath_GetResult)),
  }
};

typedef struct _PlanPath_GetResult_type_support_data_t
{
  void * data[2];
} _PlanPath_GetResult_type_support_data_t;

static _PlanPath_GetResult_type_support_data_t _PlanPath_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_GetResult_service_typesupport_map = {
  2,
  "local_planning",
  &_PlanPath_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t PlanPath_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<local_planning::action::PlanPath_GetResult_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<local_planning::action::PlanPath_GetResult_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<local_planning::action::PlanPath_GetResult_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<local_planning::action::PlanPath_GetResult>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<local_planning::action::PlanPath_GetResult>,
  &local_planning__action__PlanPath_GetResult__get_type_hash,
  &local_planning__action__PlanPath_GetResult__get_type_description,
  &local_planning__action__PlanPath_GetResult__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace local_planning

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<local_planning::action::PlanPath_GetResult>()
{
  return &::local_planning::action::rosidl_typesupport_cpp::PlanPath_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, local_planning, action, PlanPath_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<local_planning::action::PlanPath_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "local_planning/action/detail/plan_path__functions.h"
// already included above
// #include "local_planning/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace local_planning
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _PlanPath_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlanPath_FeedbackMessage_type_support_ids_t;

static const _PlanPath_FeedbackMessage_type_support_ids_t _PlanPath_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PlanPath_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlanPath_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlanPath_FeedbackMessage_type_support_symbol_names_t _PlanPath_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, local_planning, action, PlanPath_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, local_planning, action, PlanPath_FeedbackMessage)),
  }
};

typedef struct _PlanPath_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _PlanPath_FeedbackMessage_type_support_data_t;

static _PlanPath_FeedbackMessage_type_support_data_t _PlanPath_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlanPath_FeedbackMessage_message_typesupport_map = {
  2,
  "local_planning",
  &_PlanPath_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_PlanPath_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_PlanPath_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlanPath_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlanPath_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &local_planning__action__PlanPath_FeedbackMessage__get_type_hash,
  &local_planning__action__PlanPath_FeedbackMessage__get_type_description,
  &local_planning__action__PlanPath_FeedbackMessage__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace local_planning

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<local_planning::action::PlanPath_FeedbackMessage>()
{
  return &::local_planning::action::rosidl_typesupport_cpp::PlanPath_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, local_planning, action, PlanPath_FeedbackMessage)() {
  return get_message_type_support_handle<local_planning::action::PlanPath_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "local_planning/action/detail/plan_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace local_planning
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t PlanPath_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL,
  &local_planning__action__PlanPath__get_type_hash,
  &local_planning__action__PlanPath__get_type_description,
  &local_planning__action__PlanPath__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace local_planning

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<local_planning::action::PlanPath>()
{
  using ::local_planning::action::rosidl_typesupport_cpp::PlanPath_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  PlanPath_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::local_planning::action::PlanPath::Impl::SendGoalService>();
  PlanPath_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::local_planning::action::PlanPath::Impl::GetResultService>();
  PlanPath_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::local_planning::action::PlanPath::Impl::CancelGoalService>();
  PlanPath_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::local_planning::action::PlanPath::Impl::FeedbackMessage>();
  PlanPath_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::local_planning::action::PlanPath::Impl::GoalStatusMessage>();
  return &PlanPath_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, local_planning, action, PlanPath)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<local_planning::action::PlanPath>();
}

#ifdef __cplusplus
}
#endif
