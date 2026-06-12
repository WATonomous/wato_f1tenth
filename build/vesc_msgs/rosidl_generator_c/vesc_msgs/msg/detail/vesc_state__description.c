// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from vesc_msgs:msg/VescState.idl
// generated code does not contain a copyright notice

#include "vesc_msgs/msg/detail/vesc_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
const rosidl_type_hash_t *
vesc_msgs__msg__VescState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa3, 0x27, 0x63, 0x8b, 0x63, 0x5a, 0xed, 0xa6,
      0x40, 0x96, 0xcc, 0xa0, 0xc6, 0x4c, 0x8d, 0x49,
      0x70, 0x21, 0x51, 0xa5, 0xdd, 0xf8, 0x5f, 0xb1,
      0x78, 0x7d, 0xe0, 0x14, 0x48, 0x8f, 0x54, 0xb3,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char vesc_msgs__msg__VescState__TYPE_NAME[] = "vesc_msgs/msg/VescState";

// Define type names, field names, and default values
static char vesc_msgs__msg__VescState__FIELD_NAME__temp_fet[] = "temp_fet";
static char vesc_msgs__msg__VescState__FIELD_NAME__temp_motor[] = "temp_motor";
static char vesc_msgs__msg__VescState__FIELD_NAME__current_motor[] = "current_motor";
static char vesc_msgs__msg__VescState__FIELD_NAME__current_input[] = "current_input";
static char vesc_msgs__msg__VescState__FIELD_NAME__avg_id[] = "avg_id";
static char vesc_msgs__msg__VescState__FIELD_NAME__avg_iq[] = "avg_iq";
static char vesc_msgs__msg__VescState__FIELD_NAME__duty_cycle[] = "duty_cycle";
static char vesc_msgs__msg__VescState__FIELD_NAME__speed[] = "speed";
static char vesc_msgs__msg__VescState__FIELD_NAME__voltage_input[] = "voltage_input";
static char vesc_msgs__msg__VescState__FIELD_NAME__charge_drawn[] = "charge_drawn";
static char vesc_msgs__msg__VescState__FIELD_NAME__charge_regen[] = "charge_regen";
static char vesc_msgs__msg__VescState__FIELD_NAME__energy_drawn[] = "energy_drawn";
static char vesc_msgs__msg__VescState__FIELD_NAME__energy_regen[] = "energy_regen";
static char vesc_msgs__msg__VescState__FIELD_NAME__displacement[] = "displacement";
static char vesc_msgs__msg__VescState__FIELD_NAME__distance_traveled[] = "distance_traveled";
static char vesc_msgs__msg__VescState__FIELD_NAME__fault_code[] = "fault_code";
static char vesc_msgs__msg__VescState__FIELD_NAME__pid_pos_now[] = "pid_pos_now";
static char vesc_msgs__msg__VescState__FIELD_NAME__controller_id[] = "controller_id";
static char vesc_msgs__msg__VescState__FIELD_NAME__ntc_temp_mos1[] = "ntc_temp_mos1";
static char vesc_msgs__msg__VescState__FIELD_NAME__ntc_temp_mos2[] = "ntc_temp_mos2";
static char vesc_msgs__msg__VescState__FIELD_NAME__ntc_temp_mos3[] = "ntc_temp_mos3";
static char vesc_msgs__msg__VescState__FIELD_NAME__avg_vd[] = "avg_vd";
static char vesc_msgs__msg__VescState__FIELD_NAME__avg_vq[] = "avg_vq";

static rosidl_runtime_c__type_description__Field vesc_msgs__msg__VescState__FIELDS[] = {
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__temp_fet, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__temp_motor, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__current_motor, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__current_input, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__avg_id, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__avg_iq, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__duty_cycle, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__voltage_input, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__charge_drawn, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__charge_regen, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__energy_drawn, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__energy_regen, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__displacement, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__distance_traveled, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__fault_code, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__pid_pos_now, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__controller_id, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__ntc_temp_mos1, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__ntc_temp_mos2, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__ntc_temp_mos3, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__avg_vd, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__FIELD_NAME__avg_vq, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
vesc_msgs__msg__VescState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {vesc_msgs__msg__VescState__TYPE_NAME, 23, 23},
      {vesc_msgs__msg__VescState__FIELDS, 23, 23},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Vedder VESC open source motor controller state (telemetry)\n"
  "\n"
  "# fault codes\n"
  "int32 FAULT_CODE_NONE=0\n"
  "int32 FAULT_CODE_OVER_VOLTAGE=1\n"
  "int32 FAULT_CODE_UNDER_VOLTAGE=2\n"
  "int32 FAULT_CODE_DRV8302=3\n"
  "int32 FAULT_CODE_ABS_OVER_CURRENT=4\n"
  "int32 FAULT_CODE_OVER_TEMP_FET=5\n"
  "int32 FAULT_CODE_OVER_TEMP_MOTOR=6\n"
  "\n"
  "# follow the bledc firwmare: commands.c\n"
  "float64 temp_fet             # fet temperature \n"
  "float64 temp_motor           # motor temperature\n"
  "float64 current_motor        # motor current (ampere) avg_motor_current\n"
  "float64 current_input        # input current (ampere) avg_input_current          \n"
  "float64 avg_id\n"
  "float64 avg_iq\n"
  "float64 duty_cycle           # duty cycle (0 to 1) duty_cycle_now\n"
  "float64 speed                # motor electrical speed (revolutions per minute) rpm\n"
  "\n"
  "float64 voltage_input        # input voltage (volt)\n"
  "float64 charge_drawn         # electric charge drawn from input (ampere-hours)\n"
  "float64 charge_regen         # electric charge regenerated to input (ampere-hour) amp_hours_charged    \n"
  "float64 energy_drawn         # energy drawn from input (watt-hour)\n"
  "float64 energy_regen         # energy regenerated to input (watt_hours_charged)\n"
  "int32   displacement         # net tachometer (counts) tachometer\n"
  "int32   distance_traveled    # total tachnometer (counts) tachometer_abs\n"
  "int32   fault_code\n"
  "\n"
  "float64 pid_pos_now\n"
  "int32 controller_id\n"
  "\n"
  "\n"
  "float64 ntc_temp_mos1\n"
  "float64 ntc_temp_mos2\n"
  "float64 ntc_temp_mos3\n"
  "float64 avg_vd\n"
  "float64 avg_vq\n"
  "\n"
  "#float64 temperature_pcb      # temperature of printed circuit board (degrees Celsius)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
vesc_msgs__msg__VescState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {vesc_msgs__msg__VescState__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1535, 1535},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
vesc_msgs__msg__VescState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *vesc_msgs__msg__VescState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
