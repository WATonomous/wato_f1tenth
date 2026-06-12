// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from vesc_msgs:msg/VescStateStamped.idl
// generated code does not contain a copyright notice

#include "vesc_msgs/msg/detail/vesc_state_stamped__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
const rosidl_type_hash_t *
vesc_msgs__msg__VescStateStamped__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9a, 0x95, 0x43, 0xe6, 0x72, 0x17, 0xb2, 0xe8,
      0x5a, 0x8f, 0xbe, 0x0d, 0x6b, 0x14, 0x0e, 0xaf,
      0xa0, 0x20, 0xd9, 0xf4, 0xac, 0x08, 0xe1, 0x1b,
      0xb5, 0x4c, 0xc4, 0xba, 0x6d, 0x4d, 0x44, 0x4c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "vesc_msgs/msg/detail/vesc_state__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
static const rosidl_type_hash_t vesc_msgs__msg__VescState__EXPECTED_HASH = {1, {
    0xa3, 0x27, 0x63, 0x8b, 0x63, 0x5a, 0xed, 0xa6,
    0x40, 0x96, 0xcc, 0xa0, 0xc6, 0x4c, 0x8d, 0x49,
    0x70, 0x21, 0x51, 0xa5, 0xdd, 0xf8, 0x5f, 0xb1,
    0x78, 0x7d, 0xe0, 0x14, 0x48, 0x8f, 0x54, 0xb3,
  }};
#endif

static char vesc_msgs__msg__VescStateStamped__TYPE_NAME[] = "vesc_msgs/msg/VescStateStamped";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";
static char vesc_msgs__msg__VescState__TYPE_NAME[] = "vesc_msgs/msg/VescState";

// Define type names, field names, and default values
static char vesc_msgs__msg__VescStateStamped__FIELD_NAME__header[] = "header";
static char vesc_msgs__msg__VescStateStamped__FIELD_NAME__state[] = "state";

static rosidl_runtime_c__type_description__Field vesc_msgs__msg__VescStateStamped__FIELDS[] = {
  {
    {vesc_msgs__msg__VescStateStamped__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescStateStamped__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {vesc_msgs__msg__VescState__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription vesc_msgs__msg__VescStateStamped__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
  {
    {vesc_msgs__msg__VescState__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
vesc_msgs__msg__VescStateStamped__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {vesc_msgs__msg__VescStateStamped__TYPE_NAME, 30, 30},
      {vesc_msgs__msg__VescStateStamped__FIELDS, 2, 2},
    },
    {vesc_msgs__msg__VescStateStamped__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&vesc_msgs__msg__VescState__EXPECTED_HASH, vesc_msgs__msg__VescState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = vesc_msgs__msg__VescState__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Timestamped VESC open source motor controller state (telemetry)\n"
  "\n"
  "std_msgs/Header  header\n"
  "VescState state";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
vesc_msgs__msg__VescStateStamped__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {vesc_msgs__msg__VescStateStamped__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 107, 107},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
vesc_msgs__msg__VescStateStamped__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *vesc_msgs__msg__VescStateStamped__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    sources[3] = *vesc_msgs__msg__VescState__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
