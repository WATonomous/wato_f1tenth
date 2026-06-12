// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sample_msgs:msg/Filtered.idl
// generated code does not contain a copyright notice

#include "sample_msgs/msg/detail/filtered__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
const rosidl_type_hash_t *
sample_msgs__msg__Filtered__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc3, 0x33, 0x19, 0x47, 0x80, 0x41, 0x2e, 0xcb,
      0x19, 0x29, 0xae, 0x39, 0xb0, 0x7b, 0xe9, 0x98,
      0x40, 0x83, 0xc6, 0x88, 0xbc, 0xc4, 0x63, 0xa4,
      0x58, 0x96, 0x71, 0x3d, 0xa0, 0x08, 0xf9, 0xa0,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "sample_msgs/msg/detail/metadata__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t sample_msgs__msg__Metadata__EXPECTED_HASH = {1, {
    0x0c, 0xfa, 0x03, 0x78, 0x36, 0x96, 0xd2, 0x93,
    0xfd, 0x84, 0x6f, 0x3a, 0xc2, 0xa8, 0x86, 0x19,
    0x93, 0xef, 0x4c, 0x29, 0xe6, 0x48, 0xf6, 0xe6,
    0xd1, 0x9e, 0x2a, 0xd0, 0xb8, 0xd9, 0xf9, 0xd4,
  }};
#endif

static char sample_msgs__msg__Filtered__TYPE_NAME[] = "sample_msgs/msg/Filtered";
static char sample_msgs__msg__Metadata__TYPE_NAME[] = "sample_msgs/msg/Metadata";

// Define type names, field names, and default values
static char sample_msgs__msg__Filtered__FIELD_NAME__pos_x[] = "pos_x";
static char sample_msgs__msg__Filtered__FIELD_NAME__pos_y[] = "pos_y";
static char sample_msgs__msg__Filtered__FIELD_NAME__pos_z[] = "pos_z";
static char sample_msgs__msg__Filtered__FIELD_NAME__metadata[] = "metadata";
static char sample_msgs__msg__Filtered__FIELD_NAME__timestamp[] = "timestamp";

static rosidl_runtime_c__type_description__Field sample_msgs__msg__Filtered__FIELDS[] = {
  {
    {sample_msgs__msg__Filtered__FIELD_NAME__pos_x, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_msgs__msg__Filtered__FIELD_NAME__pos_y, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_msgs__msg__Filtered__FIELD_NAME__pos_z, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_msgs__msg__Filtered__FIELD_NAME__metadata, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {sample_msgs__msg__Metadata__TYPE_NAME, 24, 24},
    },
    {NULL, 0, 0},
  },
  {
    {sample_msgs__msg__Filtered__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription sample_msgs__msg__Filtered__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {sample_msgs__msg__Metadata__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sample_msgs__msg__Filtered__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sample_msgs__msg__Filtered__TYPE_NAME, 24, 24},
      {sample_msgs__msg__Filtered__FIELDS, 5, 5},
    },
    {sample_msgs__msg__Filtered__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&sample_msgs__msg__Metadata__EXPECTED_HASH, sample_msgs__msg__Metadata__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = sample_msgs__msg__Metadata__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32  pos_x\n"
  "float32  pos_y\n"
  "float32  pos_z\n"
  "Metadata metadata\n"
  "int64    timestamp";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sample_msgs__msg__Filtered__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sample_msgs__msg__Filtered__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 81, 81},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sample_msgs__msg__Filtered__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sample_msgs__msg__Filtered__get_individual_type_description_source(NULL),
    sources[1] = *sample_msgs__msg__Metadata__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
