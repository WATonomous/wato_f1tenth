// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sample_msgs:msg/Unfiltered.idl
// generated code does not contain a copyright notice

#include "sample_msgs/msg/detail/unfiltered__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
const rosidl_type_hash_t *
sample_msgs__msg__Unfiltered__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe3, 0x94, 0x0c, 0x0c, 0x54, 0x6e, 0x27, 0xfb,
      0xfd, 0x63, 0xf1, 0x0b, 0x87, 0x6d, 0xc7, 0xad,
      0xab, 0x76, 0x45, 0x2b, 0xf4, 0x04, 0xb4, 0x2b,
      0x3d, 0xc3, 0x23, 0x15, 0x5a, 0x03, 0x17, 0xb2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char sample_msgs__msg__Unfiltered__TYPE_NAME[] = "sample_msgs/msg/Unfiltered";

// Define type names, field names, and default values
static char sample_msgs__msg__Unfiltered__FIELD_NAME__data[] = "data";
static char sample_msgs__msg__Unfiltered__FIELD_NAME__timestamp[] = "timestamp";
static char sample_msgs__msg__Unfiltered__FIELD_NAME__valid[] = "valid";

static rosidl_runtime_c__type_description__Field sample_msgs__msg__Unfiltered__FIELDS[] = {
  {
    {sample_msgs__msg__Unfiltered__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_msgs__msg__Unfiltered__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_msgs__msg__Unfiltered__FIELD_NAME__valid, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sample_msgs__msg__Unfiltered__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sample_msgs__msg__Unfiltered__TYPE_NAME, 26, 26},
      {sample_msgs__msg__Unfiltered__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string data\n"
  "int64  timestamp\n"
  "bool   valid";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sample_msgs__msg__Unfiltered__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sample_msgs__msg__Unfiltered__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 41, 41},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sample_msgs__msg__Unfiltered__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sample_msgs__msg__Unfiltered__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
