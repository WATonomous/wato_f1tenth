// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sample_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#include "sample_msgs/msg/detail/metadata__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
const rosidl_type_hash_t *
sample_msgs__msg__Metadata__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0c, 0xfa, 0x03, 0x78, 0x36, 0x96, 0xd2, 0x93,
      0xfd, 0x84, 0x6f, 0x3a, 0xc2, 0xa8, 0x86, 0x19,
      0x93, 0xef, 0x4c, 0x29, 0xe6, 0x48, 0xf6, 0xe6,
      0xd1, 0x9e, 0x2a, 0xd0, 0xb8, 0xd9, 0xf9, 0xd4,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char sample_msgs__msg__Metadata__TYPE_NAME[] = "sample_msgs/msg/Metadata";

// Define type names, field names, and default values
static char sample_msgs__msg__Metadata__FIELD_NAME__version[] = "version";
static char sample_msgs__msg__Metadata__FIELD_NAME__compression_method[] = "compression_method";
static char sample_msgs__msg__Metadata__FIELD_NAME__creation_date[] = "creation_date";

static rosidl_runtime_c__type_description__Field sample_msgs__msg__Metadata__FIELDS[] = {
  {
    {sample_msgs__msg__Metadata__FIELD_NAME__version, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_msgs__msg__Metadata__FIELD_NAME__compression_method, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_msgs__msg__Metadata__FIELD_NAME__creation_date, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sample_msgs__msg__Metadata__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sample_msgs__msg__Metadata__TYPE_NAME, 24, 24},
      {sample_msgs__msg__Metadata__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int8 DEFAULT = 0\n"
  "int8 DICTIONARY = 1\n"
  "int8 RUN_LENGTH = 2\n"
  "\n"
  "int8 version\n"
  "int8 compression_method\n"
  "int16 creation_date";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sample_msgs__msg__Metadata__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sample_msgs__msg__Metadata__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 114, 114},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sample_msgs__msg__Metadata__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sample_msgs__msg__Metadata__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
