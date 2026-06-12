// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sample_msgs:msg/FilteredArray.idl
// generated code does not contain a copyright notice

#include "sample_msgs/msg/detail/filtered_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sample_msgs
const rosidl_type_hash_t *
sample_msgs__msg__FilteredArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdb, 0xae, 0xab, 0x32, 0xe2, 0x1f, 0xc6, 0x38,
      0x2a, 0x2e, 0x84, 0x57, 0x9f, 0x2d, 0x82, 0x3f,
      0xef, 0xd4, 0x5e, 0xe3, 0x59, 0xa0, 0x32, 0x7c,
      0xa1, 0x6a, 0x86, 0x86, 0x6d, 0xf5, 0x8a, 0xe6,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "sample_msgs/msg/detail/metadata__functions.h"
#include "sample_msgs/msg/detail/filtered__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t sample_msgs__msg__Filtered__EXPECTED_HASH = {1, {
    0xc3, 0x33, 0x19, 0x47, 0x80, 0x41, 0x2e, 0xcb,
    0x19, 0x29, 0xae, 0x39, 0xb0, 0x7b, 0xe9, 0x98,
    0x40, 0x83, 0xc6, 0x88, 0xbc, 0xc4, 0x63, 0xa4,
    0x58, 0x96, 0x71, 0x3d, 0xa0, 0x08, 0xf9, 0xa0,
  }};
static const rosidl_type_hash_t sample_msgs__msg__Metadata__EXPECTED_HASH = {1, {
    0x0c, 0xfa, 0x03, 0x78, 0x36, 0x96, 0xd2, 0x93,
    0xfd, 0x84, 0x6f, 0x3a, 0xc2, 0xa8, 0x86, 0x19,
    0x93, 0xef, 0x4c, 0x29, 0xe6, 0x48, 0xf6, 0xe6,
    0xd1, 0x9e, 0x2a, 0xd0, 0xb8, 0xd9, 0xf9, 0xd4,
  }};
#endif

static char sample_msgs__msg__FilteredArray__TYPE_NAME[] = "sample_msgs/msg/FilteredArray";
static char sample_msgs__msg__Filtered__TYPE_NAME[] = "sample_msgs/msg/Filtered";
static char sample_msgs__msg__Metadata__TYPE_NAME[] = "sample_msgs/msg/Metadata";

// Define type names, field names, and default values
static char sample_msgs__msg__FilteredArray__FIELD_NAME__packets[] = "packets";

static rosidl_runtime_c__type_description__Field sample_msgs__msg__FilteredArray__FIELDS[] = {
  {
    {sample_msgs__msg__FilteredArray__FIELD_NAME__packets, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {sample_msgs__msg__Filtered__TYPE_NAME, 24, 24},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription sample_msgs__msg__FilteredArray__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {sample_msgs__msg__Filtered__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
  {
    {sample_msgs__msg__Metadata__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sample_msgs__msg__FilteredArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sample_msgs__msg__FilteredArray__TYPE_NAME, 29, 29},
      {sample_msgs__msg__FilteredArray__FIELDS, 1, 1},
    },
    {sample_msgs__msg__FilteredArray__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&sample_msgs__msg__Filtered__EXPECTED_HASH, sample_msgs__msg__Filtered__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = sample_msgs__msg__Filtered__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sample_msgs__msg__Metadata__EXPECTED_HASH, sample_msgs__msg__Metadata__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = sample_msgs__msg__Metadata__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "Filtered[] packets";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sample_msgs__msg__FilteredArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sample_msgs__msg__FilteredArray__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 18, 18},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sample_msgs__msg__FilteredArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sample_msgs__msg__FilteredArray__get_individual_type_description_source(NULL),
    sources[1] = *sample_msgs__msg__Filtered__get_individual_type_description_source(NULL);
    sources[2] = *sample_msgs__msg__Metadata__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
