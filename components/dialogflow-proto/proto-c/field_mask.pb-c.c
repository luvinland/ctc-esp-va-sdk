/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: google/protobuf/field_mask.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif

#include "field_mask.pb-c.h"
void   google__protobuf__field_mask__init
                     (Google__Protobuf__FieldMask         *message)
{
  static const Google__Protobuf__FieldMask init_value = GOOGLE__PROTOBUF__FIELD_MASK__INIT;
  *message = init_value;
}
size_t google__protobuf__field_mask__get_packed_size
                     (const Google__Protobuf__FieldMask *message)
{
  assert(message->base.descriptor == &google__protobuf__field_mask__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t google__protobuf__field_mask__pack
                     (const Google__Protobuf__FieldMask *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &google__protobuf__field_mask__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t google__protobuf__field_mask__pack_to_buffer
                     (const Google__Protobuf__FieldMask *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &google__protobuf__field_mask__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
Google__Protobuf__FieldMask *
       google__protobuf__field_mask__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Google__Protobuf__FieldMask *)
     protobuf_c_message_unpack (&google__protobuf__field_mask__descriptor,
                                allocator, len, data);
}
void   google__protobuf__field_mask__free_unpacked
                     (Google__Protobuf__FieldMask *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &google__protobuf__field_mask__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
static const ProtobufCFieldDescriptor google__protobuf__field_mask__field_descriptors[1] =
{
  {
    "paths",
    1,
    PROTOBUF_C_LABEL_REPEATED,
    PROTOBUF_C_TYPE_STRING,
    offsetof(Google__Protobuf__FieldMask, n_paths),
    offsetof(Google__Protobuf__FieldMask, paths),
    NULL,
    &protobuf_c_empty_string,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned google__protobuf__field_mask__field_indices_by_name[] = {
  0,   /* field[0] = paths */
};
static const ProtobufCIntRange google__protobuf__field_mask__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 1 }
};
const ProtobufCMessageDescriptor google__protobuf__field_mask__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "google.protobuf.FieldMask",
  "FieldMask",
  "Google__Protobuf__FieldMask",
  "google.protobuf",
  sizeof(Google__Protobuf__FieldMask),
  1,
  google__protobuf__field_mask__field_descriptors,
  google__protobuf__field_mask__field_indices_by_name,
  1,  google__protobuf__field_mask__number_ranges,
  (ProtobufCMessageInit) google__protobuf__field_mask__init,
  NULL,NULL,NULL    /* reserved[123] */
};
