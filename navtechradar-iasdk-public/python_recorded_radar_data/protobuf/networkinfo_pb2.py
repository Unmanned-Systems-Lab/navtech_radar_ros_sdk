# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: networkinfo.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='networkinfo.proto',
  package='Navtech.Core.Configuration.Protobuf',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n\x11networkinfo.proto\x12#Navtech.Core.Configuration.Protobuf\"\xa2\x01\n\x0bNetworkInfo\x12@\n\x05state\x18\x01 \x01(\x0e\x32\x31.Navtech.Core.Configuration.Protobuf.NetworkState\x12\x42\n\x06\x64uplex\x18\x02 \x01(\x0e\x32\x32.Navtech.Core.Configuration.Protobuf.NetworkDuplex\x12\r\n\x05speed\x18\x03 \x01(\r*B\n\x0cNetworkState\x12\x0c\n\x08NET_DOWN\x10\x00\x12\n\n\x06NET_UP\x10\x01\x12\x18\n\x0bNET_UNKNOWN\x10\xff\xff\xff\xff\xff\xff\xff\xff\xff\x01*+\n\rNetworkDuplex\x12\x0c\n\x08NET_HALF\x10\x00\x12\x0c\n\x08NET_FULL\x10\x01')
)

_NETWORKSTATE = _descriptor.EnumDescriptor(
  name='NetworkState',
  full_name='Navtech.Core.Configuration.Protobuf.NetworkState',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NET_DOWN', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NET_UP', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NET_UNKNOWN', index=2, number=-1,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=223,
  serialized_end=289,
)
_sym_db.RegisterEnumDescriptor(_NETWORKSTATE)

NetworkState = enum_type_wrapper.EnumTypeWrapper(_NETWORKSTATE)
_NETWORKDUPLEX = _descriptor.EnumDescriptor(
  name='NetworkDuplex',
  full_name='Navtech.Core.Configuration.Protobuf.NetworkDuplex',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NET_HALF', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NET_FULL', index=1, number=1,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=291,
  serialized_end=334,
)
_sym_db.RegisterEnumDescriptor(_NETWORKDUPLEX)

NetworkDuplex = enum_type_wrapper.EnumTypeWrapper(_NETWORKDUPLEX)
NET_DOWN = 0
NET_UP = 1
NET_UNKNOWN = -1
NET_HALF = 0
NET_FULL = 1



_NETWORKINFO = _descriptor.Descriptor(
  name='NetworkInfo',
  full_name='Navtech.Core.Configuration.Protobuf.NetworkInfo',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='state', full_name='Navtech.Core.Configuration.Protobuf.NetworkInfo.state', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='duplex', full_name='Navtech.Core.Configuration.Protobuf.NetworkInfo.duplex', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='speed', full_name='Navtech.Core.Configuration.Protobuf.NetworkInfo.speed', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=59,
  serialized_end=221,
)

_NETWORKINFO.fields_by_name['state'].enum_type = _NETWORKSTATE
_NETWORKINFO.fields_by_name['duplex'].enum_type = _NETWORKDUPLEX
DESCRIPTOR.message_types_by_name['NetworkInfo'] = _NETWORKINFO
DESCRIPTOR.enum_types_by_name['NetworkState'] = _NETWORKSTATE
DESCRIPTOR.enum_types_by_name['NetworkDuplex'] = _NETWORKDUPLEX
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

NetworkInfo = _reflection.GeneratedProtocolMessageType('NetworkInfo', (_message.Message,), dict(
  DESCRIPTOR = _NETWORKINFO,
  __module__ = 'networkinfo_pb2'
  # @@protoc_insertion_point(class_scope:Navtech.Core.Configuration.Protobuf.NetworkInfo)
  ))
_sym_db.RegisterMessage(NetworkInfo)


# @@protoc_insertion_point(module_scope)
