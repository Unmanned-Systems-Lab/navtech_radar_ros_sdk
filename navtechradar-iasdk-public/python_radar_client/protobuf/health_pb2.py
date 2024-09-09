# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: health.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from protobuf import healthinfo_pb2 as healthinfo__pb2
from protobuf import networkinfo_pb2 as networkinfo__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='health.proto',
  package='Navtech.Core.Configuration.Protobuf',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n\x0chealth.proto\x12#Navtech.Core.Configuration.Protobuf\x1a\x10healthinfo.proto\x1a\x11networkinfo.proto\"\xf5\x07\n\x06Health\x12G\n\x0e\x64ietemperature\x18\x01 \x01(\x0b\x32/.Navtech.Core.Configuration.Protobuf.HealthInfo\x12G\n\x0esoctemperature\x18\x02 \x01(\x0b\x32/.Navtech.Core.Configuration.Protobuf.HealthInfo\x12G\n\x0evcotemperature\x18\x03 \x01(\x0b\x32/.Navtech.Core.Configuration.Protobuf.HealthInfo\x12K\n\x12\x61mbienttemperature\x18\x04 \x01(\x0b\x32/.Navtech.Core.Configuration.Protobuf.HealthInfo\x12\x41\n\x08rotation\x18\x05 \x01(\x0b\x32/.Navtech.Core.Configuration.Protobuf.HealthInfo\x12\x43\n\npacketrate\x18\x06 \x01(\x0b\x32/.Navtech.Core.Configuration.Protobuf.HealthInfo\x12\x46\n\rrfhealthcheck\x18\x07 \x01(\x0b\x32/.Navtech.Core.Configuration.Protobuf.HealthInfo\x12\x14\n\x0ctransmitting\x18\x08 \x01(\x08\x12\x18\n\x10\x65xpectedrotation\x18\t \x01(\r\x12\x1a\n\x12\x65xpectedpacketrate\x18\n \x01(\r\x12\x12\n\nmacaddress\x18\x0b \x01(\t\x12\x19\n\x11\x65ncodererrorcount\x18\x0c \x01(\x05\x12\x14\n\x0csystemuptime\x18\r \x01(\x02\x12\x45\n\x0cmotorcurrent\x18\x0e \x01(\x0b\x32/.Navtech.Core.Configuration.Protobuf.HealthInfo\x12\x16\n\x0esoftwareuptime\x18\x0f \x01(\x04\x12\x13\n\x0btotaluptime\x18\x10 \x01(\x04\x12\x46\n\x0cnetworkstate\x18\x11 \x01(\x0b\x32\x30.Navtech.Core.Configuration.Protobuf.NetworkInfo\x12\x19\n\x11maxclientsallowed\x18\x12 \x01(\x05\x12\x11\n\tipclients\x18\x13 \x03(\t\x12\x1c\n\x14\x65xpectedrxpacketrate\x18\x14 \x01(\r\x12\x14\n\x0cuplinkerrors\x18\x15 \x01(\r\x12\x16\n\x0e\x64ownlinkerrors\x18\x16 \x01(\r\x12\x14\n\x0cuplinkmissed\x18\x17 \x01(\r\x12\x16\n\x0e\x64ownlinkmissed\x18\x18 \x01(\r')
  ,
  dependencies=[healthinfo__pb2.DESCRIPTOR,networkinfo__pb2.DESCRIPTOR,])




_HEALTH = _descriptor.Descriptor(
  name='Health',
  full_name='Navtech.Core.Configuration.Protobuf.Health',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='dietemperature', full_name='Navtech.Core.Configuration.Protobuf.Health.dietemperature', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='soctemperature', full_name='Navtech.Core.Configuration.Protobuf.Health.soctemperature', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='vcotemperature', full_name='Navtech.Core.Configuration.Protobuf.Health.vcotemperature', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ambienttemperature', full_name='Navtech.Core.Configuration.Protobuf.Health.ambienttemperature', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rotation', full_name='Navtech.Core.Configuration.Protobuf.Health.rotation', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='packetrate', full_name='Navtech.Core.Configuration.Protobuf.Health.packetrate', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rfhealthcheck', full_name='Navtech.Core.Configuration.Protobuf.Health.rfhealthcheck', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='transmitting', full_name='Navtech.Core.Configuration.Protobuf.Health.transmitting', index=7,
      number=8, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='expectedrotation', full_name='Navtech.Core.Configuration.Protobuf.Health.expectedrotation', index=8,
      number=9, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='expectedpacketrate', full_name='Navtech.Core.Configuration.Protobuf.Health.expectedpacketrate', index=9,
      number=10, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='macaddress', full_name='Navtech.Core.Configuration.Protobuf.Health.macaddress', index=10,
      number=11, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='encodererrorcount', full_name='Navtech.Core.Configuration.Protobuf.Health.encodererrorcount', index=11,
      number=12, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='systemuptime', full_name='Navtech.Core.Configuration.Protobuf.Health.systemuptime', index=12,
      number=13, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='motorcurrent', full_name='Navtech.Core.Configuration.Protobuf.Health.motorcurrent', index=13,
      number=14, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='softwareuptime', full_name='Navtech.Core.Configuration.Protobuf.Health.softwareuptime', index=14,
      number=15, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='totaluptime', full_name='Navtech.Core.Configuration.Protobuf.Health.totaluptime', index=15,
      number=16, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='networkstate', full_name='Navtech.Core.Configuration.Protobuf.Health.networkstate', index=16,
      number=17, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='maxclientsallowed', full_name='Navtech.Core.Configuration.Protobuf.Health.maxclientsallowed', index=17,
      number=18, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ipclients', full_name='Navtech.Core.Configuration.Protobuf.Health.ipclients', index=18,
      number=19, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='expectedrxpacketrate', full_name='Navtech.Core.Configuration.Protobuf.Health.expectedrxpacketrate', index=19,
      number=20, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='uplinkerrors', full_name='Navtech.Core.Configuration.Protobuf.Health.uplinkerrors', index=20,
      number=21, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='downlinkerrors', full_name='Navtech.Core.Configuration.Protobuf.Health.downlinkerrors', index=21,
      number=22, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='uplinkmissed', full_name='Navtech.Core.Configuration.Protobuf.Health.uplinkmissed', index=22,
      number=23, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='downlinkmissed', full_name='Navtech.Core.Configuration.Protobuf.Health.downlinkmissed', index=23,
      number=24, type=13, cpp_type=3, label=1,
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
  serialized_start=91,
  serialized_end=1104,
)

_HEALTH.fields_by_name['dietemperature'].message_type = healthinfo__pb2._HEALTHINFO
_HEALTH.fields_by_name['soctemperature'].message_type = healthinfo__pb2._HEALTHINFO
_HEALTH.fields_by_name['vcotemperature'].message_type = healthinfo__pb2._HEALTHINFO
_HEALTH.fields_by_name['ambienttemperature'].message_type = healthinfo__pb2._HEALTHINFO
_HEALTH.fields_by_name['rotation'].message_type = healthinfo__pb2._HEALTHINFO
_HEALTH.fields_by_name['packetrate'].message_type = healthinfo__pb2._HEALTHINFO
_HEALTH.fields_by_name['rfhealthcheck'].message_type = healthinfo__pb2._HEALTHINFO
_HEALTH.fields_by_name['motorcurrent'].message_type = healthinfo__pb2._HEALTHINFO
_HEALTH.fields_by_name['networkstate'].message_type = networkinfo__pb2._NETWORKINFO
DESCRIPTOR.message_types_by_name['Health'] = _HEALTH
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Health = _reflection.GeneratedProtocolMessageType('Health', (_message.Message,), dict(
  DESCRIPTOR = _HEALTH,
  __module__ = 'health_pb2'
  # @@protoc_insertion_point(class_scope:Navtech.Core.Configuration.Protobuf.Health)
  ))
_sym_db.RegisterMessage(Health)


# @@protoc_insertion_point(module_scope)
