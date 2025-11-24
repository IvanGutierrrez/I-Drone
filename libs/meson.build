libs_sources = files(
  'common_libs/Recorder.cpp',
  'common_libs/Logger.cpp',
  'common_libs/Server.cpp',
  'common_libs/Enc_Dec_Algo.cpp',
  'common_libs/Enc_Dec_PLD.cpp',
  'generated_proto/messages_algo.pb.cc',
  'generated_proto/messages_pld.pb.cc',
  'structs/Structs_Algo.h',
)

libs_include = include_directories(
  '.',
  'generated_proto'
)

# Dependencias
protobuf_dep = dependency('protobuf', required: true)
boost_dep = dependency('boost', modules: ['system', 'thread'], required: true)

# Crear la biblioteca estática
idronelib = static_library('idronelib',
  libs_sources,
  include_directories: libs_include,
  dependencies: [protobuf_dep, boost_dep]
)
