libs_sources = files(
  'common_libs/recorder.cpp',
  'common_libs/logger.cpp',
)

libs_include = include_directories('.')

# Creamos una biblioteca estática (libidronelib.a)
idronelib = static_library('idronelib', libs_sources, include_directories : libs_include)
