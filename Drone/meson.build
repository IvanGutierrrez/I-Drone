drone_sources = files(
    'src/main.cpp',
    'src/Controller.cpp',
    'src/Communication_Manager.cpp',
    'src/PX4_Drone_Recorder.cpp',
    'src/Engine.cpp',
    'src/PX4_Wrapper.cpp',
    'src/Multi_Drone_Manager.cpp',
    'src/Gazebo_Cleaner.cpp'
)

mavsdk_dep = dependency('MAVSDK', required: false)

if not mavsdk_dep.found()
  mavsdk_include = include_directories('/usr/local/include')
  mavsdk_lib = meson.get_compiler('cpp').find_library('mavsdk', dirs: ['/usr/local/lib'], required: true)
  mavsdk_dep = declare_dependency(
    include_directories: mavsdk_include,
    dependencies: mavsdk_lib
  )
endif

executable(
  'Drone',
  sources : drone_sources,
  include_directories : [
    include_directories('src'),
    include_directories('../libs'),
  ],
  dependencies : [mavsdk_dep],
  link_with : idronelib,
  install : true,
)
