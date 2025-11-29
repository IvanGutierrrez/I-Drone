drone_sources = files(
    'src/main.cpp',
    'src/Controller.cpp',
    'src/Communication_Manager.cpp',
    'src/Drone_Recorder.cpp'
)

executable(
  'Drone',
  sources : drone_sources,
  include_directories : [
    include_directories('src'),
    include_directories('../libs'),
  ],
  link_with : idronelib,
  install : true,
)
