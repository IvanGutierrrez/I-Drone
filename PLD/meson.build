pld_sources = files(
    'src/main.cpp',
    'src/Communication_Manager.cpp',
    'src/State_Machine.cpp',
    'src/states/State.cpp',
    'src/states/Off_State.cpp',
    'src/states/Planner_State.cpp',
    'src/states/Drone_Mission_State.cpp'
)


executable('PLD',
  sources : pld_sources,
  include_directories : [
    include_directories('../libs'),
  ],
  link_with : idronelib,
  install : true,
)
