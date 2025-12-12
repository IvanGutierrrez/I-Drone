planner_sources = files(
    'src/main.cpp',
    'src/Planner_Manager.cpp',
    'src/Planner_Manager_Interface.cpp',
    'src/Planner_Recorder.cpp',
    'src/Communication_Manager.cpp',
    'src/Path_Cal.cpp',
    'src/Signal_Cal.cpp'
)

ortools_dep = dependency('ortools', required: true)

executable(
    'Planner',
    sources : planner_sources,
    include_directories : [
        include_directories('src'),
        include_directories('../libs'),
    ],
    link_with : idronelib,
    dependencies : [ortools_dep],
    install : true
)
