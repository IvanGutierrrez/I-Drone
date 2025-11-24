algo_sources = files(
    'src/main.cpp',
    'src/Algorithm_Manager.cpp',
    'src/Algorithm_Manager_Interface.cpp',
    'src/Algorithm_Recorder.cpp',
    'src/Communication_Manager.cpp',
    'src/Path_Cal.cpp',
    'src/Signal_Cal.cpp'
)

ortools_dep = dependency('ortools', required: true)

executable(
    'Algorithm',
    sources : algo_sources,
    include_directories : [
        include_directories('src'),
        include_directories('../libs'),
    ],
    link_with : idronelib,
    dependencies : [ortools_dep],
    install : true
)
