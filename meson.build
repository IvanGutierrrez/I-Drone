project('I-Drone', 'cpp',
  version : '1.0.0',
  default_options : [
    'buildtype=release',
    'optimization=3',
    'debug=false',
    'cpp_std=c++20',
    'warning_level=3',
    'werror=false',
    'b_lto=true',
    'b_ndebug=true',
  ]
)

# Incluir librerías comunes
subdir('libs')

# Subproyectos (cada módulo)
subdir('PLD')
subdir('RSC')
subdir('Drone')
subdir('Algorithm')
