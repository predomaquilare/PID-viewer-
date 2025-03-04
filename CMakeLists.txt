cmake_minimum_required(VERSION 3.8)
project(pid_viewer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
)

# Incluir diretórios de cabeçalho
include_directories(
  include
)

# Nomes do executável e da biblioteca
set(executable_name pid_viewer)
set(library_name ${executable_name}_core)

# Executável
add_executable(${executable_name}
  src/pid_main.cpp
)

# Biblioteca compartilhada (para o nó PID)
add_library(${library_name} SHARED
  src/pid_node.cpp
)

# Incluir diretórios de cabeçalho para a biblioteca
target_include_directories(${library_name} PRIVATE include)
target_link_libraries(${executable_name} ${library_name})
# Dependências do executável
ament_target_dependencies(${executable_name}
  ${dependencies}
)

# Dependências da biblioteca
ament_target_dependencies(${library_name}
  ${dependencies}
)

# Registrar o nó como um componente dentro da biblioteca
rclcpp_components_register_nodes(${library_name} "pid::pid")

# Instalação
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Testes
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Pular verificação de copyright (comente se adicionar copyright)
  set(ament_cmake_copyright_FOUND TRUE)
  # Pular cpplint (comente se estiver em um repositório git e com copyright)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
