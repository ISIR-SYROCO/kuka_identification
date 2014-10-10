include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PC_EIGEN_INCLUDE_DIRS}
)

link_directories(
)

#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library 
# and installs in the directory lib/orocos/
#	
orocos_component(KukaIdentificationRTNET kukaIdentification-rtnetcomponent.cpp) # ...you may add multiple source files.
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/kuka_identification/ directory
#
orocos_install_headers(kukaIdentification-rtnetcomponent.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

