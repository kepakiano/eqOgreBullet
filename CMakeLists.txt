#
# Copyright (C) 2013-2014
# Sebastian Schmitz <sschmitz@informatik.uni-siegen.de>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#

#
# CMake init stuff
#
cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
cmake_policy(SET CMP0015 NEW)

add_definitions(-std=c++0x -g)

#
# Project name and languages
#
project(eqOgreBullet C CXX)

#
# Main application: eqOgreBullet
#
find_package(GLUT REQUIRED)
find_package(Equalizer REQUIRED)
find_package(Boost REQUIRED system)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OGRE REQUIRED OGRE)
pkg_check_modules(BULLET REQUIRED bullet)

include_directories(
    ${EQUALIZER_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIR}
    ${GLEWMX_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${OGRE_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
)

link_directories(
    ${EQUALIZER_LIBRARY_DIRS}
    ${GLUT_LIBRARY_DIRS}
    ${GLEWMX_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS}
    ${OGRE_LIBRARY_DIRS}
    ${BULLET_LIBRARY_DIRS}
)

# Definition of CO_IGNORE_BYTESWAP to solve an undefined reference, see http://software.1713.n2.nabble.com/Framedata-unresolved-symbol-lunchbox-byteswap-td7583159.html
add_definitions(-DPACKAGE_NAME="eqOgreBullet" -DPACKAGE_VERSION="1.0"
  -DCO_IGNORE_BYTESWAP)

add_executable(eqOgreBullet
  main/eqogrebullet.cpp
  main/eqogrebullet.h

  equalizer/src/channel.cpp
  equalizer/src/config.cpp
  equalizer/src/framedata.cpp
  equalizer/src/initdata.cpp
  equalizer/src/node.cpp
  equalizer/src/nodefactory.cpp
  equalizer/src/pipe.cpp
  equalizer/src/serializablediffactor.cpp
  equalizer/src/serializableogreactor.cpp
  equalizer/src/window.cpp

  equalizer/headers/channel.h
  equalizer/headers/config.h
  equalizer/headers/framedata.h
  equalizer/headers/initdata.h
  equalizer/headers/node.h
  equalizer/headers/nodefactory.h
  equalizer/headers/pipe.h
  equalizer/headers/serializablediffactor.h
  equalizer/headers/serializableogreactor.h
  equalizer/headers/window.h

  ogre/src/ogreapplication.cpp
  ogre/headers/ogreapplication.h

  bullet/src/bulletphysics.cpp
  bullet/headers/bulletphysics.h
  bullet/src/bulletrigidbody.cpp
  bullet/headers/bulletrigidbody.h

  datastructures/headers/vec3.h
  datastructures/headers/vec4.h
  datastructures/headers/ogreactor.h
  datastructures/headers/diffactor.h
  datastructures/headers/bulletactor.h
  datastructures/headers/actor.h
)

if(EQUALIZER_USE_X11)
  list(APPEND EQUALIZER_LIBRARIES X11)
endif()

set_target_properties(eqOgreBullet PROPERTIES OUTPUT_NAME eqOgreBullet)
target_link_libraries(eqOgreBullet
  ${EQUALIZER_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${GLEWMX_LIBRARIES}
  ${OPENGL_gl_LIBRARY}
  ${OPENGL_glu_LIBRARY}
  ${OGRE_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ${BULLET_LIBRARIES}
)

install(TARGETS eqOgreBullet RUNTIME DESTINATION bin)
install(FILES
  configs/vr_lab_simulation.eqc
  DESTINATION configs
)
