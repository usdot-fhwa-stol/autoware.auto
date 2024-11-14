# Copyright 2021 the Autoware Foundation
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Use a reduced set of compile options for packages which require it
function(autoware_set_compile_options_reduced_warning target)

  message("${target} is using reduced compiler warnings. Extra care should be taken with code inspection.")

  if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    # Causes the visibility macros to use dllexport rather than dllimport,
    # which is appropriate when building the dll but not consuming it.
    string(TOUPPER ${target} PROJECT_NAME_UPPER)
    target_compile_options(${target} PRIVATE "/bigobj")
    target_compile_definitions(${target} PRIVATE
      ${PROJECT_NAME_UPPER}_BUILDING_DLL
      -D_CRT_NONSTDC_NO_WARNINGS
      -D_CRT_SECURE_NO_WARNINGS
      -D_WINSOCK_DEPRECATED_NO_WARNINGS)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(${target} PRIVATE
      -Werror
      -Wno-useless-cast
      -fvisibility=hidden)
    # C++-only options
    target_compile_options(${target}
      PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -Woverloaded-virtual> -Wno-error=useless-cast -Wno-error=maybe-uninitialized)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${target}
      PUBLIC $<$<COMPILE_LANGUAGE:CXX>:>)
    target_compile_options(${target} PRIVATE -Wlogical-op -frecord-gcc-switches)
  endif()

endfunction()
