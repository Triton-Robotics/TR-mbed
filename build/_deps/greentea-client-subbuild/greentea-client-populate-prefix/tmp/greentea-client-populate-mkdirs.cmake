# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/winstonchou/Documents/GitHub/TR-mbed/build/_deps/greentea-client-src"
  "/home/winstonchou/Documents/GitHub/TR-mbed/build/_deps/greentea-client-build"
  "/home/winstonchou/Documents/GitHub/TR-mbed/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix"
  "/home/winstonchou/Documents/GitHub/TR-mbed/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/tmp"
  "/home/winstonchou/Documents/GitHub/TR-mbed/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp"
  "/home/winstonchou/Documents/GitHub/TR-mbed/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src"
  "/home/winstonchou/Documents/GitHub/TR-mbed/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/winstonchou/Documents/GitHub/TR-mbed/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/winstonchou/Documents/GitHub/TR-mbed/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
