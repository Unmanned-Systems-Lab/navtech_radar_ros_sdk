#!/bin/sh

build_root=build

if [ ! -d source/googletest ]; then
  echo "Getting Google Test/Mock Framework"
  pushd src
  git clone https://github.com/google/googletest.git
  pushd googletest
  git checkout tags/release-1.10.0 -b release-1.10.0
  popd
  popd
fi

# --------------------------------------------------------------------
# CMake invocations
#
run_cmake_host()
{
  echo "-------------------------------------------------------------------------------------------------------------------"
  echo "Local host build (${1})"
  echo "-------------------------------------------------------------------------------------------------------------------"

  build_dir=${build_root}/linux/$1
  ver_num=$2

  if [ -d "${build_dir}" ]; then
      echo "Removing old build folder, ${build_dir}"
      rm -rf ${build_dir}
  fi
  mkdir -p ${build_dir}

  cmake -DCMAKE_BUILD_TYPE=$1 \
        -DPLATFORM=linux_x86_64 \
        -S src \
        -B ${build_dir}
}


run_cmake_host Debug
run_cmake_host Release
