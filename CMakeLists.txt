# Copyright (c) Open Enclave SDK contributors.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.11)

project("TC Malloc for Open Enclave" LANGUAGES C CXX)

# Currently the `OpenEnclave` package depends on `project()`.
find_package(OpenEnclave CONFIG REQUIRED)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(SOURCES
    ${SOURCE_DIR}/common.cc
    ${SOURCE_DIR}/internal_logging.cc
    ${SOURCE_DIR}/system-alloc.cc
    ${SOURCE_DIR}/central_freelist.cc
    ${SOURCE_DIR}/tcmalloc.cc
    ${SOURCE_DIR}/page_heap.cc
    ${SOURCE_DIR}/sampler.cc
    ${SOURCE_DIR}/span.cc
    ${SOURCE_DIR}/stack_trace_table.cc
    ${SOURCE_DIR}/static_vars.cc
    ${SOURCE_DIR}/symbolize.cc
    ${SOURCE_DIR}/thread_cache.cc    
    ${SOURCE_DIR}/malloc_extension.cc
    ${SOURCE_DIR}/malloc_hook.cc
    ${SOURCE_DIR}/maybe_threads.cc
    ${SOURCE_DIR}/base/spinlock.cc)

add_library(
  oetcmalloc
  STATIC
  ${SOURCES}
  enclave/sgx.c
  enclave/allocator.cpp)

if (WIN32)
  maybe_build_using_clangw(oetcmalloc)
endif ()

target_link_libraries(oetcmalloc openenclave::oelibcxx openenclave::oelibc)

target_compile_options(
  oetcmalloc
  PRIVATE
  -DHAVE_CONFIG_H
  -DNO_TCMALLOC_SAMPLES
  -DNO_TCMALLOC_SAMPLES
  -DNO_HEAP_CHECK
  -DNDEBUG
  -ffreestanding
  -Wall
  -Wno-error
  -Wwrite-strings
  -Woverloaded-virtual
  -Wno-sign-compare
  -fno-builtin-malloc
  -fno-builtin-free
  -fno-builtin-realloc
  -fno-builtin-calloc
  -fno-builtin-cfree
  -fno-builtin-memalign
  -fno-builtin-posix_memalign
  -fno-builtin-valloc
  -fno-builtin-pvalloc
  -Wno-unused-result
  -fsized-deallocation
  -faligned-new
  -DNO_FRAME_POINTER
  -Wno-sign-conversion
  -Wno-error
  -Wno-unused-local-typedefs
  -ftls-model=local-exec
  -fno-exceptions
  -nostdinc
  -nostdinc++
  -I${SOURCE_DIR}
  -I${CMAKE_CURRENT_SOURCE_DIR}/enclave/gen
  -fPIE
  -DPIC)

