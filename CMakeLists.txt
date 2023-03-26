cmake_minimum_required(VERSION 3.21)

project(ParticleAPI)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_BUILD_TYPE Release)

file(GLOB_RECURSE SRC_FILES_DIR
    ${PROJECT_SOURCE_DIR}/src/*.c
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*.h
)

include_directories(
    ${CMAKE_SOURCE_DIR}/LiteLoader/include
    ${CMAKE_SOURCE_DIR}/LiteLoader/include/llapi
    ${CMAKE_SOURCE_DIR}/LiteLoader/third-party/include
)

link_directories(
    ${CMAKE_SOURCE_DIR}/LiteLoader/lib
)

add_definitions(
    -D"NDEBUG"
    -D"NOMINMAX"
    -D"TEMPLATE_EXPORTS"
    -D"UNICODE"
    -D"WIN32_LEAN_AND_MEAN"
    -D"_AMD64_"
    -D"_CRT_SECURE_NO_WARNINGS"
    -D"_UNICODE"
    -D"_USRDLL"
    -D"_WINDLL"
    -D"_WINDOWS"
)

add_compile_options(
    /diagnostics:column
    /EHsc
    /FC
    /GL
    /MD
    /nologo
    /permissive-
    /sdl
    /utf-8
    /Zc:inline
    /Zi
)

add_link_options(
    /DEBUG
    /DELAYLOAD:"bedrock_server.dll"
    /DLL
    /IGNORE:4199
    /INCREMENTAL:NO
    /LTCG
    /MANIFESTUAC:NO
    /MACHINE:X64
    /NOLOGO
    /OPT:ICF
    /OPT:REF
    /SUBSYSTEM:CONSOLE
)

add_library(ParticleAPI SHARED ${SRC_FILES_DIR})

target_link_libraries(ParticleAPI LiteLoader
    "${CMAKE_SOURCE_DIR}/LiteLoader/lib/bedrock_server_api.lib"
    "${CMAKE_SOURCE_DIR}/LiteLoader/lib/bedrock_server_var.lib"
    SymDBHelper
)

add_custom_command(
    TARGET ParticleAPI POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:ParticleAPI> ${CMAKE_BINARY_DIR}/output/plugins/LiteLoader/ParticleAPI.dll
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_PDB_FILE:ParticleAPI> ${CMAKE_BINARY_DIR}/pdb/ParticleAPI.pdb
    COMMENT "Copying ParticleAPI DLL and PDB to output directory"
    VERBATIM
)