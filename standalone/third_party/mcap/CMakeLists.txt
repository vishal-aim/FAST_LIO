# Vendored mcap C++ reader/writer (see mcap.cpp for version/provenance).
# Built as its own static library so consumers just link "mcap" instead of
# managing the MCAP_IMPLEMENTATION-defining translation unit themselves.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED IMPORTED_TARGET liblz4)
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)

add_library(mcap STATIC mcap.cpp)
target_include_directories(mcap PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")
target_link_libraries(mcap PUBLIC PkgConfig::LZ4 PkgConfig::ZSTD)
# The vendored headers use std::optional/std::string_view/structured bindings.
target_compile_features(mcap PUBLIC cxx_std_17)
