# handle dependencies
include(FetchContent)

# === fmt ===
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG 6.1.2)
FetchContent_GetProperties(fmt)
if(NOT fmt_POPULATED)
  set(FMT_INSTALL ON)
  FetchContent_Populate(fmt)
  add_subdirectory(${fmt_SOURCE_DIR} ${fmt_BINARY_DIR})
endif()

# === spdlog ===
# Force spdlog to use downloaded fmt library, using the header-only target
set(SPDLOG_FMT_EXTERNAL_HO
    ON
    CACHE INTERNAL "") # Forces the value
FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG v1.7.0)
FetchContent_GetProperties(spdlog)
if(NOT spdlog_POPULATED)
  set(SPDLOG_INSTALL ON)
  FetchContent_Populate(spdlog)
  add_subdirectory(${spdlog_SOURCE_DIR} ${spdlog_BINARY_DIR})
endif()
