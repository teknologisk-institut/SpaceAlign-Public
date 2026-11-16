#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "MAPLE_K::MAPLE_K" for configuration "Release"
set_property(TARGET MAPLE_K::MAPLE_K APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(MAPLE_K::MAPLE_K PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/MAPLE_K-1.0/libMAPLE_K.a"
  )

list(APPEND _cmake_import_check_targets MAPLE_K::MAPLE_K )
list(APPEND _cmake_import_check_files_for_MAPLE_K::MAPLE_K "${_IMPORT_PREFIX}/lib/MAPLE_K-1.0/libMAPLE_K.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
