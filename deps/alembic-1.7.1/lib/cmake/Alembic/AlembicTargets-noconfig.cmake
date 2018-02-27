#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Alembic::Alembic" for configuration ""
set_property(TARGET Alembic::Alembic APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(Alembic::Alembic PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "/usr/lib/x86_64-linux-gnu/libImath.so;/usr/lib/x86_64-linux-gnu/libIlmThread.so;/usr/lib/x86_64-linux-gnu/libIex.so;/usr/lib/x86_64-linux-gnu/libIexMath.so;/usr/lib/x86_64-linux-gnu/libHalf.so"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libAlembic.so"
  IMPORTED_SONAME_NOCONFIG "libAlembic.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS Alembic::Alembic )
list(APPEND _IMPORT_CHECK_FILES_FOR_Alembic::Alembic "${_IMPORT_PREFIX}/lib/libAlembic.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
