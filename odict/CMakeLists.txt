cmake_minimum_required (VERSION 2.8.11)
project (odict)
find_package(Doxygen)


add_executable (odict odict.c serialization.c)
target_link_libraries (odict)

if(DOXYGEN_FOUND)
    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/../doc/odict.doxygen)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)
    add_custom_target(doc ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile_in}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../doc
            SOURCES ${doxyfile}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)

endif(DOXYGEN_FOUND)
