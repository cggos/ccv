find_package(Doxygen)
if(NOT Doxygen_FOUND)
    message(FATAL_ERROR "NOT Found Doxygen which is needed to build the documentation")
else()
    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/doxyfile.config)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/doxyfile)
    configure_file(${doxyfile_in} ${doxyfile} @ONLY)
    add_custom_target(doc
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)
    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
            DESTINATION share/doc)
endif()
