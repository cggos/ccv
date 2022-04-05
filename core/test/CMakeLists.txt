find_package(GTest REQUIRED)
if(NOT GTest_FOUND)
    message(FATAL_ERROR "NOT FOUND GTest which is needed to build test program!")
else()
    include_directories(${GTEST_INCLUDE_DIRS})

    set(CIMG_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/CImg.h)
    if(NOT EXISTS ${CIMG_HEADER})
        file(DOWNLOAD
                https://raw.githubusercontent.com/dtschump/CImg/master/CImg.h
                ${CIMG_HEADER}
                SHOW_PROGRESS)
    endif()

    set(TEST_APP_NAME "test.run")
    add_executable(${TEST_APP_NAME}
            gtest_main.cpp
            test_image.cpp)
    target_link_libraries(${TEST_APP_NAME}
            ${CMAKE_PROJECT_NAME}
            gtest pthread -lX11)
endif()


