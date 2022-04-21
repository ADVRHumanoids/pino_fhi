find_package(xbot2 REQUIRED)

add_xbot2_plugin(gcomp_example
    gcomp_example.cpp)

install(
    TARGETS gcomp_example
    DESTINATION lib
    )
