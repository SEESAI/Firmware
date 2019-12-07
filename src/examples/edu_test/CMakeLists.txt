px4_add_module(
    MODULE examples__edu_test
    MAIN edu_test
    STACK_MAIN 2000
    SRCS
        edu_test.c
    DEPENDS
    )
