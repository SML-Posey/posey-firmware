zephyr_include_directories(sh2)

FILE(GLOB_RECURSE SH2_SRC sh2/*.c)
FILE(GLOB LIB_SRC *.c)

zephyr_library_sources(${SH2_SRC})
zephyr_library_sources(${LIB_SRC})