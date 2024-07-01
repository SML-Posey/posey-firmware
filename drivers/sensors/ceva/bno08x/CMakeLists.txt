# zephyr_library()

FILE(GLOB sh2_sources sh2/*.c)
zephyr_library_sources(${sh2_sources})
zephyr_library_sources(bno08x.c)
zephyr_library_sources(bno08x_spi.c)
