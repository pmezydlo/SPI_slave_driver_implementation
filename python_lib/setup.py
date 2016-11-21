from distutils.core import setup, Extension

CFLAGS = ['-Wall', '-Werror', '-Wno-format-security']

setup (name = 'SPIslave',
        version         = '1.0',
        author          = 'Patryk Mezydlo',
        author_email    = 'mezydlo.p@gmail.com',
        description     = 'This is a demo package',
        url             = 'https://github.com/pmezydlo/SPI_slave_driver_implementation',
        license         = 'GPL v2',
        ext_modules     = [Extension('SPIslave', sources = ['py_spislave.c'], extra_compile_args=CFLAGS)] )
