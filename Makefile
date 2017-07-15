override BOARD = capybara
export BOARD

export BOARD_MAJOR = 1
export BOARD_MINOR = 1

TOOLCHAINS = \
	gcc \

include ext/maker/Makefile

# Paths to toolchains here if not in or different from defaults in Makefile.env

TOOLCHAIN_ROOT = /opt/ti/mspgcc3
