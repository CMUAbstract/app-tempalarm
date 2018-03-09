override BOARD = capybara
export BOARD

export BOARD_MAJOR = 2
export BOARD_MINOR = 0

TOOLCHAINS = \
	gcc \

include ext/maker/Makefile

# Paths to toolchains here if not in or different from defaults in Makefile.env
