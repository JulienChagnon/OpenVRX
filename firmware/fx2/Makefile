# SPDX-License-Identifier: GPL-3.0-or-later
#
# Convenience wrapper for Linux and macOS. The build logic lives in build.py
# (one implementation for every platform); it needs SDCC and Python 3.
#
#   make          build build/openvrx-fx2.ihx, .iic and refresh prebuilt/
#   make test     build and run the unit tests
#   make load     upload to a blank FX2 in RAM with fx2tool (pip install fx2)
#   make clean

PYTHON ?= python3
VID    ?= 04b4
PID    ?= 8613

all:
	$(PYTHON) build.py

test:
	$(PYTHON) build.py --test

load: all
	$(PYTHON) -m fx2.fx2tool -d $(VID):$(PID) load build/openvrx-fx2.ihx

clean:
	$(PYTHON) build.py --clean

.PHONY: all test load clean
