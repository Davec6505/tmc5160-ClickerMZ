# Root Makefile - XC32 Project for 32MZ1024EFH064
# Simple Makefile for PIC32MZ project

# Force use of bundled sh.exe (no dependency on MSYS/MinGW in system PATH)
SHELL := c:/Users/davec/.vscode/extensions/davidcoetzee.xc-project-importer-2.5.51/bin/win32/sh.exe
export PATH := c:/Users/davec/.vscode/extensions/davidcoetzee.xc-project-importer-2.5.51/bin/win32:$(PATH)

# Create /tmp in the bundled Cygwin root to suppress "could not find /tmp" warnings
$(shell "$(SHELL)" -c 'mkdir -p /tmp' 2>/dev/null)

# Name of the project binary
MODULE     := tmc5160-ClickerMZ

# Default target when running 'make' with no arguments
.DEFAULT_GOAL := build

# Device configuration
DEVICE     := 32MZ1024EFH064

# Optimization level control (optional override)
# Usage: make all OPT_LEVEL=2    (use -O2 instead of default -O1)
OPT_LEVEL ?= 1

# Memory configuration for dynamic allocation
# These control heap and stack sizes for the PIC32 application
HEAP_SIZE  := 8192    # Heap for dynamic memory allocation
STACK_SIZE := 8192   # Stack for function calls and local variables

# Compiler location and DFP (Device Family Pack) location
# Leave empty to enable auto-detection in srcs/Makefile
# Set these to override auto-detection:
#   COMPILER_LOCATION := C:/Program Files/Microchip/xc32/v5.00/bin
#   DFP := C:/Program Files/Microchip/MPLABX/v6.25/packs/Microchip/PIC32MZ-EF_DFP/1.4.168
COMPILER_LOCATION := C:/Program Files/Microchip/xc32/v5.00/bin
DFP := C:/Program Files/Microchip/MPLABX/v6.25/packs/Microchip/PIC32MZ-EF_DFP/1.4.168

# MikroC Bootloader Configuration
# Set to 'yes' if project uses custom startup (detected from MPLABX -nostartfiles flag)
USE_MIKROE_BOOTLOADER := yes

# Use bundled make.exe for recursive calls (no dependency on system PATH)
MAKE_EXE := c:/Users/davec/.vscode/extensions/davidcoetzee.xc-project-importer-2.5.51/bin/win32/make.exe
BUILD=$(MAKE_EXE)
BUILD_DIR=$(MAKE_EXE) build_dir

# Build the make command with optional path overrides
MAKE_ARGS := DEVICE=$(DEVICE) MODULE=$(MODULE) HEAP_SIZE=$(HEAP_SIZE) STACK_SIZE=$(STACK_SIZE) OPT_LEVEL=$(OPT_LEVEL) USE_MIKROE_BOOTLOADER=$(USE_MIKROE_BOOTLOADER)
ifneq ($(COMPILER_LOCATION),)
	MAKE_ARGS += COMPILER_LOCATION="$(COMPILER_LOCATION)"
endif
ifneq ($(DFP),)
	MAKE_ARGS += DFP="$(DFP)"
endif

lib:
	@echo "######  BUILDING LIBRARY  ########"
	$(BUILD) -C srcs lib $(MAKE_ARGS)
	@echo "######  LIBRARY COMPLETE (libs/libstepper.a)  ########"

# Default target: incremental build (only rebuilds changed files)
build:
	@echo "######  INCREMENTAL BUILD  ########"
	$(BUILD) -C srcs $(MAKE_ARGS)
	@echo "######  BUILD COMPLETE (bins/$(MODULE).hex)  ########"

# Clean + full rebuild
all: clean build
	@echo "######  FULL REBUILD COMPLETE  ########"

# Alias for 'all' (used by VS Code rebuild button)
rebuild: all

build_dir:
	@echo "###### BUILDING DIRECTORIES FOR OUTPUT BINARIES #######"
	$(BUILD_DIR) -C srcs
	@echo "############ BUILDING DIRECTORIES COMPLETED ###########"

debug:
	@echo "####### DEBUGGING OUTPUTS #######"
	$(BUILD) -C srcs debug COMPILER_LOCATION="$(COMPILER_LOCATION)" DFP_LOCATION="$(DFP_LOCATION)" DFP="$(DFP)" DEVICE=$(DEVICE) MODULE=$(MODULE) HEAP_SIZE=$(HEAP_SIZE) STACK_SIZE=$(STACK_SIZE)

platform:
	@echo "####### PLATFORM INFO #######"
	$(BUILD) -C srcs platform COMPILER_LOCATION="$(COMPILER_LOCATION)" DFP_LOCATION="$(DFP_LOCATION)" DFP="$(DFP)" DEVICE=$(DEVICE) MODULE=$(MODULE)

clean:
	@echo "####### CLEANING OUTPUTS #######"
ifeq ($(OS),Windows_NT)
	@powershell -NoProfile -Command "make -C srcs clean DRY_RUN=$(DRY_RUN)"
else
	$(MAKE) -C srcs clean DRY_RUN=$(DRY_RUN)
endif

help:
	@echo "=========================================="
	@echo "  XC32 Makefile - $(MODULE)"
	@echo "=========================================="
	@echo "Available targets:"
	@echo "  make build       - Incremental build (fast, only changed files)"
	@echo "  make all         - Clean + full rebuild"
	@echo "  make rebuild     - Alias for 'all' (clean + rebuild)"
	@echo "  make build_dir   - Create output directories (bins/, objs/, incs/)"
	@echo "                     and move .h files into incs/ folder"
	@echo "  make clean       - Remove build artifacts (objs/, bins/)"
	@echo "  make debug       - Show debug information"
	@echo "  make platform    - Show platform and toolchain info"
	@echo "  make help        - Show this help message"
	@echo ""
	@echo "Optimization levels:"
	@echo "  make OPT_LEVEL=0 - No optimization (faster compile)"
	@echo "  make OPT_LEVEL=1 - Size/speed balance (default)"
	@echo "  make OPT_LEVEL=2 - Optimize for speed"
	@echo "  make OPT_LEVEL=3 - Aggressive optimization"
	@echo ""
	@echo "Advanced options:"
	@echo "  make build_dir DRY_RUN=1 - Preview directory creation (safe preview)"
	@echo "  make build_dir DRY_RUN=0 - Actually create directories (default)"
	@echo "  make clean DRY_RUN=1     - Show what would be deleted (safe preview)"
	@echo "  make clean DRY_RUN=0     - Actually delete files (default)"
	@echo "  make debug               - Show compiler paths and build variables"
	@echo ""
	@echo "Project configuration:"
	@echo "  Device:      $(DEVICE)"
	@echo "  Heap:        $(HEAP_SIZE) bytes"
	@echo "  Stack:       $(STACK_SIZE) bytes"
	@echo "  Compiler:    $(COMPILER_LOCATION)"
	@echo ""
	@echo "NOTE: This project was imported from MPLABX by XC Project Importer"
	@echo "      Headers organized in incs/, sources in srcs/, outputs in bins/"
	@echo "      Use MPLABX IDE for MCC peripheral configuration, then re-import"
	@echo "=========================================="

.PHONY: build all rebuild build_dir clean debug platform help
