# ============================================================
# NeuroVive SmartPen Features Library — Makefile
# ============================================================

# Android NDK platform level
PLATFORM := 21

# Supported Android ABIs
ABIS := arm64-v8a armeabi-v7a x86_64 x86

# Output layout for Android shared libraries
LIB_ROOT := libs
LIB_NAME := libSmartPen.so

# Source and include directories
SRC_DIR := src
INC_DIR := include
OBJ_DIR := build/obj

# Compiler toolchains for each ABI
CXX_arm64-v8a   := aarch64-linux-android$(PLATFORM)-clang++
CXX_armeabi-v7a := armv7a-linux-androideabi$(PLATFORM)-clang++
CXX_x86_64      := x86_64-linux-android$(PLATFORM)-clang++
CXX_x86         := i686-linux-android$(PLATFORM)-clang++

# Common flags
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -O3 -fPIC -I$(INC_DIR)
LDFLAGS  := -shared -static-libstdc++

# File discovery
LIB_SRC := $(SRC_DIR)/SmartPen_Features.cpp

# Object files (Generated automatically)
ALL_OBJS := $(foreach abi,$(ABIS),$(OBJ_DIR)/$(abi)/SmartPen_Features.o)

# Dependency files (For header tracking)
DEPS := $(ALL_OBJS:.o=.d)

# ------------------------------------------------------------
# Primary Targets
# ------------------------------------------------------------

all: $(foreach abi,$(ABIS),$(LIB_ROOT)/$(abi)/$(LIB_NAME))

# Create ABI-specific shared libraries
$(LIB_ROOT)/%/$(LIB_NAME): $(OBJ_DIR)/%/SmartPen_Features.o
	@mkdir -p $(@D)
	$(CXX_$*) $(LDFLAGS) $^ -o $@
	@echo "[SUCCESS] Built Android library for $* : $@"

# Compile source files to ABI-specific objects
$(OBJ_DIR)/%/SmartPen_Features.o: $(LIB_SRC)
	@mkdir -p $(@D)
	$(CXX_$*) $(CXXFLAGS) -MMD -MP -c $< -o $@

# Include generated dependencies
-include $(DEPS)

# ------------------------------------------------------------
# Utility Targets
# ------------------------------------------------------------

clean:
	rm -rf build $(LIB_ROOT)
	@echo "Build environment purged."

rebuild: clean all

info:
	@echo "Android platform: android-$(PLATFORM)"
	@echo "ABIs: $(ABIS)"
	@echo "Library name: $(LIB_NAME)"
	@echo "Output root: $(LIB_ROOT)"

.PHONY: all clean rebuild info