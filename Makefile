# ============================================================
# NeuroVive SmartPen Features Library — Makefile
# ============================================================

# Compiler and standard flags
CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -O3 -fPIC -I./include
LDFLAGS  := -shared

# Directory structure
SRC_DIR   := src
INC_DIR   := include
TEST_DIR  := tests
OBJ_DIR   := build/obj
BIN_DIR   := build/bin

# Output definitions
LIB_NAME  := libSmartPen.so
LIB_OUT   := $(BIN_DIR)/$(LIB_NAME)
TEST_OUT  := $(BIN_DIR)/test_runner

# File discovery
LIB_SRC   := $(SRC_DIR)/SmartPen_Features.cpp
TEST_SRC  := $(TEST_DIR)/main.cpp

# Object files (Generated automatically)
LIB_OBJ   := $(LIB_SRC:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
TEST_OBJ  := $(TEST_SRC:$(TEST_DIR)/%.cpp=$(OBJ_DIR)/%.o)

# Dependency files (For header tracking)
DEPS      := $(LIB_OBJ:.o=.d) $(TEST_OBJ:.o=.d)

# ------------------------------------------------------------
# Primary Targets
# ------------------------------------------------------------

all: $(LIB_OUT) $(TEST_OUT)

# Create shared library
$(LIB_OUT): $(LIB_OBJ)
	@mkdir -p $(@D)
	$(CXX) $(LDFLAGS) $^ -o $@
	@echo "[SUCCESS] Shared Library built: $@"

# Create test executable
$(TEST_OUT): $(TEST_OBJ) $(LIB_OBJ)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $^ -o $@
	@echo "[SUCCESS] Test Runner built: $@"

# ------------------------------------------------------------
# Compilation Rules
# ------------------------------------------------------------

# Compile source files to objects with dependency generation
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(OBJ_DIR)/%.o: $(TEST_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

# Include generated dependencies
-include $(DEPS)

# ------------------------------------------------------------
# Utility Targets
# ------------------------------------------------------------

test: $(TEST_OUT)
	@echo "Executing NeuroVive Unit Tests..."
	@LD_LIBRARY_PATH=$(BIN_DIR):$(LD_LIBRARY_PATH) ./$(TEST_OUT)

memcheck: $(TEST_OUT)
	@echo "Starting Valgrind Memory Audit..."
	valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes ./$(TEST_OUT)

clean:
	rm -rf build
	@echo "Build environment purged."

rebuild: clean all

info:
	@echo "Architecture: NeuroVive - SmartPenFeatures Library"
	@echo "Compiler:     $(CXX)"
	@echo "Optimizations: High (-O3)"
	@echo "Standards:    C++17"

.PHONY: all test memcheck clean info