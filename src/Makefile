# Makefile (lives in src/)
# Builds executable in the current src/ directory.
# Object files go to src/build/ and are wiped by `make clean`.

# --- toolchain & flags ---
CXX       := g++
CXXFLAGS  := -std=c++17 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS  := -I"../lib/inertia_tree_lib" -I"../lib/json/single_include"

# --- exe name (handles Windows .exe) ---
APP    := inertia_tree
ifeq ($(OS),Windows_NT)
  EXEEXT := .exe
else
  EXEEXT :=
endif
TARGET := $(APP)$(EXEEXT)

# --- dirs ---
BUILD_DIR := build
LIB_DIR   := ../lib/inertia_tree_lib
SRC_DIR   := .

# --- sources & objects ---
LIB_SRCS := $(wildcard $(LIB_DIR)/*.cpp)
LIB_BASENAMES := $(notdir $(LIB_SRCS))
LIB_OBJS := $(addprefix $(BUILD_DIR)/,$(LIB_BASENAMES:.cpp=.o))

MAIN_SRC := $(SRC_DIR)/main.cpp
MAIN_OBJ := $(BUILD_DIR)/main.o

OBJS := $(MAIN_OBJ) $(LIB_OBJS)

# --- default ---
.PHONY: all
all: $(TARGET)

# --- link ---
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o "$(TARGET)"

# --- make build dir ---
.PHONY: dirs
dirs:
ifeq ($(OS),Windows_NT)
	@if not exist "$(BUILD_DIR)" mkdir "$(BUILD_DIR)"
else
	@mkdir -p "$(BUILD_DIR)"
endif

# --- compile rules ---
$(BUILD_DIR)/main.o: $(MAIN_SRC) | dirs
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c "$<" -o "$@"

# Build each lib/*.cpp into build/*.o
$(BUILD_DIR)/%.o: $(LIB_DIR)/%.cpp | dirs
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c "$<" -o "$@"

# --- utilities ---
.PHONY: clean run debug release
clean:
ifeq ($(OS),Windows_NT)
	@cmd /C if exist "$(BUILD_DIR)" rmdir /S /Q "$(BUILD_DIR)"
	@cmd /C del /Q "$(TARGET)" 2>nul
else
	@rm -rf "$(BUILD_DIR)"
	@rm -f "$(TARGET)"
endif

run: all
ifeq ($(OS),Windows_NT)
	"./$(TARGET)"
else
	./$(TARGET)
endif

debug: CXXFLAGS := -std=c++17 -g -O0 -Wall -Wextra -Wpedantic
debug: clean all

release: CXXFLAGS := -std=c++17 -O3 -DNDEBUG -Wall -Wextra -Wpedantic
release: clean all
