# Makefile (lives in src/)
# Builds executable in the current src/ directory.

# --- toolchain & flags ---
CXX       := g++
CXXFLAGS  := -std=c++17 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS  := -I"../lib/inertia_tree_lib"

# --- exe name (handles Windows .exe) ---
ifeq ($(OS),Windows_NT)
  EXEEXT := .exe
else
  EXEEXT :=
endif
TARGET := inertia_demo$(EXEEXT)

# --- sources & objects ---
LIB_DIR  := ../lib/inertia_tree_lib
LIB_SRCS := $(wildcard $(LIB_DIR)/*.cpp)
LIB_OBJS := $(notdir $(LIB_SRCS:.cpp=.o))

MAIN_SRC := main.cpp
MAIN_OBJ := main.o

OBJS := $(MAIN_OBJ) $(LIB_OBJS)

# --- default ---
.PHONY: all
all: $(TARGET)

# --- link ---
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o "$(TARGET)"

# --- compile rules ---
$(MAIN_OBJ): $(MAIN_SRC)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c "$(MAIN_SRC)" -o "$(MAIN_OBJ)"

# build each lib/*.cpp into a local .o (no subdirs)
%.o: $(LIB_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c "$<" -o "$@"

# --- utilities ---
.PHONY: clean run debug release
clean:
	- rm -f *.o "$(TARGET)"
	- cmd /C del /Q *.o 2>nul
	- cmd /C del /Q "$(TARGET)" 2>nul

run: all
	"./$(TARGET)"

debug: CXXFLAGS := -std=c++17 -g -O0 -Wall -Wextra -Wpedantic
debug: clean all

release: CXXFLAGS := -std=c++17 -O3 -DNDEBUG -Wall -Wextra -Wpedantic
release: clean all
