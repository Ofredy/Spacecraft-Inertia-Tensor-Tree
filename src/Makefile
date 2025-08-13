# Use cmd.exe so Windows commands work (mkdir, rmdir, del)
SHELL := cmd
.SHELLFLAGS := /C

CXX       := g++
AR        := ar
CXXFLAGS  := -std=c++2a -O2 -Wall -Wextra -I"../lib"
LDFLAGS   :=

# Paths (keep them simple; quotes added in commands to survive spaces)
BUILD_DIR := ../build
LIB_DIR   := ../lib

BIN_NAME  := iss_inertia
APP_BIN   := $(BIN_NAME).exe

# Sources/objects
LIB_SRCS    := $(wildcard $(LIB_DIR)/*.cpp)
LIB_OBJS    := $(patsubst $(LIB_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(LIB_SRCS))
STATIC_LIB  := $(BUILD_DIR)/libinertia.a

APP_SRCS := $(wildcard *.cpp)
APP_OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(APP_SRCS))

.PHONY: all clean dirs run

all: dirs $(APP_BIN)

dirs:
	@if not exist "$(BUILD_DIR)" mkdir "$(BUILD_DIR)"

$(APP_BIN): $(STATIC_LIB) $(APP_OBJS)
	$(CXX) -o "$(APP_BIN)" $(APP_OBJS) "$(STATIC_LIB)" $(LDFLAGS)

$(STATIC_LIB): $(LIB_OBJS)
	$(AR) rcs "$(STATIC_LIB)" $(LIB_OBJS)

# Compile app .cpp in src/ -> ../build/*.o
$(BUILD_DIR)/%.o: %.cpp
	@if not exist "$(BUILD_DIR)" mkdir "$(BUILD_DIR)"
	$(CXX) $(CXXFLAGS) -c "$<" -o "$@"

# Compile lib .cpp in ../lib -> ../build/*.o
$(BUILD_DIR)/%.o: $(LIB_DIR)/%.cpp
	@if not exist "$(BUILD_DIR)" mkdir "$(BUILD_DIR)"
	$(CXX) $(CXXFLAGS) -c "$<" -o "$@"

run: all
	".\$(APP_BIN)"

clean:
	@echo Cleaning...
	@if exist "$(BUILD_DIR)" rmdir /S /Q "$(BUILD_DIR)"
	@if exist "$(APP_BIN)"  del /Q "$(APP_BIN)"
