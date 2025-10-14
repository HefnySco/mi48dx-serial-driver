# Makefile for compiling serial_mi48 with multiple example binaries

# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -Wall -std=c++17 `pkg-config --cflags opencv4`

# Linker flags
LDFLAGS = -lserialport `pkg-config --libs opencv4`

# Target executables
TARGETS = simple_display advanced_display raw_display

# Common source file
COMMON_SRC = serial_mi48.cpp

# Header files
HEADERS = serial_mi48.hpp

# Object files for common source
COMMON_OBJ = $(COMMON_SRC:.cpp=.o)

# Source files for each target
SIMPLE_SRC = examples/simple_display.cpp
ADVANCED_SRC = examples/advanced_display.cpp
RAW_SRC = examples/raw_display.cpp

# Object files for each target
SIMPLE_OBJ = $(SIMPLE_SRC:.cpp=.o)
ADVANCED_OBJ = $(ADVANCED_SRC:.cpp=.o)
RAW_OBJ = $(RAW_SRC:.cpp=.o)

# Default target
all: $(TARGETS)

# Link object files to create simple_display executable
simple_display: $(COMMON_OBJ) $(SIMPLE_OBJ)
	$(CXX) $(COMMON_OBJ) $(SIMPLE_OBJ) -o $@ $(LDFLAGS)

# Link object files to create advanced_display executable
advanced_display: $(COMMON_OBJ) $(ADVANCED_OBJ)
	$(CXX) $(COMMON_OBJ) $(ADVANCED_OBJ) -o $@ $(LDFLAGS)

# Link object files to create raw_display executable
raw_display: $(COMMON_OBJ) $(RAW_OBJ)
	$(CXX) $(COMMON_OBJ) $(RAW_OBJ) -o $@ $(LDFLAGS)

# Compile common source file to object file
$(COMMON_OBJ): $(COMMON_SRC) $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile example source files to object files
$(SIMPLE_OBJ): $(SIMPLE_SRC) $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(ADVANCED_OBJ): $(ADVANCED_SRC) $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(RAW_OBJ): $(RAW_SRC) $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up
clean:
	rm -f $(COMMON_OBJ) $(SIMPLE_OBJ) $(ADVANCED_OBJ) $(RAW_OBJ) $(TARGETS)

# Phony targets
.PHONY: all clean