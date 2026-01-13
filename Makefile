# Makefile for compiling serial_mi48 library and example binaries

# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -Wall -std=c++17 -fPIC `pkg-config --cflags opencv4`

# Linker flags
LDFLAGS = -lserialport `pkg-config --libs opencv4`

# Output directories
BIN_DIR = bin
LIB_DIR = bin

# Library names
STATIC_LIB = $(LIB_DIR)/libmi48.a
SHARED_LIB = $(LIB_DIR)/libmi48.so

# Target executables (with bin directory)
TARGETS = $(BIN_DIR)/simple_display $(BIN_DIR)/advanced_display $(BIN_DIR)/raw_display $(BIN_DIR)/dual_camera

# Common source file
COMMON_SRC = serial_mi48.cpp

# Header files
HEADERS = serial_mi48.hpp

# Object files for common source
COMMON_OBJ = $(COMMON_SRC:.cpp=.o)

# Object files for library (compiled with -fPIC for shared library)
LIB_OBJ = serial_mi48_lib.o

# Source files for each target
SIMPLE_SRC = examples/simple_display.cpp
ADVANCED_SRC = examples/advanced_display.cpp
RAW_SRC = examples/raw_display.cpp
DUAL_SRC = examples/dual_camera.cpp

# Object files for each target
SIMPLE_OBJ = $(SIMPLE_SRC:.cpp=.o)
ADVANCED_OBJ = $(ADVANCED_SRC:.cpp=.o)
RAW_OBJ = $(RAW_SRC:.cpp=.o)
DUAL_OBJ = $(DUAL_SRC:.cpp=.o)

# Default target - build libraries and examples
all: $(BIN_DIR) libraries $(TARGETS)

# Create output directory
$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# Build both static and shared libraries
libraries: $(STATIC_LIB) $(SHARED_LIB)

# Create static library
$(STATIC_LIB): $(LIB_OBJ) | $(BIN_DIR)
	ar rcs $@ $^

# Create shared library
$(SHARED_LIB): $(LIB_OBJ) | $(BIN_DIR)
	$(CXX) -shared -o $@ $^ $(LDFLAGS)

# Compile library source file with -fPIC for shared library
$(LIB_OBJ): $(COMMON_SRC) $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Link object files to create simple_display executable
$(BIN_DIR)/simple_display: $(COMMON_OBJ) $(SIMPLE_OBJ) | $(BIN_DIR)
	$(CXX) $(COMMON_OBJ) $(SIMPLE_OBJ) -o $@ $(LDFLAGS)

# Link object files to create advanced_display executable
$(BIN_DIR)/advanced_display: $(COMMON_OBJ) $(ADVANCED_OBJ) | $(BIN_DIR)
	$(CXX) $(COMMON_OBJ) $(ADVANCED_OBJ) -o $@ $(LDFLAGS)

# Link object files to create raw_display executable
$(BIN_DIR)/raw_display: $(COMMON_OBJ) $(RAW_OBJ) | $(BIN_DIR)
	$(CXX) $(COMMON_OBJ) $(RAW_OBJ) -o $@ $(LDFLAGS)

# Link object files to create dual_camera executable
$(BIN_DIR)/dual_camera: $(COMMON_OBJ) $(DUAL_OBJ) | $(BIN_DIR)
	$(CXX) $(COMMON_OBJ) $(DUAL_OBJ) -o $@ $(LDFLAGS) -lpthread

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

$(DUAL_OBJ): $(DUAL_SRC) $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up
clean:
	rm -f $(COMMON_OBJ) $(LIB_OBJ) $(SIMPLE_OBJ) $(ADVANCED_OBJ) $(RAW_OBJ) $(DUAL_OBJ) $(TARGETS) $(STATIC_LIB) $(SHARED_LIB)
	rm -rf $(BIN_DIR)

# Install libraries (optional)
install: libraries
	mkdir -p /usr/local/lib
	mkdir -p /usr/local/include
	cp $(STATIC_LIB) /usr/local/lib/
	cp $(SHARED_LIB) /usr/local/lib/
	cp $(HEADERS) /usr/local/include/
	ldconfig

# Uninstall libraries (optional)
uninstall:
	rm -f /usr/local/lib/libmi48.a
	rm -f /usr/local/lib/libmi48.so
	rm -f /usr/local/include/$(HEADERS)
	ldconfig

# Help target
help:
	@echo "Available targets:"
	@echo "  all       - Build libraries and examples"
	@echo "  libraries - Build static and shared libraries only"
	@echo "  clean     - Remove all built files"
	@echo "  install   - Install libraries to /usr/local"
	@echo "  uninstall - Remove libraries from /usr/local"
	@echo "  help      - Show this help message"
	@echo ""
	@echo "Output directory: $(BIN_DIR)/"
	@echo "Libraries: $(STATIC_LIB), $(SHARED_LIB)"
	@echo "Examples: $(TARGETS)"

# Phony targets
.PHONY: all libraries clean install uninstall help