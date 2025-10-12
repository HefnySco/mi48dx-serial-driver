# Makefile for compiling serial_mi84 and main

# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -Wall -std=c++17

# Linker flags
LDFLAGS = -lserialport

# Target executable
TARGET = serial_sender

# Source files
SOURCES = serial_mi84.cpp main.cpp

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# Header files
HEADERS = serial_mi48.hpp

# Default target
all: $(TARGET)

# Link object files to create executable
$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(TARGET) $(LDFLAGS)

# Compile source files to object files
%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up
clean:
	rm -f $(OBJECTS) $(TARGET)

# Phony targets
.PHONY: all clean