#include "serial_mi48.hpp"
#include <cctype>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <unistd.h>
#include <vector>

// Constructor and destructor
SerialCommandSender::SerialCommandSender()
    : port(nullptr), m_frame_callback(nullptr), m_streaming(false), m_resolution{DEFAULT_ROWS, DEFAULT_COLS} {}

SerialCommandSender::~SerialCommandSender() { close_port(); }

/**
 * @brief Registers a callback function to be called when a new frame is
 * received.
 * @param callback The function to be called. It must match the FrameCallback
 * signature.
 */
void SerialCommandSender::register_frame_callback(FrameCallback callback) {
  m_frame_callback = std::move(callback);
}

// Implementation of methods (unchanged from previous version)
bool SerialCommandSender::open_port(const std::string &port_path) {
  std::cout << "Attempting to connect to port: " << port_path << " at "
            << BAUD_RATE << " baud...\n";

  enum sp_return result = sp_get_port_by_name(port_path.c_str(), &port);
  if (result != SP_OK) {
    std::cerr << "\n--- ERROR ---\n";
    std::cerr << "Could not find serial port " << port_path << ".\n";
    list_available_ports();
    return false;
  }

  result = sp_open(port, SP_MODE_READ_WRITE);
  if (result != SP_OK) {
    std::cerr << "\n--- ERROR ---\n";
    std::cerr << "Could not open serial port " << port_path << ".\n";
    std::cerr << "Make sure the device is connected and you have permission.\n";
    std::cerr << "Details: " << sp_last_error_message() << "\n";
    sp_free_port(port);
    port = nullptr;
    list_available_ports();
    return false;
  }

  sp_set_baudrate(port, BAUD_RATE);
  sp_set_bits(port, 8);
  sp_set_parity(port, SP_PARITY_NONE);
  sp_set_stopbits(port, 1);
  result = sp_set_flowcontrol(port, SP_FLOWCONTROL_RTSCTS);
  if (result != SP_OK) {
    std::cerr << "Warning: Failed to enable RTS/CTS flow control: " << sp_last_error_message() << "\n";
  }
  sp_set_dtr(port, SP_DTR_ON);
  sp_set_rts(port, SP_RTS_ON);

  std::cout << "Successfully opened port.\n";

  // Flush buffers to clear any stale data (matches Python initialization)
  std::cout << "Flushing serial buffers...\n";
  sp_flush(port, SP_BUF_BOTH);

  // Stop any ongoing stream to ensure clean state (matches Python's
  // stop_stream() in open())
  std::cout << "Stopping any ongoing stream...\n";
  int response;
  if (!stop_stream()) {
    std::cerr << "Warning: Failed to stop stream during initialization\n";
  }

  // Verify camera communication by reading SENXOR_TYPE (matches Python's
  // refresh_regmap())
  std::cout << "Verifying camera communication...\n";

  // Read FRAME_MODE to verify stop_stream worked (matches Python)
  if (!get_frame_mode(response)) {
    std::cerr << "Warning: Failed to read FRAME_MODE\n";
  }

  // Read STATUS register to check for errors (matches Python)
  if (!get_status(response)) {
    std::cerr << "Warning: Failed to read STATUS\n";
  }

  // Read SENXOR_TYPE to get camera resolution
  if (!get_senxor_type(response)) {
    std::cerr << "ERROR: Failed to read SENXOR_TYPE. Camera not responding.\n";
    sp_close(port);
    sp_free_port(port);
    port = nullptr;
    return false;
  }

  std::cout << "Successfully initialized camera.\n";
  std::cout << "Pausing briefly for final stabilization...\n";
  usleep(500000); // Reduced from 2 seconds to 0.5 seconds since we verified
                  // communication
  return true;
}

void SerialCommandSender::close_port() {
  if (port != nullptr) {
    sp_close(port);
    sp_free_port(port);
    port = nullptr;
    std::cout << "\nSerial port closed.\n";
  }
}

bool SerialCommandSender::initialize_camera(bool verbose) {
  if (port == nullptr) {
    std::cerr << "ERROR: Port not opened. Call open_port() first.\n";
    return false;
  }

  if (verbose) std::cout << "\n=== MI48 Camera Initialization Sequence ===\n";

  // Step 1: Check if EVK has bridge board (EVK_TEST register)
  int evk_test_value;
  if (get_evk_test(evk_test_value)) {
    bool has_bridge = (evk_test_value == 0xFF);
    if (verbose) std::cout << "EVK Bridge detected: " << (has_bridge ? "YES" : "NO") << "\n";
    
    // Step 2: Power up sensor if no bridge detected
    if (!has_bridge) {
      if (verbose) std::cout << "Powering up sensor (SENXOR_POWERUP = 0x13)...\n";
      int powerup_response;
      set_senxor_powerup(0x13, powerup_response);
      usleep(100000); // 100ms delay
    }
  }

  // Step 3: Check and stop any existing capture
  int current_mode;
  if (get_frame_mode(current_mode)) {
    if (verbose) std::cout << "Current FRAME_MODE: 0x" << std::hex << current_mode << std::dec << "\n";
    if (current_mode & 0x03) { // GET_SINGLE_FRAME or CONTINUOUS_STREAM bits set
      if (verbose) std::cout << "Stopping existing capture...\n";
      stop_stream();
      usleep(100000); // 100ms delay
    }
  }

  // Step 4: Get camera information
  if (verbose) std::cout << "\n=== Camera Information ===\n";
  int camera_type, module_type, evk_id;
  if (get_senxor_type(camera_type)) {
    if (verbose) std::cout << "Camera Type: " << camera_type << "\n";
  }
  if (get_module_type(module_type)) {
    if (verbose) std::cout << "Module Type: " << module_type << "\n";
  }
  if (get_evk_id(evk_id)) {
    if (verbose) std::cout << "EVK ID: " << evk_id << "\n";
  }

  // Get sensor ID (serial number)
  if (verbose) {
    std::cout << "Sensor ID: ";
    for (int i = 0; i < 6; i++) {
      int id_byte;
      bool success = false;
      switch (i) {
        case 0: success = get_senxor_id_0(id_byte); break;
        case 1: success = get_senxor_id_1(id_byte); break;
        case 2: success = get_senxor_id_2(id_byte); break;
        case 3: success = get_senxor_id_3(id_byte); break;
        case 4: success = get_senxor_id_4(id_byte); break;
        case 5: success = get_senxor_id_5(id_byte); break;
      }
      if (success) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << id_byte;
      }
    }
    std::cout << std::dec << "\n";
  }

  // Get firmware version
  int fw_ver1, fw_ver2;
  if (get_fw_version_1(fw_ver1) && get_fw_version_2(fw_ver2)) {
    int major = (fw_ver1 >> 4) & 0xF;
    int minor = fw_ver1 & 0xF;
    int build = fw_ver2;
    if (verbose) std::cout << "Firmware Version: " << major << "." << minor << "." << build << "\n";
  }

  // Step 5: Wait for bootup to complete (check STATUS register)
  if (verbose) std::cout << "\n=== Waiting for Bootup ===\n";
  bool boot_complete = false;
  int max_boot_attempts = 50;
  for (int attempt = 0; attempt < max_boot_attempts && !boot_complete; ++attempt) {
    int status;
    if (get_status(status)) {
      bool booting = (status & 0x20); // BOOTING_UP flag
      if (!booting) {
        boot_complete = true;
        if (verbose) std::cout << "Bootup complete. STATUS: 0x" << std::hex << status << std::dec << "\n";
        
        // Check for errors
        if (status & 0x02 && verbose) std::cout << "WARNING: Readout too slow\n";
        if (status & 0x04) {
          std::cerr << "ERROR: SenXor interface error\n";
          return false;
        }
        if (status & 0x08) {
          std::cerr << "ERROR: Capture error\n";
          return false;
        }
        if (status & 0x10 && verbose) std::cout << "INFO: Data ready\n";
      } else {
        if (verbose) std::cout << "Booting... (attempt " << (attempt + 1) << ")\n";
        usleep(25000); // 25ms delay
      }
    }
  }

  if (!boot_complete) {
    std::cerr << "ERROR: Camera failed to complete bootup\n";
    return false;
  }

  // Step 6: Configure camera settings
  if (verbose) std::cout << "\n=== Configuring Camera ===\n";
  
  // Set frame rate (divisor = 4 for ~6.4 fps on MI0801)
  int frame_rate_response;
  set_frame_rate(4, frame_rate_response);
  if (verbose) std::cout << "Frame rate divisor set to 4\n";

  // Disable all filters initially
  int filter_ctrl_response;
  set_filter_ctrl(0x00, filter_ctrl_response);
  if (verbose) std::cout << "Filters disabled (FILTER_CTRL = 0x00)\n";

  // Set sensitivity factor to 1.00 (0x64 = 100%)
  int sens_factor_response;
  set_sens_factor(0x64, sens_factor_response);
  if (verbose) std::cout << "Sensitivity factor set to 1.00 (0x64)\n";

  // Set emissivity to 0.95 (95%)
  int emissivity_response;
  set_emissivity(0x5F, emissivity_response); // 0x5F = 95 decimal
  if (verbose) std::cout << "Emissivity set to 0.95 (95%)\n";

  // Flush buffers after configuration to ensure clean state
  if (verbose) std::cout << "Flushing buffers after configuration...\n";
  sp_flush(port, SP_BUF_BOTH);
  usleep(100000); // 100ms delay for settings to take effect

  if (verbose) std::cout << "\n=== Camera Initialization Complete ===\n\n";
  return true;
}

void SerialCommandSender::list_available_ports() {
  std::cout << "\nAvailable ports:\n";
  struct sp_port **port_list;
  int error = sp_list_ports(&port_list);
  if (error == SP_OK) {
    if (*port_list == NULL) {
      std::cout << "  No serial ports found.\n";
      return;
    }
    for (int i = 0; port_list[i] != NULL; i++) {
      std::cout << "  " << sp_get_port_name(port_list[i]) << ": "
                << sp_get_port_description(port_list[i]) << "\n";
    }
    sp_free_port_list(port_list);
  } else {
    std::cerr << "  Error listing ports: " << sp_last_error_message() << "\n";
  }
}

std::string
SerialCommandSender::generate_read_command(const std::string &reg_name) {
  auto it = MI48_REGMAP.find(reg_name);
  if (it == MI48_REGMAP.end()) {
    throw std::invalid_argument("Unknown register: " + reg_name);
  }
  uint8_t reg_addr = it->second;
  char command[19];
  snprintf(command, sizeof(command), "   #000ARREG%02XXXXX", reg_addr);
  return std::string(command);
}

std::string
SerialCommandSender::generate_write_command(const std::string &reg_name,
                                            uint8_t value) {
  auto it = MI48_REGMAP.find(reg_name);
  if (it == MI48_REGMAP.end()) {
    throw std::invalid_argument("Unknown register: " + reg_name);
  }
  uint8_t reg_addr = it->second;
  char command[21];
  snprintf(command, sizeof(command), "   #000CWREG%02X%02XXXXX", reg_addr,
           value);
  return std::string(command);
}

bool SerialCommandSender::send_command(const Command &cmd,
                                       int &response_value) {
  response_value = UINT8_MAX;

  std::string full_command =
      cmd.is_write ? generate_write_command(cmd.reg_name, cmd.value)
                   : generate_read_command(cmd.reg_name);

  std::cout << "Sending: '" << full_command
            << "' (Bytes: " << full_command.size() << ")\n";

  // Flush input buffer
  sp_flush(port, SP_BUF_INPUT);

  // Send the command
  int bytes_written =
      sp_blocking_write(port, full_command.data(), full_command.size(), 0);
  if (bytes_written != (int)full_command.size()) {
    std::cerr << "❌ ERROR: Failed to write full command. Wrote "
              << bytes_written << " of " << full_command.size() << " bytes.\n";
    return false;
  }

  // Read the response (for both read and write commands)
  std::cout << "Waiting for response (max " << TIMEOUT_MILLISECONDS
            << " ms) << is_write:" << cmd.is_write << "...\n";

  char buffer[256]; // Max response size
  int bytes_read = 0;
  std::string response_string_raw = "";
  long start_time = clock();

  // Loop to read data until timeout or sufficient data received
  while ((clock() - start_time) * 1000 / CLOCKS_PER_SEC <
         TIMEOUT_MILLISECONDS) {
    bytes_read = sp_nonblocking_read(port, buffer, sizeof(buffer) - 1);

    if (bytes_read > 0) {
      buffer[bytes_read] = '\0'; // Null-terminate
      response_string_raw.append(buffer, bytes_read);
      // For write commands, expect at least 12 bytes (#000ARREGXXYYYY); for
      // read, at least 1 byte
      if ((cmd.is_write && response_string_raw.size() >= 12) ||
          (!cmd.is_write && response_string_raw.size() >= 12)) {
        break;
      }
    }
    usleep(10000); // Small delay to prevent busy-waiting
  }

  // Post-process the response
  if (!response_string_raw.empty()) {
    // Trim leading/trailing whitespace
    std::string response_string = response_string_raw;
    size_t last = response_string.find_last_not_of(" \t\n\r");
    if (std::string::npos != last) {
      response_string = response_string.substr(0, last + 1);
    } else {
      response_string = "";
    }

    // For write commands, extract the 2 characters before the last 4 and
    // convert to hex
    if (!cmd.is_write && response_string.size() >= 12) {
      std::string value_str =
          response_string.substr(12, 2); // Extract XX from #000ARREGXXYYYY
      try {
        response_value = std::stoi(value_str, nullptr, 16);
        char hex_buf[5];
        snprintf(hex_buf, sizeof(hex_buf), "0x%02X", response_value);
        response_string = hex_buf; // Store as hex string, e.g., "0x0D"

        std::cout << "RES:" << response_string << " int:" << response_value
                  << std::endl;
        std::cout << value_str << std::endl;
      } catch (const std::exception &e) {
        std::cerr << "❌ ERROR: Failed to parse write response value: "
                  << e.what() << "\n";
      }
    } else {
      response_value = 0; // write success.
    }

    // Output raw bytes (displaying non-printable chars as hex)
    std::string raw_bytes_display = "";
    for (char c : response_string_raw) {
      if (c < 32 || c > 126) {
        char hex_buf[5];
        snprintf(hex_buf, sizeof(hex_buf), "\\x%02X", (unsigned char)c);
        raw_bytes_display += hex_buf;
      } else {
        raw_bytes_display += c;
      }
    }

    std::cout << "✅ RECEIVED RESPONSE (RAW BYTES): " << raw_bytes_display
              << "\n";
    std::cout << "✅ RECEIVED RESPONSE (STRING):   " << response_string << "\n";

    usleep(COMMAND_DELAY_MS * 1000); // Pause before next command
    return true;
  } else {
    std::cout << "❌ TIMEOUT: No response received after "
              << TIMEOUT_MILLISECONDS << " ms for command '" << full_command
              << "'.\n";
    return false;
  }
}

void SerialCommandSender::send_and_receive_serial_command() {
  if (!port) {
    std::cerr << "\n--- ERROR ---\n";
    std::cerr << "Serial port is not open. Call open_port() first.\n";
    return;
  }

  std::cout << "Starting sequence of " << COMMAND_LIST.size()
            << " commands...\n";
  try {
    bool received_data = false;
    std::cout << "\n--- Command Cycle ---\n";
    for (size_t i = 0; i < COMMAND_LIST.size(); ++i) {
      std::cout << "\n--- Command " << i + 1 << "/" << COMMAND_LIST.size()
                << " ---\n";
      int response_value;
      bool success = send_command(COMMAND_LIST[i], response_value);
      if (success && response_value != UINT8_MAX) {
        usleep(100000);
        received_data = true;
      }
    }
    if (!received_data) {
      std::cout << "\nNo data received in this cycle, stopping.\n";
    }
  } catch (const std::exception &e) {
    std::cerr << "\n--- COMMUNICATION ERROR ---\n";
    std::cerr << "An error occurred during communication: " << e.what() << "\n";
  }
}

/**
 * @brief Validates the checksum of a GFRA frame.
 * Checksum is sum of (length bytes + cmd bytes + data bytes) & 0xFFFF
 */
static bool validate_checksum(const std::string &data, size_t frame_start,
                              size_t frame_len, bool verbose = false) {
  if (frame_start + 8 + frame_len > data.size()) {
    if (verbose) std::cerr << "Checksum validation: insufficient data\n";
    return false;
  }

  // Parse expected checksum from last 4 bytes of frame body
  size_t checksum_start = frame_start + 8 + frame_len - 4;
  std::string checksum_str = data.substr(checksum_start, 4);
  uint16_t expected_checksum;
  try {
    expected_checksum = std::stoi(checksum_str, nullptr, 16);
  } catch (...) {
    if (verbose) std::cerr << "Checksum validation: failed to parse checksum string\n";
    return false;
  }

  // Calculate actual checksum: sum of length bytes + cmd + data (excluding checksum)
  uint32_t actual_checksum = 0;
  // Length bytes (4 bytes at offset 4)
  for (size_t i = frame_start + 4; i < frame_start + 8; ++i) {
    actual_checksum += (uint8_t)data[i];
  }
  // CMD + data (excluding final 4 checksum bytes)
  for (size_t i = frame_start + 8; i < checksum_start; ++i) {
    actual_checksum += (uint8_t)data[i];
  }
  actual_checksum &= 0xFFFF;

  if (actual_checksum != expected_checksum) {
    if (verbose) {
      std::cerr << "Checksum mismatch: calculated 0x" << std::hex << actual_checksum 
                << ", expected 0x" << expected_checksum << std::dec 
                << " (frame_len=" << frame_len << ")\n";
    }
    return false;
  }

  return true;
}

/**
 * @brief Loops reading raw serial data until no data is received, displaying
 * temperatures for 1-in-5 rows and columns.
 */
void SerialCommandSender::loop_on_read() {
#define DEBUG_HEADER 0
  if (!port) {
    std::cerr << "\n--- ERROR ---\n";
    std::cerr << "Serial port is not open. Call open_port() first.\n";
    return;
  }

  sp_return result = sp_flush(port, SP_BUF_BOTH);
  if (result != SP_OK) {
    std::cerr << "Failed to flush serial port: " << sp_last_error_message()
              << "\n";
    return;
  }

  //----------------------------------------------------------------------------------------------------------------------
  // GFRA FORMAT (from Python library):
  //
  //         |   CMD  |   RESERVED   |  HEADER |     DATA      | CHECKSUM |
  //  MI08   |  GFRA  |    80 * 2    |  80 * 2 |  80 * 62 * 2  |    4     |
  //         data at slice(320, 10240), body_len: 10248 (0x2808)
  //  MI16   |  GFRA  |  3 * 160 * 2 | 160 * 2 | 160 * 120 * 2 |    4     |
  //         data at slice(1280, 39680), body_len: 39688 (0x9B08)
  //  MI05   |  GFRA  |    50 * 2    |  50 * 2 |  50 * 50 * 2  |    4     |
  //         data at slice(200, 5200), body_len: 5208 (0x1458)
  //
  //----------------------------------------------------------------------------------------------------------------------

  const uint16_t rows = m_resolution.rows;
  const uint16_t cols = m_resolution.cols;
  const size_t pixel_data_size = (size_t)rows * cols * BYTES_PER_PIXEL;

  // Calculate reserved + header size based on resolution (matches Python GFRAData)
  // MI08 (80x62): reserved=160, header=160, data_start=320
  // MI16 (160x120): reserved=960, header=320, data_start=1280
  // MI05 (50x50): reserved=100, header=100, data_start=200
  size_t reserved_size;
  size_t header_row_size;
  if (cols == MI16_COLS) {
    // MI16
    reserved_size = MI16_RESERVED_SIZE;
    header_row_size = MI16_HEADER_ROW_SIZE;
  } else if (cols == MI05_COLS) {
    // MI05
    reserved_size = MI05_RESERVED_SIZE;
    header_row_size = MI05_HEADER_ROW_SIZE;
  } else {
    // MI08 (default)
    reserved_size = MI08_RESERVED_SIZE;
    header_row_size = MI08_HEADER_ROW_SIZE;
  }
  const size_t gfra_data_offset = reserved_size + header_row_size;  // Offset from GFRA cmd to pixel data
  const size_t frame_body_len = gfra_data_offset + pixel_data_size + CHECKSUM_SIZE;

  // Total expected: header + gfra_data_offset + pixel_data + checksum
  const size_t minimum_frame_size = FRAME_HEADER_SIZE + gfra_data_offset + pixel_data_size + CHECKSUM_SIZE;
  constexpr size_t MAX_BUFFER_SIZE = 65536;

  int consecutive_errors = 0;
  constexpr int MAX_CONSECUTIVE_ERRORS = 10;
  constexpr size_t MAX_ACCUMULATED_SIZE = MAX_BUFFER_SIZE * 2; // Limit accumulated buffer size

  try {
    m_streaming = true;
    char buffer[MAX_BUFFER_SIZE];
    std::string accumulated_data;

    while (m_streaming) {
      long start_time = clock();
      bool got_data_this_cycle = false;

      // Read data until we have a complete frame or timeout
      while ((clock() - start_time) * 1000 / CLOCKS_PER_SEC <
             TIMEOUT_MILLISECONDS) {
        int current_read =
            sp_nonblocking_read(port, buffer, sizeof(buffer) - 1);
        if (current_read > 0) {
          accumulated_data.append(buffer, current_read);
          got_data_this_cycle = true;

          // Prevent buffer overflow - if accumulated data is too large, flush and resync
          if (accumulated_data.size() > MAX_ACCUMULATED_SIZE) {
            std::cerr << "WARNING: Buffer overflow (" << accumulated_data.size() 
                      << " bytes), flushing and resyncing\n";
            sp_flush(port, SP_BUF_INPUT);
            accumulated_data.clear();
            consecutive_errors++;
            break;
          }

          // Check if we have enough data for a frame
          if (accumulated_data.size() >= minimum_frame_size) {
            break;
          }
        }
        usleep(5000);
      }

      if (!got_data_this_cycle) {
        // No data received at all this cycle
        std::cout << "\nNo data received, stopping stream.\n";
        m_streaming = false;
        continue;
      }

      // Try to find and parse a complete frame
      bool frame_processed = false;
      size_t search_start = 0;

      while (search_start + 12 <= accumulated_data.size()) {
        // Look for "   #" header
        size_t header_pos = accumulated_data.find("   #", search_start);
        if (header_pos == std::string::npos) {
          // No header found, discard data up to end minus potential partial header
          if (accumulated_data.size() > 3) {
            accumulated_data.erase(0, accumulated_data.size() - 3);
          }
          break;
        }

        // Check if we have enough bytes to read the length field
        if (header_pos + 8 > accumulated_data.size()) {
          // Partial header, keep waiting
          break;
        }

        // Validate hex digits for length
        bool valid_length = true;
        for (size_t i = header_pos + 4; i < header_pos + 8; ++i) {
          if (!isxdigit(accumulated_data[i])) {
            valid_length = false;
            break;
          }
        }

        if (!valid_length) {
          // Invalid length field, skip this header
          search_start = header_pos + 1;
          continue;
        }

        // Parse length
        std::string len_str = accumulated_data.substr(header_pos + 4, 4);
        size_t body_len;
        try {
          body_len = std::stoul(len_str, nullptr, 16);
        } catch (...) {
          search_start = header_pos + 1;
          continue;
        }

        // Check if we have the complete frame
        size_t total_frame_len = 8 + body_len;  // header(4) + length(4) + body
        if (header_pos + total_frame_len > accumulated_data.size()) {
          // Incomplete frame, wait for more data
          break;
        }

        // Check if this is a GFRA frame
        std::string cmd = accumulated_data.substr(header_pos + 8, 4);
        if (cmd != "GFRA") {
          // Not a GFRA frame, skip it
          accumulated_data.erase(0, header_pos + total_frame_len);
          search_start = 0;
          continue;
        }

#if DEBUG_HEADER
        std::cout << "Found GFRA at " << header_pos << ", body_len=" << body_len << "\n";
#endif

        // Validate checksum
        if (!validate_checksum(accumulated_data, header_pos, body_len, true)) {
          std::cerr << "WARNING: Checksum mismatch, discarding frame\n";
          accumulated_data.erase(0, header_pos + total_frame_len);
          search_start = 0;
          consecutive_errors++;
          if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
            std::cerr << "ERROR: Too many consecutive errors, stopping\n";
            m_streaming = false;
          }
          continue;
        }

        // Extract pixel data
        // Data starts at: header_pos + FRAME_HEADER_SIZE + gfra_data_offset
        size_t data_start = header_pos + FRAME_HEADER_SIZE + gfra_data_offset;
        if (data_start + pixel_data_size > accumulated_data.size()) {
          std::cerr << "WARNING: Insufficient pixel data, discarding frame\n";
          accumulated_data.erase(0, header_pos + total_frame_len);
          search_start = 0;
          consecutive_errors++;
          continue;
        }

        const char *data_ptr = accumulated_data.data();
        const size_t total_pixels = rows * cols;
        std::vector<float> temperatures(total_pixels);
        bool frame_valid = true;

        // Convert int16_t values to Celsius (little-endian)
        for (size_t i = 0; i < total_pixels; ++i) {
          size_t byte_index = data_start + (i * 2);
          uint16_t raw_data_val =
              (uint16_t)(((uint8_t)data_ptr[byte_index + 1] << 8) |
                         ((uint8_t)data_ptr[byte_index]));

          // Temperature in deciKelvin, convert to Celsius
          const float temp_celsius = ((float)raw_data_val / 10.0) + KELVIN_0;
          temperatures[i] = temp_celsius;
        }

        // Invoke callback if frame is valid
        if (frame_valid && m_frame_callback && temperatures.size() == total_pixels) {
          m_frame_callback(temperatures, rows, cols);
          consecutive_errors = 0;  // Reset error counter on success
        }

        frame_processed = true;

        // Remove processed frame from buffer
        accumulated_data.erase(0, header_pos + total_frame_len);
        search_start = 0;
        break;
      }

      if (!frame_processed && got_data_this_cycle) {
        consecutive_errors++;
        if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
          std::cerr << "ERROR: Too many consecutive frame errors\n";
          // Flush and try to recover
          sp_flush(port, SP_BUF_INPUT);
          accumulated_data.clear();
          consecutive_errors = 0;
        }
      }
    }
    std::cout << "\nStream ended.\n";
  } catch (const std::exception &e) {
    std::cerr << "\n--- COMMUNICATION ERROR ---\n";
    std::cerr << "An error occurred during communication: " << e.what() << "\n";
  }
}

void SerialCommandSender::stop_loop() {
  m_streaming = false;
}

bool SerialCommandSender::start_stream(bool with_header) {
  int response_value;
  uint8_t mode = with_header ? 0x02 : 0x22;
  return set_frame_mode(mode, response_value);
}

bool SerialCommandSender::stop_stream() {
  int response_value;
  return set_frame_mode(0x00, response_value);
}

bool SerialCommandSender::get_evk_test(int &response_value) {
  Command cmd = {"EVK_TEST", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_evk_id(int &response_value) {
  Command cmd = {"EVK_ID", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_powerup(int &response_value) {
  Command cmd = {"SENXOR_POWERUP", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::set_senxor_powerup(uint8_t value, int &response_value) {
  Command cmd = {"SENXOR_POWERUP", true, value};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_frame_mode(int &response_value) {
  Command cmd = {"FRAME_MODE", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::set_frame_mode(uint8_t value, int &response_value) {
  Command cmd = {"FRAME_MODE", true, value};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_fw_version_1(int &response_value) {
  Command cmd = {"FW_VERSION_1", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_fw_version_2(int &response_value) {
  Command cmd = {"FW_VERSION_2", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_frame_rate(int &response_value) {
  Command cmd = {"FRAME_RATE", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::set_frame_rate(uint8_t value, int &response_value) {
  Command cmd = {"FRAME_RATE", true, value};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_power_down_1(int &response_value) {
  Command cmd = {"POWER_DOWN_1", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::set_power_down_1(uint8_t value, int &response_value) {
  Command cmd = {"POWER_DOWN_1", true, value};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_status(int &response_value) {
  Command cmd = {"STATUS", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_power_down_2(int &response_value) {
  Command cmd = {"POWER_DOWN_2", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::set_power_down_2(uint8_t value, int &response_value) {
  Command cmd = {"POWER_DOWN_2", true, value};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_type(int &response_value) {
  Command cmd = {"SENXOR_TYPE", false, 0};
  bool succ = send_command(cmd, response_value);
  if (succ && response_value != UINT8_MAX) {
    m_resolution = FPA_SHAPE.at(response_value);
    std::cout << "Camera resolution: " << m_resolution.rows << "x"
              << m_resolution.cols << "\n";
  }
  return succ;
}

bool SerialCommandSender::get_module_type(int &response_value) {
  Command cmd = {"MODULE_TYPE", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_sens_factor(int &response_value) {
  Command cmd = {"SENS_FACTOR", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::set_sens_factor(uint8_t value, int &response_value) {
  Command cmd = {"SENS_FACTOR", true, value};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_emissivity(int &response_value) {
  Command cmd = {"EMISSIVITY", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::set_emissivity(uint8_t value, int &response_value) {
  Command cmd = {"EMISSIVITY", true, value};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_offset_corr(int &response_value) {
  Command cmd = {"OFFSET_CORR", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::set_offset_corr(uint8_t value, int &response_value) {
  Command cmd = {"OFFSET_CORR", true, value};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_filter_ctrl(int &response_value) {
  Command cmd = {"FILTER_CTRL", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::set_filter_ctrl(uint8_t value, int &response_value) {
  Command cmd = {"FILTER_CTRL", true, value};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_filter_1_lsb(int &response_value) {
  Command cmd = {"FILTER_1_LSB", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::set_filter_1_lsb(uint8_t value, int &response_value) {
  Command cmd = {"FILTER_1_LSB", true, value};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_filter_1_msb(int &response_value) {
  Command cmd = {"FILTER_1_MSB", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::set_filter_1_msb(uint8_t value, int &response_value) {
  Command cmd = {"FILTER_1_MSB", true, value};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_filter_2(int &response_value) {
  Command cmd = {"FILTER_2", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::set_filter_2(uint8_t value, int &response_value) {
  Command cmd = {"FILTER_2", true, value};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_flash_ctrl(int &response_value) {
  Command cmd = {"FLASH_CTRL", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::set_flash_ctrl(uint8_t value, int &response_value) {
  Command cmd = {"FLASH_CTRL", true, value};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_id_0(int &response_value) {
  Command cmd = {"SENXOR_ID_0", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_id_1(int &response_value) {
  Command cmd = {"SENXOR_ID_1", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_id_2(int &response_value) {
  Command cmd = {"SENXOR_ID_2", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_id_3(int &response_value) {
  Command cmd = {"SENXOR_ID_3", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_id_4(int &response_value) {
  Command cmd = {"SENXOR_ID_4", false, 0};
  return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_id_5(int &response_value) {
  Command cmd = {"SENXOR_ID_5", false, 0};
  return send_command(cmd, response_value);
}