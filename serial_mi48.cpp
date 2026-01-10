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
// Assuming these are defined elsewhere
#define TIMEOUT_MILLISECONDS 1000
#define COMMAND_DELAY_MS 10
#define KELVIN_0 -273.15

// Constructor and destructor
SerialCommandSender::SerialCommandSender()
    : port(nullptr), m_resolution(DEFAULT_ROWS, DEFAULT_COLS) {}

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
 * @brief Loops reading raw serial data until no data is received, displaying
 * temperatures for 1-in-5 rows and columns.
 */
void SerialCommandSender::loop_on_read() {
#define DEBUG_HEADER 1
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
  // GFRA FORMAT:
  //
  //         |   CMD  |   RESERVED   |  HEADER |     DATA      | CHECKSUM |
  //  MI08   |  GFRA  |    80 * 2    |  80 * 2 |  80 * 62 * 2  |    4     |
  //  data_len: 10240 body_len: 10248(0x2808) MI16   |  GFRA  |  3 * 160 * 2 |
  //  160 * 2 | 160 * 120 * 2 |    4     | data_len: 39680 body_len:
  //  39688(0x9B08) MI05   |  GFRA  |    50 * 2    |  50 * 2 |  50 * 50 * 2  |
  //  4     | data_len:  5200 body_len: 5208(0x1458)
  //
  //----------------------------------------------------------------------------------------------------------------------

  const uint16_t rows = m_resolution.first;
  const uint16_t cols = m_resolution.second;
  size_t expected_data_size = (size_t)rows * cols * 2; // 2 bytes per pixel
  size_t minimum_size =
      12 + (80 * 4) + expected_data_size; // "   #xxxxGFRA" (12 bytes) + 80
                                          // words (320 bytes) + frame data
  constexpr size_t MAX_BUFFER_SIZE = 19201;

  try {
    bool received_data = true;
    char buffer[MAX_BUFFER_SIZE];
    while (received_data) {
      received_data = false;
      std::string response_string_raw;
      response_string_raw.clear(); // Explicitly clear the string
      long start_time = clock();

      // Read data until "   #xxxxGFRA" + 80 words + frame data or timeout
      while ((clock() - start_time) * 1000 / CLOCKS_PER_SEC <
             TIMEOUT_MILLISECONDS) {
        int current_read =
            sp_nonblocking_read(port, buffer, sizeof(buffer) - 1);
        if (current_read > 0) {
          response_string_raw.append(buffer, current_read);
          received_data = true;
          // Check if we have "   #xxxxGFRA" (3 spaces + # + 4 hex digits +
          // GFRA) and enough data after it This matches the USB protocol format
          // from the Python implementation
          for (size_t i = 0; i <= response_string_raw.size() - 12; ++i) {
            // Look for "   #" (3 spaces followed by #)
            if (i + 12 <= response_string_raw.size() &&
                response_string_raw[i] == ' ' &&
                response_string_raw[i + 1] == ' ' &&
                response_string_raw[i + 2] == ' ' &&
                response_string_raw[i + 3] == '#' &&
                isxdigit(response_string_raw[i + 4]) &&
                isxdigit(response_string_raw[i + 5]) &&
                isxdigit(response_string_raw[i + 6]) &&
                isxdigit(response_string_raw[i + 7]) &&
                response_string_raw.substr(i + 8, 4) == "GFRA") {
              size_t required_size =
                  i + 12 + (80 * 4) +
                  expected_data_size; // Data from start of "   #xxxxGFRA"
              if (response_string_raw.size() >= required_size) {
                minimum_size =
                    required_size; // Update minimum_size to include offset
                break;
              }
            }
          }
          if (response_string_raw.size() >= minimum_size) {
            break;
          }
        }
        usleep(10000);
      }

      // Data Processing
      if (received_data && response_string_raw.size() >= minimum_size) {
        const char *data_ptr = response_string_raw.data();
        const size_t total_pixels = rows * cols;
        size_t start_index = 0;
        size_t header_size = 0;
        bool found = false;

        // Find "   #xxxxGFRA" (3 spaces + # + 4 hex digits + GFRA)
        for (size_t i = 0; i <= response_string_raw.size() - 12; ++i) {
          if (response_string_raw[i] == ' ' &&
              response_string_raw[i + 1] == ' ' &&
              response_string_raw[i + 2] == ' ' &&
              response_string_raw[i + 3] == '#' &&
              isxdigit(response_string_raw[i + 4]) &&
              isxdigit(response_string_raw[i + 5]) &&
              isxdigit(response_string_raw[i + 6]) &&
              isxdigit(response_string_raw[i + 7]) &&
              response_string_raw.substr(i + 8, 4) == "GFRA") {
#ifdef DEBUG_HEADER
            std::cout << "   #xxxxGFRA found at index: " << i
                      << " (xxxx = " << response_string_raw.substr(i + 4, 4)
                      << ")\n";
#endif
            header_size =
                i + 12 +
                (80 * 4); // Header ends after "   #xxxxGFRA" + 80 words
            start_index = header_size; // Frame data starts immediately after
            found = true;
            break;
          }
        }

        if (!found) {
          std::cout << "TIMEOUT: '   #xxxxGFRA' not found in received data\n";
          continue;
        }

        std::cout << "Header ends at index: " << header_size << "\n";

#ifdef DEBUG_HEADER
        // Print header bytes in hex
        for (size_t i = 0; i < header_size && i < response_string_raw.size();
             ++i) {
          std::cout << std::hex << std::setfill('0');
          std::cout << std::setw(2) << (int)(uint8_t)data_ptr[i] << ",";
        }
        std::cout << "\n DATA \n";
        // Print first 450 bytes of data in hex
        for (size_t i = start_index;
             i < start_index + 450 && i < response_string_raw.size(); ++i) {
          std::cout << std::hex << std::setfill('0');
          std::cout << std::setw(2) << (int)(uint8_t)data_ptr[i] << ",";
        }
        std::cout << "\n ";
#endif

        // Check if enough data remains after start_index
        if (start_index + expected_data_size > response_string_raw.size()) {
          std::cout << "TIMEOUT: Insufficient data after header ("
                    << expected_data_size << " bytes needed, "
                    << (response_string_raw.size() - start_index)
                    << " bytes available)\n";
          continue;
        }

        std::vector<float> temperatures(total_pixels);
        // Convert int16_t values to Celsius (little-endian)
        for (size_t i = 0; i < total_pixels; ++i) {
          size_t byte_index = start_index + (i * 2);
          if (byte_index + 1 >= response_string_raw.size()) {
            std::cerr << "❌ ERROR: Insufficient data for pixel " << i
                      << " at byte " << byte_index << "\n";
            break;
          }

          u_int16_t raw_data_val =
              (u_int16_t)(((u_int16_t)(data_ptr[byte_index + 1]) << 8) |
                          ((u_int16_t)data_ptr[byte_index] & 0x00ff));
          if (raw_data_val > 5000) {
            // Set up hex formatting once
            std::cout << std::hex << std::setfill('0');
            // 1. Display High Byte (data_ptr[byte_index + 1])
            std::cout << "High Byte: 0x" << std::setw(2)
                      << (int)(unsigned char)data_ptr[byte_index + 1];
            // 2. Display Low Byte (data_ptr[byte_index])
            std::cout << " | Low Byte: 0x" << std::setw(2)
                      << (int)(unsigned char)data_ptr[byte_index];
            // 3. Display Combined raw_data_val
            std::cout << " | Combined Raw: 0x" << std::setw(4) << raw_data_val;
            // 4. Display Combined raw_data_val (Decimal)
            // Switch to decimal mode and use a standard fill/width
            std::cout << std::dec << std::setfill(' ');
            std::cout << " | Combined Dec: " << std::setw(5)
                      << raw_data_val; // Use a reasonable decimal width
            // Reset formatting
            std::cout << "-----------------------------------index:"
                      << byte_index;
            std::cout << "\n";
            raw_data_val = temperatures[i - 1];
            response_string_raw.clear();
            break;
          }

          const float temp_celsius = ((float)raw_data_val / 10.0) + KELVIN_0;
          temperatures[i] = temp_celsius;
        }

        if ((m_frame_callback) &&
            (total_pixels > 0 && temperatures.size() == total_pixels)) {
          // Invoke the callback with the new frame data
          m_frame_callback(temperatures, rows, cols);
        }
        response_string_raw.clear();
        // response_string_raw.erase(0, header_size + expected_data_size);
        usleep(COMMAND_DELAY_MS * 1000);
      } else {
        std::cout << "TIMEOUT: No full frame (" << minimum_size
                  << " bytes) received after " << TIMEOUT_MILLISECONDS
                  << " ms. Received " << response_string_raw.size()
                  << " bytes.\n";
      }
    }
    std::cout << "\nNo data received in the last read cycle, stopping.\n";
  } catch (const std::exception &e) {
    std::cout << "\033[2J\033[H";
    std::cerr << "\n--- COMMUNICATION ERROR ---\n";
    std::cerr << "An error occurred during communication: " << e.what() << "\n";
  }
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
    std::cout << "Camera resolution: " << m_resolution.first << "x"
              << m_resolution.second << "\n";
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