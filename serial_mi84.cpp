#include "serial_mi48.hpp"
#include <unistd.h>
#include <ctime>
#include <numeric>
#include <map>
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>
#include <string>
#include <cctype>
#include <iomanip>
// Assuming these are defined elsewhere
#define TIMEOUT_MILLISECONDS 1000
#define COMMAND_DELAY_MS 100
#define KELVIN_0 -273.15

// Constructor and destructor
SerialCommandSender::SerialCommandSender() : port(nullptr), m_resolution(DEFAULT_ROWS, DEFAULT_COLS) {}

SerialCommandSender::~SerialCommandSender()
{
    close_port();
}

// Implementation of methods (unchanged from previous version)
bool SerialCommandSender::open_port(const std::string &port_path)
{
    std::cout << "Attempting to connect to port: " << port_path << " at " << BAUD_RATE << " baud...\n";

    enum sp_return result = sp_get_port_by_name(port_path.c_str(), &port);
    if (result != SP_OK)
    {
        std::cerr << "\n--- ERROR ---\n";
        std::cerr << "Could not find serial port " << port_path << ".\n";
        list_available_ports();
        return false;
    }

    result = sp_open(port, SP_MODE_READ_WRITE);
    if (result != SP_OK)
    {
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
    std::cout << "Pausing for 2 seconds to allow device initialization...\n";
    usleep(2000000);
    return true;
}

void SerialCommandSender::close_port()
{
    if (port != nullptr)
    {
        sp_close(port);
        sp_free_port(port);
        port = nullptr;
        std::cout << "\nSerial port closed.\n";
    }
}

void SerialCommandSender::list_available_ports()
{
    std::cout << "\nAvailable ports:\n";
    struct sp_port **port_list;
    int error = sp_list_ports(&port_list);
    if (error == SP_OK)
    {
        if (*port_list == NULL)
        {
            std::cout << "  No serial ports found.\n";
            return;
        }
        for (int i = 0; port_list[i] != NULL; i++)
        {
            std::cout << "  " << sp_get_port_name(port_list[i])
                      << ": " << sp_get_port_description(port_list[i]) << "\n";
        }
        sp_free_port_list(port_list);
    }
    else
    {
        std::cerr << "  Error listing ports: " << sp_last_error_message() << "\n";
    }
}

std::string SerialCommandSender::generate_read_command(const std::string &reg_name)
{
    auto it = MI48_REGMAP.find(reg_name);
    if (it == MI48_REGMAP.end())
    {
        throw std::invalid_argument("Unknown register: " + reg_name);
    }
    uint8_t reg_addr = it->second;
    char command[18];
    snprintf(command, sizeof(command), "#000CRREG%02XXXXXXX", reg_addr);
    return std::string(command);
}

std::string SerialCommandSender::generate_write_command(const std::string &reg_name, uint8_t value)
{
    auto it = MI48_REGMAP.find(reg_name);
    if (it == MI48_REGMAP.end())
    {
        throw std::invalid_argument("Unknown register: " + reg_name);
    }
    uint8_t reg_addr = it->second;
    char command[18];
    snprintf(command, sizeof(command), "#000CWREG%02X%02XXXXX", reg_addr, value);
    return std::string(command);
}

bool SerialCommandSender::send_command(const Command &cmd, int &response_value)
{
    response_value = UINT8_MAX;

    std::string full_command = cmd.is_write ? generate_write_command(cmd.reg_name, cmd.value) : generate_read_command(cmd.reg_name);

    std::cout << "Sending: '" << full_command << "' (Bytes: " << full_command.size() << ")\n";

    // Flush input buffer
    sp_flush(port, SP_BUF_INPUT);

    // Send the command
    int bytes_written = sp_blocking_write(port, full_command.data(), full_command.size(), 0);
    if (bytes_written != (int)full_command.size())
    {
        std::cerr << "❌ ERROR: Failed to write full command. Wrote "
                  << bytes_written << " of " << full_command.size() << " bytes.\n";
        return false;
    }

    // Read the response (for both read and write commands)
    std::cout << "Waiting for response (max " << TIMEOUT_MILLISECONDS << " ms)...\n";

    char buffer[256]; // Max response size
    int bytes_read = 0;
    std::string response_string_raw = "";
    long start_time = clock();

    // Loop to read data until timeout or sufficient data received
    while ((clock() - start_time) * 1000 / CLOCKS_PER_SEC < TIMEOUT_MILLISECONDS)
    {
        bytes_read = sp_nonblocking_read(port, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0)
        {
            buffer[bytes_read] = '\0'; // Null-terminate
            response_string_raw.append(buffer, bytes_read);
            // For write commands, expect at least 12 bytes (#000ARREGXXYYYY); for read, at least 1 byte
            if ((cmd.is_write && response_string_raw.size() >= 12) || (!cmd.is_write && response_string_raw.size() >= 1))
            {
                break;
            }
        }
        usleep(10000); // Small delay to prevent busy-waiting
    }

    // Post-process the response
    if (!response_string_raw.empty())
    {
        // Trim leading/trailing whitespace
        std::string response_string = response_string_raw;
        size_t last = response_string.find_last_not_of(" \t\n\r");
        if (std::string::npos != last)
        {
            response_string = response_string.substr(0, last + 1);
        }
        else
        {
            response_string = "";
        }

        // For write commands, extract the 2 characters before the last 4 and convert to hex
        if (!cmd.is_write && response_string.size() >= 12)
        {
            std::string value_str = response_string.substr(12, 2); // Extract XX from #000ARREGXXYYYY
            try
            {
                response_value = std::stoi(value_str, nullptr, 16);
                char hex_buf[5];
                snprintf(hex_buf, sizeof(hex_buf), "0x%02X", response_value);
                response_string = hex_buf; // Store as hex string, e.g., "0x0D"

                std::cout << "RES:" << response_string << " int:" << response_value << std::endl;
                std::cout << value_str << std::endl;
            }
            catch (const std::exception &e)
            {
                std::cerr << "❌ ERROR: Failed to parse write response value: " << e.what() << "\n";
            }
        }
        else
        {
            response_value = 0; // write success.
        }

        // Output raw bytes (displaying non-printable chars as hex)
        std::string raw_bytes_display = "";
        for (char c : response_string_raw)
        {
            if (c < 32 || c > 126)
            {
                char hex_buf[5];
                snprintf(hex_buf, sizeof(hex_buf), "\\x%02X", (unsigned char)c);
                raw_bytes_display += hex_buf;
            }
            else
            {
                raw_bytes_display += c;
            }
        }

        std::cout << "✅ RECEIVED RESPONSE (RAW BYTES): " << raw_bytes_display << "\n";
        std::cout << "✅ RECEIVED RESPONSE (STRING):   " << response_string << "\n";

        usleep(COMMAND_DELAY_MS * 1000); // Pause before next command
        return true;
    }
    else
    {
        std::cout << "❌ TIMEOUT: No response received after " << TIMEOUT_MILLISECONDS
                  << " ms for command '" << full_command << "'.\n";
        return false;
    }
}

void SerialCommandSender::send_and_receive_serial_command()
{
    if (!port)
    {
        std::cerr << "\n--- ERROR ---\n";
        std::cerr << "Serial port is not open. Call open_port() first.\n";
        return;
    }

    std::cout << "Starting sequence of " << COMMAND_LIST.size() << " commands...\n";
    try
    {
        bool received_data = false;
        std::cout << "\n--- Command Cycle ---\n";
        for (size_t i = 0; i < COMMAND_LIST.size(); ++i)
        {
            std::cout << "\n--- Command " << i + 1 << "/" << COMMAND_LIST.size() << " ---\n";
            int response_value;
            bool success = send_command(COMMAND_LIST[i], response_value);
            if (success && response_value != UINT8_MAX)
            {
                received_data = true;
            }
        }
        if (!received_data)
        {
            std::cout << "\nNo data received in this cycle, stopping.\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "\n--- COMMUNICATION ERROR ---\n";
        std::cerr << "An error occurred during communication: " << e.what() << "\n";
    }
}

/**
 * @brief Loops reading raw serial data until no data is received, displaying frames as colored ASCII art on a fixed screen.
 */

// void SerialCommandSender::loop_on_read() {
//     if (!port) {
//         std::cerr << "\n--- ERROR ---\n";
//         std::cerr << "Serial port is not open. Call open_port() first.\n";
//         return;
//     }

//     const uint16_t rows = m_resolution.first;
//     const uint16_t cols = m_resolution.second;
//     size_t expected_data_size = (size_t)rows * cols * sizeof(int16_t);
//     constexpr size_t MAX_BUFFER_SIZE = 19201;

//     // Define 5 grayscale levels using ASCII characters
//     const char grayscale_levels[5] = {' ', '.', ':', '#', '@'};

//     try {
//         bool received_data = true;
//         while (received_data) {
//             received_data = false;
//             char buffer[MAX_BUFFER_SIZE];
//             std::string response_string_raw = "";
//             long start_time = clock();

//             // Read data until full frame or timeout
//             while ((clock() - start_time) * 1000 / CLOCKS_PER_SEC < TIMEOUT_MILLISECONDS) {
//                 int current_read = sp_nonblocking_read(port, buffer, sizeof(buffer) - 1);
//                 if (current_read > 0) {
//                     response_string_raw.append(buffer, current_read);
//                     received_data = true;
//                     if (response_string_raw.size() >= expected_data_size + 9) { // Minimum size for #xxxxGFRA + data
//                         break;
//                     }
//                 }
//                 usleep(10000);
//             }

//             // Clear screen and move cursor to top-left
//             std::cout << "\033[2J\033[H";

//             // Data Processing
//             if (received_data && response_string_raw.size() >= expected_data_size + 9) {
//                 const char *data_ptr = response_string_raw.data();
//                 const size_t total_pixels = rows * cols;
//                 size_t start_index = 0;
//                 size_t header_size = 0;

//                 // Search for #xxxxGFRA
//                 bool found = false;
//                 for (size_t i = 0; i <= response_string_raw.size() - 9; ++i) {
//                     if (response_string_raw[i] == '#' &&
//                         isxdigit(response_string_raw[i+1]) && isxdigit(response_string_raw[i+2]) &&
//                         isxdigit(response_string_raw[i+3]) && isxdigit(response_string_raw[i+4]) &&
//                         response_string_raw.substr(i+5, 4) == "GFRA") {
//                         std::cout << "#xxxxGFRA found at index: " << i << " (xxxx = "
//                                   << response_string_raw.substr(i+1, 4) << ")\n";
//                         header_size = i + 9; // Header ends after #xxxxGFRA
//                         start_index = header_size; // Frame data starts immediately after
//                         found = true;
//                         break;
//                     }
//                 }

//                 if (!found) {
//                     header_size = response_string_raw.size() - expected_data_size; // Fallback
//                     start_index = header_size;
//                     std::cout << "header_size:" << header_size << "\n";
//                 }

//                 // Check if enough data remains after start_index
//                 if (start_index + expected_data_size > response_string_raw.size()) {
//                     std::cout << "TIMEOUT: Insufficient data after header (" << expected_data_size
//                               << " bytes needed, " << (response_string_raw.size() - start_index)
//                               << " bytes available)\n";
//                     continue;
//                 }

//                 std::vector<double> temperatures(total_pixels);

//                 // Convert int16_t values to Kelvin
//                 for (size_t i = 0; i < total_pixels; ++i) {
//                     size_t byte_index = start_index + (i * 2);
//                     if (byte_index + 1 >= response_string_raw.size()) {
//                         std::cerr << "❌ ERROR: Insufficient data for pixel " << i << " at byte " << byte_index << "\n";
//                         break;
//                     }
//                     int16_t raw_data_val = (int16_t)(((uint8_t)data_ptr[byte_index + 1] << 8) | (uint8_t)data_ptr[byte_index]);
//                     const double temp_celsius = (double)raw_data_val / 10.0;
//                     temperatures[i] = temp_celsius + KELVIN_0;
//                 }

//                 // Display frame metadata
//                 std::cout << "Thermal Frame: " << rows << "x" << cols << " (" << response_string_raw.size() << " bytes)\n";
//                 std::cout << "Header Size: " << header_size << " bytes\n";
//                 std::cout << "Total Pixels: " << total_pixels << "\n";

//                 // Calculate statistics
//                 if (total_pixels > 0 && temperatures.size() == total_pixels) {
//                     auto min_it = std::min_element(temperatures.begin(), temperatures.end());
//                     auto max_it = std::max_element(temperatures.begin(), temperatures.end());
//                     double sum = std::accumulate(temperatures.begin(), temperatures.end(), 0.0);
//                     double avg = sum / total_pixels;

//                     std::cout << "\n--- Frame Statistics (Kelvin) ---\n";
//                     std::cout << "MAX Temperature: " << *max_it << "\n";
//                     std::cout << "MIN Temperature: " << *min_it << "\n";
//                     std::cout << "AVG Temperature: " << avg << "\n";

//                     // Map temperatures to grayscale levels
//                     double temp_min = *min_it;
//                     double temp_max = *max_it;
//                     double temp_range = temp_max - temp_min;
//                     if (temp_range == 0) temp_range = 1.0; // Avoid division by zero

//                     std::cout << "\n--- Thermal Image (ASCII Grayscale) ---\n";
//                     for (uint16_t r = 0; r < rows; ++r) {
//                         for (uint16_t c = 0; c < cols; ++c) {
//                             size_t index = (size_t)r * cols + c;
//                             if (index < temperatures.size()) {
//                                 // Normalize temperature to [0,1]
//                                 double normalized = (temperatures[index] - temp_min) / temp_range;
//                                 // Map to 5 levels (0 to 4)
//                                 int level = static_cast<int>(normalized * 4.999); // 4.999 to ensure proper rounding
//                                 level = std::max(0, std::min(4, level)); // Clamp to [0,4]
//                                 std::cout << grayscale_levels[level];
//                             }
//                         }
//                         std::cout << "\n";
//                     }
//                 }

//                 response_string_raw.erase(0, header_size + expected_data_size);
//                 usleep(COMMAND_DELAY_MS * 1000);
//             } else {
//                 std::cout << "TIMEOUT: No full frame (" << expected_data_size << " bytes) received after " << TIMEOUT_MILLISECONDS
//                           << " ms. Received " << response_string_raw.size() << " bytes.\n";
//             }
//         }
//         std::cout << "\nNo data received in the last read cycle, stopping.\n";
//     } catch (const std::exception& e) {
//         std::cout << "\033[2J\033[H";
//         std::cerr << "\n--- COMMUNICATION ERROR ---\n";
//         std::cerr << "An error occurred during communication: " << e.what() << "\n";
//     }
// }

/**
 * @brief Loops reading raw serial data until no data is received, displaying temperatures for 1-in-5 rows and columns.
 */
void SerialCommandSender::loop_on_read()
{
    if (!port)
    {
        std::cerr << "\n--- ERROR ---\n";
        std::cerr << "Serial port is not open. Call open_port() first.\n";
        return;
    }

    const uint16_t rows = m_resolution.first;
    const uint16_t cols = m_resolution.second;
    size_t expected_data_size = (size_t)rows * cols * sizeof(int16_t);
    constexpr size_t MAX_BUFFER_SIZE = 19201;

    try
    {
        bool received_data = true;
        while (received_data)
        {
            received_data = false;
            char buffer[MAX_BUFFER_SIZE];
            int bytes_read = 0;
            std::string response_string_raw = "";
            long start_time = clock();

            // Read data until full frame or timeout
            while ((clock() - start_time) * 1000 / CLOCKS_PER_SEC < TIMEOUT_MILLISECONDS)
            {
                int current_read = sp_nonblocking_read(port, buffer, sizeof(buffer) - 1);
                if (current_read > 0)
                {
                    response_string_raw.append(buffer, current_read);
                    received_data = true;
                    if (response_string_raw.size() >= expected_data_size + 9 + 161)
                    { // Minimum size for #xxxxGFRA + data
                        break;
                    }
                }
                else
                {
                    std::cout << "NO DATA " << std::endl;
                }
                usleep(10000);
            }

            // Clear screen and move cursor to top-left
            std::cout << "\033[2J\033[H";

            // Data Processing
            if (received_data && response_string_raw.size() >= expected_data_size + 9)
            {
                const char *data_ptr = response_string_raw.data();
                const size_t total_pixels = rows * cols;
                size_t start_index = 0;
                size_t header_size = 0;

                // Search for #xxxxGFRA
                bool found = false;
                for (size_t i = 0; i <= response_string_raw.size() - 9; ++i)
                {
                    if (response_string_raw[i] == '#' &&
                        isxdigit(response_string_raw[i + 1]) && isxdigit(response_string_raw[i + 2]) &&
                        isxdigit(response_string_raw[i + 3]) && isxdigit(response_string_raw[i + 4]) &&
                        response_string_raw.substr(i + 5, 4) == "GFRA")
                    {
                        std::cout << "#xxxxGFRA found at index: " << i << " (xxxx = "
                                  << response_string_raw.substr(i + 1, 4) << ")\n";
                        size_t j = i + 9; // Start after #xxxxGFRA
                        size_t zero_count = 0;
                        // Scan for a sequence of at least 10 zeros
                        while (j < response_string_raw.size() && zero_count < 10)
                        {
                            if (response_string_raw[j] == 0)
                                zero_count++;
                            else
                                zero_count = 0; // Reset count if non-zero is found
                            j++;
                        }
                        // Continue until a non-zero character is found
                        while (j < response_string_raw.size() && response_string_raw[j] == 0)
                            j++;
                        if (j < response_string_raw.size())
                        {
                            header_size = j; // Header ends at the first non-zero character
                            start_index = j; // Frame data starts at the first non-zero character
                            std::cout << "Header ends at index (first non-zero after >=10 zeros): " << header_size << "\n";
                        }
                        else
                        {
                            std::cerr << "❌ ERROR: No non-zero character found after >=10 zeros\n";
                            header_size = i + 9; // Fallback to end of #xxxxGFRA
                            start_index = header_size;
                        }
                        found = true;
                        break;
                    }
                }
                if (!found)
                    break;

                // header_size = response_string_raw.size() - expected_data_size - 4; // Fallback
                // start_index = header_size;
                std::cout << "header_size:" << header_size << "\n";

                for (size_t i = 0; i < start_index; ++i)
                {
                    std::cout << std::hex << std::setfill('0');
                    std::cout << std::setw(2) << (int)(uint8_t)data_ptr[i] << ",";
                }

                std::cout << "\n DATA \n";

                for (size_t i = start_index; i < start_index + 450; ++i)
                {
                    std::cout << std::hex << std::setfill('0');
                    std::cout << std::setw(2) << (int)(uint8_t)data_ptr[i] << ",";
                }

                std::cout << "\n ";

                // Check if enough data remains after start_index
                if (start_index + expected_data_size > response_string_raw.size())
                {
                    std::cout << "TIMEOUT: Insufficient data after header (" << expected_data_size
                              << " bytes needed, " << (response_string_raw.size() - start_index)
                              << " bytes available)\n";
                    // received_data = false;
                    continue;
                }

                std::vector<float> temperatures(total_pixels);
                u_int16_t max_temp = 0;
                u_int16_t min_temp = UINT16_MAX;
                // Convert int16_t values to Kelvin
                for (size_t i = 0; i < total_pixels; ++i)
                {
                    size_t byte_index = start_index + (i * 2);
                    if (byte_index + 1 >= response_string_raw.size())
                    {
                        std::cerr << "❌ ERROR: Insufficient data for pixel " << i << " at byte " << byte_index << "\n";
                        break;
                    }

                    u_int16_t raw_data_val = (u_int16_t)(((u_int16_t)(data_ptr[byte_index + 1]) << 8) | ((u_int16_t)data_ptr[byte_index] & 0x00ff));
                    if (raw_data_val > 5000)
                    {
                        // Set up hex formatting once
                        std::cout << std::hex << std::setfill('0');
                        // 1. Display High Byte (data_ptr[byte_index + 1])
                        std::cout << "High Byte: 0x" << std::setw(2) << (int)(unsigned char)data_ptr[byte_index + 1];
                        // 2. Display Low Byte (data_ptr[byte_index])
                        std::cout << " | Low Byte: 0x" << std::setw(2) << (int)(unsigned char)data_ptr[byte_index];
                        // 3. Display Combined raw_data_val
                        std::cout << " | Combined Raw: 0x" << std::setw(4) << raw_data_val;
                        // 4. Display Combined raw_data_val (Decimal)
                        // Switch to decimal mode and use a standard fill/width
                        std::cout << std::dec << std::setfill(' ');
                        std::cout << " | Combined Dec: " << std::setw(5) << raw_data_val; // Use a reasonable decimal width

                        // Reset formatting
                        std::cout << "-----------------------------------index:" << byte_index;
                        std::cout << "\n";
                        break;
                    }

                    if (min_temp > raw_data_val)
                        min_temp = raw_data_val;
                    if (max_temp < raw_data_val)
                        max_temp = raw_data_val;
                    const float temp_celsius = (float)raw_data_val / 10.0;
                    temperatures[i] = temp_celsius + KELVIN_0;
                }
                std::cout << std::dec << "min_temp: " << min_temp << "  max_temp: " << max_temp << std::endl;
                // Display frame metadata
                std::cout << "Thermal Frame: " << rows << "x" << cols << " (" << response_string_raw.size() << " bytes)\n";
                std::cout << "Header Size: " << header_size << " bytes\n";
                std::cout << "Total Pixels: " << total_pixels << "\n";
                // Calculate and display statistics
                if (total_pixels > 0 && temperatures.size() == total_pixels)
                {
                    auto min_it = std::min_element(temperatures.begin(), temperatures.end());
                    auto max_it = std::max_element(temperatures.begin(), temperatures.end());
                    double sum = std::accumulate(temperatures.begin(), temperatures.end(), 0.0);
                    double avg = sum / total_pixels;

                    std::cout << "\n--- Frame Statistics (Kelvin) ---\n";
                    std::cout << "MAX Temperature: " << *max_it << "\n";
                    std::cout << "MIN Temperature: " << *min_it << "\n";
                    std::cout << "AVG Temperature: " << avg << "\n";

                    // Display 1-in-5 temperature samples
                    std::cout << "\n--- 1-in-5 Temperature Sample (K) ---\n";
                    for (uint16_t r = 0; r < rows; r += 1)
                    {
                        std::cout << "Row " << r << ": ";
                        for (uint16_t c = 0; c < cols; c += 1)
                        {
                            size_t index = (size_t)r * cols + c;
                            if (index < temperatures.size())
                            {
                                std::cout << "[" << c << "]=" << temperatures[index] << " ";
                            }
                        }
                        std::cout << "\n";
                    }
                }

                response_string_raw.erase(0, header_size + expected_data_size);
                usleep(COMMAND_DELAY_MS * 1000);
            }
            else
            {
                std::cout << "TIMEOUT: No full frame (" << expected_data_size << " bytes) received after " << TIMEOUT_MILLISECONDS
                          << " ms. Received " << response_string_raw.size() << " bytes.\n";
                // received_data = false;
            }
        }
        std::cout << "\nNo data received in the last read cycle, stopping.\n";
    }
    catch (const std::exception &e)
    {
        std::cout << "\033[2J\033[H";
        std::cerr << "\n--- COMMUNICATION ERROR ---\n";
        std::cerr << "An error occurred during communication: " << e.what() << "\n";
    }
}

bool SerialCommandSender::start_stream(bool with_header)
{
    int response_value;
    uint8_t mode = with_header ? 0x02 : 0x22;
    return set_frame_mode(mode, response_value);
}

bool SerialCommandSender::stop_stream()
{
    int response_value;
    return set_frame_mode(0x00, response_value);
}

bool SerialCommandSender::get_evk_test(int &response_value)
{
    Command cmd = {"EVK_TEST", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_evk_id(int &response_value)
{
    Command cmd = {"EVK_ID", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_powerup(int &response_value)
{
    Command cmd = {"SENXOR_POWERUP", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_frame_mode(int &response_value)
{
    Command cmd = {"FRAME_MODE", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::set_frame_mode(uint8_t value, int &response_value)
{
    Command cmd = {"FRAME_MODE", true, value};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_fw_version_1(int &response_value)
{
    Command cmd = {"FW_VERSION_1", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_fw_version_2(int &response_value)
{
    Command cmd = {"FW_VERSION_2", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_frame_rate(int &response_value)
{
    Command cmd = {"FRAME_RATE", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::set_frame_rate(uint8_t value, int &response_value)
{
    Command cmd = {"FRAME_RATE", true, value};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_power_down_1(int &response_value)
{
    Command cmd = {"POWER_DOWN_1", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::set_power_down_1(uint8_t value, int &response_value)
{
    Command cmd = {"POWER_DOWN_1", true, value};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_status(int &response_value)
{
    Command cmd = {"STATUS", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_power_down_2(int &response_value)
{
    Command cmd = {"POWER_DOWN_2", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::set_power_down_2(uint8_t value, int &response_value)
{
    Command cmd = {"POWER_DOWN_2", true, value};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_type(int &response_value)
{
    Command cmd = {"SENXOR_TYPE", false, 0};
    bool succ = send_command(cmd, response_value);
    if (succ && response_value != UINT8_MAX)
    {
        m_resolution = FPA_SHAPE.at(response_value);
        std::cout << "Camera resolution: " << m_resolution.first << "x" << m_resolution.second << "\n";
    }
    return succ;
}

bool SerialCommandSender::get_module_type(int &response_value)
{
    Command cmd = {"MODULE_TYPE", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_sens_factor(int &response_value)
{
    Command cmd = {"SENS_FACTOR", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::set_sens_factor(uint8_t value, int &response_value)
{
    Command cmd = {"SENS_FACTOR", true, value};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_emissivity(int &response_value)
{
    Command cmd = {"EMISSIVITY", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::set_emissivity(uint8_t value, int &response_value)
{
    Command cmd = {"EMISSIVITY", true, value};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_offset_corr(int &response_value)
{
    Command cmd = {"OFFSET_CORR", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::set_offset_corr(uint8_t value, int &response_value)
{
    Command cmd = {"OFFSET_CORR", true, value};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_filter_ctrl(int &response_value)
{
    Command cmd = {"FILTER_CTRL", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::set_filter_ctrl(uint8_t value, int &response_value)
{
    Command cmd = {"FILTER_CTRL", true, value};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_filter_1_lsb(int &response_value)
{
    Command cmd = {"FILTER_1_LSB", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::set_filter_1_lsb(uint8_t value, int &response_value)
{
    Command cmd = {"FILTER_1_LSB", true, value};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_filter_1_msb(int &response_value)
{
    Command cmd = {"FILTER_1_MSB", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::set_filter_1_msb(uint8_t value, int &response_value)
{
    Command cmd = {"FILTER_1_MSB", true, value};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_filter_2(int &response_value)
{
    Command cmd = {"FILTER_2", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::set_filter_2(uint8_t value, int &response_value)
{
    Command cmd = {"FILTER_2", true, value};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_flash_ctrl(int &response_value)
{
    Command cmd = {"FLASH_CTRL", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::set_flash_ctrl(uint8_t value, int &response_value)
{
    Command cmd = {"FLASH_CTRL", true, value};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_id_0(int &response_value)
{
    Command cmd = {"SENXOR_ID_0", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_id_1(int &response_value)
{
    Command cmd = {"SENXOR_ID_1", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_id_2(int &response_value)
{
    Command cmd = {"SENXOR_ID_2", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_id_3(int &response_value)
{
    Command cmd = {"SENXOR_ID_3", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_id_4(int &response_value)
{
    Command cmd = {"SENXOR_ID_4", false, 0};
    return send_command(cmd, response_value);
}

bool SerialCommandSender::get_senxor_id_5(int &response_value)
{
    Command cmd = {"SENXOR_ID_5", false, 0};
    return send_command(cmd, response_value);
}