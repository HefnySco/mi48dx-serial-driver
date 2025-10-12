#include "serial_mi48.hpp"
#include <unistd.h>
#include <ctime>
#include <numeric> // For std::accumulate
#include <map>
#include <algorithm>
#include <cstdint>         // For uint8_t

// Constructor and destructor
SerialCommandSender::SerialCommandSender() : port(nullptr), m_resolution(DEFAULT_ROWS, DEFAULT_COLS) {}

SerialCommandSender::~SerialCommandSender()
{
    close_port();
}

// Implementation of methods (unchanged from previous version)
bool SerialCommandSender::open_port(const std::string& port_path) {
    std::cout << "Attempting to connect to port: " << port_path << " at " << BAUD_RATE << " baud...\n";

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

void SerialCommandSender::loop_on_read()
{
    if (!port)
    {
        std::cerr << "\n--- ERROR ---\n";
        std::cerr << "Serial port is not open. Call open_port() first.\n";
        return;
    }

    uint16_t rows = DEFAULT_ROWS; // Replace with m_resolution.first if in a class
    uint16_t cols = DEFAULT_COLS; // Replace with m_resolution.second if in a class
    size_t expected_data_size = (size_t)rows * cols * sizeof(int16_t);

    constexpr size_t MAX_BUFFER_SIZE = 19201;

    std::cout << "Starting read-only loop until no data received...\n";
    std::cout << "Expected Frame Size: " << rows << "x" << cols << " (" << expected_data_size << " bytes)\n";

    try
    {
        bool received_data = true;

        while (received_data)
        {
            received_data = false; // Reset flag for this cycle

            char buffer[MAX_BUFFER_SIZE];
            int bytes_read = 0;
            std::string response_string_raw = "";
            long start_time = clock();

            std::cout << "\n--- Read Cycle ---\n";

            // Loop to read data until the full frame is received or timeout
            while ((clock() - start_time) * 1000 / CLOCKS_PER_SEC < TIMEOUT_MILLISECONDS)
            {
                int current_read = sp_nonblocking_read(port, buffer, sizeof(buffer) - 1);

                if (current_read > 0)
                {
                    response_string_raw.append(buffer, current_read);
                    received_data = true; // Data received

                    if (response_string_raw.size() >= expected_data_size)
                    {
                        break; // Stop accumulating data for this frame
                    }
                }
                usleep(10000); // Small delay to prevent busy-waiting
            }

            // --- Data Processing ---

            if (received_data && response_string_raw.size() >= expected_data_size)
            {
                const char *data_ptr = response_string_raw.data();
                const size_t total_pixels = rows * cols;

                // Vector to hold the temperature data in degrees Kelvin
                std::vector<double> temperatures(total_pixels);

                // 1. Read and Convert ALL int16_t values
                for (size_t i = 0; i < total_pixels; ++i)
                {
                    // Interpret two bytes as a signed 16-bit integer (little-endian assumed)
                    int16_t raw_data_val = (int16_t)(((uint8_t)data_ptr[i * 2 + 1] << 8) |
                                                     (uint8_t)data_ptr[i * 2]);

                    const double temp_celsius = (double)raw_data_val / 10.0;
                    temperatures[i] = temp_celsius - KELVIN_0;
                }

                std::cout << "✅ RECEIVED FRAME (" << response_string_raw.size() << " bytes).\n";
                std::cout << "   Total Pixels: " << total_pixels << "\n";

                // 2. Calculate MIN, MAX, and AVG
                if (total_pixels > 0)
                {
                    // Find min and max using std::min_element and std::max_element
                    auto min_it = std::min_element(temperatures.begin(), temperatures.end());
                    auto max_it = std::max_element(temperatures.begin(), temperatures.end());

                    // Calculate average using std::accumulate
                    double sum = std::accumulate(temperatures.begin(), temperatures.end(), 0.0);
                    double avg = sum / total_pixels;

                    std::cout << "\n   --- Frame Statistics (Kelvin) ---\n";
                    std::cout << "   MAX Temperature: " << *max_it << "\n";
                    std::cout << "   MIN Temperature: " << *min_it << "\n";
                    std::cout << "   AVG Temperature: " << avg << "\n";
                }

                // 3. Sample and Display 1-in-5 temperatures
                std::cout << "\n   --- 1-in-5 Temperature Sample (K) ---\n";

                for (uint16_t r = 0; r < rows; r += 5) // Step 5 for Rows
                {
                    std::cout << "   Row " << r << ": ";
                    for (uint16_t c = 0; c < cols; c += 5) // Step 5 for Columns
                    {
                        size_t index = (size_t)r * cols + c;

                        if (index < temperatures.size())
                        {
                            std::cout << "[" << c << "]=" << temperatures[index] << " ";
                        }
                    }
                    std::cout << "\n";
                }

                // If necessary, remove the processed frame data from the raw buffer
                response_string_raw.erase(0, expected_data_size);

                usleep(COMMAND_DELAY_MS * 1000); // Pause before next read cycle
            }
            else
            {
                std::cout << "❌ TIMEOUT: No full frame (" << expected_data_size << " bytes) received after " << TIMEOUT_MILLISECONDS
                          << " ms. Received " << response_string_raw.size() << " bytes.\n";
                received_data = false; // Ensure outer loop exits
            }
        }
        std::cout << "\nNo data received in the last read cycle, stopping.\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "\n--- COMMUNICATION ERROR ---\n";
        std::cerr << "An error occurred during communication: " << e.what() << "\n";
    }
}

bool SerialCommandSender::start_stream(bool with_header)
{
    int response_value;
    uint8_t mode = with_header ? CONTINUOUS_STREAM : (CONTINUOUS_STREAM | 0x04);
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