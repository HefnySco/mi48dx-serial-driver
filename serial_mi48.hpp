#ifndef SERIAL_COMMAND_SENDER_HPP
#define SERIAL_COMMAND_SENDER_HPP

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <cstdint>
#include <libserialport.h>
#include <functional> 
#include <atomic>     
class SerialCommandSender
{

public:
    // Library version information
    static constexpr int VERSION_MAJOR = 1;
    static constexpr int VERSION_MINOR = 0;
    static constexpr int VERSION_PATCH = 0;
    static constexpr const char* VERSION_STRING = "1.0.0";
    
    // Method to get version information
    static std::string get_version();
    // Define the callback function signature that takes a const reference to the temperature vector
    using FrameCallback = std::function<void(const std::vector<float> &temperatures, const uint16_t rows, const uint16_t cols)>;

    SerialCommandSender();
    ~SerialCommandSender();


    // Method to register the user's callback function
    void register_frame_callback(FrameCallback callback);


    bool open_port(const std::string &port_path);
    void close_port();
    void send_and_receive_serial_command();
    void loop_on_read();
    void stop_loop();
    bool start_stream(bool with_header);
    bool stop_stream();
    bool get_evk_test(int &response_value);
    bool get_evk_id(int &response_value);
    bool get_senxor_powerup(int &response_value);
    bool get_frame_mode(int &response_value);
    bool set_frame_mode(uint8_t value, int &response_value);
    bool get_fw_version_1(int &response_value);
    bool get_fw_version_2(int &response_value);
    bool get_frame_rate(int &response_value);
    bool set_frame_rate(uint8_t value, int &response_value);
    bool get_power_down_1(int &response_value);
    bool set_power_down_1(uint8_t value, int &response_value);
    bool get_status(int &response_value);
    bool get_power_down_2(int &response_value);
    bool set_power_down_2(uint8_t value, int &response_value);
    bool get_senxor_type(int &response_value);
    bool get_module_type(int &response_value);
    bool get_sens_factor(int &response_value);
    bool set_sens_factor(uint8_t value, int &response_value);
    bool get_emissivity(int &response_value);
    bool set_emissivity(uint8_t value, int &response_value);
    bool get_offset_corr(int &response_value);
    bool set_offset_corr(uint8_t value, int &response_value);
    bool get_filter_ctrl(int &response_value);
    bool set_filter_ctrl(uint8_t value, int &response_value);
    bool get_filter_1_lsb(int &response_value);
    bool set_filter_1_lsb(uint8_t value, int &response_value);
    bool get_filter_1_msb(int &response_value);
    bool set_filter_1_msb(uint8_t value, int &response_value);
    bool get_filter_2(int &response_value);
    bool set_filter_2(uint8_t value, int &response_value);
    bool get_flash_ctrl(int &response_value);
    bool set_flash_ctrl(uint8_t value, int &response_value);
    bool get_senxor_id_0(int &response_value);
    bool get_senxor_id_1(int &response_value);
    bool get_senxor_id_2(int &response_value);
    bool get_senxor_id_3(int &response_value);
    bool get_senxor_id_4(int &response_value);
    bool get_senxor_id_5(int &response_value);

private:
    // --- Configuration ---
    static constexpr const char *PORT_NAME = "/dev/ttyACM0";
    static constexpr int BAUD_RATE = 115200;
    static constexpr int TIMEOUT_MILLISECONDS = 2000;
    static constexpr int COMMAND_DELAY_MS = 10;

    // Resolution type and mapping
    struct Resolution {
        uint16_t rows;
        uint16_t cols;
    };

    // Callback
    FrameCallback m_frame_callback; // Member to store the callback function
    std::atomic<bool> m_streaming;  // Control flag for the streaming loop
    Resolution m_resolution; // Camera resolution

    
    // MI48-specific constants
    static constexpr uint16_t GET_SINGLE_FRAME = 0x01;
    static constexpr uint16_t CONTINUOUS_STREAM = 0x02;
    static constexpr double KELVIN_0 = -273.15;
    static constexpr uint16_t DEFAULT_ROWS = 62;
    static constexpr uint16_t DEFAULT_COLS = 80;

    // GFRA frame format constants
    static constexpr size_t BYTES_PER_PIXEL = 2;
    static constexpr size_t CHECKSUM_SIZE = 4;
    static constexpr size_t FRAME_HEADER_SIZE = 12;  // "   #" (4) + length (4) + "GFRA" (4)
    
    // Camera-specific frame offsets
    static constexpr size_t MI08_COLS = 80;
    static constexpr size_t MI08_RESERVED_SIZE = 160;    // 80 * 2
    static constexpr size_t MI08_HEADER_ROW_SIZE = 160;  // 80 * 2
    
    static constexpr size_t MI16_COLS = 160;
    static constexpr size_t MI16_RESERVED_SIZE = 960;    // 3 * 160 * 2
    static constexpr size_t MI16_HEADER_ROW_SIZE = 320;  // 160 * 2
    
    static constexpr size_t MI05_COLS = 50;
    static constexpr size_t MI05_RESERVED_SIZE = 100;    // 50 * 2
    static constexpr size_t MI05_HEADER_ROW_SIZE = 100;  // 50 * 2

    // MI48 Register Map
    static const std::map<std::string, uint8_t> MI48_REGMAP;

    static const std::map<uint8_t, Resolution> FPA_SHAPE;

    // Command structure
    struct Command
    {
        std::string reg_name;
        bool is_write;
        uint8_t value;
    };

    // Array of commands
    static const std::vector<Command> COMMAND_LIST;

    struct sp_port *port;    // Serial port handle

    void list_available_ports();
    std::string generate_read_command(const std::string &reg_name);
    std::string generate_write_command(const std::string &reg_name, uint8_t value);
    bool send_command(const Command &cmd, int &response_value);
};

// Static member definitions
inline const std::map<std::string, uint8_t> SerialCommandSender::MI48_REGMAP = {
    {"EVK_TEST", 0x00}, {"EVK_ID", 0xA5}, {"SENXOR_POWERUP", 0xB0}, {"FRAME_MODE", 0xB1}, {"FW_VERSION_1", 0xB2}, {"FW_VERSION_2", 0xB3}, {"FRAME_RATE", 0xB4}, {"POWER_DOWN_1", 0xB5}, {"STATUS", 0xB6}, {"POWER_DOWN_2", 0xB7}, {"SENXOR_TYPE", 0xBA}, {"MODULE_TYPE", 0xBB}, {"SENS_FACTOR", 0xC2}, {"EMISSIVITY", 0xCA}, {"OFFSET_CORR", 0xCB}, {"FILTER_CTRL", 0xD0}, {"FILTER_1_LSB", 0xD1}, {"FILTER_1_MSB", 0xD2}, {"FILTER_2", 0xD3}, {"FLASH_CTRL", 0xD8}, {"SENXOR_ID_0", 0xE0}, {"SENXOR_ID_1", 0xE1}, {"SENXOR_ID_2", 0xE2}, {"SENXOR_ID_3", 0xE3}, {"SENXOR_ID_4", 0xE4}, {"SENXOR_ID_5", 0xE5}};

// Camera types & correspondant resolutions.
// Note: Frame format is cols×rows, so MI08 80×62 means {rows=80, cols=62}
inline const std::map<uint8_t, SerialCommandSender::Resolution> SerialCommandSender::FPA_SHAPE = {
    {0, {80, 62}}, {1, {80, 62}}, {2, {32, 32}}, {3, {80, 62}}, {4, {80, 62}}, {8, {160, 120}}};

// Command initialization sequence.
inline const std::vector<SerialCommandSender::Command> SerialCommandSender::COMMAND_LIST = {
    {"EVK_TEST", false, 0},
    {"POWER_DOWN_1", false, 0},
    {"POWER_DOWN_2", false, 0},
    {"FRAME_MODE", true, 0},
    {"FRAME_MODE", false, 0},
    {"FRAME_MODE", false, 0},
    {"FRAME_MODE", false, 0},
    {"SENXOR_TYPE", false, 0},
    {"MODULE_TYPE", false, 0},
    {"EVK_ID", false, 0},
    {"SENXOR_ID_0", false, 0},
    {"SENXOR_ID_1", false, 0},
    {"SENXOR_ID_2", false, 0},
    {"SENXOR_ID_3", false, 0},
    {"SENXOR_ID_4", false, 0},
    {"SENXOR_ID_5", false, 0},
    {"FW_VERSION_1", false, 0},
    {"FW_VERSION_2", false, 0},
    {"FRAME_RATE", true, 5},
    {"FILTER_CTRL", true, 0},
    {"SENS_FACTOR", true, 100},
    {"EMISSIVITY", true, 95}};

#endif // SERIAL_COMMAND_SENDER_HPP