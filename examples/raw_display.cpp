#include "../serial_mi48.hpp"
#include <string>
#include <iomanip>
#include <numeric>
#include <vector>
#include <iostream>

void my_frame_handler(const std::vector<float> &temperatures, const uint16_t rows, const uint16_t cols)
{
    if (temperatures.empty()) {
        std::cout << "No data received.\n";
        return;
    }

    const uint16_t total_pixels = rows * cols;
    

    // Calculate statistics
    auto min_it = std::min_element(temperatures.begin(), temperatures.end());
    auto max_it = std::max_element(temperatures.begin(), temperatures.end());
    double sum = std::accumulate(temperatures.begin(), temperatures.end(), 0.0);
    double avg = sum / total_pixels;

    // Convert to Celsius
    double min_temp_c = *min_it ;
    double max_temp_c = *max_it ;
    double avg_c = avg ;

    // Clear console and print statistics
    std::cout << "\033[2J\033[H"; // Clear screen
    std::cout << "--- Frame Statistics (Celsius) ---\n";
    std::cout << "MAX Temperature: " << std::fixed << std::setprecision(1) << max_temp_c << " C\n";
    std::cout << "MIN Temperature: " << std::fixed << std::setprecision(1) << min_temp_c << " C\n";
    std::cout << "AVG Temperature: " << std::fixed << std::setprecision(1) << avg_c << " C\n";

    // Print 1-in-5 temperature sample grid
    std::cout << "\n--- 1-in-5 Temperature Sample (C) ---\n";
    for (uint16_t r = 0; r < rows; r += 5) {
        std::cout << "Row " << std::setw(3) << r << ": ";
        for (uint16_t c = 0; c < cols; c += 5) {
            size_t index = (size_t)c * rows + r; // Match original column-major order
            if (index < temperatures.size()) {
                std::cout << "[" << std::setw(2) << c << "]=" 
                          << std::fixed << std::setprecision(1) << (temperatures[index]) << " ";
            }
        }
        std::cout << "\n";
    }
}


int main(int argc, char *argv[])
{
    std::string port_path = "/dev/ttyACM0";
    if (argc > 1)
    {
        port_path = argv[1];
    }
    else
    {
        std::cout << "No port path provided, using default: " << port_path << "\n";
    }

    SerialCommandSender sender;
    if (sender.open_port(port_path))
    {
        sender.send_and_receive_serial_command();
        int camera_type;
        sender.get_senxor_type(camera_type);
    }
    else
    {
        std::cerr << "Failed to open port: " << port_path << "\n";
        return 1;
    }

    sender.register_frame_callback(my_frame_handler);
    sender.start_stream(true);
    sender.loop_on_read();

    return 0;
}