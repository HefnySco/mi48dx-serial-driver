#include "serial_mi48.hpp"
#include <string>
#include <iomanip>
#include <numeric> // For std::accumulate
#include <thread>  // For std::thread
#include <chrono>  // For std::this_thread::sleep_for

void my_frame_handler(const std::vector<float> &temperatures, const uint16_t rows, const uint16_t cols)
{
    if (temperatures.empty())
    {
        return;
    }
    const uint16_t total_pixels = rows * cols;

    // Clear screen and move cursor to top-left
    std::cout << "\033[2J\033[H";
    
    // Example processing: calculate the average temperature of the frame
    auto min_it = std::min_element(temperatures.begin(), temperatures.end());
    auto max_it = std::max_element(temperatures.begin(), temperatures.end());
    double sum = std::accumulate(temperatures.begin(), temperatures.end(), 0.0);
    double avg = sum / (total_pixels);

    std::cout << "\n--- Frame Statistics (Kelvin) ---\n";
    std::cout << "MAX Temperature: " << *max_it << "\n";
    std::cout << "MIN Temperature: " << *min_it << "\n";
    std::cout << "AVG Temperature: " << avg << "\n";

    // Display 1-in-5 temperature samples
    std::cout << "\n--- 1-in-5 Temperature Sample (K) ---\n";
    for (uint16_t r = 0; r < rows; r += 5)
    {
        std::cout << "Row " << r << ": ";
        for (uint16_t c = 0; c < cols; c += 5)
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
int main(int argc, char *argv[])
{
    std::string port_path = "/dev/ttyACM0"; // Default port
    if (argc > 1)
    {
        port_path = argv[1]; // Use provided port path
    }
    else
    {
        std::cout << "No port path provided, using default: " << port_path << "\n";
    }

    SerialCommandSender sender;
    if (sender.open_port(port_path))
    {
        // sender.stop_stream();
        sender.send_and_receive_serial_command();
        int camera_type;
        sender.get_senxor_type(camera_type);
    }

    sender.register_frame_callback(my_frame_handler);
    sender.start_stream(true);
    sender.loop_on_read();
    // Port is closed automatically by destructor
    return 0;
}