#include "../serial_mi48.hpp"
#include "../thermal_visualization.hpp"
#include <string>
#include <numeric>
#include <iostream>
#include <opencv2/opencv.hpp>

void my_frame_handler(const std::vector<float> &temperatures, const uint16_t rows, const uint16_t cols)
{
    if (temperatures.empty())
    {
        return;
    }

    // Use the new thermal visualization class with RGB palette
    static ThermalVisualization visualizer;
    cv::Mat thermal_image = visualizer.visualizeTemperatures(
        temperatures, rows, cols, ThermalPalette::RAINBOW, -273.15f, 100.0f, 4, true
    );

    // Calculate statistics
    auto min_it = std::min_element(temperatures.begin(), temperatures.end());
    auto max_it = std::max_element(temperatures.begin(), temperatures.end());
    double sum = std::accumulate(temperatures.begin(), temperatures.end(), 0.0);
    const uint16_t total_pixels = rows * cols;
    double avg = sum / total_pixels;

    // Add text annotations
    std::string stats_text = cv::format("Max: %.1f C  Min: %.1f C  Avg: %.1f C", *max_it, *min_it, avg);
    cv::putText(thermal_image, stats_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1);

    // Display the thermal image
    cv::imshow("Thermal Image", thermal_image);
    cv::waitKey(1); // Allow OpenCV to process window events

    // Optional: Keep console output
    std::cout << "\033[2J\033[H";
    std::cout << "\n--- Frame Statistics (C) ---\n";
    std::cout << "MAX Temperature: " << *max_it << "\n";
    std::cout << "MIN Temperature: " << *min_it << "\n";
    std::cout << "AVG Temperature: " << avg << "\n";

    std::cout << "\n--- 1-in-5 Temperature Sample (C) ---\n";
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
    if (!sender.open_port(port_path))
    {
        std::cerr << "Failed to open port: " << port_path << "\n";
        return 1;
    }

    // Initialize camera with full MI48 initialization sequence
    if (!sender.initialize_camera(true))
    {
        std::cerr << "Failed to initialize camera\n";
        sender.close_port();
        return 1;
    }

    // Register callback and start streaming
    std::cout << "=== Starting Stream ===\n";
    sender.register_frame_callback(my_frame_handler);
    
    if (!sender.start_stream(true))
    {
        std::cerr << "ERROR: Failed to start stream\n";
        sender.close_port();
        return 1;
    }
    
    std::cout << "Stream started successfully. Press Ctrl+C to exit.\n\n";
    
    // Main loop
    sender.loop_on_read();
    
    // Cleanup
    cv::destroyAllWindows();
    return 0;
}