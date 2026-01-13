#include "../serial_mi48.hpp"
#include <string>
#include <numeric>
#include <opencv2/opencv.hpp>

void my_frame_handler(const std::vector<float> &temperatures, const uint16_t rows, const uint16_t cols)
{
    if (temperatures.empty())
    {
        return;
    }
    const uint16_t total_pixels = rows * cols;

    // Create a matrix from temperature data
    cv::Mat frame(rows, cols, CV_32F);
    for (uint16_t c = 0; c < cols; ++c) {
        for (uint16_t r = 0; r < rows; ++r) {
            frame.at<float>(r, c) = temperatures[c * rows + r];
        }
    }

    // Normalize temperatures to 0-255 range for display
    double min_temp, max_temp;
    cv::minMaxLoc(frame, &min_temp, &max_temp);
    cv::Mat frame_normalized;
    frame.convertTo(frame_normalized, CV_8U, 255.0 / (max_temp - min_temp), -min_temp * 255.0 / (max_temp - min_temp));

    // Invert the normalized frame to make hottest points blue and coolest points red
    cv::Mat frame_inverted = 255 - frame_normalized;

    // Apply OpenCV colormap (e.g., COLORMAP_RAINBOW)
    cv::Mat thermal_image;
    cv::applyColorMap(frame_inverted, thermal_image, cv::COLORMAP_RAINBOW);

    // Resize for better visibility (e.g., scale by 4 as in Python code)
    cv::Mat thermal_display;
    cv::resize(thermal_image, thermal_display, cv::Size(cols * 4, rows * 4), 0, 0, cv::INTER_NEAREST);

    // Rotate the image 90 degrees clockwise to correct -90 degree rotation
    cv::Mat thermal_display_rotated;
    cv::rotate(thermal_display, thermal_display_rotated, cv::ROTATE_90_CLOCKWISE);

    // Calculate statistics
    auto min_it = std::min_element(temperatures.begin(), temperatures.end());
    auto max_it = std::max_element(temperatures.begin(), temperatures.end());
    double sum = std::accumulate(temperatures.begin(), temperatures.end(), 0.0);
    double avg = sum / total_pixels;

    // Add text annotations
    std::string stats_text = cv::format("Max: %.1f C  Min: %.1f C  Avg: %.1f C", *max_it, *min_it, avg);
    cv::putText(thermal_display_rotated, stats_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1);

    // Display the thermal image
    cv::imshow("Thermal Image", thermal_display_rotated);
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