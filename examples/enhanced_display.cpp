#include "../serial_mi48.hpp"
#include "../thermal_visualization.hpp"
#include <string>
#include <numeric>
#include <opencv2/opencv.hpp>

void enhanced_frame_handler(const std::vector<float> &temperatures, const uint16_t rows, const uint16_t cols)
{
    if (temperatures.empty())
    {
        return;
    }

    static ThermalVisualization visualizer;
    static ThermalPalette current_palette = ThermalPalette::VIVID;
    static bool show_help = true;

    // Create visualization with current palette
    cv::Mat thermal_image = visualizer.visualizeTemperatures(
        temperatures, rows, cols, current_palette, -273.15f, 100.0f, 4, true
    );

    // Calculate statistics
    auto min_it = std::min_element(temperatures.begin(), temperatures.end());
    auto max_it = std::max_element(temperatures.begin(), temperatures.end());
    double sum = std::accumulate(temperatures.begin(), temperatures.end(), 0.0);
    double avg = sum / temperatures.size();

    // Add text annotations
    std::string palette_name = ThermalVisualization::getAvailablePalettes()[static_cast<int>(current_palette)];
    std::string stats_text = cv::format("Max: %.1f C  Min: %.1f C  Avg: %.1f C", *max_it, *min_it, avg);
    std::string palette_text = cv::format("Palette: %s", palette_name.c_str());
    
    cv::putText(thermal_image, stats_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1);
    cv::putText(thermal_image, palette_text, cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1);

    // Show help text on first few frames
    if (show_help) {
        std::string help_text = "Keys: [1-9] Palette selection [H] Toggle help [Q] Quit";
        cv::putText(thermal_image, help_text, cv::Point(10, thermal_image.rows - 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        
        // Show palette options
        auto palettes = ThermalVisualization::getAvailablePalettes();
        for (size_t i = 0; i < std::min(palettes.size(), size_t(9)); ++i) {
            std::string option_text = cv::format("[%zu] %s", i + 1, palettes[i].c_str());
            cv::putText(thermal_image, option_text, cv::Point(10, 85 + i * 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(180, 180, 180), 1);
        }
    }

    // Display the thermal image
    cv::imshow("Enhanced Thermal Display", thermal_image);
    
    // Handle keyboard input
    char key = cv::waitKey(1) & 0xFF;
    if (key >= '1' && key <= '9') {
        int palette_index = key - '1';
        auto palettes = ThermalVisualization::getAvailablePalettes();
        if (palette_index < palettes.size()) {
            current_palette = ThermalVisualization::paletteFromString(palettes[palette_index]);
            std::cout << "Switched to palette: " << palettes[palette_index] << "\n";
        }
    } else if (key == 'h' || key == 'H') {
        show_help = !show_help;
    } else if (key == 'q' || key == 'Q' || key == 27) {  // ESC key
        cv::destroyAllWindows();
        exit(0);
    }

    // Optional: Keep console output (reduced frequency)
    static int frame_counter = 0;
    if (++frame_counter % 20 == 0) {  // Print every 20 frames
        std::cout << "\033[2J\033[H";
        std::cout << "\n--- Enhanced Thermal Display ---\n";
        std::cout << "Current Palette: " << palette_name << "\n";
        std::cout << "MAX Temperature: " << *max_it << " C\n";
        std::cout << "MIN Temperature: " << *min_it << " C\n";
        std::cout << "AVG Temperature: " << avg << " C\n";
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

    std::cout << "\n=== Enhanced Thermal Display with Multiple Palettes ===\n";
    std::cout << "Available palettes:\n";
    auto palettes = ThermalVisualization::getAvailablePalettes();
    for (size_t i = 0; i < palettes.size(); ++i) {
        std::cout << "  [" << (i + 1) << "] " << palettes[i] << "\n";
    }
    std::cout << "\nControls:\n";
    std::cout << "  [1-9] - Select palette\n";
    std::cout << "  [H] - Toggle help display\n";
    std::cout << "  [Q] or [ESC] - Quit\n\n";

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
    sender.register_frame_callback(enhanced_frame_handler);
    
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
