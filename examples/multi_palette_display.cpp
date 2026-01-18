#include "../serial_mi48.hpp"
#include "../thermal_visualization.hpp"
#include <string>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <iostream>

class MultiPaletteDisplay {
private:
    ThermalVisualization visualizer_;
    std::vector<ThermalPalette> palettes_;
    size_t current_palette_index_;
    bool auto_rotate_;
    int frame_count_;
    int rotate_interval_;
    
public:
    MultiPaletteDisplay(bool auto_rotate = true, int rotate_interval = 30) 
        : current_palette_index_(0), auto_rotate_(auto_rotate), frame_count_(0), rotate_interval_(rotate_interval) {
        
        palettes_ = {
            ThermalPalette::RAINBOW,
            ThermalPalette::VIVID,
            ThermalPalette::RGB,
            ThermalPalette::GRAYSCALE,
            ThermalPalette::MEDICAL,
            ThermalPalette::IRON,
            ThermalPalette::FIRE,
            ThermalPalette::OCEAN
        };
    }
    
    void frame_handler(const std::vector<float> &temperatures, const uint16_t rows, const uint16_t cols) {
        if (temperatures.empty()) {
            return;
        }

        // Auto-rotate palettes if enabled
        if (auto_rotate_ && frame_count_ % rotate_interval_ == 0) {
            current_palette_index_ = (current_palette_index_ + 1) % palettes_.size();
        }
        frame_count_++;

        ThermalPalette current_palette = palettes_[current_palette_index_];
        
        // Create visualization with current palette
        cv::Mat thermal_image = visualizer_.visualizeTemperatures(
            temperatures, rows, cols, current_palette, -273.15f, 100.0f, 4, true
        );

        // Calculate statistics
        auto min_it = std::min_element(temperatures.begin(), temperatures.end());
        auto max_it = std::max_element(temperatures.begin(), temperatures.end());
        double sum = std::accumulate(temperatures.begin(), temperatures.end(), 0.0);
        double avg = sum / temperatures.size();

        // Add text annotations
        std::string palette_name = ThermalVisualization::getAvailablePalettes()[current_palette_index_];
        std::string stats_text = cv::format("Max: %.1f C  Min: %.1f C  Avg: %.1f C", *max_it, *min_it, avg);
        std::string palette_text = cv::format("Palette: %s [%zu/%zu]", palette_name.c_str(), 
                                             current_palette_index_ + 1, palettes_.size());
        
        if (auto_rotate_) {
            palette_text += cv::format(" (Auto-rotate every %d frames)", rotate_interval_);
        }

        cv::putText(thermal_image, stats_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        cv::putText(thermal_image, palette_text, cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        
        // Add controls info
        std::string controls_text = "Controls: [N]ext palette [P]rev palette [A]uto-rotate toggle [Q]uit";
        cv::putText(thermal_image, controls_text, cv::Point(10, thermal_image.rows - 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);

        // Display the thermal image
        cv::imshow("Multi-Palette Thermal Display", thermal_image);
        
        // Handle keyboard input
        char key = cv::waitKey(1) & 0xFF;
        handleKeyPress(key);
        
        // Optional: Keep console output
        if (frame_count_ % 10 == 0) {  // Print every 10 frames to reduce spam
            std::cout << "\033[2J\033[H";
            std::cout << "\n--- Frame " << frame_count_ << " ---\n";
            std::cout << "Palette: " << palette_name << "\n";
            std::cout << "MAX Temperature: " << *max_it << " C\n";
            std::cout << "MIN Temperature: " << *min_it << " C\n";
            std::cout << "AVG Temperature: " << avg << " C\n";
        }
    }
    
    void handleKeyPress(char key) {
        switch (key) {
            case 'n':
            case 'N':
                // Next palette
                current_palette_index_ = (current_palette_index_ + 1) % palettes_.size();
                std::cout << "Switched to palette: " << ThermalVisualization::getAvailablePalettes()[current_palette_index_] << "\n";
                break;
            case 'p':
            case 'P':
                // Previous palette
                current_palette_index_ = (current_palette_index_ == 0) ? palettes_.size() - 1 : current_palette_index_ - 1;
                std::cout << "Switched to palette: " << ThermalVisualization::getAvailablePalettes()[current_palette_index_] << "\n";
                break;
            case 'a':
            case 'A':
                // Toggle auto-rotate
                auto_rotate_ = !auto_rotate_;
                std::cout << "Auto-rotate " << (auto_rotate_ ? "enabled" : "disabled") << "\n";
                break;
            case 'q':
            case 'Q':
            case 27:  // ESC key
                std::cout << "Exiting...\n";
                cv::destroyAllWindows();
                exit(0);
                break;
        }
    }
    
    void printUsage() {
        std::cout << "\n=== Multi-Palette Thermal Display ===\n";
        std::cout << "Available palettes:\n";
        auto palettes = ThermalVisualization::getAvailablePalettes();
        for (size_t i = 0; i < palettes.size(); ++i) {
            std::cout << "  " << (i + 1) << ". " << palettes[i] << "\n";
        }
        std::cout << "\nControls:\n";
        std::cout << "  [N] - Next palette\n";
        std::cout << "  [P] - Previous palette\n";
        std::cout << "  [A] - Toggle auto-rotate\n";
        std::cout << "  [Q] or [ESC] - Quit\n";
        std::cout << "\nStarting with auto-rotate " << (auto_rotate_ ? "enabled" : "disabled") << "\n";
    }
};

int main(int argc, char *argv[]) {
    std::string port_path = "/dev/ttyACM0";
    bool auto_rotate = true;
    int rotate_interval = 30;
    
    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port_path = argv[++i];
        } else if (arg == "--no-auto") {
            auto_rotate = false;
        } else if (arg == "--interval" && i + 1 < argc) {
            rotate_interval = std::stoi(argv[++i]);
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "Options:\n";
            std::cout << "  --port <path>     Serial port path (default: /dev/ttyACM0)\n";
            std::cout << "  --no-auto         Disable auto-rotation of palettes\n";
            std::cout << "  --interval <n>    Auto-rotate interval in frames (default: 30)\n";
            std::cout << "  --help            Show this help\n";
            return 0;
        }
    }

    MultiPaletteDisplay display(auto_rotate, rotate_interval);
    display.printUsage();

    SerialCommandSender sender;
    if (!sender.open_port(port_path)) {
        std::cerr << "Failed to open port: " << port_path << "\n";
        return 1;
    }

    // Initialize camera with full MI48 initialization sequence
    if (!sender.initialize_camera(true)) {
        std::cerr << "Failed to initialize camera\n";
        sender.close_port();
        return 1;
    }

    // Register callback and start streaming
    std::cout << "=== Starting Stream ===\n";
    sender.register_frame_callback([&display](const std::vector<float>& temps, uint16_t rows, uint16_t cols) {
        display.frame_handler(temps, rows, cols);
    });
    
    if (!sender.start_stream(true)) {
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
