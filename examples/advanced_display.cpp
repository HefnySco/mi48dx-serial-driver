#include "../serial_mi48.hpp"
#include <string>
#include <iomanip>
#include <numeric>
#include <thread>
#include <chrono>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cstdlib>

// Rolling Average Filter for temporal smoothing
class RollingAverageFilter {
public:
    RollingAverageFilter(size_t N = 3) : N_(N), buffer_() {}
    
    cv::Mat operator()(const cv::Mat& frame) {
        buffer_.push_back(frame.clone());
        if (buffer_.size() > N_) {
            buffer_.erase(buffer_.begin());
        }
        
        cv::Mat sum = cv::Mat::zeros(frame.rows, frame.cols, CV_32F);
        for (const auto& buf : buffer_) {
            sum += buf;
        }
        return sum / buffer_.size();
    }

private:
    size_t N_;
    std::vector<cv::Mat> buffer_;
};

void my_frame_handler(const std::vector<float> &temperatures, const uint16_t rows, const uint16_t cols, 
                      int smooth_level = 5, int temporal_smooth = 3, bool use_clahe = false, 
                      const std::string& colormap = "COLORMAP_RAINBOW")
{
    if (temperatures.empty()) {
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

    // Apply temporal smoothing
    static RollingAverageFilter temporal_filter(temporal_smooth);
    cv::Mat frame_smoothed = temporal_filter(frame);

    // Normalize temperatures to 0-255 range for display (inverted for blue=hot, red=cool)
    double min_temp, max_temp;
    cv::minMaxLoc(frame_smoothed, &min_temp, &max_temp);
    cv::Mat frame_normalized;
    frame_smoothed.convertTo(frame_normalized, CV_8U, 255.0 / (max_temp - min_temp), -min_temp * 255.0 / (max_temp - min_temp));
    
    // Invert the normalized frame to make hottest points blue and coolest points red
    cv::Mat frame_inverted = 255 - frame_normalized;

    // Apply spatial smoothing (median blur)
    cv::Mat smoothed_frame;
    cv::medianBlur(frame_inverted, smoothed_frame, smooth_level);

    // Apply CLAHE if enabled
    cv::Mat enhanced_frame = smoothed_frame;
    if (use_clahe) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
        clahe->apply(smoothed_frame, enhanced_frame);
    }

    // Apply OpenCV colormap
    cv::Mat thermal_image;
    int colormap_id;
    if (colormap == "COLORMAP_JET") colormap_id = cv::COLORMAP_JET;
    else if (colormap == "COLORMAP_HOT") colormap_id = cv::COLORMAP_HOT;
    else if (colormap == "COLORMAP_VIRIDIS") colormap_id = cv::COLORMAP_VIRIDIS;
    else colormap_id = cv::COLORMAP_RAINBOW; // Default
    cv::applyColorMap(enhanced_frame, thermal_image, colormap_id);

    // Resize for better visibility (scale by 4)
    cv::Mat thermal_display;
    cv::resize(thermal_image, thermal_display, cv::Size(cols * 4, rows * 4), 0, 0, cv::INTER_NEAREST);

    // Rotate the image 90 degrees clockwise to correct -90 degree rotation
    cv::Mat thermal_display_rotated;
    cv::rotate(thermal_display, thermal_display_rotated, cv::ROTATE_90_CLOCKWISE);

    // Find min and max temperature points
    cv::Point min_loc, max_loc;
    cv::minMaxLoc(frame_smoothed, &min_temp, &max_temp, &min_loc, &max_loc);

    // Convert temperatures to Celsius for display
    double min_temp_c = min_temp;
    double max_temp_c = max_temp;
    double avg_c = (std::accumulate(temperatures.begin(), temperatures.end(), 0.0) / total_pixels);

    // Scale min/max locations for display
    int scale = 4;
    cv::Point min_loc_scaled(min_loc.x * scale, min_loc.y * scale);
    cv::Point max_loc_scaled(max_loc.x * scale, max_loc.y * scale);

    // Annotate min and max points with "+" and temperature values
    cv::putText(thermal_display_rotated, "+", min_loc_scaled, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(thermal_display_rotated, cv::format("%.1fC", min_temp_c), 
                cv::Point(min_loc_scaled.x + 10, min_loc_scaled.y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1);
    cv::putText(thermal_display_rotated, "+", max_loc_scaled, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(thermal_display_rotated, cv::format("%.1fC", max_temp_c), 
                cv::Point(max_loc_scaled.x + 10, max_loc_scaled.y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1);

    // Add statistics text
    std::string stats_text = cv::format("Max: %.1fC  Min: %.1fC  Avg: %.1fC", max_temp_c, min_temp_c, avg_c);
    cv::putText(thermal_display_rotated, stats_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1);

    // Display the thermal image
    cv::imshow("Thermal Image", thermal_display_rotated);
    cv::waitKey(1); // Allow OpenCV to process window events

    // Console output
    std::cout << "\033[2J\033[H";
    std::cout << "\n--- Frame Statistics (Celsius) ---\n";
    std::cout << "MAX Temperature: " << max_temp_c / 10.0 << "\n";
    std::cout << "MIN Temperature: " << min_temp_c / 10.0 << "\n";
    std::cout << "AVG Temperature: " << avg_c / 10.0 << "\n";

    std::cout << "\n--- 1-in-5 Temperature Sample (C) ---\n";
    for (uint16_t r = 0; r < rows; r += 5) {
        std::cout << "Row " << r << ": ";
        for (uint16_t c = 0; c < cols; c += 5) {
            size_t index = (size_t)r * cols + c;
            if (index < temperatures.size()) {
                std::cout << "[" << c << "]=" << (temperatures[index]) << " ";
            }
        }
        std::cout << "\n";
    }
}

int main(int argc, char *argv[])
{
    std::string port_path = "/dev/ttyACM0";
    int smooth_level = 5;
    int temporal_smooth = 3;
    bool use_clahe = false;
    std::string colormap = "COLORMAP_RAINBOW";

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port_path = argv[++i];
        }
        else if (arg == "--smooth-level" && i + 1 < argc) {
            smooth_level = std::atoi(argv[++i]);
            if (smooth_level % 2 == 0 || smooth_level < 3) {
                std::cerr << "Error: Smooth level must be an odd integer >= 3\n";
                return 1;
            }
        }
        else if (arg == "--temporal-smooth" && i + 1 < argc) {
            temporal_smooth = std::atoi(argv[++i]);
            if (temporal_smooth < 1) {
                std::cerr << "Error: Temporal smooth must be an integer >= 1\n";
                return 1;
            }
        }
        else if (arg == "--clahe") {
            use_clahe = true;
        }
        else if (arg == "--colormap" && i + 1 < argc) {
            colormap = argv[++i];
            if (colormap != "COLORMAP_RAINBOW" && colormap != "COLORMAP_JET" &&
                colormap != "COLORMAP_HOT" && colormap != "COLORMAP_VIRIDIS") {
                std::cerr << "Error: Invalid colormap. Use COLORMAP_RAINBOW, COLORMAP_JET, COLORMAP_HOT, or COLORMAP_VIRIDIS\n";
                return 1;
            }
        }
        else {
            std::cerr << "Usage: " << argv[0] << " [--port <port>] [--smooth-level <odd_int>] [--temporal-smooth <int>] [--clahe] [--colormap <name>]\n";
            return 1;
        }
    }

    std::cout << "Using port: " << port_path << ", smooth-level: " << smooth_level 
              << ", temporal-smooth: " << temporal_smooth << ", clahe: " << (use_clahe ? "on" : "off")
              << ", colormap: " << colormap << "\n";

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

    sender.register_frame_callback([=](const std::vector<float>& temps, uint16_t r, uint16_t c) {
        my_frame_handler(temps, r, c, smooth_level, temporal_smooth, use_clahe, colormap);
    });
    sender.start_stream(true);
    sender.loop_on_read();

    cv::destroyAllWindows(); // Clean up OpenCV windows on exit
    return 0;
}