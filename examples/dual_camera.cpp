#include "../serial_mi48.hpp"
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>

// --- Global state ---
std::atomic<bool> g_running{true};
std::mutex g_frame_mutex;
cv::Mat g_thermal_frame;
uint16_t g_thermal_rows = 0;
uint16_t g_thermal_cols = 0;

// Display modes
enum class DisplayMode {
    SEPARATE = 1,
    SIDE_BY_SIDE = 2,
    OVERLAY = 3,
    PIP = 4
};

// Calibration parameters
struct CalibrationParams {
    double scale_x = 1.0;
    double scale_y = 1.0;
    int offset_x = 0;
    int offset_y = 0;
    double rotation = 0.0;
    double alpha = 0.5;
};

void thermal_frame_handler(const std::vector<float> &temperatures, const uint16_t rows, const uint16_t cols)
{
    if (temperatures.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(g_frame_mutex);
    g_thermal_rows = rows;
    g_thermal_cols = cols;

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

    // Invert and apply colormap
    cv::Mat frame_inverted = 255 - frame_normalized;
    cv::applyColorMap(frame_inverted, g_thermal_frame, cv::COLORMAP_JET);

    // Rotate 90 degrees clockwise
    cv::rotate(g_thermal_frame, g_thermal_frame, cv::ROTATE_90_CLOCKWISE);
}

cv::Mat stretch_image(const cv::Mat& image, double scale_x, double scale_y, 
                      int offset_x, int offset_y, double rotation)
{
    int h = image.rows;
    int w = image.cols;
    cv::Point2f center(w / 2.0f, h / 2.0f);

    // Create rotation matrix
    cv::Mat rot_matrix = cv::getRotationMatrix2D(center, rotation, 1.0);

    // Add scaling
    rot_matrix.at<double>(0, 0) *= scale_x;
    rot_matrix.at<double>(0, 1) *= scale_x;
    rot_matrix.at<double>(1, 0) *= scale_y;
    rot_matrix.at<double>(1, 1) *= scale_y;

    // Add translation
    rot_matrix.at<double>(0, 2) += offset_x;
    rot_matrix.at<double>(1, 2) += offset_y;

    cv::Mat result;
    cv::warpAffine(image, result, rot_matrix, cv::Size(w, h),
                   cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    return result;
}

cv::Mat overlay_thermal_on_rgb(const cv::Mat& rgb_image, const cv::Mat& thermal_image,
                               const CalibrationParams& params)
{
    // Resize thermal to match RGB
    cv::Mat thermal_resized;
    cv::resize(thermal_image, thermal_resized, rgb_image.size(), 0, 0, cv::INTER_LINEAR);

    // Apply stretching/transformation
    cv::Mat thermal_stretched = stretch_image(thermal_resized, params.scale_x, params.scale_y,
                                               params.offset_x, params.offset_y, params.rotation);

    // Blend images
    cv::Mat result;
    cv::addWeighted(rgb_image, 1.0 - params.alpha, thermal_stretched, params.alpha, 0, result);
    return result;
}

cv::Mat side_by_side(const cv::Mat& rgb_image, const cv::Mat& thermal_image)
{
    // Resize thermal to match RGB height
    cv::Mat thermal_resized;
    double scale = static_cast<double>(rgb_image.rows) / thermal_image.rows;
    cv::resize(thermal_image, thermal_resized, cv::Size(), scale, scale, cv::INTER_LINEAR);

    // Concatenate horizontally
    cv::Mat result;
    cv::hconcat(rgb_image, thermal_resized, result);
    return result;
}

cv::Mat picture_in_picture(const cv::Mat& rgb_image, const cv::Mat& thermal_image,
                           double pip_scale = 0.3)
{
    cv::Mat result = rgb_image.clone();

    // Scale thermal for PiP
    int pip_width = static_cast<int>(rgb_image.cols * pip_scale);
    int pip_height = static_cast<int>(rgb_image.rows * pip_scale);
    cv::Mat thermal_pip;
    cv::resize(thermal_image, thermal_pip, cv::Size(pip_width, pip_height), 0, 0, cv::INTER_LINEAR);

    // Position in bottom-right corner with margin
    int margin = 10;
    int x = rgb_image.cols - pip_width - margin;
    int y = rgb_image.rows - pip_height - margin;

    // Copy thermal PiP to result
    thermal_pip.copyTo(result(cv::Rect(x, y, pip_width, pip_height)));

    // Draw border
    cv::rectangle(result, cv::Point(x - 2, y - 2), 
                  cv::Point(x + pip_width + 2, y + pip_height + 2),
                  cv::Scalar(255, 255, 255), 2);

    return result;
}

cv::Mat draw_calibration_grid(const cv::Mat& image, int grid_size = 50)
{
    cv::Mat result = image.clone();
    int h = image.rows;
    int w = image.cols;

    // Draw vertical lines
    for (int x = 0; x < w; x += grid_size) {
        cv::line(result, cv::Point(x, 0), cv::Point(x, h), cv::Scalar(0, 255, 0), 1);
    }

    // Draw horizontal lines
    for (int y = 0; y < h; y += grid_size) {
        cv::line(result, cv::Point(0, y), cv::Point(w, y), cv::Scalar(0, 255, 0), 1);
    }

    // Draw center crosshair
    cv::line(result, cv::Point(w/2 - 20, h/2), cv::Point(w/2 + 20, h/2), cv::Scalar(0, 0, 255), 2);
    cv::line(result, cv::Point(w/2, h/2 - 20), cv::Point(w/2, h/2 + 20), cv::Scalar(0, 0, 255), 2);

    return result;
}

void draw_help_overlay(cv::Mat& image)
{
    std::vector<std::string> help_text = {
        "Controls:",
        "1 - Separate windows",
        "2 - Side-by-side",
        "3 - Overlay (default)",
        "4 - Picture-in-Picture",
        "+ / = - Increase thermal alpha",
        "- - Decrease thermal alpha",
        "w/s - Scale Y +/-",
        "a/d - Scale X +/-",
        "Arrow keys - Offset",
        "z/x - Rotate +/-",
        "c - Toggle calibration grid",
        "p - Print current values",
        "r - Reset transformation",
        "h - Toggle help (this)",
        "q / ESC - Quit"
    };

    // Draw semi-transparent background
    cv::Mat overlay = image.clone();
    cv::rectangle(overlay, cv::Point(5, 5), cv::Point(350, 20 + static_cast<int>(help_text.size()) * 20),
                  cv::Scalar(0, 0, 0), -1);
    cv::addWeighted(overlay, 0.7, image, 0.3, 0, image);

    // Draw text
    int start_y = 20;
    for (size_t i = 0; i < help_text.size(); ++i) {
        cv::putText(image, help_text[i], cv::Point(10, start_y + static_cast<int>(i) * 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
}

void print_usage(const char* program_name)
{
    std::cout << "Usage: " << program_name << " [thermal_port] [video_device]\n";
    std::cout << "  thermal_port  - Serial port for MI48 (default: /dev/ttyACM0)\n";
    std::cout << "  video_device  - Video device index or path (default: 0)\n";
    std::cout << "\nExample: " << program_name << " /dev/ttyACM0 0\n";
}

int main(int argc, char *argv[])
{
    std::string thermal_port = "/dev/ttyACM0";
    int video_device = 0;

    if (argc > 1) {
        thermal_port = argv[1];
    }
    if (argc > 2) {
        video_device = std::atoi(argv[2]);
    }

    std::cout << "Dual Camera Thermal-RGB Fusion\n";
    std::cout << "Thermal port: " << thermal_port << "\n";
    std::cout << "Video device: " << video_device << "\n";

    // Initialize thermal sensor
    SerialCommandSender sender;
    if (!sender.open_port(thermal_port)) {
        std::cerr << "Failed to open thermal port: " << thermal_port << "\n";
        return 1;
    }

    sender.send_and_receive_serial_command();
    int camera_type;
    sender.get_senxor_type(camera_type);

    sender.register_frame_callback(thermal_frame_handler);

    // Initialize RGB camera
    cv::VideoCapture rgb_cam(video_device);
    if (!rgb_cam.isOpened()) {
        std::cerr << "Failed to open video device: " << video_device << "\n";
        return 1;
    }

    // Set camera resolution
    rgb_cam.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    rgb_cam.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // Start thermal streaming in background thread
    std::thread thermal_thread([&sender]() {
        sender.start_stream(true);
        sender.loop_on_read();
    });

    // Display state
    DisplayMode mode = DisplayMode::OVERLAY;
    CalibrationParams calib;
    bool show_help = false;
    bool calibration_mode = false;
    const double scale_step = 0.02;
    const int offset_step = 2;
    const double rotation_step = 0.5;

    std::string window_name = "Dual Camera Fusion";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    std::cout << "\nPress 'h' for help\n";

    while (g_running) {
        cv::Mat rgb_frame;
        if (!rgb_cam.read(rgb_frame)) {
            std::cerr << "Failed to read RGB frame\n";
            continue;
        }

        // Get thermal frame copy
        cv::Mat thermal_frame;
        {
            std::lock_guard<std::mutex> lock(g_frame_mutex);
            if (!g_thermal_frame.empty()) {
                thermal_frame = g_thermal_frame.clone();
            }
        }

        if (thermal_frame.empty()) {
            // Show only RGB if thermal not yet available
            cv::imshow(window_name, rgb_frame);
            int key = cv::waitKey(30) & 0xFF;
            if (key == 'q' || key == 27) break;
            continue;
        }

        cv::Mat display;

        switch (mode) {
            case DisplayMode::SEPARATE:
                {
                    cv::Mat thermal_display;
                    cv::resize(thermal_frame, thermal_display, cv::Size(thermal_frame.cols * 4, thermal_frame.rows * 4));
                    if (calibration_mode) {
                        thermal_display = draw_calibration_grid(thermal_display);
                        rgb_frame = draw_calibration_grid(rgb_frame);
                    }
                    cv::imshow("Thermal", thermal_display);
                    cv::imshow("RGB", rgb_frame);
                    display = rgb_frame;
                }
                break;

            case DisplayMode::SIDE_BY_SIDE:
                display = side_by_side(rgb_frame, thermal_frame);
                if (calibration_mode) {
                    display = draw_calibration_grid(display);
                }
                break;

            case DisplayMode::OVERLAY:
                display = overlay_thermal_on_rgb(rgb_frame, thermal_frame, calib);
                {
                    std::string info = cv::format("Scale: %.2fx%.2f | Offset: (%d, %d) | Rot: %.1fdeg | Alpha: %.1f",
                                                   calib.scale_x, calib.scale_y, calib.offset_x, calib.offset_y,
                                                   calib.rotation, calib.alpha);
                    cv::putText(display, info, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                }
                if (calibration_mode) {
                    display = draw_calibration_grid(display);
                }
                break;

            case DisplayMode::PIP:
                display = picture_in_picture(rgb_frame, thermal_frame);
                if (calibration_mode) {
                    display = draw_calibration_grid(display);
                }
                break;
        }

        if (show_help) {
            draw_help_overlay(display);
        }

        if (mode != DisplayMode::SEPARATE) {
            cv::imshow(window_name, display);
        }

        int key = cv::waitKey(30) & 0xFF;

        if (key == 'q' || key == 27) {
            g_running = false;
            break;
        }
        else if (key == 'h') {
            show_help = !show_help;
            std::cout << "Help overlay: " << (show_help ? "ON" : "OFF") << "\n";
        }
        else if (key == '1') {
            mode = DisplayMode::SEPARATE;
            std::cout << "Mode: Separate windows\n";
        }
        else if (key == '2') {
            mode = DisplayMode::SIDE_BY_SIDE;
            cv::destroyWindow("Thermal");
            cv::destroyWindow("RGB");
            std::cout << "Mode: Side-by-side\n";
        }
        else if (key == '3') {
            mode = DisplayMode::OVERLAY;
            cv::destroyWindow("Thermal");
            cv::destroyWindow("RGB");
            std::cout << "Mode: Overlay\n";
        }
        else if (key == '4') {
            mode = DisplayMode::PIP;
            cv::destroyWindow("Thermal");
            cv::destroyWindow("RGB");
            std::cout << "Mode: Picture-in-Picture\n";
        }
        else if (key == '+' || key == '=') {
            calib.alpha = std::min(1.0, calib.alpha + 0.1);
            std::cout << "Alpha: " << calib.alpha << "\n";
        }
        else if (key == '-') {
            calib.alpha = std::max(0.0, calib.alpha - 0.1);
            std::cout << "Alpha: " << calib.alpha << "\n";
        }
        else if (key == 'w') {
            calib.scale_y += scale_step;
            std::cout << "Scale Y: " << calib.scale_y << "\n";
        }
        else if (key == 's') {
            calib.scale_y = std::max(0.1, calib.scale_y - scale_step);
            std::cout << "Scale Y: " << calib.scale_y << "\n";
        }
        else if (key == 'd') {
            calib.scale_x += scale_step;
            std::cout << "Scale X: " << calib.scale_x << "\n";
        }
        else if (key == 'a') {
            calib.scale_x = std::max(0.1, calib.scale_x - scale_step);
            std::cout << "Scale X: " << calib.scale_x << "\n";
        }
        else if (key == 82) { // Up arrow
            calib.offset_y -= offset_step;
            std::cout << "Offset Y: " << calib.offset_y << "\n";
        }
        else if (key == 84) { // Down arrow
            calib.offset_y += offset_step;
            std::cout << "Offset Y: " << calib.offset_y << "\n";
        }
        else if (key == 81) { // Left arrow
            calib.offset_x -= offset_step;
            std::cout << "Offset X: " << calib.offset_x << "\n";
        }
        else if (key == 83) { // Right arrow
            calib.offset_x += offset_step;
            std::cout << "Offset X: " << calib.offset_x << "\n";
        }
        else if (key == 'z') {
            calib.rotation -= rotation_step;
            std::cout << "Rotation: " << calib.rotation << " deg\n";
        }
        else if (key == 'x') {
            calib.rotation += rotation_step;
            std::cout << "Rotation: " << calib.rotation << " deg\n";
        }
        else if (key == 'c') {
            calibration_mode = !calibration_mode;
            std::cout << "Calibration grid: " << (calibration_mode ? "ON" : "OFF") << "\n";
        }
        else if (key == 'p') {
            std::cout << "\n--- Current Transformation Values ---\n";
            std::cout << "Scale X: " << calib.scale_x << "\n";
            std::cout << "Scale Y: " << calib.scale_y << "\n";
            std::cout << "Offset X: " << calib.offset_x << "\n";
            std::cout << "Offset Y: " << calib.offset_y << "\n";
            std::cout << "Rotation: " << calib.rotation << " deg\n";
            std::cout << "Alpha: " << calib.alpha << "\n";
            std::cout << "-------------------------------------\n";
        }
        else if (key == 'r') {
            calib = CalibrationParams();
            std::cout << "Reset transformation\n";
        }
    }

    // Cleanup
    g_running = false;
    sender.stop_stream();
    rgb_cam.release();
    cv::destroyAllWindows();

    if (thermal_thread.joinable()) {
        thermal_thread.join();
    }

    std::cout << "Dual camera viewer closed.\n";
    return 0;
}
