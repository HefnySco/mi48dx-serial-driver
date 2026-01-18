#ifndef THERMAL_VISUALIZATION_HPP
#define THERMAL_VISUALIZATION_HPP

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <cstdint>

enum class ThermalPalette {
    RAINBOW,        // OpenCV COLORMAP_RAINBOW (current)
    VIVID,          // High-contrast vivid colors
    RGB,            // RGB-based thermal mapping
    GRAYSCALE,      // Simple grayscale
    MEDICAL,        // Medical thermal palette
    IRON,           // Iron/black hot palette
    FIRE,           // Fire palette
    OCEAN,          // Ocean/blue palette
    CUSTOM          // User-defined palette
};

class ThermalVisualization {
public:
    ThermalVisualization();
    ~ThermalVisualization() = default;

    // Main visualization function
    cv::Mat visualizeTemperatures(const std::vector<float>& temperatures, 
                                 uint16_t rows, uint16_t cols,
                                 ThermalPalette palette = ThermalPalette::RAINBOW,
                                 float min_temp = -273.15f, float max_temp = 100.0f,
                                 int scale_factor = 4, bool rotate = true);

    // Set custom palette colors
    void setCustomPalette(const std::vector<cv::Vec3b>& colors);

    // Get available palettes as strings
    static std::vector<std::string> getAvailablePalettes();
    static ThermalPalette paletteFromString(const std::string& palette_name);

    // Utility functions
    static cv::Mat applyVividPalette(const cv::Mat& normalized_temp);
    static cv::Mat applyRGBPalette(const cv::Mat& normalized_temp);
    static cv::Mat applyMedicalPalette(const cv::Mat& normalized_temp);
    static cv::Mat applyIronPalette(const cv::Mat& normalized_temp);
    static cv::Mat applyFirePalette(const cv::Mat& normalized_temp);
    static cv::Mat applyOceanPalette(const cv::Mat& normalized_temp);

private:
    std::vector<cv::Vec3b> custom_palette_;
    bool has_custom_palette_;

    // Helper functions
    cv::Mat normalizeTemperatures(const std::vector<float>& temperatures, 
                                 uint16_t rows, uint16_t cols,
                                 float min_temp, float max_temp);
    cv::Mat createColorMatrix(const cv::Mat& normalized_temp, ThermalPalette palette);
    
    // Color interpolation helper (static)
    static cv::Vec3b interpolateColor(float value, const std::vector<cv::Vec3b>& palette);
};

#endif // THERMAL_VISUALIZATION_HPP
