#include "thermal_visualization.hpp"
#include <algorithm>
#include <cmath>

ThermalVisualization::ThermalVisualization() : has_custom_palette_(false) {}

cv::Mat ThermalVisualization::visualizeTemperatures(const std::vector<float>& temperatures, 
                                                   uint16_t rows, uint16_t cols,
                                                   ThermalPalette palette,
                                                   float min_temp, float max_temp,
                                                   int scale_factor, bool rotate) {
    if (temperatures.empty()) {
        return cv::Mat();
    }

    // Normalize temperatures to 0-1 range
    cv::Mat normalized = normalizeTemperatures(temperatures, rows, cols, min_temp, max_temp);
    
    // Convert to 0-255 range for processing
    cv::Mat normalized_8u;
    normalized.convertTo(normalized_8u, CV_8U, 255.0);
    
    // Apply color palette
    cv::Mat colored;
    switch (palette) {
        case ThermalPalette::RAINBOW:
            cv::applyColorMap(normalized_8u, colored, cv::COLORMAP_RAINBOW);
            cv::cvtColor(colored, colored, cv::COLOR_BGR2RGB);
            break;
        case ThermalPalette::VIVID:
            colored = applyVividPalette(normalized);
            break;
        case ThermalPalette::RGB:
            colored = applyRGBPalette(normalized);
            break;
        case ThermalPalette::GRAYSCALE:
            cv::cvtColor(normalized_8u, colored, cv::COLOR_GRAY2BGR);
            break;
        case ThermalPalette::MEDICAL:
            colored = applyMedicalPalette(normalized);
            break;
        case ThermalPalette::IRON:
            colored = applyIronPalette(normalized);
            break;
        case ThermalPalette::FIRE:
            colored = applyFirePalette(normalized);
            break;
        case ThermalPalette::OCEAN:
            colored = applyOceanPalette(normalized);
            break;
        case ThermalPalette::CUSTOM:
            if (has_custom_palette_) {
                colored = createColorMatrix(normalized, palette);
            } else {
                cv::applyColorMap(normalized_8u, colored, cv::COLORMAP_RAINBOW);
            }
            break;
    }

    // Resize for better visibility
    cv::Mat resized;
    cv::resize(colored, resized, cv::Size(cols * scale_factor, rows * scale_factor), 
               0, 0, cv::INTER_NEAREST);

    // Rotate 90 degrees clockwise to correct -90 degree rotation
    if (rotate) {
        cv::Mat rotated;
        cv::rotate(resized, rotated, cv::ROTATE_90_CLOCKWISE);
        return rotated;
    }

    return resized;
}

cv::Mat ThermalVisualization::normalizeTemperatures(const std::vector<float>& temperatures, 
                                                   uint16_t rows, uint16_t cols,
                                                   float min_temp, float max_temp) {
    cv::Mat frame(rows, cols, CV_32F);
    
    // Convert vector to matrix
    for (uint16_t c = 0; c < cols; ++c) {
        for (uint16_t r = 0; r < rows; ++r) {
            frame.at<float>(r, c) = temperatures[c * rows + r];
        }
    }

    // Find actual min and max in the data if not provided
    if (min_temp == -273.15f && max_temp == 100.0f) {
        double min_d, max_d;
        cv::minMaxLoc(frame, &min_d, &max_d);
        min_temp = static_cast<float>(min_d);
        max_temp = static_cast<float>(max_d);
    }

    // Normalize to 0-1 range
    cv::Mat normalized;
    frame.convertTo(normalized, CV_32F, 1.0 / (max_temp - min_temp), 
                    -min_temp / (max_temp - min_temp));
    
    // Clamp values to [0, 1]
    cv::threshold(normalized, normalized, 1.0, 1.0, cv::THRESH_TRUNC);
    cv::threshold(normalized, normalized, 0.0, 0.0, cv::THRESH_TOZERO);

    return normalized;
}

cv::Mat ThermalVisualization::applyVividPalette(const cv::Mat& normalized_temp) {
    cv::Mat result(normalized_temp.rows, normalized_temp.cols, CV_8UC3);
    
    // Vivid palette: Blue -> Cyan -> Green -> Yellow -> Red -> Magenta
    std::vector<cv::Vec3b> vivid_palette = {
        cv::Vec3b(255, 0, 0),      // Blue (BGR)
        cv::Vec3b(255, 255, 0),    // Cyan (BGR)
        cv::Vec3b(0, 255, 0),      // Green (BGR)
        cv::Vec3b(0, 255, 255),    // Yellow (BGR)
        cv::Vec3b(0, 0, 255),      // Red (BGR)
        cv::Vec3b(255, 0, 255)     // Magenta (BGR)
    };

    for (int r = 0; r < normalized_temp.rows; ++r) {
        for (int c = 0; c < normalized_temp.cols; ++c) {
            float value = normalized_temp.at<float>(r, c);
            result.at<cv::Vec3b>(r, c) = interpolateColor(value, vivid_palette);
        }
    }

    return result;
}

cv::Mat ThermalVisualization::applyRGBPalette(const cv::Mat& normalized_temp) {
    cv::Mat result(normalized_temp.rows, normalized_temp.cols, CV_8UC3);
    
    for (int r = 0; r < normalized_temp.rows; ++r) {
        for (int c = 0; c < normalized_temp.cols; ++c) {
            float value = normalized_temp.at<float>(r, c);
            
            // RGB thermal mapping: Cold (blue) to Hot (red/white)
            uint8_t red, green, blue;
            
            if (value < 0.25f) {
                // Blue to Cyan
                float t = value * 4.0f;
                red = 0;
                green = static_cast<uint8_t>(t * 255);
                blue = 255;
            } else if (value < 0.5f) {
                // Cyan to Green
                float t = (value - 0.25f) * 4.0f;
                red = 0;
                green = 255;
                blue = static_cast<uint8_t>((1.0f - t) * 255);
            } else if (value < 0.75f) {
                // Green to Yellow
                float t = (value - 0.5f) * 4.0f;
                red = static_cast<uint8_t>(t * 255);
                green = 255;
                blue = 0;
            } else {
                // Yellow to White
                float t = (value - 0.75f) * 4.0f;
                red = 255;
                green = 255;
                blue = static_cast<uint8_t>(t * 255);
            }
            
            result.at<cv::Vec3b>(r, c) = cv::Vec3b(blue, green, red);
        }
    }

    return result;
}

cv::Mat ThermalVisualization::applyMedicalPalette(const cv::Mat& normalized_temp) {
    cv::Mat result(normalized_temp.rows, normalized_temp.cols, CV_8UC3);
    
    // Medical palette: Black -> Blue -> Green -> Yellow -> Red -> White
    std::vector<cv::Vec3b> medical_palette = {
        cv::Vec3b(0, 0, 0),        // Black (BGR)
        cv::Vec3b(128, 0, 0),      // Dark Blue (BGR)
        cv::Vec3b(255, 128, 0),    // Blue (BGR)
        cv::Vec3b(128, 255, 0),    // Cyan-Green (BGR)
        cv::Vec3b(0, 255, 128),    // Green-Yellow (BGR)
        cv::Vec3b(0, 128, 255),    // Orange (BGR)
        cv::Vec3b(0, 0, 255),      // Red (BGR)
        cv::Vec3b(255, 255, 255)   // White (BGR)
    };

    for (int r = 0; r < normalized_temp.rows; ++r) {
        for (int c = 0; c < normalized_temp.cols; ++c) {
            float value = normalized_temp.at<float>(r, c);
            result.at<cv::Vec3b>(r, c) = interpolateColor(value, medical_palette);
        }
    }

    return result;
}

cv::Mat ThermalVisualization::applyIronPalette(const cv::Mat& normalized_temp) {
    cv::Mat result(normalized_temp.rows, normalized_temp.cols, CV_8UC3);
    
    // Iron palette: Black -> Dark red -> Red -> Orange -> Yellow -> White
    std::vector<cv::Vec3b> iron_palette = {
        cv::Vec3b(0, 0, 0),        // Black (BGR)
        cv::Vec3b(0, 0, 32),       // Dark red (BGR)
        cv::Vec3b(0, 0, 128),      // Red (BGR)
        cv::Vec3b(0, 64, 255),     // Red-orange (BGR)
        cv::Vec3b(0, 128, 255),    // Orange (BGR)
        cv::Vec3b(0, 192, 255),    // Yellow-orange (BGR)
        cv::Vec3b(128, 255, 255),  // Light yellow (BGR)
        cv::Vec3b(255, 255, 255)   // White (BGR)
    };

    for (int r = 0; r < normalized_temp.rows; ++r) {
        for (int c = 0; c < normalized_temp.cols; ++c) {
            float value = normalized_temp.at<float>(r, c);
            result.at<cv::Vec3b>(r, c) = interpolateColor(value, iron_palette);
        }
    }

    return result;
}

cv::Mat ThermalVisualization::applyFirePalette(const cv::Mat& normalized_temp) {
    cv::Mat result(normalized_temp.rows, normalized_temp.cols, CV_8UC3);
    
    // Fire palette: Black -> Red -> Yellow -> White
    std::vector<cv::Vec3b> fire_palette = {
        cv::Vec3b(0, 0, 0),        // Black (BGR)
        cv::Vec3b(0, 0, 64),       // Dark red (BGR)
        cv::Vec3b(0, 0, 255),      // Red (BGR)
        cv::Vec3b(0, 128, 255),    // Orange (BGR)
        cv::Vec3b(0, 255, 255),    // Yellow (BGR)
        cv::Vec3b(128, 255, 255),  // Light yellow (BGR)
        cv::Vec3b(255, 255, 255)   // White (BGR)
    };

    for (int r = 0; r < normalized_temp.rows; ++r) {
        for (int c = 0; c < normalized_temp.cols; ++c) {
            float value = normalized_temp.at<float>(r, c);
            result.at<cv::Vec3b>(r, c) = interpolateColor(value, fire_palette);
        }
    }

    return result;
}

cv::Mat ThermalVisualization::applyOceanPalette(const cv::Mat& normalized_temp) {
    cv::Mat result(normalized_temp.rows, normalized_temp.cols, CV_8UC3);
    
    // Ocean palette: Black -> Deep blue -> Blue -> Cyan -> Light cyan -> White
    std::vector<cv::Vec3b> ocean_palette = {
        cv::Vec3b(0, 0, 0),        // Black (BGR)
        cv::Vec3b(32, 0, 0),       // Deep blue (BGR)
        cv::Vec3b(128, 0, 0),      // Blue (BGR)
        cv::Vec3b(255, 128, 0),    // Light blue (BGR)
        cv::Vec3b(255, 192, 0),    // Cyan (BGR)
        cv::Vec3b(255, 255, 128),  // Light cyan (BGR)
        cv::Vec3b(255, 255, 192),  // Very light cyan (BGR)
        cv::Vec3b(255, 255, 255)   // White (BGR)
    };

    for (int r = 0; r < normalized_temp.rows; ++r) {
        for (int c = 0; c < normalized_temp.cols; ++c) {
            float value = normalized_temp.at<float>(r, c);
            result.at<cv::Vec3b>(r, c) = interpolateColor(value, ocean_palette);
        }
    }

    return result;
}

cv::Mat ThermalVisualization::createColorMatrix(const cv::Mat& normalized_temp, ThermalPalette palette) {
    cv::Mat result(normalized_temp.rows, normalized_temp.cols, CV_8UC3);
    
    for (int r = 0; r < normalized_temp.rows; ++r) {
        for (int c = 0; c < normalized_temp.cols; ++c) {
            float value = normalized_temp.at<float>(r, c);
            result.at<cv::Vec3b>(r, c) = interpolateColor(value, custom_palette_);
        }
    }

    return result;
}

cv::Vec3b ThermalVisualization::interpolateColor(float value, const std::vector<cv::Vec3b>& palette) {
    if (palette.empty()) {
        return cv::Vec3b(0, 0, 0);
    }
    
    if (palette.size() == 1) {
        return palette[0];
    }

    // Clamp value to [0, 1]
    value = std::max(0.0f, std::min(1.0f, value));
    
    // Find the two colors to interpolate between
    float scaled_value = value * (palette.size() - 1);
    int index = static_cast<int>(scaled_value);
    float fraction = scaled_value - index;
    
    // Handle edge case
    if (index >= palette.size() - 1) {
        return palette.back();
    }
    
    // Linear interpolation
    const cv::Vec3b& color1 = palette[index];
    const cv::Vec3b& color2 = palette[index + 1];
    
    cv::Vec3b result;
    result[0] = static_cast<uint8_t>(color1[0] + fraction * (color2[0] - color1[0]));
    result[1] = static_cast<uint8_t>(color1[1] + fraction * (color2[1] - color1[1]));
    result[2] = static_cast<uint8_t>(color1[2] + fraction * (color2[2] - color1[2]));
    
    return result;
}

void ThermalVisualization::setCustomPalette(const std::vector<cv::Vec3b>& colors) {
    custom_palette_ = colors;
    has_custom_palette_ = true;
}

std::vector<std::string> ThermalVisualization::getAvailablePalettes() {
    return {
        "RAINBOW",
        "VIVID", 
        "RGB",
        "GRAYSCALE",
        "MEDICAL",
        "IRON",
        "FIRE",
        "OCEAN",
        "CUSTOM"
    };
}

ThermalPalette ThermalVisualization::paletteFromString(const std::string& palette_name) {
    if (palette_name == "RAINBOW") return ThermalPalette::RAINBOW;
    if (palette_name == "VIVID") return ThermalPalette::VIVID;
    if (palette_name == "RGB") return ThermalPalette::RGB;
    if (palette_name == "GRAYSCALE") return ThermalPalette::GRAYSCALE;
    if (palette_name == "MEDICAL") return ThermalPalette::MEDICAL;
    if (palette_name == "IRON") return ThermalPalette::IRON;
    if (palette_name == "FIRE") return ThermalPalette::FIRE;
    if (palette_name == "OCEAN") return ThermalPalette::OCEAN;
    if (palette_name == "CUSTOM") return ThermalPalette::CUSTOM;
    
    // Default to RAINBOW if not found
    return ThermalPalette::RAINBOW;
}
