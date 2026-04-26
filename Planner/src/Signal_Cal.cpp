/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Signal_Cal.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "./Signal_Cal.h"
#include <vector>
#include <map>
#include <algorithm>
#include <sstream>
#if defined(__has_include)
#if __has_include(<format>)
#include <format>
#endif
#endif
#include "common_libs/Logger.h"

namespace {
CoverageMatrix merge_max(const CoverageMatrix& a, const CoverageMatrix& b)
{
    if (a.empty()) return b;
    if (b.empty()) return a;

    if (a.size() != b.size() || a[0].size() != b[0].size()) {
        return a;
    }

    CoverageMatrix merged = a;
    for (size_t r = 0; r < merged.size(); ++r) {
        for (size_t c = 0; c < merged[r].size(); ++c) {
            merged[r][c] = std::max(merged[r][c], b[r][c]);
        }
    }
    return merged;
}

std::string format_cfg_output_file(const std::string& base_output_file, size_t idx)
{
#if defined(__cpp_lib_format) && (__cpp_lib_format >= 201907L)
    return std::format("{}_cfg_{}", base_output_file, idx);
#else
    std::ostringstream oss;
    oss << base_output_file << "_cfg_" << idx;
    return oss.str();
#endif
}

std::string format_config_indexed_message(const std::string& prefix, size_t idx)
{
#if defined(__cpp_lib_format) && (__cpp_lib_format >= 201907L)
    return std::format("{}{}", prefix, idx);
#else
    std::ostringstream oss;
    oss << prefix << idx;
    return oss.str();
#endif
}
}

std::map<RGB, double> Signal_Cal::read_DCF(const std::string& dcf_filename) const{
    std::ifstream file(dcf_filename);
    std::map<RGB, double> colorToDbm;
    
    if (!file.is_open()) {
        Logger::log_message(Logger::Type::ERROR, "Cannot open DCF file: " + dcf_filename);
        return colorToDbm;
    }
    
    std::string line;

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        line.erase(0, line.find_first_not_of(" \t"));

        auto colon_pos = line.find(':');
        if (colon_pos == std::string::npos) continue;

        double dbm = std::stod(line.substr(0, colon_pos));
        std::string rgb_str = line.substr(colon_pos + 1);

        rgb_str.erase(0, rgb_str.find_first_not_of(" \t"));

        int r, g, b;
        char comma;
        std::stringstream ss(rgb_str);
        ss >> r >> comma >> g >> comma >> b;

        colorToDbm[{static_cast<unsigned char>(r),
                    static_cast<unsigned char>(g),
                    static_cast<unsigned char>(b)}] = dbm;
    }

    return colorToDbm;
}

CoverageMatrix Signal_Cal::read_Coverage_File(const std::string& ppm_filename, const std::string& dcf_filename) const
{
    std::ifstream file(ppm_filename, std::ios::binary);
    if (!file) {
        Logger::log_message(Logger::Type::ERROR, "Cannot open PPM file: " + ppm_filename);
        return {};
    }

    std::string magic;
    file >> magic;
    if (magic != "P6") {
        Logger::log_message(Logger::Type::ERROR, "Error in PPM version, only accept P6");
        return {};
    }

    std::ifstream::int_type ch = file.peek();
    while (ch == '#') {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ch = file.peek();
    }

    int cols = 0, rows = 0, maxval = 0;
    file >> cols >> rows >> maxval;

    file.get();
    while (isspace(file.peek())) file.get();

    auto colorMap = read_DCF(dcf_filename);

    if (colorMap.empty())
        return {};

    auto findColor = [&](RGB c) -> double {
        for (auto& [first, second] : colorMap) {
            if (std::abs(int(c.r) - int(first.r)) <= 1 &&
                std::abs(int(c.g) - int(first.g)) <= 1 &&
                std::abs(int(c.b) - int(first.b)) <= 1)
                return second;
        }
        return -120.0;
    };

    CoverageMatrix matrix(rows, std::vector<double>(cols, -120.0));

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            unsigned char rgb[3];
            file.read(reinterpret_cast<char*>(rgb), 3);
            if (!file) {
                Logger::log_message(Logger::Type::ERROR, "Error reading PPM, EOF unexpected");
                return {};
            }

            RGB color{rgb[0], rgb[1], rgb[2]};

            double value = findColor(color);
            matrix[row][col] = value;
        }
    }

    return matrix;
}

std::vector<double> Signal_Cal::parse_Bounds(const std::string& str) const
{
    std::vector<double> values;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, '|')) {
        if (!token.empty()) {
            values.push_back(std::stod(token));
        }
    }

    return values;
}

std::vector<Struct_Planner::Coordinate> matrixToVector(
    const CoverageMatrix& matrix,
    double latMax,
    double latMin,
    double lonMin,
    double lonMax,
    double threshold
) {
    auto rows = static_cast<int>(matrix.size());
    auto cols = static_cast<int>(matrix[0].size());

    double latStep = (latMax - latMin) / rows;
    double lonStep = (lonMax - lonMin) / cols;

    std::vector<Struct_Planner::Coordinate> points;

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            double value = matrix[row][col];
            if (value >= threshold) {
                Struct_Planner::Coordinate p(lonMin + col * lonStep,latMax - row * latStep);
                points.push_back(p);
            }
        }
    }

    return points;
}

std::vector<Struct_Planner::Coordinate> Signal_Cal::calculate_signal(const Struct_Planner::Config_struct &global_config, const std::vector<Struct_Planner::SignalServerConfig> &signal_server_confs) const
{
    std::vector<Struct_Planner::Coordinate> points_empty;
    if (signal_server_confs.empty()) {
        Logger::log_message(Logger::Type::ERROR, "Signal config list is empty");
        return points_empty;
    }

    CoverageMatrix merged_matrix;
    std::vector<double> bounds;

    for (size_t i = 0; i < signal_server_confs.size(); ++i) {
        auto cfg = signal_server_confs[i];
        cfg.filePaths.outputFile = format_cfg_output_file(signal_server_confs[i].filePaths.outputFile, i);

        std::string cmd;
        if (!cfg.toCommand(global_config.signal_server_path, cmd)) {
            Logger::log_message(
                Logger::Type::ERROR,
                format_config_indexed_message("Error creating Signal-Server command for config ", i));
            return points_empty;
        }

        cmd += " > output.txt 2>&1";
        Logger::log_message(Logger::Type::INFO, "Executing Signal-Server command: " + cmd);

        if (std::system(cmd.c_str()) != 0) {
            Logger::log_message(
                Logger::Type::ERROR,
                format_config_indexed_message("Error executing Signal-Server command for config ", i));
            return points_empty;
        }

        std::string path = std::string(global_config.executable_path) + "/output.txt";
        std::ifstream in(path);
        if (!in.is_open()) {
            Logger::log_message(Logger::Type::ERROR, "Cannot open output.txt file: " + path);
            return points_empty;
        }

        std::string line;
        if (!std::getline(in, line)) {
            Logger::log_message(Logger::Type::ERROR, "Error getting Signal-Server output");
            return points_empty;
        }
        in.close();

        auto values = parse_Bounds(line);
        if (values.size() != 4) {
            Logger::log_message(Logger::Type::ERROR, "Error decoding Signal-Server output: " + line);
            return points_empty;
        }

        if (bounds.empty()) {
            bounds = values;
        }

        std::string dcfFilename = cfg.filePaths.outputFile + ".dcf";
        std::string ppmPath = global_config.executable_path + "/" + cfg.filePaths.outputFile + ".ppm";
        std::string dcfPath = global_config.executable_path + "/" + dcfFilename;

        Logger::log_message(Logger::Type::INFO, "PPM path: " + ppmPath);
        Logger::log_message(Logger::Type::INFO, "DCF path: " + dcfPath);

        auto matrix = read_Coverage_File(ppmPath, dcfPath);
        if (matrix.empty()) {
            Logger::log_message(
                Logger::Type::ERROR,
                format_config_indexed_message("Error reading coverage file for config ", i));
            return points_empty;
        }

        merged_matrix = merge_max(merged_matrix, matrix);
    }

    if (bounds.size() != 4 || merged_matrix.empty()) {
        Logger::log_message(Logger::Type::ERROR, "Error creating merged coverage matrix");
        return points_empty;
    }

    auto vector = matrixToVector(merged_matrix, bounds[0], bounds[2], bounds[3], bounds[1], global_config.threshold);
    
    if (vector.empty())
        Logger::log_message(Logger::Type::ERROR, "Error parsing matrix into a vector");

    return vector;

}