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
#include "common_libs/Logger.h"

Signal_Cal::Signal_Cal()
{
}

std::map<RGB, double> Signal_Cal::read_DCF(const std::string& dcf_filename) {
    std::ifstream file(dcf_filename);
    std::map<RGB, double> colorToDbm;
    
    if (!file.is_open()) {
        Logger::log_message(Logger::TYPE::ERROR, "Cannot open DCF file: " + dcf_filename);
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

CoverageMatrix Signal_Cal::read_Coverage_File(const std::string& ppm_filename, const std::string& dcf_filename, const std::vector<double> &lat_lon, const double &threshold)
{
    std::ifstream file(ppm_filename, std::ios::binary);
    if (!file) {
        Logger::log_message(Logger::TYPE::ERROR, "Cannot open PPM file: " + ppm_filename);
        return {};
    }

    std::string magic;
    file >> magic;
    if (magic != "P6") {
        Logger::log_message(Logger::TYPE::ERROR, "Error in PPM version, only accept P6");
        return {};
    }

    char ch = file.peek();
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
        for (auto& kv : colorMap) {
            RGB cc = kv.first;
            if (std::abs(int(c.r) - int(cc.r)) <= 1 &&
                std::abs(int(c.g) - int(cc.g)) <= 1 &&
                std::abs(int(c.b) - int(cc.b)) <= 1)
                return kv.second;
        }
        return -120.0;
    };

    CoverageMatrix matrix(rows, std::vector<double>(cols, -120.0));

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            unsigned char rgb[3];
            file.read(reinterpret_cast<char*>(rgb), 3);
            if (!file) {
                Logger::log_message(Logger::TYPE::ERROR, "Error reading PPM, EOF unexpected");
                return {};
            }

            RGB color{rgb[0], rgb[1], rgb[2]};

            double value = findColor(color);
            matrix[row][col] = value;
        }
    }

    return matrix;
}

std::vector<double> Signal_Cal::parse_Bounds(const std::string& str)
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

std::vector<Struct_Algo::Coordinate> matrixToVector(
    const CoverageMatrix& matrix,
    double latMax,
    double latMin,
    double lonMin,
    double lonMax,
    double threshold
) {
    int rows = matrix.size();
    int cols = matrix[0].size();

    double latStep = (latMax - latMin) / rows;
    double lonStep = (lonMax - lonMin) / cols;

    std::vector<Struct_Algo::Coordinate> points;

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            double value = matrix[row][col];
            if (value >= threshold) {
                Struct_Algo::Coordinate p(lonMin + col * lonStep,latMax - row * latStep);
                points.push_back(p);
            }
        }
    }

    return points;
}

std::vector<Struct_Algo::Coordinate> Signal_Cal::calculate_signal(const Struct_Algo::Config_struct &global_config, const Struct_Algo::SignalServerConfig &signal_server_conf)
{
    std::vector<Struct_Algo::Coordinate> points_empty;
    std::string cmd;
    if (!signal_server_conf.toCommand(global_config.signal_server_path,cmd)) {
        Logger::log_message(Logger::TYPE::ERROR, "Error creating Signal-Server command");
        return points_empty;
    }

    cmd += " > output.txt 2>&1";
    
    Logger::log_message(Logger::TYPE::INFO, "Executing Signal-Server command: " + cmd);

    if (std::system(cmd.c_str()) == 0) {
        Logger::log_message(Logger::TYPE::INFO, "Signal-Server command execute succesfully");
    } else {
        Logger::log_message(Logger::TYPE::ERROR, "Error executing Signal-Server command");
        return points_empty;
    }

    std::string path = std::string(global_config.executable_path) + "/output.txt";
    std::ifstream in(path);
    if (!in.is_open()) {
        Logger::log_message(Logger::TYPE::ERROR, "Cannot open output.txt file: " + path);
        return points_empty;
    }

    std::string line;
    if (!std::getline(in, line)) {
        Logger::log_message(Logger::TYPE::ERROR, "Error getting Signal-Server output");
        return points_empty;
    }
    in.close();

    std::vector<double> values = parse_Bounds(line);

    if (values.size() != 4) {
        Logger::log_message(Logger::TYPE::ERROR, "Error decoding Signal-Server output: " + line);
        return points_empty;
    }

    std::string dcfFilename = signal_server_conf.outputFile + ".dcf";
    std::string ppmPath = global_config.executable_path + "/" + signal_server_conf.outputFile + ".ppm";
    std::string dcfPath = global_config.executable_path + "/" + dcfFilename;
    
    Logger::log_message(Logger::TYPE::INFO, "PPM path: " + ppmPath);
    Logger::log_message(Logger::TYPE::INFO, "DCF path: " + dcfPath);

    auto matrix = read_Coverage_File(ppmPath, dcfPath, values, global_config.threshold);

    if (matrix.empty())
    {
        Logger::log_message(Logger::TYPE::ERROR, "Error reading coverage file");
        return points_empty;
    }
    
    auto vector = matrixToVector(matrix,values[0],values[2],values[3],values[1],global_config.threshold);
    
    if (vector.empty())
        Logger::log_message(Logger::TYPE::ERROR, "Error parsing matrix into a vector");

    return vector;

}