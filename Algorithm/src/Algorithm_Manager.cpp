/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Algorithm_Manager.cpp               
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Algorithm_Manager.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <tuple>
#include <iomanip>
#include "structs/Structs_Algo.h"
#include "common_libs/Logger.h"

Algorithm_Manager::Algorithm_Manager(std::shared_ptr<Communication_Manager> &comm_mng, 
                                     std::shared_ptr<Algorithm_Recorder> &rec_mng,
                                     const std::string &path_signal,
                                     const std::string &executable_path,
                                     const double &threshold): comm_mng_ptr_(std::move(comm_mng)),
                                                              recorder_ptr_(std::move(rec_mng)),
                                                              path_signal_(path_signal),
                                                              executable_path_(executable_path),
                                                              threshold_(threshold)
{
    comm_mng_ptr_->set_calculate_handler(std::bind(&Algorithm_Manager::calculate, this, std::placeholders::_1));
}

Algorithm_Manager::~Algorithm_Manager()
{

}

std::map<RGB, double> Algorithm_Manager::read_DCF(const std::string& dcf_filename) {
    std::ifstream file(dcf_filename);
    std::map<RGB, double> colorToDbm;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        double dbm;
        char colon;
        int r, g, b;
        ss >> dbm >> colon >> r >> g >> b;
        colorToDbm[{r, g, b}] = dbm;
    }
    return colorToDbm;
}

std::vector<CoveragePoint> Algorithm_Manager::read_Coverage_File(const std::string& ppm_filename, const std::string& dcf_Filename, const std::vector<double> &lat_lon)
{
    std::ifstream file(ppm_filename, std::ios::binary);
    if (!file) return {};

    std::string magic;
    int cols, rows, maxval;
    file >> magic >> cols >> rows >> maxval;
    file.ignore(1); // skip single whitespace after header

    auto colorMap = read_DCF(dcf_Filename);

    std::vector<CoveragePoint> points;
    points.reserve(cols * rows);

    double latMax = lat_lon[0];
    double lonMax = lat_lon[1];
    double latMin = lat_lon[2];
    double lonMin = lat_lon[3];

    double latStep = (latMax - latMin) / rows;
    double lonStep = (lonMax - lonMin) / cols;

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            unsigned char rgb[3];
            file.read(reinterpret_cast<char*>(rgb), 3);
            RGB color{rgb[0], rgb[1], rgb[2]};

            double value = 0;
            auto it = colorMap.find(color);
            if (it != colorMap.end()) value = it->second;

            CoveragePoint p;
            p.lat = latMax - row * latStep;
            p.lon = lonMin + col * lonStep;
            p.value = value;
            p.hasCoverage = (value >= threshold_);

            points.push_back(p);
        }
    }

    return points;
}

std::vector<double> Algorithm_Manager::parse_Bounds(const std::string& str)
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

void Algorithm_Manager::calculate(const Struct_Algo::SignalServerConfig &config)
{
    comm_mng_ptr_->set_status(Struct_Algo::Status::CALCULATING);
    std::string cmd;
    if (!config.toCommand(path_signal_,cmd)) {
        Logger::log_message(Logger::TYPE::ERROR, "Error creating Signal-Server command");
        comm_mng_ptr_->set_status(Struct_Algo::Status::ERROR);
        return;
    }

    cmd += " > output.txt 2>&1";
    
    Logger::log_message(Logger::TYPE::INFO, "Executing Signal-Server command: " + cmd);

    if (std::system(cmd.c_str()) == 0) {
        Logger::log_message(Logger::TYPE::INFO, "Signal-Server command execute succesfully");
    } else {
        Logger::log_message(Logger::TYPE::ERROR, "Error executing Signal-Server command");
        comm_mng_ptr_->set_status(Struct_Algo::Status::ERROR);
        return;
    }

    std::string path = std::string(executable_path_) + "/output.txt";
    std::ifstream in(path);
    if (!in.is_open()) {
        Logger::log_message(Logger::TYPE::ERROR, "Cannot open output.txt file: " + path);
        comm_mng_ptr_->set_status(Struct_Algo::Status::ERROR);
        return;
    }

    std::string line;
    if (!std::getline(in, line)) {
        Logger::log_message(Logger::TYPE::ERROR, "Error getting Signal-Server output");
        comm_mng_ptr_->set_status(Struct_Algo::Status::ERROR);
        return;
    }
    in.close();

    std::vector<double> values = parse_Bounds(line);

    if (values.size() != 4) {
        Logger::log_message(Logger::TYPE::ERROR, "Error decoding Signal-Server output: " + line);
        comm_mng_ptr_->set_status(Struct_Algo::Status::ERROR);
        return;
    }

    std::string dcfFilename = config.outputFile.substr(0, config.outputFile.size() - 3) + "dcf";

    auto points = read_Coverage_File(executable_path_ + "/" + config.outputFile + ".ppm",executable_path_ + "/" + dcfFilename,values);

    if (points.empty()) {
        Logger::log_message(Logger::TYPE::ERROR, "Error getting Signal-Server output's values: " + line);
        comm_mng_ptr_->set_status(Struct_Algo::Status::ERROR);
        return;
    }

    // TODO Move to recorder class
    Logger::log_message(Logger::TYPE::INFO, "Writting csv coverage map");
    std::ofstream out(executable_path_ + "/coverage_map.csv");
    out << "lat,lon,coverage\n";
    for (const auto& p : points)
        out << p.lat << "," << p.lon << "," << (p.hasCoverage ? 1 : 0) << "\n";

    //TODO Calculate optime path
    Logger::log_message(Logger::TYPE::INFO, "Algorithm_Manager task finish correctly");
    comm_mng_ptr_->set_status(Struct_Algo::Status::FINISH);
}