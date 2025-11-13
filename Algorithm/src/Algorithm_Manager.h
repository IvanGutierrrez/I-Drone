/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Algorithm_Manager.h                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <iostream>
#include <memory>
#include <map>
#include "Algorithm_Manager_Interface.h"
#include "Communication_Manager.h"
#include "Algorithm_Recorder.h"

struct CoveragePoint {
    double lat;
    double lon;
    double value;   // señal en dBm
    bool hasCoverage;
};

struct RGB {
    int r, g, b;
    bool operator<(const RGB &other) const {
        return std::tie(r, g, b) < std::tie(other.r, other.g, other.b);
    }
};

class Algorithm_Manager: public Algorithm_Manager_Interface {

private:
    std::shared_ptr<Communication_Manager> comm_mng_ptr_;
    std::shared_ptr<Algorithm_Recorder> recorder_ptr_;
    std::string path_signal_;
    std::string executable_path_;
    double threshold_;

    std::vector<CoveragePoint> read_Coverage_File(const std::string& filename, const std::string& dcfFilename, const std::vector<double> &lat_lot);
    std::vector<double> parse_Bounds(const std::string& str);
    std::map<RGB, double> read_DCF(const std::string& dcf_filename);

public:
    Algorithm_Manager(std::shared_ptr<Communication_Manager> &comm_mng, 
                      std::shared_ptr<Algorithm_Recorder> &rec_mng, 
                      const std::string &path_signal,
                      const std::string &executable_path,
                      const double &threshold);
    ~Algorithm_Manager() override;
    void calculate(const Struct_Algo::SignalServerConfig &config) override;
};