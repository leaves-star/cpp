#ifndef __CPUMONITOR_H__
#define __CPUMONITOR_H__

#include <string>
#include <fstream>
#include <iostream> 

class CpuMonitor {
public:
    double GetTemperature() {
        if (cpu_temp_fs_.is_open()) {
            std::string buf;
            cpu_temp_fs_ >> buf;
            cpu_temp_fs_.seekg(0);
            try {
                double temperature = std::stoi(buf) / 10;
                temperature /= 100;
                return temperature;
            } catch (...) {
            }
        }
        
        double temperature = -1.0;
        for (int i = 0; temperature < 0; ++i) {
            temperature = GetTemperature(i);
            if (temperature == -2.0) {
                return -1.0;
            }
        }
        
        return temperature;
    }
    
private:
    double GetTemperature(int n) {
        std::string path = "/sys/class/thermal/thermal_zone" + std::to_string(n);
        std::fstream f;
        f.open(path + "/type", std::ios::in);
        if (!f.good()) {
            return -2.0;
        }
        
        std::string buf;
        f >> buf;
        f.seekg(0);
        
//#ifdef __aarch64__
//        std::string type = "CPU-therm";
//#else
//        std::string type = "x86_pkg_temp";
//#endif
        std::string type = "cpu-thermal";

        if (type != buf) {
            return -1.0;
        }
        cpu_temp_fs_.open(path + "/temp", std::ios::in);
        cpu_temp_fs_ >> buf;
        cpu_temp_fs_.seekg(0);
        try {
            double temperature = std::stoi(buf) / 10;
            temperature /= 100;
            return temperature;
        } catch (...) {
        }
        return -1.0;
    }
    
private:
    std::fstream cpu_temp_fs_;
};

#endif //__CPUMONITOR_H__
