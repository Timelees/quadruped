#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include "log_save.h"

int main(){
    namespace fs = std::filesystem;
    using quadruped::logs::LogSave;
    
    LogSave logger;
    logger.setEnabled(true);        // 设置日志是否启用
    // prepare data and header
    Eigen::Vector3d v(1.1, 2.2, 3.3);
    std::string header = "x,y,z";
    std::string filename = "test_output.csv";

    // call the function under test
    logger.saveDataToCSV(filename, header, v);

    // read back the file and print to stdout for verification
    // Mirror the same package-source data path used by saveDataToCSV
    fs::path srcFilePath = fs::path(__FILE__);
    fs::path pkgDir = srcFilePath.parent_path().parent_path();
    fs::path p = pkgDir / "data" / filename;
    if (!fs::exists(p)) {
        std::cerr << "Failed: file not created: " << p << std::endl;
        return 2;
    }
    std::ifstream ifs(p);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open file: " << p << std::endl;
        return 3;
    }
    std::string line;
    std::cout << "Contents of " << p << ":\n";
    while (std::getline(ifs, line)) {
        std::cout << line << "\n";
    }
    ifs.close();

    return 0;
}