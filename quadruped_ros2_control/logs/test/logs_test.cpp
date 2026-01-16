#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include "log_save.h"

int main() {
    namespace fs = std::filesystem;
    using quadruped::logs::LogSave;
    
    LogSave logger;
    logger.setEnabled(true);        // 设置日志是否启用
    // Use default path (will create directories)
    std::string timestamp = logger.timestampNow();
    std::string path = std::string("Mylogs/") + std::string("logs") + "_" + timestamp + ".txt";
    std::cout << "Default log file: " << path << std::endl;
    logger.setDefaultFilePath(path);
    if (!logger.open("")) {
        std::cerr << "Failed to open default log file" << std::endl;
        return 1;
    }
 
    logger.appendLine("Test default path - line 1");
    logger.appendLine("Test default path - line 2");


    // Test Matrix
    ocs2::matrix_t mat(3, 3);
    mat << 1, 2, 3,
           4, 5, 6,
           7, 8, 9;
    logger.appendData(mat, 3, 3);
    ocs2::vector_t vec(3);
    vec << 10, 20, 30;
    logger.appendData(vec, 3, 1);
    
    logger.flush();
    logger.close();
    
    // Verify default file exists and has at least 2 lines
    const fs::path defaultPath("Mylogs/test_output/logs.txt");
    if (!fs::exists(defaultPath)) {
        std::cerr << "Default log file not found: " << defaultPath << std::endl;
        return 2;
    }
    std::ifstream ifs(defaultPath);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(ifs, line)) {
        if (!line.empty()) lines.push_back(line);
    }
    ifs.close();
    if (lines.size() < 2) {
        std::cerr << "Default log file has insufficient lines (" << lines.size() << ")" << std::endl;
        return 3;
    }

    std::cout << "Default path test passed, lines=" << lines.size() << std::endl;

    // Test custom path
    const std::string custom = "Mylogs/test_output/logs_test_custom.txt";
    if (!logger.open(custom)) {
        std::cerr << "Failed to open custom log file: " << custom << std::endl;
        return 4;
    }
    logger.appendLine("Custom path test line");
    logger.flush();
    logger.close();

    if (!fs::exists(custom)) {
        std::cerr << "Custom log file not found: " << custom << std::endl;
        return 5;
    }

    std::cout << "Custom path test passed: " << custom << std::endl;
    std::cout << "All tests passed." << std::endl;
    return 0;
}
