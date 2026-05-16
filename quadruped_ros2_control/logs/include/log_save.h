// Simple thread-safe logger that appends lines to a file with timestamps.
#pragma once

#include <atomic>
#include <fstream>
#include <mutex>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <vector>
#include <initializer_list>
#include <eigen3/Eigen/Dense>
#include <ocs2_core/Types.h>

namespace quadruped::logs {

class LogSave {
public:
    LogSave() = default;
    ~LogSave();

    // Open file for appending. If already open, it will be closed and reopened.
    // If called with no argument or empty string, uses the default file path set by setDefaultFilePath().
    bool open(const std::string &path = "");

    // Set/get default file path used when open() is called with empty path
    void setDefaultFilePath(const std::string &path) { filePath_ = path; }
    std::string getDefaultFilePath() const { return filePath_; }

    // Append a line to the file.
    void appendLine(const std::string &line);   
    // Append Matrix or Vector data to the file.
    void appendData(const ocs2::matrix_t &data, const int &matrix_row, const int &matrix_col);
    void appendData(const ocs2::vector_t &data, const int &vector_row, const int &vector_col);


    // Save data to the csv file
    void saveDataToCSV(const std::string &filename, const std::string &header, const Eigen::Vector3d &data);

    // Save data with controller/sim time as the first CSV column (header gets "time," prefix)
    void saveDataToCSV(const std::string &filename, const std::string &header, double timestamp,
                       const Eigen::Vector3d &data);
    void saveDataToCSV(const std::string &filename, const std::string &header, double timestamp,
                       std::initializer_list<double> data);

    // Save variable-length row to CSV (more columns)
    void saveDataToCSV(const std::string &filename, const std::string &header, const Eigen::VectorXd &data);
    void saveDataToCSV(const std::string &filename, const std::string &header, const std::vector<double> &data);
    void saveDataToCSV(const std::string &filename, const std::string &header, std::initializer_list<double> data);

    // Flush the underlying file stream.
    void flush();

    // Close the file.
    void close();

    // Enable/disable logging (if disabled, appendLine is a no-op)
    void setEnabled(bool enabled);
    bool isEnabled() const { return enabled_.load(); }
    
    std::string timestampNow() const;
private:


    std::mutex fileMutex_;
    std::ofstream fileStream_;
    std::atomic<bool> enabled_{true};
    std::string filePath_;
};

} // namespace quadruped::logs
