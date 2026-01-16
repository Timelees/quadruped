#include "log_save.h"

namespace fs = std::filesystem;

namespace quadruped::logs {

LogSave::~LogSave() {
    close();
}
// 打开文件
bool LogSave::open(const std::string &path) {
    std::lock_guard<std::mutex> lock(fileMutex_);
    // If logging is disabled, do not create or open any files.
    if (!enabled_.load()) return false;

    std::string usePath = path.empty() ? filePath_ : path;
    if (usePath.empty()) return false; // don't create unnamed files

    // create parent directory if necessary
    try {
        fs::path p(usePath);
        if (p.has_parent_path()) {
            fs::create_directories(p.parent_path());
        }
    } catch (const std::exception &e) {
        (void)e;
        return false;
    }
    if (fileStream_.is_open()) fileStream_.close();
    fileStream_.open(usePath, std::ios::out | std::ios::app);
    return fileStream_.is_open();
}
// 添加一行
void LogSave::appendLine(const std::string &line) {
    if (!enabled_.load()) return;
    std::lock_guard<std::mutex> lock(fileMutex_);
    if (!fileStream_.is_open()) return;
    fileStream_ << "[" << timestampNow() << "]" << line << "\n";
}
// 添加矩阵数据或向量数据
void LogSave::appendData(const ocs2::matrix_t &data, const int &matrix_row, const int &matrix_col){
    if(!enabled_.load()) return;
    std::lock_guard<std::mutex> lock(fileMutex_);
    if(!fileStream_.is_open()) return;
    // Print a small header describing matrix size
    fileStream_ << "[" << timestampNow() << "] Matrix " << matrix_row << "x" << matrix_col << "\n";
    // Configure formatting
    std::ios oldState(nullptr);
    oldState.copyfmt(fileStream_);
    fileStream_ << std::fixed << std::setprecision(4);
    // Column headers
    fileStream_ << std::setw(8) << "";
    for (int j = 0; j < matrix_col; ++j) {
        fileStream_ << std::setw(8) << ("c" + std::to_string(j));
    }
    fileStream_ << "\n";
    // Rows
    for(int i = 0; i < matrix_row; i++){
        fileStream_ << std::setw(8) << ("r" + std::to_string(i)) << ": ";
        for(int j = 0; j < matrix_col; j++){
            fileStream_ << std::setw(8) << data(i, j);
        }
        fileStream_ << "\n";   
    }
    // restore stream state
    fileStream_.copyfmt(oldState);

}
void LogSave::appendData(const ocs2::vector_t &data, const int &vector_row, const int &vector_col){
    if(!enabled_.load()) return;
    std::lock_guard<std::mutex> lock(fileMutex_);
    if(!fileStream_.is_open()) return;
    // Print a small header describing vector-as-matrix size
    fileStream_ << "[" << timestampNow() << "] Vector as " << vector_row << "x" << vector_col << "\n";
    std::ios oldState(nullptr);
    oldState.copyfmt(fileStream_);
    fileStream_ << std::fixed << std::setprecision(4);
    // Column headers
    fileStream_ << std::setw(8) << "";
    for (int j = 0; j < vector_col; ++j) {
        fileStream_ << std::setw(8) << ("c" + std::to_string(j));
    }
    fileStream_ << "\n";
    // Rows
    for(int i = 0; i < vector_row; i++){
        fileStream_ << std::setw(8) << ("r" + std::to_string(i)) << ": ";
        for(int j = 0; j < vector_col; j++){
            fileStream_ << std::setw(8) << data(i + j * vector_row);
        }
        fileStream_ << "\n";   
    }
    // restore stream state
    fileStream_.copyfmt(oldState);
}

// 保存三维向量到 csv 文件，路径位于包 logs 下的 data 目录
void LogSave::saveDataToCSV(const std::string &filename, const std::string &header, const Eigen::Vector3d &data) {
    if (!enabled_.load()) return;
    if (filename.empty()) return;

    std::lock_guard<std::mutex> lock(fileMutex_);

    try {
        // Prefer the package's source `data` directory next to this source file
        fs::path srcFilePath = fs::path(__FILE__);
        // go up to package directory: .../src/quadruped_ros2_control/logs/src/log_save.cpp -> .../src/quadruped_ros2_control/logs/
        fs::path pkgDir = srcFilePath.parent_path().parent_path();
        fs::path dir = pkgDir / "data";
        fs::create_directories(dir);
        fs::path filePath = dir / filename;

        // If file not exists or empty, we'll write header first
        bool writeHeader = false;
        if (!fs::exists(filePath)) {
            writeHeader = !header.empty();
        } else {
            try {
                writeHeader = (fs::file_size(filePath) == 0) && !header.empty();
            } catch (...) {
                writeHeader = false;
            }
        }

        std::ofstream ofs(filePath, std::ios::out | std::ios::app);
        if (!ofs.is_open()) return;

        if (writeHeader) {
            ofs << header << "\n";
        }

        // Append data as CSV row (no extra timestamp to keep CSV clean)
        ofs << data(0) << "," << data(1) << "," << data(2) << "\n";
        ofs.flush();
        ofs.close();
    } catch (const std::exception &e) {
        (void)e;
        return;
    }
}

// 刷新缓冲区
void LogSave::flush() {
    std::lock_guard<std::mutex> lock(fileMutex_);
    if (fileStream_.is_open()) fileStream_.flush();
}
// 关闭文件
void LogSave::close() {
    std::lock_guard<std::mutex> lock(fileMutex_);
    if (fileStream_.is_open()) {
        fileStream_.flush();
        fileStream_.close();
    }
}
// 获取当前时间戳
std::string LogSave::timestampNow() const {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto s = time_point_cast<std::chrono::seconds>(now);
    auto ms = duration_cast<milliseconds>(now - s).count();
    std::time_t t = system_clock::to_time_t(now);
    std::tm tm;
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "." << std::setw(3) << std::setfill('0') << ms;
    return oss.str();
}

void LogSave::setEnabled(bool enabled) {
    bool prev = enabled_.exchange(enabled);
    if (!enabled) {
        // If disabling, close any open file and don't create new ones.
        std::lock_guard<std::mutex> lock(fileMutex_);
        if (fileStream_.is_open()) {
            fileStream_.flush();
            fileStream_.close();
        }
    } else {
        // If enabling and we have a default path, try to open it.
        if (!filePath_.empty()) {
            open(filePath_);
        }
    }
}

} // namespace quadruped::logs
