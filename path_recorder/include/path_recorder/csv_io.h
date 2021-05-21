#ifndef PATH_RECORDER_CSV_IO_H
#define PATH_RECORDER_CSV_IO_H

#include <fstream>
#include <string>

namespace f1tenth_racecar
{
namespace utils
{
class CsvIO
{
private:
  std::fstream file_;
  std::string file_path_;

  bool fail_ = false;

  bool fileExists(const std::string file_name);

public:
  CsvIO();

  bool read(const std::string file_path);

  void write(const std::string file_path);

  bool isOpen();

  bool eof();

  std::string filePath();

  CsvIO& write(const double data);

  CsvIO& operator<<(const std::string data);

  CsvIO& operator<<(const double data);

  CsvIO& operator>>(std::string& data);

  CsvIO& operator>>(double& data);

  operator bool() const;
};
}  // namespace utils
}  // namespace f1tenth_racecar

#endif  // PATH_RECORDER_CSV_IO_H
