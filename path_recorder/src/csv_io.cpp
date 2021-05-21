#include <fstream>
#include <string>
#include <sstream>

#include <geometry_msgs/Pose.h>

#include "path_recorder/csv_io.h"

namespace f1tenth_racecar
{
namespace utils
{
CsvIO::CsvIO()
{
}

bool CsvIO::fileExists(const std::string file_name)
{
  file_.open(file_name);
  bool exists = file_.is_open();
  file_.close();
  return exists;
}

bool CsvIO::read(const std::string file_path)
{
  file_path_ = file_path;
  file_.open(file_path_, std::ios::in);

  return file_.is_open();
}

bool CsvIO::eof()
{
  return file_.eof();
}

void CsvIO::write(const std::string file_path)
{
  file_path_ = file_path;

  const std::string token = file_path.substr(0, file_path.find(".csv"));

  for (int i = 2; fileExists(file_path_); ++i)
  {
    std::string temp = token + std::to_string(i) + ".csv";
    file_path_ = temp;
  }

  file_.open(file_path_, std::ios::out);
}

bool CsvIO::isOpen()
{
  return file_.is_open();
}

std::string CsvIO::filePath()
{
  return file_path_;
}

CsvIO& CsvIO::operator<<(const std::string data)
{
  if (data == "\n")
  {
    file_ << std::endl;
  }
  else
  {
    file_ << data << ",";
  }

  return *this;
}

CsvIO& CsvIO::operator<<(const double data)
{
  file_ << data << ",";
  return *this;
}

CsvIO& CsvIO::operator>>(std::string& data)
{
  getline(file_, data, ',');

  std::size_t pos = data.find('\n');
  if (pos != std::string::npos)
  {
    data.erase(data.begin() + pos);
  }

  fail_ = file_.fail();
  return *this;
}

CsvIO& CsvIO::operator>>(double& data)
{
  std::string data_str;
  operator>>(data_str);

  std::stringstream ss(data_str);
  ss >> data;

  fail_ = ss.fail();
  return *this;
}

CsvIO::operator bool() const
{
  return !fail_;
}
}  // namespace utils
}  // namespace f1tenth_racecar
