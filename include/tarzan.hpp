#ifndef TARZAN_HPP
#define TARZAN_HPP

#include <boost/asio.hpp>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stdint.h>

using namespace boost::asio;

class Tarzan {

private:
  /* msg struct for Tarzan msg */
  struct inverse_msg {
    double turn_table;
    double first_link;
    double second_link;
    double pitch;
    double roll;
    double x;
    double y;
    double z;
  };

  struct imu_data {
    double accel[3];
    double gyro[3];
    double mag[3];
    double gyro_offset[3];
  };

  struct imu_msg {
    struct imu_data baseLink;
    struct imu_data firstLink;
    struct imu_data secondLink;
    struct imu_data differential;
  };

  struct DiffDriveTwist {
    float linear_x;
    float angular_z;
  };

  enum msg_type { AUTONOMOUS, INVERSE };

  std::mutex write_mtx; // mutex to guard async write
  std::mutex error_mtx; // mutex to gaurd error in async write
  bool writeLocked = false;
  std::condition_variable writeCV;
  int err;

public:
  serial_port serial; // serial port

  enum Error : uint8_t {
	  WriteSuccess = 0, 
	  CobsEncodeError,
	  AsioWriteError
  };

  // msg to write
  struct tarzan_msg {
    struct DiffDriveTwist cmd;
    struct inverse_msg inv;
    struct imu_msg imu;
    enum msg_type type;
    uint32_t crc;
  };

  Tarzan(io_context &io, const std::string port, unsigned int baud_rate)
      : serial(io, port) {

    // close if port was already opened
    if (serial.is_open())
      close();

    try {
      serial.open(port);
    } catch (...) {
    }

    if (!serial.is_open())
      return;
    serial.set_option(serial_port_base::baud_rate(baud_rate));
  }

  void asyncWriteHandler(const boost::system::error_code &error,
                         std::size_t bytes_transferred);

  Error asyncWrite(const uint8_t msg[]);

  Error write_msg(const tarzan_msg &msg);

  void close();

  uint32_t crc32_ieee(const uint8_t *data, size_t len);

  uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *data, size_t len);

  const char* get_error(enum Error err);
};

#endif
