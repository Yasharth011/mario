#ifndef TARZAN_HPP
#define TARZAN_HPP

#include <boost/asio.hpp>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stdint.h>

using namespace boost::asio;

/* extra msg struct for Tarzan msg */
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

namespace tarzan {

  struct DiffDriveTwist {
    float linear_x;
    float angular_z;
  };

  enum msg_type { AUTONOMOUS, INVERSE };


  enum Error : uint8_t { WriteSuccess = 0, CobsEncodeError, AsioWriteError };

  // msg to write
  struct tarzan_msg {
    struct DiffDriveTwist cmd;
    // struct inverse_msg inv;
    // struct imu_msg imu;
    // enum msg_type type;
    uint32_t crc;
  };

  void asyncWriteHandler(const boost::system::error_code &error,
                         std::size_t bytes_transferred);

  Error asyncWrite(serial_port *serial, const uint8_t msg[]);

  Error write_msg(serial_port *serial, const tarzan_msg &msg);

  void close(serial_port *serial);

  serial_port *open(io_context &io, const std::string port, unsigned int baudrate);

  uint32_t crc32_ieee(const uint8_t *data, size_t len);

  uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *data, size_t len);

  const char *get_error(enum Error err);
};

#endif
