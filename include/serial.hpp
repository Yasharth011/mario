#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <boost/asio.hpp>
#include <cstdint>
#include <stdint.h>

using namespace boost::asio;

namespace serial {

enum Error : uint8_t { WriteSuccess = 0, ReadSuccess, CobsEncodeError, CobsDecodeError, AsioWriteError, AsioReadError };

void asyncWriteHandler(const boost::system::error_code &error,
                       std::size_t bytes_transferred);

Error asyncWrite(serial_port *serial, const uint8_t msg[], size_t MSG_LEN);

template <typename msgType>
Error write_msg(serial_port *serial, const msgType &msg, size_t MSG_LEN);

Error read_msg(serial_port *serial, uint8_t *buffer, size_t MSG_LEN);

void close(serial_port *serial);

serial_port *open(io_context &io, const std::string port,
                  unsigned int baudrate);

uint32_t crc32_ieee(const uint8_t *data, size_t len);

uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *data, size_t len);

const char *get_error(enum Error err);
}; // namespace serial

namespace tarzan {
/* extra msg for Tarzan msg */
enum msg_type { AUTONOMOUS, INVERSE, IMU };

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

struct tarzan_msg {
  struct DiffDriveTwist cmd;
  struct inverse_msg inv;
  struct imu_msg imu;
  enum msg_type type;
  uint32_t crc;
};

constexpr size_t TARZAN_MSG_LEN = sizeof(tarzan_msg)+2; // tarzan message len
							  
// construct tarzan message
struct tarzan_msg get_tarzan_msg(float linear_x, float angular_z);
}; // namespace tarzan
#endif
