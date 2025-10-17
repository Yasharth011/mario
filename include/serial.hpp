#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <boost/asio.hpp>
#include <cstdint>
#include <stdint.h>

using namespace boost::asio;

namespace serial {

enum Error : uint8_t { WriteSuccess = 0, CobsEncodeError, AsioWriteError };

void asyncWriteHandler(const boost::system::error_code &error,
                       std::size_t bytes_transferred);

Error asyncWrite(serial_port *serial, const uint8_t msg[], size_t MSG_LEN);

template <typename msgType>
Error write_msg(serial_port *serial, const msgType &msg, size_t MSG_LEN);

void close(serial_port *serial);

serial_port *open(io_context &io, const std::string port,
                  unsigned int baudrate);

uint32_t crc32_ieee(const uint8_t *data, size_t len);

uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *data, size_t len);

const char *get_error(enum Error err);
}; // namespace serial

namespace tarzan {
/* extra msg for Tarzan msg */
struct inverse_msg;

struct imu_data;

struct imu_msg;

enum msg_type { AUTONOMOUS, INVERSE, IMU };
/**/
struct DiffDriveTwist; // auto drive command

struct tarzan_msg; // message to write

// construct tarzan message
struct tarzan_msg get_tarzan_msg(float linear_x, float angular_z);
}; // namespace tarzan
#endif
