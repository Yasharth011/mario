#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <boost/asio.hpp>
#include <cobs.h>
#include <cstdint>
#include <iostream>
#include <stdint.h>

using namespace boost::asio;

namespace serial {

enum Error : uint8_t {
  WriteSuccess = 0,
  ReadSuccess,
  CobsEncodeError,
  CobsDecodeError,
  AsioWriteError,
  AsioReadError
};

void asyncWriteHandler(const boost::system::error_code &error,
                       std::size_t bytes_transferred);

Error asyncWrite(serial_port *serial, const uint8_t msg[], size_t MSG_LEN);

template <typename msgType>
Error write_msg(serial_port *serial, const msgType &msg, size_t MSG_LEN) {

  uint8_t buffer[MSG_LEN];

  if (auto result =
          cobs_encode(reinterpret_cast<void *>(buffer), MSG_LEN,
                      reinterpret_cast<const void *>(&msg), sizeof(msgType));
      result.status != COBS_ENCODE_OK) {
    return Error::CobsEncodeError;
  }

  buffer[MSG_LEN - 1] = 0x00;

  return asyncWrite(serial, buffer, MSG_LEN);
}

void asyncReadHandler(const boost::system::error_code &error,
                      std::size_t bytes_transferred);
Error asyncRead(serial_port *serial, uint8_t *read_buffer, size_t MSG_LEN);

template <typename msg_type>
Error read_msg(serial_port *serial, msg_type &buffer, size_t MSG_LEN) {

  uint8_t read_buffer[MSG_LEN];
  Error err = asyncRead(serial, read_buffer, MSG_LEN);

  if (auto result = cobs_decode(reinterpret_cast<void *>(buffer), MSG_LEN,
                                reinterpret_cast<const void *>(read_buffer),
                                sizeof(read_buffer));
      result.status != COBS_DECODE_OK) {
    return Error::CobsDecodeError;
  }

  return err;
}

void close(serial_port *serial);

serial_port *open(io_context &io, const std::string port,
                  unsigned int baudrate);

uint32_t crc32_ieee(const uint8_t *data, size_t len);

uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *data, size_t len);

const char *get_error(enum Error err);
}; // namespace serial

namespace tarzan {
/* msg for Tarzan msg */
struct DiffDriveTwist {
  float linear_x;
  float angular_z;
};

struct tarzan_msg {
  struct DiffDriveTwist cmd;
  uint32_t crc;
};

constexpr size_t TARZAN_MSG_LEN = sizeof(tarzan_msg) + 2; // tarzan message len

// construct tarzan message
struct tarzan_msg get_tarzan_msg(float linear_x, float angular_z);

}; // namespace tarzan
#endif
