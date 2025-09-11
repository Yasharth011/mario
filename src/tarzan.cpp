#include <cobs.h>
#include <cstddef>
#include <cstdint>
#include <imu.h>

#include "tarzan.hpp"

namespace tarzan {
constexpr size_t TARZAN_MSG_LEN = sizeof(tarzan_msg) + 2;

std::mutex write_mtx; // mutex to guard async write
std::mutex error_mtx; // mutex to gaurd error in async write
bool writeLocked = false;
std::condition_variable writeCV;
int err;

void asyncWriteHandler(const boost::system::error_code &error,
                       std::size_t bytes_transferred) {
  std::unique_lock<std::mutex> elk(error_mtx);
  if (error) {
    err = error.value();
  }
  elk.unlock();

  std::unique_lock<std::mutex> lk(write_mtx);
  writeLocked = false;
  lk.unlock();          // next write is possible
  writeCV.notify_one(); // if there is next write() waiting, notify it so it can
                        // continue
}

Error asyncWrite(serial_port *serial, const uint8_t msg[TARZAN_MSG_LEN]) {

  std::unique_lock<std::mutex> lock(write_mtx);

  if (writeLocked)
    writeCV.wait(lock);

  writeLocked = true; // set writeLocked true and begin writing

  lock.unlock();

  async_write(*serial, buffer(msg, TARZAN_MSG_LEN),
              [](const boost::system::error_code &error,
                 std::size_t bytes_transferred) {
                asyncWriteHandler(error, bytes_transferred);
              });
  std::unique_lock<std::mutex> elk(error_mtx);

  if (!err)
    return Error::WriteSuccess;
  else
    return Error::AsioWriteError;
}

Error write_msg(serial_port *serial, const tarzan_msg &msg) {

  uint8_t buffer[TARZAN_MSG_LEN];

  if (auto result = cobs_encode(
          reinterpret_cast<void *>(buffer), TARZAN_MSG_LEN,
          reinterpret_cast<const void *>(&msg), sizeof(struct tarzan_msg));
      result.status != COBS_ENCODE_OK) {
    return Error::CobsEncodeError;
  }

  buffer[TARZAN_MSG_LEN - 1] = 0x00;

  return asyncWrite(serial, buffer);
}

void close(serial_port *serial) {
  if (!serial->is_open())
    return;
  serial->cancel(); // cancel all pending async processes
  serial->close();  // close the serial port
}

serial_port *open(io_context &io, const std::string port,
                  unsigned int baudrate) {

  // create boost serial object
  serial_port *serial = new serial_port(io);

  // close if port was already opened
  if (serial->is_open())
    serial->close();

  try {
    serial->open(port);
    serial->set_option(serial_port_base::baud_rate(baudrate));
    return serial;
  } catch (...) {
    return nullptr;
  }
}

uint32_t crc32_ieee(const uint8_t *data, size_t len) {
  return crc32_ieee_update(0x0, data, len);
}

uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *data, size_t len) {
  /* crc table generated from polynomial 0xedb88320 */
  static const uint32_t table[16] = {
      0x00000000U, 0x1db71064U, 0x3b6e20c8U, 0x26d930acU,
      0x76dc4190U, 0x6b6b51f4U, 0x4db26158U, 0x5005713cU,
      0xedb88320U, 0xf00f9344U, 0xd6d6a3e8U, 0xcb61b38cU,
      0x9b64c2b0U, 0x86d3d2d4U, 0xa00ae278U, 0xbdbdf21cU,
  };

  crc = ~crc;

  for (size_t i = 0; i < len; i++) {
    uint8_t byte = data[i];

    crc = (crc >> 4) ^ table[(crc ^ byte) & 0x0f];
    crc = (crc >> 4) ^ table[(crc ^ ((uint32_t)byte >> 4)) & 0x0f];
  }

  return (~crc);
}

const char *get_error(enum Error err) {
  switch (err) {
  case Error::WriteSuccess:
    return "Serial Write Successfull";
    break;
  case Error::CobsEncodeError:
    return "Cobs Encode Error";
    break;
  case Error::AsioWriteError:
    return "Asio Serial Write Error";
    break;
  };
  return "Undefined Error";
}
} // namespace tarzan

#ifdef TARZAN_TEST_CPP
#include <iostream>

int main(int argc, char *argv[]) {

  std::string port = argv[1];
  boost::asio::io_context io;
  boost::asio::serial_port *nucleo = tarzan::open(io, port, 9600);

  tarzan::tarzan_msg msg;
  msg.cmd.linear_x = 1.0;
  msg.cmd.angular_z = 0.5;
  msg.crc = tarzan::crc32_ieee(
      (uint8_t *)&msg, sizeof(struct tarzan::tarzan_msg) - sizeof(msg.crc));

  std::cout << "crc = " << msg.crc;

  std::string err = get_error(tarzan::write_msg(nucleo, msg));

  std::cout<<"Error : "<<err; 

  tarzan::close(nucleo);
}
#endif
