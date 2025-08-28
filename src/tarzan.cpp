#include <cobs.h>
#include <cstddef>
#include <cstdint>

#include "tarzan.hpp"

constexpr size_t TARZAN_MSG_LEN = sizeof(Tarzan::tarzan_msg) + 2;

void Tarzan::asyncWriteHandler(const boost::system::error_code &error,
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

Tarzan::Error Tarzan::asyncWrite(const uint8_t msg[TARZAN_MSG_LEN]) {

  std::unique_lock<std::mutex> lock(write_mtx);

  if (writeLocked)
    writeCV.wait(lock);

  writeLocked = true; // set writeLocked true and begin writing

  lock.unlock();

  async_write(Tarzan::serial, buffer(msg, TARZAN_MSG_LEN),
              [this](const boost::system::error_code &error,
                     std::size_t bytes_transferred) {
                asyncWriteHandler(error, bytes_transferred);
              });
  std::unique_lock<std::mutex> elk(error_mtx);

  if (!err)
    return Tarzan::Error::WriteSuccess;
  else
    return Tarzan::Error::AsioWriteError;
}

Tarzan::Error Tarzan::write_msg(const Tarzan::tarzan_msg &msg) {

  uint8_t buffer[TARZAN_MSG_LEN];

  if (auto result = cobs_encode(
          reinterpret_cast<void *>(buffer), sizeof(TARZAN_MSG_LEN),
          reinterpret_cast<const void *>(&msg), sizeof(TARZAN_MSG_LEN));
      result.status != COBS_ENCODE_OK) {
    return Tarzan::Error::CobsEncodeError;
  }

  buffer[TARZAN_MSG_LEN - 1] = 0x00;

  return Tarzan::asyncWrite(buffer);
}

void Tarzan::close() {
  if (!Tarzan::serial.is_open())
    return;
  Tarzan::serial.cancel(); // cancel all pending async processes
  Tarzan::serial.close();  // close the serial port
}

int Tarzan::open() {

  // close if port was already opened
  if (serial.is_open())
    close();

  try {
    serial.open(Tarzan::port);
    serial.set_option(serial_port_base::baud_rate(Tarzan::baudrate));
    return 1;
  } catch (...) {
    return 0;
  }
}

uint32_t Tarzan::crc32_ieee(const uint8_t *data, size_t len) {
  return crc32_ieee_update(0x0, data, len);
}

uint32_t Tarzan::crc32_ieee_update(uint32_t crc, const uint8_t *data,
                                   size_t len) {
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

const char *get_error(enum Tarzan::Error err) {
  switch (err) {
  case Tarzan::Error::WriteSuccess:
    return "Serial Write Successfull";
    break;
  case Tarzan::Error::CobsEncodeError:
    return "Cobs Encode Error";
    break;
  case Tarzan::Error::AsioWriteError:
    return "Asio Serial Write Error";
    break;
  };
  return "Undefined Error";
}

#ifdef TARZAN_TEST_CPP
#include <iostream>

int main(int argc, char *argv[]) {

  std::string port = argv[1];
  boost::asio::io_context io;
  Tarzan nucleo(io, port, 9600);

  Tarzan::tarzan_msg msg;
  msg.cmd.linear_x = 1.0;
  msg.cmd.angular_z = 0.5;
  msg.crc = nucleo.crc32_ieee(
      (uint8_t *)&msg, sizeof(struct Tarzan::tarzan_msg) - sizeof(msg.crc));

  std::cout << "crc = " << msg.crc;
  std::cout << "writing message" << std::endl;

  nucleo.write_msg(msg);
}
#endif
