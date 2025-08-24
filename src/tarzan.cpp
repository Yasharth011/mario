#include <cobs.h>
#include <cstddef>
#include <cstdint>

#include "tarzan.hpp"

constexpr size_t TARZAN_MSG_LEN = sizeof(Tarzan::tarzan_msg) + 2;

void Tarzan::asyncWriteHandler(const boost::system::error_code &error,
                               std::size_t bytes_transferred) {
  std::unique_lock<std::mutex> elk(error_mtx);
  if (error) {
    // log error
  }
  elk.unlock();

  std::unique_lock<std::mutex> lk(write_mtx);
  writeLocked = false;
  lk.unlock();          // next write is possible
  writeCV.notify_one(); // if there is next write() waiting, notify it so it can
                        // continue
}

void Tarzan::asyncWrite(const uint8_t msg[TARZAN_MSG_LEN]) {

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
}

void Tarzan::write_msg(const Tarzan::tarzan_msg &msg) {

  uint8_t buffer[TARZAN_MSG_LEN];

  if (auto result = cobs_encode(
          reinterpret_cast<void *>(buffer), sizeof(TARZAN_MSG_LEN),
          reinterpret_cast<const void *>(&msg), sizeof(TARZAN_MSG_LEN));
      result.status != COBS_ENCODE_OK) {
    // log COBS_DECODE_ERROR
  }

  buffer[TARZAN_MSG_LEN - 1] = 0x00;

  Tarzan::asyncWrite(buffer);
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
  msg.crc = 0; 

  std::cout << "writing message" << std::endl;

  nucleo.write_msg(msg);
}
#endif
