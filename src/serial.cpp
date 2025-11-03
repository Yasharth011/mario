#include <cobs.h>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include "serial.hpp"

namespace serial {

std::mutex write_mtx;       // mutex to guard async write
std::mutex read_mtx;        // mutex to guard async read
std::mutex write_error_mtx; // mutex to gaurd error in async write
std::mutex read_error_mtx;  // mutex to gaurd error in async read
bool writeLocked = false;
bool readLocked = false;
std::condition_variable writeCV;
std::condition_variable readCV;
int write_err;
int read_err;

void asyncWriteHandler(const boost::system::error_code &error,
                       std::size_t bytes_transferred) {
  std::unique_lock<std::mutex> elk(write_error_mtx);
  if (error) {
    write_err = error.value();
  }
  elk.unlock();

  std::unique_lock<std::mutex> lk(write_mtx);
  writeLocked = false;
  lk.unlock();          // next write is possible
  writeCV.notify_one(); // if there is next write() waiting, notify it so it can
                        // continue
}

Error asyncWrite(serial_port *serial, const uint8_t msg[], size_t MSG_LEN) {

  std::unique_lock<std::mutex> lock(write_mtx);

  if (writeLocked)
    writeCV.wait(lock);

  writeLocked = true; // set writeLocked true and begin writing

  lock.unlock();

  async_write(*serial, buffer(msg, MSG_LEN),
              [](const boost::system::error_code &error,
                 std::size_t bytes_transferred) {
                asyncWriteHandler(error, bytes_transferred);
              });
  std::unique_lock<std::mutex> elk(write_error_mtx);

  if (!write_err)
    return Error::WriteSuccess;
  else
    return Error::AsioWriteError;
}

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
                      std::size_t bytes_transferred) {
  std::unique_lock<std::mutex> elk(read_error_mtx);
  if (error) {
    read_err = error.value();
  }
  elk.unlock();

  std::unique_lock<std::mutex> lk(read_mtx);
  readLocked = false;
  lk.unlock();         // next write is possible
  readCV.notify_one(); // if there is next write() waiting, notify it so it can
                       // continue
}

Error asyncRead(serial_port *serial, uint8_t *read_buffer, size_t MSG_LEN) {
  std::unique_lock<std::mutex> lock(read_mtx);

  if (readLocked)
    readCV.wait(lock);

  readLocked = true;

  lock.unlock();

  async_read(*serial, boost::asio::buffer(read_buffer, MSG_LEN),
             [](const boost::system::error_code &error,
                std::size_t bytes_transferred) {
               asyncReadHandler(error, bytes_transferred);
             });

  std::unique_lock<std::mutex> elk(read_error_mtx);

  if (!read_err)
    return Error::ReadSuccess;
  else
    return Error::AsioReadError;
}

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
} // namespace serial

namespace tarzan {
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
constexpr size_t TARZAN_MSG_LEN = sizeof(tarzan_msg) + 2; // tarzan message len

struct tarzan_msg get_tarzan_msg(float linear_x, float angular_z) {
  struct tarzan_msg msg;

  // DiffDrive var
  struct DiffDriveTwist cmd = {.linear_x = linear_x, .angular_z = angular_z};

  uint32_t crc = serial::crc32_ieee(
      (uint8_t *)&msg, sizeof(struct tarzan::tarzan_msg) - sizeof(msg.crc));

  msg = {.cmd = cmd, .inv = {0}, .imu = {0}, .crc = crc};

  // tarzan msg
  return msg;
};
}; // namespace tarzan

#ifdef SERIAL_TEST_CPP
#include <iostream>

int main(int argc, char *argv[]) {

  std::string port = argv[1];
  boost::asio::io_context io;
  boost::asio::serial_port *nucleo = serial::open(io, port, 9600);

  float linear_x = 1.0;
  float angular_z = 0.5;
  tarzan::tarzan_msg msg = tarzan::get_tarzan_msg(linear_x, angular_z);

  std::string err =
      serial::get_error(serial::write_msg(nucleo, msg, tarzan::TARZAN_MSG_LEN));

  std::cout << "Error : " << err;

  serial::close(nucleo);
}
#endif
