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

public:

  serial_port serial; // serial port

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
    serial.set_option(serial_port_base::baud_rate(baud_rate));
    serial.set_option(serial_port_base::character_size(8));
    serial.set_option(
        serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial.set_option(
        serial_port_base::flow_control(serial_port_base::flow_control::none));
  }

  void asyncWriteHandler(const boost::system::error_code &error,
                         std::size_t bytes_transferred);

  void asyncWrite(const uint8_t msg[]);

  void write_msg(const tarzan_msg &msg);

};

#endif
