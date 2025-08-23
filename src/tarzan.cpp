#include <tarzan.hpp>
#include <cobs.h>

void Serial::asyncWriteHandler(const boost::system::error_code &error,
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

void Serial::asyncWrite(const std::vector<uint8_t> &msg) {

  std::unique_lock<std::mutex> lock(write_mtx);

  if (writeLocked)
    writeCV.wait(lock);

  writeLocked = true; // set writeLocked true and begin writing

  lock.unlock();

  async_write(serial, buffer(msg, msg.size()),
              [this](const boost::system::error_code &error,
                     std::size_t bytes_transferred) {
                asyncWriteHandler(error, bytes_transferred);
              });
}

// void write_msg(const tarzan_msg &msg){
//
// } 
