#include <array>
#include <condition_variable>
#include <mutex>

namespace pandar128 {

template <typename T, int N> struct fifo {

  bool push(const T &v, const int64_t timeout_us = -1) {
    bool succeed = false;
    if (timeout_us < 0) { // wait forever
      std::unique_lock<std::mutex> l(m_);
      while (_full()) {
        cv_.wait(l);
      }
      buffer_[(wr_ptr_++).idx] = v;
      succeed = true;
    } else if (timeout_us == 0) { // no wait
      std::unique_lock<std::mutex> l(m_);
      if (!_full()) {
        buffer_[(wr_ptr_++).idx] = v;
        succeed = true;
      }
    } else { // wait for timeout
      std::unique_lock<std::mutex> l(m_);
      if (_full()) {
        cv_.wait_for(l, std::chrono::microseconds(timeout_us));
      }
      if (!_full()) { // check for spurious wakeup
        buffer_[(wr_ptr_++).idx] = v;
        succeed = true;
      }
    }

    if (succeed) {
      cv_.notify_all();
    }

    return succeed;
  }

  bool pop(T &v, const int64_t timeout_us = -1) {
    bool succeed = false;
    if (timeout_us < 0) { // wait forever
      std::unique_lock<std::mutex> l(m_);
      while (_empty()) {
        cv_.wait(l);
      }
      v = buffer_[(rd_ptr_++).idx];
      succeed = true;
    } else if (timeout_us == 0) { // no wait
      std::unique_lock<std::mutex> l(m_);
      if (!_empty()) {
        v = buffer_[(rd_ptr_++).idx];
        succeed = true;
      }
    } else { // wait for timeout
      std::unique_lock<std::mutex> l(m_);
      if (_empty()) {
        cv_.wait_for(l, std::chrono::microseconds(timeout_us));
      }
      if (!_empty()) { // check for spurious wakeup
        v = buffer_[(rd_ptr_++).idx];
        succeed = true;
      }
    }

    if (succeed) {
      cv_.notify_all();
    }

    return succeed;
  }

  bool empty() {
    std::unique_lock<std::mutex> l(m_);
    return _empty();
  }

  bool full() {
    std::unique_lock<std::mutex> l(m_);
    return _full();
  }

  int size() {
    std::unique_lock<std::mutex> l(m_);
    auto s = wr_ptr_.idx - rd_ptr_.idx;
    if (wr_ptr_.turn != rd_ptr_.turn) {
      s += N;
    }
    return s;
  }

  static int capacity() { return N; }

protected:
  bool _empty() const {
    return rd_ptr_.turn == wr_ptr_.turn && rd_ptr_.idx == wr_ptr_.idx;
  }

  bool _full() const {
    return rd_ptr_.turn != wr_ptr_.turn && rd_ptr_.idx == wr_ptr_.idx;
  }

protected:
  struct Ptr {
    int turn = 0;
    int idx = 0;

    // prefix ++
    Ptr &operator++() {
      ++idx;
      if (idx >= N) {
        idx = 0;
        ++turn;
      }
      return *this;
    }

    // postfix ++
    Ptr operator++(int) {
      auto old = *this;
      ++(*this);
      return old;
    }
  };

  Ptr rd_ptr_;
  Ptr wr_ptr_;
  std::array<T, N> buffer_;
  std::mutex m_;
  std::condition_variable cv_;
};

} // namespace pandar128
