#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

#include <arpa/inet.h>
#include <netinet/in.h> // sockaddr_in

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

namespace tools {


class Plotter {
public:
  Plotter(){
    std::string host = "127.0.0.1";
    uint16_t port = 9870;
    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

    destination_.sin_family = AF_INET;
    destination_.sin_port = ::htons(port);
    destination_.sin_addr.s_addr = ::inet_addr(host.c_str());
  }
  // Plotter(std::string host = "127.0.0.1", uint16_t port = 9870) {
  //   socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

  //   destination_.sin_family = AF_INET;
  //   destination_.sin_port = ::htons(port);
  //   destination_.sin_addr.s_addr = ::inet_addr(host.c_str());
  // };

  ~Plotter()=default;

  void plot(const nlohmann::json &json) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto data = json.dump();
    ::sendto(socket_, data.c_str(), data.length(), 0,
             reinterpret_cast<sockaddr *>(&destination_), sizeof(destination_));
  }

private:
  int socket_;
  sockaddr_in destination_;
  std::mutex mutex_;
};

inline void plotter_debug_cmd(Plotter& plotter,double &now, double vx, double vy,
                                   double wz) {
  nlohmann::json cmd_data_json;
  cmd_data_json["ts"] = now;
  cmd_data_json["cmd_vx"] = vx;
  cmd_data_json["cmd_vy"] = vy;
  cmd_data_json["cmd_wz"] = wz;
  plotter.plot(cmd_data_json);
}

} // namespace tools

#endif // TOOLS__PLOTTER_HPP