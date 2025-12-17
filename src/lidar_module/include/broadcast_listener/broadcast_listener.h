// //
// // Created by mwt on 25-2-6.
// //

// #ifndef BROADCASTLISTENER_H
// #define BROADCASTLISTENER_H

// #include <boost/asio.hpp>
// #include <boost/asio/ip/udp.hpp>
// #include <boost/asio/ip/address.hpp>

// #include <vector>
// #include <string>
// #include <thread>



// class BroadcastListener {
// public:
//     /**
//      * 构造函数：
//      * @param interface_name 要绑定的网卡名称
//      * @param port 监听的 UDP 端口
//      * @param seconds 监听时长（单位：秒）
//      */
//     BroadcastListener(const std::string &interface_name, unsigned short port, int seconds);

//     ~BroadcastListener();

//     /**
//      * 同步搜索雷达：
//      * 该方法启动异步接收操作和定时器，等待定时器超时后返回收集到的结果
//      */
//     std::vector<std::pair<std::string, std::string>> searchLidar();

// private:
//     // 异步接收数据，将接收到的雷达数据（ip和SN）保存到 lidars 中
//     void start_receive(std::vector<std::pair<std::string, std::string>> &lidars);

//     boost::asio::io_context io_context_;
//     boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
//     std::thread thread_;
//     boost::asio::ip::udp::socket socket_;
//     boost::asio::ip::udp::endpoint sender_endpoint_;
//     boost::asio::steady_timer timer_;
//     int seconds;

//     enum { max_length = 1024 };

//     char data_[max_length];
// };

// #endif // BROADCASTLISTENER_H