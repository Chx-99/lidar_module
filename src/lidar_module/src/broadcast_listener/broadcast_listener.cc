// #include "broadcast_listener/broadcast_listener.h"
// #include <iostream>
// #include "base/dataStruct.h"

// BroadcastListener::BroadcastListener(const std::string &interface_name,
//                                      unsigned short port, int seconds)
//     : work_guard_(boost::asio::make_work_guard(io_context_)),
//       socket_(io_context_),
//       timer_(io_context_),
//       seconds(seconds) {
//   // 允许地址重用
//   socket_.open(boost::asio::ip::udp::v4());
//   socket_.set_option(boost::asio::socket_base::reuse_address(true));
//   boost::asio::ip::udp::endpoint local_endpoint(
//       boost::asio::ip::address_v4::any(), port);
//   socket_.bind(local_endpoint);

// #ifdef __linux__
//   // 使用 SO_BINDTODEVICE 将 socket 限定在指定网卡上
//   if (!interface_name.empty()) {
//     int ret = setsockopt(socket_.native_handle(), SOL_SOCKET, SO_BINDTODEVICE,
//                          interface_name.c_str(), interface_name.size());
//     if (ret != 0) {
//       std::cerr << "Error binding to device " << interface_name << std::endl;
//     }
//   }
// #endif

//   // 启动 io_context_ 线程
//   thread_ = std::thread([this]() {
//     std::cout << "io_context_ thread started." << std::endl;
//     io_context_.run();
//     std::cout << "io_context_ thread stopped." << std::endl;
//   });
// }

// BroadcastListener::~BroadcastListener() {
//   std::cout << "BroadcastListener destructor called." << std::endl;
//   work_guard_.reset();
//   io_context_.stop();
//   if (thread_.joinable()) {
//     thread_.join();
//   }
// }

// std::vector<std::pair<std::string, std::string>>
// BroadcastListener::searchLidar() {
//   // 用于存放接收到的雷达信息 SN:IP
//   std::vector<std::pair<std::string, std::string>> lidars;

//   // 创建一个 promise 用于等待超时后收集到的结果
//   auto promise_ptr = std::make_shared<
//       std::promise<std::vector<std::pair<std::string, std::string>>>>();
 
//   // 启动异步接收，将数据存放到 lidars 中
//   start_receive(lidars);

//   // 设置定时器，到达指定秒数后关闭 socket，并通过 promise 返回结果
//   timer_.expires_after(std::chrono::seconds(seconds));
//   timer_.async_wait(
//       [this, promise_ptr, &lidars](const boost::system::error_code &ec) {
//         if (!ec) {
//           std::cout << "Timer expired (" << ec.message()
//                     << "). Stopping BroadcastListener." << std::endl;
//           // 关闭 socket 会取消所有挂起的异步操作
//           socket_.close();
//         }
//         // 将收集到的结果传给 promise，解除等待
//         promise_ptr->set_value(lidars);
//       });

//   // 阻塞等待 promise 被设置（即等待定时器超时后的回调执行）
//   return promise_ptr->get_future().get();
// }

// void BroadcastListener::start_receive(
//     std::vector<std::pair<std::string, std::string>> &lidars) {
//   socket_.async_receive_from(
//       boost::asio::buffer(data_, max_length), sender_endpoint_,
//       [this, &lidars](const boost::system::error_code &error,
//                       std::size_t bytes_transferred) {
//         if (error) {
//           // 如果因为超时或被取消而出错，则不继续调用 start_receive
//           if (error == boost::asio::error::operation_aborted) {
//             std::cout << "搜索结束." << std::endl;
//           } else {
//             std::cerr << "Receive error: " << error.message() << std::endl;
//           }
//           return;
//         }

//         std::cout << "Received broadcast from "
//                   << sender_endpoint_.address().to_string() << std::endl;

//         // 解析 tag（假定 map 和索引宏已正确定义）
//         std::string tag = TAG_MAP.at(
//             {data_[CMD_TYPE_INDEX], data_[CMD_SET_INDEX], data_[CMD_ID_INDEX]});
//         std::vector<uint8_t> data_v = GET_DATA_FIELD(data_, bytes_transferred);
//         if (tag == "boardcast") {
//           boardcast bc;
//           std::memcpy(&bc, data_v.data(), sizeof(boardcast));
//           std::cout << bc << std::endl;
//           if (std::find(
//                   lidars.begin(), lidars.end(),
//                   std::make_pair(bc.getSN(),
//                                  sender_endpoint_.address().to_string())) ==
//               lidars.end()) {
//             lidars.emplace_back(bc.getSN(),
//                                 sender_endpoint_.address().to_string());
//           }
//           std::cout << std::dec;
//         }

//         // 如果 socket 仍然打开，则继续接收
//         if (socket_.is_open()) {
//           start_receive(lidars);
//         }
//       });
// }

// //多网卡监听
// // std::vector<std::string> interfaces = {"eth0", "eth1", "wlan0"};
// // std::vector<std::future<std::vector<std::pair<std::string, std::string>>>>
// // futures;

// // for (const auto& iface : interfaces) {
// //     futures.push_back(std::async(std::launch::async, [iface, port, seconds]()
// //     {
// //         BroadcastListener listener(iface, port, seconds);
// //         return listener.searchLidar();
// //     }));
// // }

// // std::vector<std::pair<std::string, std::string>> all_results;
// // for (auto& fut : futures) {
// //     auto result = fut.get();
// //     all_results.insert(all_results.end(), result.begin(), result.end());
// // }