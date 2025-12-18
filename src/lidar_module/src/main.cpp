#include <chrono>
#include <iostream>
#include <vector>
#include <iomanip>

#include "base/data_struct.h"
#include "concurrentqueue.h"


int main()
{
    base_frame::Frame<base_frame::HandShake> handshake("192.168.1.103", 45000, 55000, 65000);
    std::cout << handshake << std::endl;

    for (const auto &byte : FRAME_TO_VECTOR(handshake)) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;

    base_frame::Frame<base_frame::HeartBeat> heartbeat;
    std::cout << heartbeat << std::endl;

    for (const auto &byte : FRAME_TO_VECTOR(heartbeat)) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;

    base_frame::Frame<base_frame::SetLaserStatus> set_laser_status(1);
    std::cout << set_laser_status << std::endl;

    for (const auto &byte : FRAME_TO_VECTOR(set_laser_status)) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;

    base_frame::Frame<base_frame::SetIMUFrequency> set_imu_frequency(1);
    std::cout << set_imu_frequency << std::endl;

    for (const auto &byte : FRAME_TO_VECTOR(set_imu_frequency)) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;

    moodycamel::ConcurrentQueue<int> queue;

    // 使用生产者令牌（可选，可以提高性能）
    moodycamel::ProducerToken producerToken(queue);

    // 入队操作
    queue.enqueue(42);
    // 或使用令牌
    queue.enqueue(producerToken, 43);

    // 出队操作
    int item;
    bool found = queue.try_dequeue(item);

    // 使用消费者令牌
    moodycamel::ConsumerToken consumerToken(queue);
    found = queue.try_dequeue(consumerToken, item);

    // 批量操作
    std::vector<int> items{1, 2, 3, 4, 5};
    queue.enqueue_bulk(items.data(), items.size());

    std::vector<int> results(5);
    size_t count = queue.try_dequeue_bulk(results.data(), 5);

    return 0;
}