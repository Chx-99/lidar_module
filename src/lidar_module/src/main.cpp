#include "base/data_struct.h"
#include "base/port_scan.h"
#include "concurrentqueue.h"
#include "lidar/lidar_base.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace std::chrono;

// 定义时间测量宏
#define MEASURE_TIME(operation, iterations, name)                                                               \
    auto start_##name = high_resolution_clock::now();                                                           \
    for (int i = 0; i < iterations; ++i) { operation; }                                                         \
    auto end_##name = high_resolution_clock::now();                                                             \
    auto duration_##name = duration_cast<nanoseconds>(end_##name - start_##name);                               \
    std::cout << #name << " (" << iterations << " iterations) 执行时间: " << duration_##name.count() << " 纳秒" \
              << std::endl;                                                                                     \
    std::cout << "平均每次执行时间: " << duration_##name.count() / iterations << " 纳秒" << std::endl;

int main(int argc, char** argv) {
    base_frame::Frame<base_frame::HandShake> handshake("192.168.1.103", 45000, 55000, 65000);
    std::cout << handshake << std::endl;

    for (const auto& byte : FRAME_TO_SPAN(handshake)) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;

    base_frame::Frame<base_frame::HeartBeat> heartbeat;
    std::cout << heartbeat << std::endl;

    for (const auto& byte : FRAME_TO_SPAN(heartbeat)) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;

    base_frame::Frame<base_frame::SetLaserStatus> set_laser_status(1);
    std::cout << set_laser_status << std::endl;

    for (const auto& byte : FRAME_TO_SPAN(set_laser_status)) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;

    base_frame::Frame<base_frame::SetIMUFrequency> set_imu_frequency(1);
    std::cout << set_imu_frequency << std::endl;

    for (const auto& byte : FRAME_TO_SPAN(set_imu_frequency)) {
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

    boost::asio::io_context io_context;
    lidar_module::Lidar lidar(io_context, "192.168.1.103", "192.168.1.1", "SN123456", 10);

    // ========== 性能测试 ==========
    std::cout << "\n========== 性能测试（使用 MEASURE_TIME 宏） ==========\n" << std::endl;

    // 测试1: 单次端口分配性能
    {
        std::cout << "1. 单次端口分配性能测试：" << std::endl;
        MEASURE_TIME(
            {
                auto h = network_tools::BoostPortAllocator::instance().acquire();
                h.release();
            },
            1000, single_port_allocation);
        std::cout << std::endl;
    }

    // 测试2: 批量端口分配性能
    {
        std::cout << "2. 批量端口分配性能测试（每次10个）：" << std::endl;
        MEASURE_TIME(
            {
                auto handles = network_tools::BoostPortAllocator::instance().acquireMultiple(10);
                // handles 自动释放
            },
            100, batch_port_allocation);
        std::cout << std::endl;
    }

    // 测试3: 端口获取与释放循环
    {
        std::cout << "3. 端口获取与自动释放循环测试：" << std::endl;
        MEASURE_TIME(
            {
                {
                    auto h = network_tools::BoostPortAllocator::instance().acquire();
    }
},
                500,
                acquire_release_cycle
            );
std::cout << std::endl;
}

// 测试4: 端口句柄移动性能
{
    std::cout << "4. 端口句柄移动性能测试：" << std::endl;
    std::vector<network_tools::PortHandle> handles_pool;
    handles_pool.reserve(100);

    MEASURE_TIME(
        {
            auto h = network_tools::BoostPortAllocator::instance().acquire();
            handles_pool.push_back(std::move(h));
        },
        100, port_handle_move);

    handles_pool.clear();  // 释放所有端口
    std::cout << std::endl;
}

// 测试5: 统计信息查询性能
{
    std::cout << "5. 统计信息查询性能测试：" << std::endl;
    MEASURE_TIME(
        {
            auto stats = network_tools::BoostPortAllocator::instance().statistics();
            (void)stats;  // 避免未使用警告
        },
        10000, statistics_query);
    std::cout << std::endl;
}

// 测试6: 压力测试 - 快速分配释放
{
    std::cout << "6. 压力测试 - 快速分配释放 50 个端口：" << std::endl;
    MEASURE_TIME(
        {
            std::vector<network_tools::PortHandle> temp_handles;
            temp_handles.reserve(50);
            for (int j = 0; j < 50; ++j) {
                temp_handles.push_back(network_tools::BoostPortAllocator::instance().acquire());
            }
            // temp_handles 离开作用域，全部释放
        },
        10, stress_test_50_ports);
    std::cout << std::endl;
}

// 显示最终统计
{
    std::cout << "========== 最终统计信息 ==========\n" << std::endl;
    auto final_stats = network_tools::BoostPortAllocator::instance().statistics();
    std::cout << "累计分配次数: " << final_stats.allocated << std::endl;
    std::cout << "后台扫描次数: " << final_stats.scans << std::endl;
    std::cout << "当前缓存大小: " << final_stats.cache_size << std::endl;
}

std::cout << "\n========== 性能测试结束 ==========\n" << std::endl;

return 0;
}