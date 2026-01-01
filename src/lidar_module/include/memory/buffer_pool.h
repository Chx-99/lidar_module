#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

namespace memory_pool {
/**
 * @brief 高性能无锁内存池
 *
 * @tparam T 缓冲区元素类型
 * @tparam PoolSize 池大小，建议设置为典型并发数的2-4倍
 */
template <typename T = uint8_t, size_t PoolSize = 32>
class BufferPool {
public:
    using BufferType = std::vector<T>;
    using BufferPtr = std::shared_ptr<BufferType>;

    // 池大小配置
    static constexpr size_t POOL_SIZE = PoolSize;
    static constexpr size_t DEFAULT_BUFFER_CAPACITY = 512;  // 默认预分配容量

    /**
     * @brief 构造函数，预分配池中所有缓冲区
     * @param initial_capacity 每个缓冲区的初始容量
     */
    explicit BufferPool(size_t initial_capacity = DEFAULT_BUFFER_CAPACITY) : index_(0), hit_count_(0), miss_count_(0) {
        // 预分配所有缓冲区
        for (size_t i = 0; i < POOL_SIZE; ++i) {
            pool_[i] = std::make_shared<BufferType>();
            pool_[i]->reserve(initial_capacity);  // 预留容量，避免后续重分配
        }
    }

    /**
     * @brief 从池中获取一个可用的缓冲区
     *
     * 策略：
     * 1. 轮询检查池中的缓冲区
     * 2. 如果use_count==1（仅池持有），则复用该缓冲区
     * 3. 如果所有缓冲区都在使用中，则创建新的临时缓冲区
     *
     * @return BufferPtr 指向可用缓冲区的智能指针
     */
    BufferPtr acquire() {
        // 使用原子操作轮转索引，保证线程安全
        const size_t start_idx = index_.fetch_add(1, std::memory_order_relaxed) % POOL_SIZE;

        // 从当前位置开始，遍历整个池寻找可复用的缓冲区
        for (size_t i = 0; i < POOL_SIZE; ++i) {
            const size_t idx = (start_idx + i) % POOL_SIZE;
            BufferPtr buffer = pool_[idx];

            // use_count==1 表示只有池持有该缓冲区，可以安全复用
            if (buffer.use_count() == 1) {
                // 清空内容但保留capacity，避免内存重分配
                buffer->clear();

                // 统计命中（用于性能监控）
                hit_count_.fetch_add(1, std::memory_order_relaxed);

                return buffer;
            }
        }

        // 池中所有缓冲区都在使用中，创建临时缓冲区（自动降级）
        miss_count_.fetch_add(1, std::memory_order_relaxed);

        auto temp_buffer = std::make_shared<BufferType>();
        temp_buffer->reserve(DEFAULT_BUFFER_CAPACITY);
        return temp_buffer;
    }

    /**
     * @brief 获取池的命中率统计
     * @return 命中率 [0.0, 1.0]，1.0表示所有请求都从池中获取
     */
    double getHitRate() const {
        const uint64_t hits = hit_count_.load(std::memory_order_relaxed);
        const uint64_t misses = miss_count_.load(std::memory_order_relaxed);
        const uint64_t total = hits + misses;

        if (total == 0) {
            return 1.0;  // 未使用时假设命中率100%
        }

        return static_cast<double>(hits) / static_cast<double>(total);
    }

    /**
     * @brief 获取命中次数（用于性能监控）
     */
    uint64_t getHitCount() const { return hit_count_.load(std::memory_order_relaxed); }

    /**
     * @brief 获取未命中次数（用于性能监控）
     */
    uint64_t getMissCount() const { return miss_count_.load(std::memory_order_relaxed); }

    /**
     * @brief 重置统计计数器
     */
    void resetStats() {
        hit_count_.store(0, std::memory_order_relaxed);
        miss_count_.store(0, std::memory_order_relaxed);
    }

    /**
     * @brief 获取池大小
     */
    constexpr size_t poolSize() const { return POOL_SIZE; }

private:
    // 禁止拷贝和赋值
    BufferPool(const BufferPool&) = delete;
    BufferPool& operator=(const BufferPool&) = delete;

    std::array<BufferPtr, POOL_SIZE> pool_;  // 缓冲区池
    std::atomic<size_t> index_;              // 轮转索引（无锁）
    std::atomic<uint64_t> hit_count_;        // 命中次数统计
    std::atomic<uint64_t> miss_count_;       // 未命中次数统计
};

/**
 * @brief 预定义的缓冲区池类型
 */
using ByteBufferPool = BufferPool<uint8_t, 32>;       // 通用字节缓冲池（32个）
using LargeByteBufferPool = BufferPool<uint8_t, 64>;  // 大容量字节缓冲池（64个）
using SmallByteBufferPool = BufferPool<uint8_t, 16>;  // 小容量字节缓冲池（16个）

}  // namespace memory_pool
