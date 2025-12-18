// Port allocator - header-only, Boost-based, mutex-protected implementation
#pragma once

#include <boost/asio.hpp>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <random>
#include <set>
#include <stdexcept>
#include <thread>
#include <vector>

namespace network_tools {

// Header-only implementation requires inline/linkage-safe definitions.
// 我们使用 Boost 线程/condition_variable，但用 std::mutex/std::unique_lock 保持可读性。

class PortHandle {
public:
    // PortHandle 持有一个被分配的端口号，以及一个可选的释放回调（RAII）
    PortHandle() noexcept = default;

    PortHandle(std::uint16_t port, std::function<void(std::uint16_t)> release_cb)
        : port_(port), release_cb_(std::move(release_cb)) {}

    PortHandle(PortHandle&& other) noexcept {
        std::lock_guard<std::mutex> lk(other.m_);
        port_ = other.port_;
        release_cb_ = std::move(other.release_cb_);
        other.port_ = 0;
    }

    PortHandle& operator=(PortHandle&& other) noexcept {
        if (this != &other) {
            release();
            std::lock_guard<std::mutex> lk(other.m_);
            port_ = other.port_;
            release_cb_ = std::move(other.release_cb_);
            other.port_ = 0;
        }
        return *this;
    }

    // 非拷贝
    PortHandle(const PortHandle&) = delete;
    PortHandle& operator=(const PortHandle&) = delete;

    ~PortHandle() { release(); }

    std::uint16_t port() const noexcept { return port_; }

    bool valid() const noexcept { return port_ != 0; }

    void release() noexcept {
        if (port_ != 0 && release_cb_) {
            try {
                release_cb_(port_);
            } catch (...) {
                // destructor must not throw
            }
            port_ = 0;
        }
    }

private:
    mutable std::mutex m_;
    std::uint16_t port_{0};
    std::function<void(std::uint16_t)> release_cb_;
};

class BoostPortAllocator {
public:
    struct Config {
        std::uint16_t min_port = 48000;
        std::uint16_t max_port = 54000;
        std::size_t cache_size = 32;
        std::chrono::milliseconds scan_interval{500};
    };

    struct Stats {
        std::size_t allocated = 0;
        std::size_t scans = 0;
        std::size_t cache_size = 0;
    };

    // inline 单例访问（header-only 安全）
    // 使用重载避免默认参数的成员初始化器问题
    static BoostPortAllocator& instance() {
        static Config default_cfg;
        static BoostPortAllocator inst(default_cfg);
        return inst;
    }

    static BoostPortAllocator& instance(const Config& cfg) {
        static BoostPortAllocator inst(cfg);
        return inst;
    }

    // 获取端口（RAII）
    inline PortHandle acquire() { return acquire_impl(); }

    // 批量获取
    inline std::vector<PortHandle> acquireMultiple(std::size_t n) {
        std::vector<PortHandle> out;
        out.reserve(n);
        for (std::size_t i = 0; i < n; ++i) out.push_back(acquire_impl());
        return out;
    }

    inline Stats statistics() const {
        Stats s;
        std::lock_guard<std::mutex> lk(mutex_);
        s.allocated = allocated_;
        s.scans = scans_;
        s.cache_size = cache_.size();
        return s;
    }

    // 显式释放（通常由 PortHandle 调用）
    inline void release(std::uint16_t port) {
        if (port == 0) return;
        std::lock_guard<std::mutex> lk(mutex_);
        // 归还时仅将端口放入已知集合，避免重复
        if (used_.erase(port) > 0) {
            // 放回缓存尾部
            if (cache_.size() < cfg_.cache_size) cache_.push_back(port);
        }
    }

private:
    BoostPortAllocator(const Config& cfg)
        : cfg_(cfg), rng_(std::random_device{}()), io_ctx_(), work_guard_(boost::asio::make_work_guard(io_ctx_)) {
        // 启动后台线程池：一个线程用于 io_ctx（用于检查端口时的短期 acceptor）
        bg_thread_ = std::thread([this] { io_ctx_.run(); });

        stopped_ = false;
        scanner_thread_ = std::thread([this] { scannerLoop(); });

        // 初始填充
        fillCache();
    }

    ~BoostPortAllocator() {
        {
            std::lock_guard<std::mutex> lk(mutex_);
            stopped_ = true;
        }
        cv_.notify_one();

        if (scanner_thread_.joinable()) scanner_thread_.join();

        work_guard_.reset();
        io_ctx_.stop();
        if (bg_thread_.joinable()) bg_thread_.join();
    }

    // 非拷贝
    BoostPortAllocator(const BoostPortAllocator&) = delete;
    BoostPortAllocator& operator=(const BoostPortAllocator&) = delete;

    // 内部实现：尝试从缓存获取；若缓存空则直接扫描并返回
    PortHandle acquire_impl() {
        std::uint16_t port = 0;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (!cache_.empty()) {
                port = cache_.front();
                cache_.pop_front();
                // 标记为已用
                used_.insert(port);
                allocated_++;
            }
        }

        if (port == 0) {
            // 缓存空或者未命中，通过扫描直接获取
            port = scanAndReservePort();
            if (port == 0) throw std::runtime_error("no available port");
        }

        // 返回 PortHandle，析构时会调用 release
        return PortHandle(port, [this](std::uint16_t p) { this->release(p); });
    }

    // 扫描并占用一个可用端口（真正 bind 后保留到 used_）
    std::uint16_t scanAndReservePort() {
        std::uniform_int_distribution<std::uint32_t> dist(cfg_.min_port, cfg_.max_port);

        // 尝试若干次随机采样，然后线性扫描
        for (int attempt = 0; attempt < 8; ++attempt) {
            std::uint16_t p = static_cast<std::uint16_t>(dist(rng_));
            if (tryBindAndReserve(p)) return p;
        }

        // 线性扫描
        for (std::uint16_t p = cfg_.min_port; p <= cfg_.max_port; ++p) {
            if (tryBindAndReserve(p)) return p;
        }
        return 0;
    }

    bool tryBindAndReserve(std::uint16_t port) {
        // 快速检查：是否已被本进程使用
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (used_.count(port) > 0) return false;
        }

        // 采用 Boost.Asio 尝试绑定短期 acceptor；成功则保留端口
        try {
            boost::asio::ip::tcp::acceptor acceptor(io_ctx_);
            boost::asio::ip::tcp::endpoint ep(boost::asio::ip::tcp::v4(), port);
            acceptor.open(ep.protocol());
            acceptor.set_option(boost::asio::socket_base::reuse_address(true));
            acceptor.bind(ep);

            // 绑定成功，立即关闭 acceptor BUT 为了防止 race，我们将端口标记为 used_
            {
                std::lock_guard<std::mutex> lk(mutex_);
                if (used_.count(port) > 0) return false;  // double-check
                used_.insert(port);
                allocated_++;
            }

            acceptor.close();
            return true;
        } catch (const std::exception&) { return false; }
    }

    void fillCache() {
        std::vector<std::uint16_t> found;
        found.reserve(cfg_.cache_size);

        // 随机尝试，避免长时间阻塞
        for (std::size_t i = 0; i < cfg_.cache_size * 2 && found.size() < cfg_.cache_size; ++i) {
            std::uint16_t p = static_cast<std::uint16_t>(
                std::uniform_int_distribution<std::uint32_t>(cfg_.min_port, cfg_.max_port)(rng_));
            if (tryBindAndReserve(p)) { found.push_back(p); }
        }

        // 将找到的端口加入缓存（已在 tryBindAndReserve 中被标记为 used_）
        if (!found.empty()) {
            std::lock_guard<std::mutex> lk(mutex_);
            for (auto p : found) {
                if (cache_.size() < cfg_.cache_size) cache_.push_back(p);
            }
            scans_++;
        }
    }

    void scannerLoop() {
        while (true) {
            std::unique_lock<std::mutex> lk(mutex_);
            if (cv_.wait_for(lk, cfg_.scan_interval,
                             [this] { return stopped_ || cache_.size() < cfg_.cache_size / 2; })) {
                if (stopped_) break;
            }
            // release lock while filling to avoid blocking acquire
            lk.unlock();
            fillCache();
        }
    }

private:
    Config cfg_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;

    // 缓存与使用集合
    std::deque<std::uint16_t> cache_;
    std::set<std::uint16_t> used_;

    // 状态
    std::size_t allocated_{0};
    std::size_t scans_{0};
    bool stopped_{false};

    // 随机数
    std::mt19937 rng_;

    // 后台线程
    std::thread scanner_thread_;

    // Boost.Asio 用于绑定尝试
    boost::asio::io_context io_ctx_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread bg_thread_;
};

}  // namespace network_tools
