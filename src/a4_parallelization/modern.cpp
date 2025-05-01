/**
 * @file modern.cpp
 * @brief 现代C++风格实现的并行for_each循环
 */
#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

/**
 * @brief 使用线程池的并行for_each函数
 *
 * 创建一个简单的线程池来处理任务
 *
 * @tparam Iterator 迭代器类型
 * @tparam Function 函数类型
 * @param begin 起始迭代器
 * @param end 结束迭代器
 * @param func 要应用于每个元素的函数
 */
template <typename Iterator, typename Function>
void parallel_for_each_pool(Iterator begin, Iterator end, Function func)
{
    // 获取硬件支持的线程数量
    unsigned int num_threads = std::thread::hardware_concurrency();
    // 确保至少有一个线程
    num_threads = num_threads > 0 ? num_threads : 1;

    // 计算迭代器范围大小
    size_t total_size = std::distance(begin, end);

    // 如果元素太少，不使用并行
    if (total_size < num_threads * 4) {
        std::for_each(begin, end, func);
        return;
    }

    // 每个线程处理的元素数量
    size_t block_size = total_size / num_threads;

    // 存储所有线程
    std::vector<std::thread> threads;
    threads.reserve(num_threads - 1);

    // 工作线程函数
    auto worker = [&func](Iterator block_begin, Iterator block_end) {
        std::for_each(block_begin, block_end, func);
    };

    // 启动线程处理每个块
    Iterator block_begin = begin;
    for (unsigned int i = 0; i < num_threads - 1; ++i) {
        Iterator block_end = block_begin;
        std::advance(block_end, block_size);

        threads.emplace_back(worker, block_begin, block_end);
        block_begin = block_end;
    }

    // 在当前线程处理最后一块
    worker(block_begin, end);

    // 等待所有线程完成
    for (auto& thread : threads) {
        thread.join();
    }
}

/**
 * @brief 使用异步任务的并行for_each函数
 *
 * 利用std::async和std::future创建异步任务处理
 *
 * @tparam Iterator 迭代器类型
 * @tparam Function 函数类型
 * @param begin 起始迭代器
 * @param end 结束迭代器
 * @param func 要应用于每个元素的函数
 */
template <typename Iterator, typename Function>
void parallel_for_each_async(Iterator begin, Iterator end, Function func)
{
    // 获取硬件支持的线程数量
    unsigned int num_threads = std::thread::hardware_concurrency();
    // 确保至少有一个线程
    num_threads = num_threads > 0 ? num_threads : 1;

    // 计算迭代器范围大小
    size_t total_size = std::distance(begin, end);

    // 如果元素太少，不使用并行
    if (total_size < num_threads * 4) {
        std::for_each(begin, end, func);
        return;
    }

    // 每个线程处理的元素数量
    size_t block_size = total_size / num_threads;

    // 存储所有任务的future
    std::vector<std::future<void>> futures;
    futures.reserve(num_threads - 1);

    // 工作线程函数
    auto worker = [&func](Iterator block_begin, Iterator block_end) {
        std::for_each(block_begin, block_end, func);
    };

    // 启动异步任务处理每个块
    Iterator block_begin = begin;
    for (unsigned int i = 0; i < num_threads - 1; ++i) {
        Iterator block_end = block_begin;
        std::advance(block_end, block_size);

        futures.push_back(std::async(std::launch::async, worker, block_begin, block_end));
        block_begin = block_end;
    }

    // 在当前线程处理最后一块
    worker(block_begin, end);

    // 等待所有异步任务完成
    for (auto& future : futures) {
        future.wait();
    }
}

/**
 * @brief 主函数，演示并行for_each函数的不同实现
 */
int main()
{
    constexpr size_t SIZE = 1000000;
    std::vector<int> numbers(SIZE);

    // 初始化向量
    std::iota(numbers.begin(), numbers.end(), 0);

    // 创建结果向量
    std::vector<int> squares1(SIZE);
    std::vector<int> squares2(SIZE);

    // 使用线程池的并行for_each
    std::cout << "使用线程池的并行for_each：" << std::endl;
    auto start1 = std::chrono::high_resolution_clock::now();

    parallel_for_each_pool(numbers.begin(), numbers.end(), [&squares1, &numbers](int n) {
        size_t index = &n - &numbers[0];
        squares1[index] = n * n;
    });

    auto end1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed1 = end1 - start1;
    std::cout << "执行时间：" << elapsed1.count() << " ms" << std::endl;

    // 使用异步任务的并行for_each
    std::cout << "使用异步任务的并行for_each：" << std::endl;
    auto start2 = std::chrono::high_resolution_clock::now();

    parallel_for_each_async(numbers.begin(), numbers.end(), [&squares2, &numbers](int n) {
        size_t index = &n - &numbers[0];
        squares2[index] = n * n;
    });

    auto end2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed2 = end2 - start2;
    std::cout << "执行时间：" << elapsed2.count() << " ms" << std::endl;

    // 验证结果正确性
    for (size_t i = 0; i < 10; ++i) {
        std::cout << i << "² = " << squares1[i] << " (方法1), " << squares2[i] << " (方法2)" << std::endl;
    }

    // 验证两种方法的结果是否一致
    bool results_match = squares1 == squares2;
    std::cout << "两种方法的结果" << (results_match ? "一致" : "不一致") << std::endl;

    return 0;
}