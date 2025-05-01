/**
 * @file traditional.cpp
 * @brief 传统C++风格实现的并行for_each循环
 */
#include <iostream>
#include <thread>
#include <vector>

/**
 * @brief 传统C++风格的并行for_each函数
 *
 * 将范围内的元素分割为多个块，每个线程处理一个块
 *
 * @tparam Iterator 迭代器类型
 * @tparam Function 函数类型
 * @param begin 起始迭代器
 * @param end 结束迭代器
 * @param func 要应用于每个元素的函数
 */
template <typename Iterator, typename Function>
void parallel_for_each(Iterator begin, Iterator end, Function func)
{
    // 获取硬件支持的线程数量
    unsigned int num_threads = std::thread::hardware_concurrency();
    // 确保至少有一个线程
    num_threads = num_threads > 0 ? num_threads : 1;

    // 计算迭代器范围大小
    size_t total_size = std::distance(begin, end);

    // 如果元素太少，不使用并行
    if (total_size < num_threads * 4) {
        for (Iterator it = begin; it != end; ++it) {
            func(*it);
        }
        return;
    }

    // 每个线程处理的元素数量
    size_t block_size = total_size / num_threads;

    // 存储所有线程
    std::vector<std::thread> threads;
    threads.reserve(num_threads);

    // 创建工作线程的函数
    auto worker = [&func](Iterator block_begin, Iterator block_end) {
        for (Iterator it = block_begin; it != block_end; ++it) {
            func(*it);
        }
    };

    // 启动线程处理每个块
    Iterator block_begin = begin;
    for (unsigned int i = 0; i < num_threads - 1; ++i) {
        Iterator block_end = block_begin;
        std::advance(block_end, block_size);

        threads.emplace_back(worker, block_begin, block_end);
        block_begin = block_end;
    }

    // 最后一个线程处理剩余部分
    worker(block_begin, end);

    // 等待所有线程完成
    for (auto& thread : threads) {
        thread.join();
    }
}

/**
 * @brief 主函数，演示parallel_for_each的用法
 */
int main()
{
    std::vector<int> numbers(1000);

    // 初始化向量
    for (int i = 0; i < 1000; ++i) {
        numbers[i] = i;
    }

    // 使用并行for_each计算每个数字的平方
    std::vector<int> squares(1000);

    parallel_for_each(numbers.begin(), numbers.end(), [&squares, &numbers](int n) {
        size_t index = &n - &numbers[0];
        squares[index] = n * n;
    });

    // 验证结果
    std::cout << "前10个数字的平方：" << std::endl;
    for (int i = 0; i < 10; ++i) {
        std::cout << i << "² = " << squares[i] << std::endl;
    }

    return 0;
}