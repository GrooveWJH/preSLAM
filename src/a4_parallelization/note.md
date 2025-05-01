# 传统与现代 C++ 并行 for_each 循环的实现对比

本文档比较了不使用库实现并行for_each循环的传统和现代C++方法的差异。

## 基本原理

并行for_each函数的目标是在多线程环境下处理容器中的元素，提高处理速度。基本步骤包括：

1. 确定可用的线程数量
2. 将输入数据分成多个块
3. 为每个块分配一个线程进行处理
4. 等待所有线程完成处理

## 传统实现与现代实现对比

### 传统实现 (traditional.cpp)

传统实现使用基本的C++线程管理：

```cpp
template <typename Iterator, typename Function>
void parallel_for_each(Iterator begin, Iterator end, Function func) {
    // 获取硬件支持的线程数量
    unsigned int num_threads = std::thread::hardware_concurrency();
    num_threads = num_threads > 0 ? num_threads : 1;
    
    // 计算迭代器范围大小
    size_t total_size = std::distance(begin, end);
    
    // 每个线程处理的元素数量
    size_t block_size = total_size / num_threads;
    
    // 存储所有线程
    std::vector<std::thread> threads;
    threads.reserve(num_threads);
    
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
```

特点：

- 直接使用`std::thread`创建线程
- 手动计算数据块大小和范围
- 显式管理线程生命周期
- 使用`join()`等待线程完成
- 相对较低级别的线程控制

### 现代实现 (modern.cpp)

现代实现使用更高级的异步编程工具：

```cpp
template <typename Iterator, typename Function>
void parallel_for_each_async(Iterator begin, Iterator end, Function func) {
    // 获取硬件支持的线程数量并计算块大小...
    
    // 存储所有任务的future
    std::vector<std::future<void>> futures;
    futures.reserve(num_threads - 1);
    
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
```

特点：

- 使用`std::async`创建异步任务
- 通过`std::future`管理任务结果
- 更高级别的抽象
- 更简洁的任务管理
- 可能提供更好的异常处理

## 现代C++中的其他并行实现方法

### 使用C++17标准库的执行策略

如果编译器支持C++17及以上版本，可以使用执行策略简化并行代码：

```cpp
template <typename Iterator, typename Function>
void parallel_for_each_execution(Iterator begin, Iterator end, Function func) {
    std::for_each(std::execution::par, begin, end, func);
}
```

特点：

- 极其简洁
- 由标准库管理并行执行
- 无需手动线程管理
- 需要C++17及以上支持

### 使用线程池

为了更好地控制线程资源，可以实现线程池模式：

```cpp
class ThreadPool {
private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;

public:
    ThreadPool(size_t threads) : stop(false) {
        for(size_t i = 0; i < threads; ++i) {
            workers.emplace_back([this] {
                while(true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex);
                        this->condition.wait(lock, [this] { 
                            return this->stop || !this->tasks.empty(); 
                        });
                        if(this->stop && this->tasks.empty())
                            return;
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }
                    task();
                }
            });
        }
    }

    // 添加新任务并返回future
    template<class F>
    auto enqueue(F&& f) -> std::future<decltype(f())> {
        using return_type = decltype(f());
        auto task = std::make_shared<std::packaged_task<return_type()>>(std::forward<F>(f));
        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            if(stop)
                throw std::runtime_error("ThreadPool已停止");
            tasks.emplace([task](){ (*task)(); });
        }
        condition.notify_one();
        return res;
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for(std::thread &worker: workers)
            worker.join();
    }
};
```

特点：

- 重用线程，减少线程创建和销毁的开销
- 任务队列可以动态调整工作负载
- 更好的资源管理
- 更复杂的实现，但长期运行的应用程序中更有效

## 总结比较

| 特性 | 传统方法 | 现代方法（async） | 线程池 | C++17执行策略 |
| ---- | -------- | ----------------- | ------ | ------------- |
| 代码简洁度 | 中等 | 高 | 低（实现复杂） | 最高 |
| 性能 | 良好 | 良好 | 优（重用线程） | 取决于实现 |
| 资源管理 | 手动 | 自动 | 优化 | 自动 |
| 复杂度 | 中等 | 低 | 高 | 最低 |
| 所需C++版本 | C++11 | C++11 | C++11 | C++17 |
| 适用场景 | 通用 | 通用 | 大量小任务 | 简单并行任务 |

## 语言特性对比

### 传统C++特性（传统实现中使用）：

- `std::thread`直接创建线程
- 手动线程管理和同步
- 显式数据划分
- 手动线程生命周期管理

### 现代C++特性（现代实现中使用）：

- `std::async`和`std::future`
- Lambda表达式简化任务定义
- 自动线程管理
- 异步任务抽象

## 性能考虑

不同实现方法的性能特点：

1. **传统实现**：直接控制线程，性能可预测但开销固定
2. **Async实现**：由系统决定异步运行策略，可能在某些情况下更高效
3. **线程池**：重用线程减少创建/销毁开销，适合大量小任务
4. **执行策略**：由标准库优化，可能利用特定平台特性

## 最佳实践

选择合适的并行for_each实现时，考虑因素包括：

- 数据规模和处理复杂度
- 任务独立性（是否需要同步）
- 目标平台支持的C++标准
- 并行任务的粒度

对于简单的并行任务，现代方法通常更简洁高效；对于复杂的长时间运行的应用，线程池可能提供更好的资源管理。

总的来说，C++的并行编程能力已经从直接的线程管理发展到更高级别的抽象，使开发者能够更容易地编写高效的并行代码。 