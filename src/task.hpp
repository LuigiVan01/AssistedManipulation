#include <mutex>
#include <thread>
#include <vector>
#include <condition_variable>
#include <functional>
#include <queue>
#include <future>
#include <barrier>
#include <exception>

template<typename Function, typename... Args>
using result_of_t = typename std::result_of<Function(Args...)>::type;

class ThreadPool
{
public:

    /**
     * @brief Construct a new Thread Pool object
     * 
     * @param n The number of threads in the pool.
     */
    ThreadPool(unsigned int n);

    ~ThreadPool();

    inline void worker(std::stop_token stop);

    template<typename Function, typename... Args>
    std::future<result_of_t<Function(Args...)>>
    enqueue(unsigned int priority, Function &&callable, Args&&... args);

    template<typename Function, typename... Args>
    std::future<result_of_t<Function(Args...)>>
    enqueue(Function &&callable, Args&&... args)
    {
        return enqueue(0, std::forward(function), std::forward<Args>(args)...);
    }

private:

    using Task = std::pair<int, std::function<void()>>;

    bool compare_tasks(const Task &left, const Task &right) {
        return left.first < right.first;
    }

    std::mutex m_mutex;

    std::condition_variable m_condition;

    std::stop_source m_stop;

    std::priority_queue<Task> m_tasks;

    std::vector<std::jthread> m_threads;
};

inline ThreadPool::ThreadPool(unsigned int n)
    : m_tasks(ThreadPool::compare_tasks)
{
    for (int i = 0; i < n; i++) {
        m_threads.push_back(
            std::jthread(&ThreadPool::worker, m_stop.get_token())
        );
    }
}

ThreadPool::~ThreadPool()
{
    m_stop.request_stop();
    m_condition.notify_all();
}

void ThreadPool::worker(std::stop_token stop)
{
    Task task;

    while (!stop.stop_requested() || !m_tasks.empty()) {

        {
            std::unique_lock lock(m_mutex);

            if (m_tasks.empty()) {
                m_condition.wait(
                    lock,
                    [&](){ return stop.stop_requested() || !m_tasks.empty(); }
                );

                if (m_tasks.empty()) {
                    if (stop.stop_requested())
                        return;
                    continue;
                }
            }

            task = std::move(m_tasks.top());
            m_tasks.pop();
        }

        task.second();
    }
}

template<typename Function, typename... Args>
std::future<result_of_t<Function(Args...)>>
ThreadPool::enqueue(unsigned int priority, Function &&function, Args&&... args)
{
    if (m_stop.stop_requested())
        return;

    auto task = std::make_unique<std::packaged_task<result_of_t<Callable>>>(
        std::bind(std::forward(function), std::forward<Args>(args)...)
    );

    auto future = task->get_future();

    {
        std::scoped_lock lock(m_mutex);
        m_tasks.emplace(std::make_pair(priority, [task](){ (*task)(); }));
    }

    m_tasks.notify_one();

    return future;
}

// template<typename... Ts>
// class ExceptionGroup : public std::exception
// {
// public:

//     ExceptionGroup

// private:
// };

// class TaskGroup
// {
// public:

//     TaskGroup()
//         :
//     {};

//     template<typename Function, typename... Args>
//     std::future<result_of_t<Function(Args...)>>
//     void add_task();

// private:

//     std::barrier m_barrier;
// };
