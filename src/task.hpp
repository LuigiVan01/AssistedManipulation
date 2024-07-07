#include <mutex>
#include <thread>
#include <vector>
#include <condition_variable>
#include <functional>
#include <queue>
#include <future>
#include <barrier>
#include <exception>
#include <type_traits>
#include <memory>

template<typename Callable, typename... Args>
using invoke_result_t = typename std::invoke_result<Callable, Args...>::type;

class ThreadPool
{
public:

    /**
     * @brief Construct a new Thread Pool object
     * 
     * @param n The number of threads in the pool.
     */
    inline ThreadPool(unsigned int n);

    inline ~ThreadPool();

    inline void worker(std::stop_token stop);

    template<typename Callable, typename... Args>
    std::future<invoke_result_t<Callable, Args...>>
    enqueue(unsigned int priority, Callable &&callable, Args&&... args);

    template<typename Callable, typename... Args>
    std::future<invoke_result_t<Callable, Args...>>
    enqueue(Callable &&callable, Args&&... args)
    {
        return enqueue(0, std::forward<Callable&&>(callable), std::forward<Args&&>(args)...);
    }

private:

    struct Task {
        unsigned int priority;
        std::function<void()> run;

        friend bool operator<(const Task &left, const Task &other) {
            return left.priority < other.priority;
        }
    };

    std::mutex m_mutex;

    std::condition_variable m_condition;

    std::stop_source m_stop;

    std::priority_queue<Task> m_tasks;

    std::vector<std::jthread> m_threads;
};

inline ThreadPool::ThreadPool(unsigned int n)
{
    for (unsigned int i = 0; i < n; i++) {
        m_threads.push_back(
            std::jthread(&ThreadPool::worker, this, m_stop.get_token())
        );
    }
}

inline ThreadPool::~ThreadPool()
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

        task.run();
    }
}

template<typename Callable, typename... Args>
std::future<invoke_result_t<Callable, Args...>>
ThreadPool::enqueue(unsigned int priority, Callable &&function, Args&&... args)
{
    using Result = invoke_result_t<Callable, Args...>;

    if (m_stop.stop_requested())
        throw std::runtime_error("enqueing task to stopped thread pool");

    auto task = std::make_shared<std::packaged_task<Result(Args...)>>(
        std::bind(std::forward<Callable>(function), std::forward<Args>(args)...)
    );

    std::future<Result> future = task->get_future();

    {
        std::scoped_lock lock(m_mutex);
        m_tasks.emplace(priority, [task = std::move(task)]{ (*task)(); });
    }

    m_condition.notify_one();

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
//     std::future<invoke_result_t<Function(Args...)>>
//     void add_task();

// private:

//     std::barrier m_barrier;
// };
