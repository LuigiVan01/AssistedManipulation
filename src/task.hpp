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

/**
 * @brief Convenience type for retrieving type of std::invoke_result.
 */
template<typename Callable, typename... Args>
using invoke_result_t = typename std::invoke_result<Callable, Args...>::type;

/**
 * @brief A collection of threads perform tasks asynchronously.
 * 
 * To perform a function asynchronously, a function and it's arguments are
 * passed to the enqueue method. This packs the function and arguments into a
 * std::packaged_task object, that is placed into a priority queue. Each worker
 * thread will pop tasks off the queue in order of priority, and run in that
 * thread asynchronously. If the priority queue runs out of tasks, the worker
 * will wait on a condition variable that is signalled when a new task is
 * pushed to the queue.
 */
class ThreadPool
{
public:

    /**
     * @brief Create a new thread pool of n threads.
     * 
     * @param n The number of threads in the pool.
     */
    inline ThreadPool(unsigned int n);

    /**
     * @brief Requests the worker threads to stop and join.
     */
    inline ~ThreadPool();

    /**
     * @brief Add a new task to the thread pool queue with a priority.
     * 
     * @tparam Callable The type of the callable, such as a lambda or function.
     * @tparam Args The types of all the function arguments.
     * 
     * @param priority The priority of the task. The greater the number the
     * higher the priority.
     * @param callable The function to run as a task.
     * @param args The arguments to the task function.
     * 
     * @returns A future object that contains the result of the function. This
     * future can be waited on for the task to complete.
     */
    template<typename Callable, typename... Args>
    std::future<invoke_result_t<Callable, Args...>>
    enqueue(unsigned int priority, Callable &&callable, Args&&... args);

    /**
     * @brief Add a new task to the thread pool queue with the lowest priority.
     * 
     * @tparam Callable The type of the callable, such as a lambda or function.
     * @tparam Args The types of all the function arguments.
     * 
     * @param callable The function to run as a task.
     * @param args The arguments to the task function.
     * 
     * @returns A future object that contains the result of the function. This
     * future can be waited on for the task to complete.
     */
    template<typename Callable, typename... Args>
    std::future<invoke_result_t<Callable, Args...>>
    enqueue(Callable &&callable, Args&&... args)
    {
        return enqueue(0, std::forward<Callable>(callable), std::forward<Args>(args)...);
    }

private:

    /**
     * @brief A structure containing the task data.
     */
    struct Task {

        /// The priority of the task. Greater numbers have higher priorities.
        unsigned int priority;

        /// A function that performs the task. This will always be a lambda
        /// that calls the std::bind() object (that packs a function and its
        /// arguments into a callable.
        std::function<void()> run;

        /**
         * @brief Compare the priorities of two tasks. Used to order the queue.
         * 
         * @param first The task on the first.
         * @param second The task on the second.
         * @returns If the first task has lower priority than the second.
         */
        friend bool operator<(const Task &first, const Task &second) {
            return first.priority < second.priority;
        }
    };

    /**
     * @brief The routine of each worker.
     * 
     * @param stop A stop token that can be checked if the worker thread should
     * terminate.
     */
    inline void worker(std::stop_token stop);

    /// Mutex protecting concurrent access to the priority queue.
    std::mutex m_mutex;

    /// Condition to wait on when a worker has no tasks to run.
    std::condition_variable m_condition;

    /// An atomic boolean stop flag that signals the workers to terminate.
    std::stop_source m_stop;

    /// The queue of tasks to perform. Tasks are picked up by workers and run.
    std::priority_queue<Task> m_tasks;

    /// The worker threads themselves. Automatically joined on destruction.
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
    // Request all the worker threads to stop, then wake them up to terminate.
    m_stop.request_stop();
    m_condition.notify_all();
}

void ThreadPool::worker(std::stop_token stop)
{
    // The task to perform. Required since popping and running occur in
    // different scopes.
    Task task;

    for (;;) {
        {
            std::unique_lock lock(m_mutex);

            // Does not wait if a task exists or stop is requested or a task exists.
            m_condition.wait(
                lock,
                [&]{ return stop.stop_requested() || !m_tasks.empty(); }
            );

            if (m_tasks.empty()) {

                // If there are tasks, they must be run until the queue is empty
                // incase other threads are being blocked by futures. Only
                // shutdown if empty.
                if (stop.stop_requested())
                    return;

                // Continue waiting for a task to run.
                continue;
            }

            task = std::move(m_tasks.top());
            m_tasks.pop();
        }

        // Run the task after releasing the lock in the above scope.
        task.run();
    }
}

template<typename Callable, typename... Args>
std::future<invoke_result_t<Callable, Args...>>
ThreadPool::enqueue(unsigned int priority, Callable &&function, Args&&... args)
{
    // The type of the result of calling the function.
    using Result = invoke_result_t<Callable, Args...>;

    if (m_stop.stop_requested())
        throw std::runtime_error("enqueuing task to stopped thread pool");

    // Must be a std::shared_ptr since a std::unique_ptr cannot be copied in the
    // subsequent lambda expression.
    auto task = std::make_shared<std::packaged_task<Result(Args...)>>(
        std::bind(std::forward<Callable>(function), std::forward<Args>(args)...)
    );

    std::future<Result> future = task->get_future();

    {
        std::scoped_lock lock(m_mutex);
        m_tasks.emplace(priority, [task = std::move(task)]{ (*task)(); });
    }

    // If there is a thread waiting, then that thread will pick the task up.
    // Otherwise the next time a thread checks if the task queue is empty, it
    // will pick it up. Therefore only need to notify one thread.
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
