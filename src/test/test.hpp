#pragma once

#include <stdexcept>
#include <iostream>
#include <unordered_map>
#include <functional>
#include <string>
#include <chrono>
#include <future>
#include <filesystem>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * @brief Used as a generic pointer to test classes, and defines the test
 * interface.
 */
class Test 
{
public:

    /**
     * @brief Run the test.
     * @returns If the test ran successfully.
     */
    virtual bool run() = 0;
};

/**
 * @brief The suite of tests that can be run.
 */
class TestSuite
{
public:

    /**
     * @brief Function the takes a json configuration and returns a test
     * instance to run.
     */
    using TestCreator = std::function<std::unique_ptr<Test>(json&)>;

    /**
     * @brief Register a test at runtime.
     * 
     * @param name The name of the test.
     * @param creator The function that creates the test.
     * 
     * @throws std::runtime_error if the test name already exists.
     */
    static void register_test(std::string name, TestCreator creator)
    {
        using namespace std::string_literals;

        if (s_tests.contains(name)) {
            throw std::runtime_error(
                "test with name "s + name + " already exists"
            );
        }

        s_tests.emplace(name, creator);
    }

    /**
     * @brief Run all tests.
     * @returns If all the tests were successful.
     */
    static bool run()
    {
        bool success = true;
        for (auto &[name, creator] : s_tests)
            success |= run(name);
        return success;
    }

    /**
     * @brief Run multiple tests.
     * 
     * @param names The names of the tests to run.
     * @returns If all the tests were successful.
     */
    static bool run(const std::vector<std::string> &names)
    {
        bool success = true;
        for (const auto &name : names)
            success |= run(name);
        return success;
    }

    /**
     * @brief Run a test.
     * 
     * @param name The name of the test to run.
     * @returns If the test was successful.
     */
    static bool run(std::string name, json configuration = nullptr)
    {
        using namespace std::string_literals;
        using namespace std::chrono_literals;
        using namespace std::chrono;

        auto it = s_tests.find(name);
        if (it == s_tests.end()) {
            std::cerr << "test \"" << name << "\" does not exist" << std::endl;
            return false;
        }

        auto test = it->second(configuration);
        if (!test) {
            std::cerr << "failed to create test \"" << name << "\""
                      << " with configuration " << configuration.dump(4)
                      << std::endl;
            return false;
        }

        std::cout << "test \"" << name << "\" " << std::flush;

        // Display a dot for every second waiting, in real time.
        std::promise<void> promise;
        std::future<void> signal = promise.get_future();
        auto future = std::async([&signal]{
            for (;;) {
                if (signal.wait_for(1s) != std::future_status::ready)
                    std::cout << '.' << std::flush;
                else
                    return;
            }
        });

        // Time the duration of the test.
        auto start = steady_clock::now();
        bool success = test->run();
        auto stop = steady_clock::now();

        // Join the dot display thread.
        promise.set_value();
        future.wait();

        if (success)
            std::cout << " okay"; 
        else
            std::cout << " fail";

        auto duration = duration_cast<microseconds>(stop - start);
        auto min = duration_cast<minutes>(duration);
        auto sec = duration_cast<seconds>(duration - min);
        auto ms = duration_cast<milliseconds>(duration - min - sec);

        std::cout << " (" << min << " " << sec << " " << ms << ")" << std::endl;

        return success;
    }

private:

    // Allow subclasses of Test to add to s_tests.
    template <class Derived> friend class RegisteredTest;

    /// Registered subclasses of Test.
    inline static std::unordered_map<std::string, TestCreator> s_tests;
};

/**
 * @brief A test that is automatically registered at initialisation time.
 * 
 * Assumes the derived class has `static const char *TEST_NAME` defined. 
 * 
 * @tparam Derived The subclass of Test.
 */
template<typename Derived>
class RegisteredTest : public Test
{
private:

    /**
     * @brief Registers the test. Private, cannot be called from subclasses.
     */
    static bool register_test()
    {
        using namespace std::string_literals;

        if (TestSuite::s_tests.contains(Derived::TEST_NAME)) {
            throw std::runtime_error(
                "test with name "s + Derived::TEST_NAME + " already exists"
            );
        }
 
        TestSuite::s_tests[Derived::TEST_NAME] = [](json &configuration){
            return Derived::create(configuration);
        };

        return true;
    }

    /// Register the test during static initialisation.
    static inline bool registered = register_test();
};
