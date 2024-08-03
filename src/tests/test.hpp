#pragma once

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <functional>
#include <string>

/**
 * @brief Used as a generic pointer to test classes, and defines the test
 * interface.
 */
class Test 
{
public:

    /**
     * @brief Create an instance of the test.
     * @returns A pointer to the test on success, or nullptr on failure.
     */
    virtual std::unique_ptr<Test> create() = 0;

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

    using TestCreator = std::function<std::unique_ptr<Test>()>;

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
    static bool run(std::string name)
    {
        using namespace std::string_literals;

        auto it = s_tests.find(name);
        if (it == s_tests.end()) {
            std::cerr << "test \"" << name << "\" does not exist" << std::endl;
            return false;
        }

        auto test = it->second();
        if (!test) {
            std::cerr << "failed to create test \"" << name << "\"" << std::endl;
            return false;
        }

        bool success = test->run();

        if (success)
            std::cout << "test \"" << name << "\" okay" << std::endl; 
        else
            std::cout << "test \"" << name << "\" fail" << std::endl;

        return success;
    }

private:

    // Allow subclasses of Test to add to s_tests.
    template <class Derived> friend class RegisteredTest;

    /// Registered subclasses of Test.
    static std::unordered_map<std::string, TestCreator> s_tests;
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

        TestSuite::s_tests.emplace(Derived::TEST_NAME, Derived::create);
        return true;
    }

    /// Register the test during static initialisation.
    static inline bool registered = register_test();
};
