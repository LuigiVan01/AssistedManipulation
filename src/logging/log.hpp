#pragma once

#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>

#include "controller/controller.hpp"

class Logger
{
public:

    template<typename... Args>
    void log(Args... args) = 0;
};

template<typename T>
concept Iterable = requires(T iterable)
{
    std::begin(iterable) != std::end(iterable);
    ++std::declval<decltype(std::begin(iterable))&>();
    *std::begin(iterable);
};

class CSVLogger
{
public:

    static inline std::unique_ptr<CSVLogger> create(std::string filename)
    {
        using namespace std::string_literals;

        std::filesystem::path file {filename};

        if (!std::filesystem::create_directories(file)) {
            std::cerr << "logging file directories for \"" << file
                      << "\" could not be created" << std::endl;
                return nullptr;
        }

        std::fstream stream {file.c_str(), std::ios::out};

        if (!stream.is_open()) {
            std::cerr << "failed to open log file " << file << std::endl;
            return nullptr;
        }

        return std::unique_ptr<CSVLogger>(new CSVLogger(std::move(stream)));
    }

    inline ~CSVLogger()
    {
        m_stream.flush();
        m_stream.close();
    }

    template<typename... Args>
    void log(Args... args)
    {
        using namespace std::string_literals;

        if (!m_stream.is_open())
            throw std::runtime_error("failed to open file "s + m_stream);

        m_stream << (log_value(args) + ", "s + ...);
    }

private:

    template<typename T>
    std::ostream &log_value(std::ostream &out, const T &iterable)
        requires Iterable<T>
    {
        auto it = std::begin(iterable);

        if (it != std::end(iterable))
            m_stream << *it;

        for (; it != std::end(iterable); ++it)
            m_stream << "," << *it;

        return out;
    }

    template<typename T>
    std::ostream &log_value(std::ostream &out, const T &value) {
        m_stream << value;
        return out;
    }

    inline CSVLogger(std::fstream &&out)
        : m_stream(std::move(out))
    {}

    std::fstream m_stream;
};
