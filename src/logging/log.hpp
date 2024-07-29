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

class CSVLogger
{
public:

    struct Configuration {

        // Folder to store logs in.
        std::string file;
    };

    inline std::unique_ptr<CSVLogger> create(Configuration &&configuration)
    {
        using namespace std::string_literals;

        std::filesystem::path file {configuration.file};

        if (!std::filesystem::create_directories(file)) {
            std::cerr << "logging file directories for \"" << file
                      << "\" could not be created" << std::endl;
                return nullptr;
        }

        std::fstream stream {file.c_str(), std::ios::out};

        if (!m_stream.is_open()) {
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

        m_stream << (std::to_string(args) + ", "s + ...);
    }

private:

    inline CSVLogger(std::fstream &&out)
        : m_stream(std::move(out))
    {}

    std::fstream m_stream;
};
