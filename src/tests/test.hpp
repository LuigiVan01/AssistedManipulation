#pragma once

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>

using namespace std::string_literals;

class Logger
{
public:

    template<typename... Args>
    void log(Args... args) = 0;
};

class CSVLogger
{
public:

    inline CSVLogger(std::filesystem::path filename)
        : m_stream(filename.c_str(), std::ios::out)
    {
        if (!m_stream.is_open())
            throw std::runtime_error("failed to open file "s + filename);
    }

    inline ~CSVLogger()
    {
        m_stream.flush();
        m_stream.close();
    }

    template<typename... Args>
    void log(Args... args)
    {
        if (!m_stream.is_open())
            throw std::runtime_error("failed to open file "s + filename);
        m_stream << (std::to_string(args) + ", "s + ...);
        m_stream.flush();
    }

private:

    std::fstream m_stream;
};

// class RosLogger

template<typename Base>
class Test : public Base
{
public:

    Test(const std::string &name, )

private:

};
