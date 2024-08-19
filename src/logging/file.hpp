#pragma once

#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>

namespace logger {

/**
 * @brief A file logging class.
 */
class File
{
public:

    /**
     * @brief Create a new file logger.
     * 
     * @param path The path to the file.
     * 
     * @return A pointer to the file logger or nullptr on failure. 
     */
    static inline std::unique_ptr<File> create(std::filesystem::path path)
    {
        using namespace std::string_literals;

        // Create the parent directories of the file if they do not exist.
        if (!std::filesystem::exists(path.parent_path())) {
            std::error_code code;
            bool created_directory = std::filesystem::create_directories(
                path.parent_path(),
                code
            );

            if (!created_directory) {
                std::cerr << "failed to create log file " << path
                        << ". " << code.message() << std::endl;
                return nullptr;
            }
        }

        // Open the CSV file.
        std::fstream stream {path, std::ios::out};

        if (!stream.is_open()) {
            std::cerr << "failed to open log file "<< path << std::endl;
            return nullptr;
        }

        return std::unique_ptr<File>(new File(std::move(stream)));
    }

    inline std::fstream &get_stream() {
        return m_stream;
    }

    template<typename T>
    inline void write(T &&value) {
        m_stream << value;
    }

    template<typename T>
    inline std::fstream &operator<<(T &&value)
    {
        m_stream << value;
        return m_stream;
    }

    inline ~File()
    {
        m_stream.flush();
        m_stream.close();
    }

private:

    /**
     * @brief Initialise the file logger.
     * @param out The open file stream to write to.
     */
    inline File(std::fstream &&out)
        : m_stream(std::move(out))
    {}

    /// The file to write to.
    std::fstream m_stream;
};

} // namespace logger
