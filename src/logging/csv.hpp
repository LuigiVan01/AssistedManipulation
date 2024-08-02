#pragma once

#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>

namespace logger {

/**
 * @brief A concept that checks if a data structure is iterable.
 */
template<typename T>
concept Iterable = requires(T iterable)
{
    std::begin(iterable) != std::end(iterable);
    ++std::declval<decltype(std::begin(iterable))&>();
    *std::begin(iterable);
};

/**
 * @brief A CSV logging class.
 */
class CSV
{
public:

    struct Configuration {

        /// The file to log to.
        std::filesystem::path path;

        /// The csv column headers.
        std::vector<std::string> header;
    };

    /**
     * @brief Create a new CSV logger.
     * 
     * @param configuration 
     * @return std::unique_ptr<CSV> 
     */
    static inline std::unique_ptr<CSV> create(
        const Configuration &configuration
    ) {
        using namespace std::string_literals;

        // Create the parent directories of the file if they do not exist.
        if (!std::filesystem::exists(configuration.path.parent_path())) {
            std::error_code code;
            bool created_directory = std::filesystem::create_directories(
                configuration.path.parent_path(),
                code
            );

            if (!created_directory) {
                std::cerr << "failed to create csv log file " << configuration.path
                        << ". " << code.message() << std::endl;
                return nullptr;
            }
        }

        // Open the CSV file.
        std::fstream stream {configuration.path, std::ios::out};

        if (!stream.is_open()) {
            std::cerr << "failed to open log file "
                      << configuration.path << std::endl;
            return nullptr;
        }

        // Output the csv header.
        if (!configuration.header.empty()) {
            stream << configuration.header[0];
            for (int i = 1; i < configuration.header.size(); i++)
                stream << ", " << configuration.header[i];
            stream << '\n';
        }

        return std::unique_ptr<CSV>(new CSV(std::move(stream)));
    }

    inline ~CSV()
    {
        m_stream.flush();
        m_stream.close();
    }

    /**
     * @brief Write a row the CSV file.
     * 
     * This function generically handles writing variables to a CSV file. If the
     * variable is iterable, then the csv file will iterate over each element
     * and log it separately.
     * 
     * @param arg The first argument passed to write.
     * @param args Optional other arguments to write to the file.
     */
    template<typename Arg, typename... Args>
    void write(Arg &&arg, Args&&... args)
    {
        using namespace std::string_literals;

        if (!m_stream.is_open())
            throw std::runtime_error("logging to unopen csv file.");

        // Log the first value.
        write_value(std::forward<Arg&&>(arg));

        // Log comma separated values.
        ((m_stream << ", ", write_value(std::forward<Args&&>(args))), ...);

        // End with newline. End of newline at end of file.
        m_stream << '\n';
        m_stream.flush();
    }

private:

    /**
     * @brief Initialise the CSV file writer.
     * @param out The open file stream to write to.
     */
    inline CSV(std::fstream &&out)
        : m_stream(std::move(out))
    {}

    /**
     * @brief Write an iterable to the file, comma separated.
     * 
     * @tparam T The type of iterable to write.
     * @param iterable The iterable to write.
     */
    template<typename T>
    void write_value(const T &iterable) requires Iterable<T>
    {
        auto it = std::begin(iterable);

        if (it != std::end(iterable))
            m_stream << *it;

        // Comma separation after the first element.
        for (++it; it != std::end(iterable); ++it)
            m_stream << ", " << *it;
    }

    /**
     * @brief Write a value to the CSV file.
     * 
     * @tparam T The type of the value.
     * @param value The value to write.
     */
    template<typename T>
    void write_value(const T &value) {
        m_stream << value;
    }

    /// The file to write to.
    std::fstream m_stream;
};

} // namespace logger
