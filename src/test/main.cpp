#include <iostream>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <optional>
#include <vector>

#include "test/test.hpp"

/**
 * @brief Parse command line arguments.
 * 
 * @param argc The number of command line arguments.
 * @param argv The command line arguments to parse.
 * 
 * @returns A tuple containing if the parse succeeded, the parsed flags, and the
 * parsed keyword arguments.
 */
std::tuple<
    bool,
    std::unordered_set<char>,
    std::unordered_map<std::string, std::vector<std::string>>
>
parse(int argc, char **argv)
{
    std::unordered_set<char> flags;
    std::unordered_map<std::string, std::vector<std::string>> args;

    int state = 0;
    std::string key;

    for (int i = 0; i < argc; i++) {
        char *arg = argv[i];

        for (char *c = arg; c && *c;) {
            switch (state)
            {
                case 0: {
                    // Expect all arguments to begin with a dash.
                    if (*c != '-')
                        return std::make_tuple(false, flags, args);

                    c++;
                    state = 1;
                    continue;
                }

                case 1: {
                    // If there is a second dash, interpret as a key.
                    if (*c == '-') {
                        c++;
                        state = 2;
                        continue;
                    }

                    // Otherwise all subsequent characters are flags.
                    for (; *c; c++)
                        flags.emplace(*c);

                    state = 0;
                    break;
                }

                case 2: {
                    // Get the key, expect a value.
                    key = std::string(c);
                    state = 3;
                    c = nullptr;
                    break;
                }

                case 3: {
                    auto value = std::string(c);

                    // Add the value to the vector for this key.
                    auto it = args.find(key);
                    if (it == args.end())
                        args[key] = {value};
                    else
                        it->second.push_back(value);

                    state = 0;
                    c = nullptr;
                    break;
                }
            }
        }
    }

    return std::make_tuple(true, flags, args);
}

int main(int argc, char **argv)
{
    // Ignore executable name.
    argc -= 1;
    argv += 1;

    auto [success, flags, args] = parse(argc, argv);
    if (!success) {
        std::cerr << "invalid command line arguments" << std::endl;
        return 1;
    }

    auto it = args.find("test");
    if (it == args.end())
        return (int)TestSuite::run();
    else
        return (int)TestSuite::run(it->second);
}
