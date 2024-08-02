#include <cstdlib>
#include <csignal>
#include <iostream>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <optional>

class Args
{
public:

    static inline std::optional<Args> parse(int argc, char **argv)
    {
        // Ignore program executable.
        argv += 1;
        argc -= 1;

        Args args;
        if (args.init(argc, argv))
            return args;
        return std::nullopt;
    }

    inline bool get_flag(char flag) {
        m_flags.contains(flag);
    }

    inline const std::unordered_set<char> &get_flags() {
        return m_flags;
    }

    inline std::optional<std::string> get_arg(const std::string &arg)
    {
        if (!m_args.contains(arg))
            return m_args[arg];
        return std::nullopt;
    }

    inline const std::unordered_map<std::string, std::string> &get_args() {
        return m_args;
    }

private:

    enum State {
        INIT,
        FLAG,
        KEY,
        VALUE,
        BAD,
    };

    Args()
        : m_state(INIT)
    {}

    bool init(int argc, char **argv)
    {
        for (int i = 0; i < argc; i++) {
            char *arg = argv[i];

            for (char *c = arg; *c; ++c) {
                switch (m_state)
                {
                    case INIT:  handle_init(c); continue;
                    case FLAG:  handle_flag(c); continue;
                    case KEY:   handle_key(c); break;
                    case VALUE: handle_value(c); break;
                    case BAD:   return false;
                }
            }
        }
    }

    inline void handle_init(char *c)
    {
        if (*c != '-') {
            m_state = BAD;
        }
        else {
            m_state = FLAG;
        }
    }

    inline void handle_flag(char *c)
    {
        if (*c == '-') {
            m_state = KEY;
            return;
        }

        for (char *flag = c; *flag; ++flag)
            m_flags.emplace(*flag);

        m_state = INIT;
    }

    inline void handle_key(char *c)
    {
        m_key = std::string(c);
        m_state = VALUE;
    }

    inline void handle_value(char *c)
    {
        m_args.emplace(std::make_pair(m_key, std::string(c)));
        m_state = INIT;
    }

    State m_state;

    std::string m_key;

    std::unordered_set<char> m_flags;

    std::unordered_map<std::string, std::string> m_args;
};

void interrupt_handler(int signal)
{

}

int main(int argc, char **argv)
{
    std::signal(SIGINT, interrupt_handler);

    auto args = Args::parse(argc, argv);
    if (!args) {
        std::cerr << "invalid command line arguments" << std::endl;
        return 1;
    }

    for (auto x : args->get_flags())
        std::cout << "flag " << x << std::endl;

    for (auto [x, y] : args->get_args())
        std::cout << "key value " << x << ":" << y << std::endl;

    return 0;
}
