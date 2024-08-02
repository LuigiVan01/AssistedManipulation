#pragma once

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>

/**
 * @brief The base test class an 
 * 
 * @tparam Base 
 */
template<typename Base>
class Test : public Base
{
public:

    Test(const std::string &name);

private:

    inline static std::unique_ptr<Test> make()

    using Construct = std::function<Base()>;

    std::unordered_map<std::string, C
};
