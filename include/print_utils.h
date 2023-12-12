#include <array>
#include <iostream>

// json import
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "data_types.h"

// operator for array printing
namespace {
    template <class T, size_t N>
    std::ostream &operator<<(std::ostream &ostream, const std::array<T, N> &array) {
        ostream << "[";
        std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
        std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
        ostream << "]";
        return ostream;
    }
}

void print_data(json parameter, bool &running, Data2Print &data_print, double &time_local);