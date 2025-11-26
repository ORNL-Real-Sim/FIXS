#pragma once

#include <random>
#include <unordered_set>

template<typename T>
T random_select_from_set(const std::unordered_set<T>& set) {
    if (set.empty()) throw std::runtime_error("Set is empty!");

    // Random number generation
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, set.size() - 1);

    int index = dis(gen);
    auto it = set.begin();
    std::advance(it, index);  // move iterator forward by index
    return *it;
}