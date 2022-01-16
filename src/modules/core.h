#pragma once

#include "module.h"
#include <memory>
#include <utility>

struct output_element_t {
    const Module_ptr module;
    const std::string property_name;
    const unsigned int precision;
};

class Core;
using Core_ptr = std::shared_ptr<Core>;

class Core : public Module {
private:
    std::list<struct output_element_t> output_list;

public:
    Core(const std::string name);
    void step();
    void call(const std::string method_name, const std::vector<Expression_ptr> arguments);
    double get(const std::string property_name) const;
    void set(std::string property_name, double value);
    std::string get_output() const;
};