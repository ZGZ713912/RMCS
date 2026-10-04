#pragma once

#include <map>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include <rmcs_executor/component.hpp>

namespace rmcs_executor {

// Test-only pairing of explicitly selected plugins; no hardware or executor thread.
class Executor {
public:
    static std::vector<Component*> with_partners(std::span<Component*> roots) {
        std::vector<Component*> components;
        const auto add = [&](auto&& self, Component* component) -> void {
            components.push_back(component);
            for (auto& partner : component->partner_component_list_)
                self(self, partner.get());
        };
        for (auto* root : roots)
            add(add, root);
        return components;
    }

    static std::vector<Component*> ordered(std::span<Component*> components) {
        pair(components);
        std::map<std::string, Component*> owners;
        for (auto* component : components)
            for (auto& output : component->output_list_)
                owners.emplace(output.name, component);
        std::map<Component*, int> mark;
        std::vector<Component*> order;
        const auto visit = [&](auto&& self, Component* component) -> void {
            if (mark[component] == 2)
                return;
            if (mark[component] == 1)
                throw std::runtime_error("cyclic test component graph");
            mark[component] = 1;
            for (auto& input : component->input_list_)
                if (owners.contains(input.name))
                    self(self, owners.at(input.name));
            mark[component] = 2;
            order.push_back(component);
        };
        for (auto* component : components)
            visit(visit, component);
        return order;
    }

    static void pair(std::span<Component*> components) {
        std::map<std::string, Component::OutputDeclaration*> outputs;
        Component::OutputInfoMap info;
        for (auto* component : components)
            for (auto& output : component->output_list_) {
                if (!outputs.emplace(output.name, &output).second)
                    throw std::runtime_error("duplicate test output " + output.name);
                info.emplace(output.name, Component::OutputInfo{output.type, output.kind});
            }
        for (auto* component : components)
            component->before_pairing(info);
        for (auto* component : components)
            for (auto& input : component->input_list_) {
                const auto output = outputs.find(input.name);
                if (output == outputs.end() && !input.required)
                    continue;
                if (output == outputs.end() || input.type != output->second->type)
                    throw std::runtime_error("unpaired test input " + input.name);
                input.bind(input.binding, output->second->binding);
            }
    }
};

} // namespace rmcs_executor
