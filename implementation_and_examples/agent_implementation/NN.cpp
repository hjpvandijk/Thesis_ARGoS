#include "NN.h"
// Define the DQN agent
    DQNAgent::DQNAgent() : net(create_q_network()), optimizer() {
        net.weight_init(weight_init::xavier());
        net.bias_init(weight_init::xavier());
        net.init_weight();
    }

    int DQNAgent::get_action(const vec_t& state) {
        vec_t q_values = net.predict(state);
        return std::distance(q_values.begin(), std::max_element(q_values.begin(), q_values.end()));
    }

    void DQNAgent::train(const std::vector<vec_t>& states, const std::vector<int>& actions,
               const std::vector<float>& rewards, const std::vector<vec_t>& next_states) {
        std::vector<vec_t> targets;
        for (size_t i = 0; i < states.size(); ++i) {
            vec_t q_values = net.predict(states[i]);
            float target_q = rewards[i];
            if (!next_states[i].empty()) {
                vec_t next_q_values = net.predict(next_states[i]);
                target_q += gamma * *std::max_element(next_q_values.begin(), next_q_values.end());
            }
            q_values[actions[i]] = target_q;
            targets.push_back(q_values);
        }
        net.fit<mse>(optimizer, states, targets, 1, 1);
    }

    network<sequential> DQNAgent::create_q_network() {
        network<sequential> net;
        net << conv(10, 10, 3, 1, 16) << relu()
            << max_pool(8, 8, 16, 2)
            << conv(4, 4, 3, 16, 32) << relu()
            << max_pool(2, 2, 32, 2)
            << fc(1 * 1 * 32, 128) << relu()
            << fc(128, 4);
        return net;
    }

    void ReplayBuffer::add(const Experience& experience) {
        if (buffer.size() >= capacity) {
            buffer.pop_front();
        }
        buffer.push_back(experience);
    }

    std::vector<ReplayBuffer::Experience> ReplayBuffer::sample(size_t batch_size) {
        std::vector<Experience> batch;
        std::sample(buffer.begin(), buffer.end(), std::back_inserter(batch),
                    batch_size, std::mt19937{std::random_device{}()});
        return batch;
    }
