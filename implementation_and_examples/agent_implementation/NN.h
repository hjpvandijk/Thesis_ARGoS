#include <tiny_dnn/tiny_dnn.h>
#include <vector>
#include <iostream>
#include <random>
#include <algorithm>
#include <deque>
#include <memory>
#include <unordered_map>

using namespace tiny_dnn;
using namespace tiny_dnn::activation;
using namespace tiny_dnn::layers;
// Define the DQN agent
class DQNAgent {
public:
    DQNAgent();

    int get_action(const vec_t& state);

    void train(const std::vector<vec_t>& states, const std::vector<int>& actions,
               const std::vector<float>& rewards, const std::vector<vec_t>& next_states);

private:
    network<sequential> create_q_network();

    network<sequential> net;
    adam optimizer;
    float gamma = 0.99;
};

// Replay buffer to store experiences
class ReplayBuffer {
public:
    struct Experience {
        vec_t state;
        int action;
        float reward;
        vec_t next_state;
    };

    void add(const Experience& experience);

    std::vector<Experience> sample(size_t batch_size);

private:
    std::deque<Experience> buffer;
    size_t capacity = 10000;
};