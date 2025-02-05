#include "NN.h"
#include "agent.h"

// Define the DQN agent
    DQNAgent::DQNAgent(Agent* agent) : net(create_q_network(agent)), optimizer() {
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
            argos::LOG << "State size: " << states[i].size() << " and target size: " << q_values.size() << std::endl;
        }
//        net.fit<mse>(optimizer, states, targets, 1, 1);
    }

/**
 * Applying convolutional (conv) and max pooling (max_pool) layers multiple times in a neural network helps to progressively extract more complex and abstract features from the input data. Here's why:
 * First Convolutional Layer:
 * The first conv layer captures basic features such as edges, textures, and simple patterns.
 * The first max_pool layer reduces the spatial dimensions, making the network more computationally efficient and providing some translation invariance.
 * Second Convolutional Layer:
 * The second conv layer builds on the features extracted by the first layer, capturing more complex patterns and structures.
 * The second max_pool layer further reduces the spatial dimensions and helps in summarizing the features extracted by the second conv layer.
 * By stacking multiple conv and max_pool layers, the network can learn hierarchical feature representations, which are crucial for tasks like image recognition and classification.
 * @return
 */
    network<sequential> DQNAgent::create_q_network(Agent* agent) {
    network<sequential> net;

    //Calculate width based on agent search radius and quadtree resolution
    const int input_width = std::min(2 * agent->config.FRONTIER_SEARCH_RADIUS / agent->quadtree->getResolution(), agent->quadtree->getRootBox().size / agent->quadtree->getResolution());
    const int input_height = std::min(2 * agent->config.FRONTIER_SEARCH_RADIUS / agent->quadtree->getResolution(), agent->quadtree->getRootBox().size / agent->quadtree->getResolution());
    argos::LOG << "Input width: " << input_width << " and height: " << input_height << std::endl;

//    const int input_width = 64; // Width of the input image
//    const int input_height = 64; // Height of the input image
    const int input_channels = 1; // Number of input channels (e.g., 1 for grayscale)

    //Increase: Adding more filters can help the network capture more detailed and diverse features, potentially improving performance on complex tasks.
    //Decrease: Reducing the number of filters can decrease the computational cost and memory usage, which might be necessary for resource-constrained environments.
    const int conv1_filters = 16; // Number of filters in the first convolutional layer

    // Increase: Larger kernels can capture more context and larger patterns in the input data, which might be beneficial for tasks requiring a broader view.
    // Decrease: Smaller kernels focus on finer details and can help in capturing small and local patterns, which might be useful for tasks requiring high precision.
    const int conv1_kernel_size = 3; // Kernel size for the first convolutional layer

    //Increase:
    //Reduces Spatial Dimensions: Larger pooling sizes reduce the spatial dimensions more aggressively, which can lead to faster computations and reduced memory usage.
    //Summarizes Features: Helps in summarizing the features, making the network more robust to small translations and distortions in the input data.
    //Decrease:
    //Preserves Spatial Information: Smaller pooling sizes preserve more spatial information, which can be beneficial for tasks requiring high spatial resolution.
    //Finer Details: Helps in capturing finer details and small patterns in the input data, which might be useful for tasks requiring high precision.
    const int pool1_size = 2; // Pooling size for the first max pooling layer

    //Increase:
    //Capacity: More units can increase the model's capacity to learn complex patterns and representations, potentially improving performance on complex tasks.
    //Feature Extraction: Helps in capturing more detailed and diverse features from the previous layers.
    //Decrease:
    //Overfitting: Fewer units can help reduce the risk of overfitting, especially if the dataset is small or noisy.
    //Computational Efficiency: Reducing the number of units can decrease the computational cost and memory usage, which might be necessary for resource-constrained environments.
    const int conv2_filters = 32; // Number of filters in the second convolutional layer
    const int conv2_kernel_size = 3; // Kernel size for the second convolutional layer
    const int pool2_size = 2; // Pooling size for the second max pooling layer

    //Capacity:
    //Increase: More units can increase the model's capacity to learn complex patterns and representations, potentially improving performance on complex tasks.
    //Decrease: Fewer units can reduce the model's capacity, which might be beneficial for simpler tasks or to prevent overfitting.
    //Feature Extraction:
    //Increase: Helps in capturing more detailed and diverse features from the previous layers.
    //Decrease: May limit the ability to capture detailed features, which might be sufficient for simpler tasks.
    //Overfitting:
    //Increase: More units can increase the risk of overfitting, especially if the dataset is small or noisy.
    //Decrease: Fewer units can help reduce the risk of overfitting.
    //Computational Efficiency:
    //Increase: More units can increase the computational cost and memory usage.
    //Decrease: Fewer units can decrease the computational cost and memory usage, which might be necessary for resource-constrained environments.
    const int fc1_units = 128; // Number of units in the first fully connected layer
    const int fc2_units = 64; // Number of units in the second fully connected layer
    const int output_units = 4; // Number of output units (e.g., number of actions)

    const int non_spatial_data_size = 4 + 2 + 1; // Proximity (4) + Position (2) + Orientation (1)


//    net << conv(input_width, input_height, conv1_kernel_size, input_channels, conv1_filters) << relu() // First convolutional layer with ReLU activation
//        << max_pool(input_width - conv1_kernel_size + 1, input_height - conv1_kernel_size + 1, conv1_filters, pool1_size) // First max pooling layer
//        << conv((input_width - conv1_kernel_size + 1) / pool1_size, (input_height - conv1_kernel_size + 1) / pool1_size, conv2_kernel_size, conv1_filters, conv2_filters) << relu() // Second convolutional layer with ReLU activation
//        << max_pool((input_width - conv1_kernel_size + 1) / pool1_size - conv2_kernel_size + 1, (input_height - conv1_kernel_size + 1) / pool1_size - conv2_kernel_size + 1, conv2_filters, pool2_size);// Second max pooling layer
//
//    net << fc(14 * 14 * conv2_filters, fc1_units) << relu(); // First fully connected layer with ReLU activation
////        << fc(((input_width - conv1_kernel_size + 1) / pool1_size - conv2_kernel_size + 1) / pool2_size * ((input_height - conv1_kernel_size + 1) / pool1_size - conv2_kernel_size + 1) / pool2_size * conv2_filters, fc1_units + non_spatial_data_size) << relu() // First fully connected layer with ReLU activation
//        //        << fc(fc1_units, output_units); // Output layer
//        // Add non-spatial data (proximity, position, orientation) to the state
//    net << fc(fc1_units + non_spatial_data_size, fc2_units) << relu(); // Second fully connected layer
//
//        // Output layer
//    net << fc(fc2_units, output_units);



// Convolutional layers for processing the local map
//    net << conv(input_width, input_height, conv1_kernel_size, input_channels, conv1_filters) << relu() // First convolutional layer
//        << max_pool(input_width - conv1_kernel_size + 1, input_height - conv1_kernel_size + 1, conv1_filters, pool1_size) // First max pooling layer
//        << conv((input_width - conv1_kernel_size + 1) / pool1_size, (input_height - conv1_kernel_size + 1) / pool1_size, conv2_kernel_size, conv1_filters, conv2_filters) << relu() // Second convolutional layer
//        << max_pool((input_width - conv1_kernel_size + 1) / pool1_size - conv2_kernel_size + 1, (input_height - conv1_kernel_size + 1) / pool1_size - conv2_kernel_size + 1, conv2_filters, pool2_size); // Second max pooling layer
//
//    // Flatten the output of the convolutional layers
//    const int conv_output_size = 14 * 14 * conv2_filters; // Flattened size of the convolutional output
//    net << fc(conv_output_size, fc1_units) << relu(); // First fully connected layer
//
//    // Combine the convolutional output with the non-spatial data
//    net << concat(fc1_units, non_spatial_data_size); // Concatenate spatial and non-spatial data
//
//    // Second fully connected layer
//    net << fc(fc1_units + non_spatial_data_size, fc2_units) << relu();
//
//    // Output layer
//    net << fc(fc2_units, output_units);

// Convolutional layers for spatial data (64x64 map)
    net << conv(64, 64, 3, 1, 32) << relu()
        << max_pool(62, 62, 32, 2)
        << conv(31, 31, 3, 32, 64) << relu()
        << max_pool(29, 29, 64, 2)
        << fc(14 * 14 * 64, 256) << relu();

// Fully connected layers for non-spatial data
    net << fc(7, 64) << relu();

// Merge spatial and non-spatial branches
    net << concat(256, 64)
        << fc(320, 128) << relu()
        << fc(128, 64) << relu()
        << fc(64, 4);



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
