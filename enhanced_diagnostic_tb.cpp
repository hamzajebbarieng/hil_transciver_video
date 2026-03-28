// enhanced_diagnostic_tb.cpp

#include <iostream>
#include <vector>

// Module 1: Input Signal
void displayInputSignal(const std::vector<int>& inputSignal) {
    std::cout << "Input Signal: ";
    for (const auto& sample : inputSignal) {
        std::cout << sample << " ";
    }
    std::cout << std::endl;
}

// Module 2: COFDM Modulation
void displayModulatedSignal(const std::vector<int>& modulatedSignal) {
    std::cout << "Modulated Signal: ";
    for (const auto& sample : modulatedSignal) {
        std::cout << sample << " ";
    }
    std::cout << std::endl;
}

// Module 3: Channel Simulation
void displayReceivedSignal(const std::vector<int>& receivedSignal) {
    std::cout << "Received Signal: ";
    for (const auto& sample : receivedSignal) {
        std::cout << sample << " ";
    }
    std::cout << std::endl;
}

// Module 4: COFDM Demodulation
void displayDemodulatedSignal(const std::vector<int>& demodulatedSignal) {
    std::cout << "Demodulated Signal: ";
    for (const auto& sample : demodulatedSignal) {
        std::cout << sample << " ";
    }
    std::cout << std::endl;
}

int main() {
    // Simulated signals for demonstration
    std::vector<int> inputSignal = {1, 0, 1, 1, 0};
    std::vector<int> modulatedSignal = {0, 1, 0, 1, 1};
    std::vector<int> receivedSignal = {1, 0, 0, 1, 0};
    std::vector<int> demodulatedSignal = {1, 0, 1, 0, 1};

    // Display diagnostics for each stage of the pipeline
    displayInputSignal(inputSignal);
    displayModulatedSignal(modulatedSignal);
    displayReceivedSignal(receivedSignal);
    displayDemodulatedSignal(demodulatedSignal);

    return 0;
}