#pragma once

#include <cstddef>
#include <initializer_list>
#include <utility>
#include <vector>

namespace pros {

class Distance {
public:
    explicit Distance(int port) : port(port) {}

    Distance(std::initializer_list<int> distances)
        : distances(distances) {}

    Distance(std::vector<int> distances, std::vector<int> confidences = {})
        : distances(std::move(distances)),
          confidences(std::move(confidences)) {}

    int get() {
        if (distances.empty()) {
            return 0;
        }

        const int value = distances[distanceIndex % distances.size()];
        ++distanceIndex;
        return value;
    }

    int get_confidence() {
        if (confidences.empty()) {
            return 100;
        }

        const int value = confidences[confidenceIndex % confidences.size()];
        ++confidenceIndex;
        return value;
    }

    int port = 0;

private:
    std::vector<int> distances;
    std::vector<int> confidences;
    std::size_t distanceIndex = 0;
    std::size_t confidenceIndex = 0;
};

}  // namespace pros
