#include <ras_utils/ras_utils.h>
#include <vector>

namespace RAS_Utils
{

    int sign(double a)
    {
        if (a >= 0) return 1;
        else        return -1;
    }

    void print(std::string text) {
        ROS_INFO(text.c_str());
    }

    void print(std::string text, float value) {
        ROS_INFO((text + ": %f").c_str(), value);
    }

    void print(std::string text, float value1, float value2) {
        ROS_INFO((text + ": %f | %f").c_str(), value1, value2);
    }

    void print(const std::vector<String> & texts, const std::vector<float> & values) {
        if(names.size() != values.size()) {
            print("!!! ERROR !!! Vectors in print function have different sizes");
            return;
        }
        for(int i = 0; i < names.size(); i++) {
            print(texts[i], values[i]);
        }
    }
}
