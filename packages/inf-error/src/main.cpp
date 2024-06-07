#include <iostream>
#include <string>
#include <limits>
#include <cmath>

int main() {
    bool on_jetson = false;
    std::string system_type = on_jetson ? "jetson" : "x86";
    std::cout << "System: " << system_type << "\n\n";

    std::cout << "val: " << INFINITY << "\n";
    std::cout << "isfinite():\t" << std::isfinite(INFINITY) << "\n";
    std::cout << "isinf():\t" << std::isinf(INFINITY) << "\n";
    std::cout << "isnormal():\t" << std::isnormal(INFINITY) << "\n";

    std::cout << "____\n\n";

    std::cout << "val: " << NAN << "\n";
    std::cout << "isnormal():\t" << std::isnormal(NAN) << "\n";
    std::cout << "isnan():\t" << std::isnan(NAN) << "\n";
}