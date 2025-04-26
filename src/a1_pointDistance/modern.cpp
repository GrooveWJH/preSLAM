#include <__math/exponential_functions.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <numeric>
#include <stdexcept>
#include <vector>

double distance_modern(const std::vector<double>& p1, const std::vector<double>& p2)
{
    if (p1.size() != p2.size())
        throw std::invalid_argument("Points must have the same dimension.");

    if (p1.empty())
        return 0;

    std::vector<double> diff_sq(p1.size());

    std::transform(
        p1.begin(),
        p1.end(),
        p2.begin(),
        std::back_inserter(diff_sq),
        [](double val1, double val2) {
            double diff = val1 - val2;
            return diff * diff;
        });

    return std::sqrt(std::accumulate(diff_sq.begin(), diff_sq.end(),
        0.0));
}

int main()
{
    std::vector<double> p_a = { 1.0, 2.0 },
                        p_b = { 4.0, 6.0 };
    std::vector<double> p_c = { 1.0, 2.0, 3.0 },
                        p_d = { 4.0, 5.0, 6.0 };
    std::vector<double> p_e = { 1.0, 2.0, 3.0, 4.0 },
                        p_f = { 5.0, 6.0, 7.0, 8.0 };

    try {
        double dist2D = distance_modern(p_a, p_b);
        std::cout << "Modern Distance between p_a and p_b (2D): " << dist2D << std::endl; // 5
        double dist3D = distance_modern(p_c, p_d);
        std::cout << "Modern Distance between p_c and p_d (3D): " << dist3D << std::endl; // 5.19615
        double dist4D = distance_modern(p_e, p_f);
        std::cout << "Modern Distance between p_e and p_f (4D): " << dist4D << std::endl; // 8
        // 尝试不同维度的点会抛出异常
        distance_modern(p_a, p_c);
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}
