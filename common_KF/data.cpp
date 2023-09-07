#include "matplotlibcpp.h"
#include <cmath>

namespace plt = matplotlibcpp;

int main()
{
    // Prepare data.
    int n = 100;
    std::vector<double> x(n), y(n), z(n), w(n,2);
    for(int i=0; i<n; ++i) {
        x.at(i) = i;
        y.at(i) = 3*x.at(i);
        z.at(i) = 10*x.at(i) ;
    }
    plt::figure_size(1200, 780);
    plt::scatter(x, y) ;

    plt::plot(x, z,"r");
    plt::title("Sample figure");
    // plt::legend();
    plt::save("./basic.png");
}
