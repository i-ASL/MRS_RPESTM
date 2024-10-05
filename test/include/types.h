#include <stdint.h>
#include <eigen3/Eigen/Dense>


namespace kf
{
    using float32_t = float;

    template<size_t ROW, size_t COL>
    using Matrix = Eigen::Matrix<float32_t, ROW, COL>;

    template<size_t ROW>
    using Vector = Eigen::Matrix<float32_t, ROW, 1>;
}