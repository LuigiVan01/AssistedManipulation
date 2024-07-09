#pragma once

#include <vector>

#include "controller/gram_savitzky_golay/gram_savitzky_golay.h"

#include <deque>
#include <iomanip>
#include <iostream>
#include <vector>
#include <map>
#include <iostream>

struct MovingExtendedWindow {

    MovingExtendedWindow(const int size, const int w);

    void trim(const double t);

    void add_point(const double u, const double t);

    void set(const double u, const double t);

    /**
     * Extend the window until the end with the latest received measurement
     */
    void extend();

    /**
     * Extract a window centered at the query time
     * @param t: query time
     * @return
     */
    std::vector<double> extract(const double t);

    int window;
    double last_trim_t;
    size_t start_idx;
    std::vector<double> uu;
    std::vector<double> tt;
};

class SavitzkyGolayFilter
{
public:

    SavitzkyGolayFilter() = default;

    SavitzkyGolayFilter(
        const int steps,
        const int nu,
        const int window,
        const unsigned int poly_order,
        const unsigned int der_order = 0,
        const double time_step = 1.
    );

    /**
     Filter with custom filtering per input channel
    **/
    SavitzkyGolayFilter(
        const int steps,
        const int nu,
        const std::vector<int>& window,
        const std::vector<unsigned int>& poly_order,
        const unsigned int der_order = 0,
        const double time_step = 1.
    );

    ~SavitzkyGolayFilter() = default;

    void reset(const double t);

    void add_measurement(
        const Eigen::VectorXd& u,
        const double t
    );

    void apply(
        Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<> > u,
        const double t
    );

    inline std::vector<MovingExtendedWindow>& get_windows() {
        return m_windows;
    }

private:

    std::vector<MovingExtendedWindow> m_windows;

    std::vector<gram_sg::SavitzkyGolayFilter> m_filters;
};
