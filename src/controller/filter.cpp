#include "controller/filter.hpp"

#include <iostream>
#include <algorithm>

std::ostream& operator<<(std::ostream &os, const MovingExtendedWindow &w) {
    os << "\nuu: [";
    for (size_t i = 0; i < w.uu.size(); i++) {
        os << w.uu[i] << " ";
    }
    os << "]\ntt: [";
    for (size_t i = 0; i < w.uu.size(); i++) {
        os << w.tt[i] << " ";
    }
    os << "]" << std::endl;
    return os;
}

MovingExtendedWindow::MovingExtendedWindow(const int size, const int w)
    : window(w)
    , last_trim_t(-1)
    , start_idx(window)
{
    // the first and last input need half horizon in the past and in the future
    // in order to be centered leaving with a total size equal to weigths size
    // (2 * window) + 1 + horizon/2
    // + horizon/2 = 2 * window + 1 + horizon
    uu.resize(size + 2 * window + 1, 0);

    // initialization to -1 makes sure everything starts correctly with a
    // positive initial time
    tt.resize(size + 2 * window + 1, -1);
}

void MovingExtendedWindow::trim(const double t)
{
    if (t < last_trim_t) {
        std::stringstream ss;
        ss << "Resetting the window back in the past. Can reset only to larger "
                "times than last "
                "reset!!!"
            << "last reset=" << last_trim_t << ", trying to reset to t=" << t;
        throw std::runtime_error(ss.str());
    }

    last_trim_t = t;

    // search in the last inserted times the closest smaller than the current
    size_t trim_idx = start_idx;
    for (size_t i = 0; i < start_idx; i++) {
        if (tt[i] >= t) {
            trim_idx = i;
            break;
        }
    }

    size_t offset = trim_idx - window;

    std::rotate(tt.begin(), tt.begin() + offset, tt.end());
    std::rotate(uu.begin(), uu.begin() + offset, uu.end());

    // extend the trimmed portion with the last elements in the vector
    if (offset > 0) {
        std::fill(tt.end() - offset, tt.end(), *(tt.end() - offset - 1));
        std::fill(uu.end() - offset, uu.end(), *(uu.end() - offset - 1));
    }

    start_idx = window;
    tt[start_idx] = t;
}

void MovingExtendedWindow::add_point(const double u, const double t) {
    if (t < tt[start_idx]) {
        std::stringstream ss;
        ss << std::setprecision(4)
            << "Adding measurement older then new time: " << t << " < "
            << tt[start_idx] << std::endl;
        ss << "start_idx: " << start_idx << std::endl;
        ss << "last trim time: " << last_trim_t << std::endl;
        ss << "window: " << *this << std::endl;
        throw std::runtime_error(ss.str());
    }

    assert(start_idx < uu.size());
    uu[start_idx] = u;
    tt[start_idx] = t;

    extend();
    start_idx++;
}

std::vector<double> MovingExtendedWindow::extract(const double t)
{
    auto lower = std::lower_bound(tt.begin(), tt.end(), t);  // index to the first element larger than t
    assert(lower != tt.end());
    size_t idx = std::distance(tt.begin(), lower);

    return std::vector<double>(
        uu.begin() + idx - window,
        uu.begin() + idx + window + 1
    );
}

void MovingExtendedWindow::set(const double u, const double t)
{
    auto lower = std::lower_bound(tt.begin(), tt.end(), t);  // index to the first element larger than t
    assert(lower != tt.end());
    size_t idx = std::distance(tt.begin(), lower) - 1;
    uu[idx] = u;
}

void MovingExtendedWindow::extend()
{
    std::fill(uu.begin() + start_idx + 1, uu.end(), uu[start_idx]);
    std::fill(tt.begin() + start_idx + 1, tt.end(), tt[start_idx]);
}

SavitzkyGolayFilter::SavitzkyGolayFilter(
    const int steps,
    const int nu,
    const int window,
    const unsigned int poly_order,
    const unsigned int der_order,
    const double time_step
) {
    m_filters.resize(nu, gram_sg::SavitzkyGolayFilter(window, 0, poly_order, der_order));
    m_windows.resize(nu, MovingExtendedWindow(steps, window));
};

SavitzkyGolayFilter::SavitzkyGolayFilter(
    const int steps,
    const int nu,
    const std::vector<int>& window,
    const std::vector<unsigned int>& poly_order,
    const unsigned int der_order,
    const double time_step
) {
    if (window.size() == 1) {
        m_filters.resize(nu, gram_sg::SavitzkyGolayFilter(window[0], 0, poly_order[0], der_order));
        m_windows.resize(nu, MovingExtendedWindow(steps, window[0]));
        return;
    }

    for (int i = 0; i < nu; i++) {
        m_filters.emplace_back(window[i], 0, poly_order[i], der_order);
        m_windows.emplace_back(steps, window[i]);
    }
};

void SavitzkyGolayFilter::reset(const double t)
{
    for (auto& w : m_windows) {
        w.trim(t);
    }
}

void SavitzkyGolayFilter::add_measurement(const Eigen::VectorXd& u, const double t)
{
    assert(u.size() == m_windows.size());
    for (long int i = 0; i < u.size(); i++) {
        m_windows[i].add_point(u(i), t);
    }
}

void SavitzkyGolayFilter::apply(
    Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<>> u,
    const double t
) {
    for (long int i = 0; i < u.size(); i++) {
        u[i] = m_filters[i].filter(m_windows[i].extract(t));
        m_windows[i].set(u[i], t); // update the window with the filtered value
    }
}
