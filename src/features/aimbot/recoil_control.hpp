#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <core/math/vector.hpp>

namespace features::aimbot::detail {

class recoil_motion
{
public:
    using clock = std::chrono::steady_clock;

    struct packet
    {
        int dx{};
        int dy{};
        float degrees_per_count{};

        [[nodiscard]] explicit operator bool() const noexcept { return dx || dy; }
    };

    void reset() noexcept { *this = {}; }

    void discard_pending() noexcept
    {
        m_applied = m_sent;
        m_velocity = {};
        m_valid = false;
    }

    [[nodiscard]] bool update(foundation::vec2 desired, float dt,
        float response_ms, float smoothness) noexcept
    {
        if (!std::isfinite(desired.x) || !std::isfinite(desired.y)
            || std::abs(desired.x) > 180.0f || std::abs(desired.y) > 180.0f
            || !std::isfinite(dt) || dt <= 0.0f || dt > 0.1f
            || !std::isfinite(response_ms) || !std::isfinite(smoothness))
        {
            discard_pending();
            return false;
        }

        const auto smooth = std::clamp(smoothness, 0.0f, 100.0f) * 0.01f;
        const auto response = std::clamp(response_ms * std::lerp(1.0f, 1.35f, smooth),
            1.0f, 300.0f) * 0.001f;
        const auto omega = 2.0f / response;
        const auto decay = std::exp(-omega * dt);
        m_max_speed = std::lerp(360.0f, 80.0f, smooth);
        const auto step = [&](float current, float target, float& velocity)
        {
            const auto error = current - target;
            const auto term = velocity + omega * error;
            const auto next = target + (error + term * dt) * decay;
            velocity = std::clamp((velocity - omega * term * dt) * decay,
                -m_max_speed, m_max_speed);
            const auto bounded = current + std::clamp(next - current,
                -m_max_speed * dt, m_max_speed * dt);
            if ((target - current) * (target - bounded) <= 0.0f)
            {
                velocity = 0.0f;
                return target;
            }
            return bounded;
        };
        m_applied.x = step(m_applied.x, desired.x, m_velocity.x);
        m_applied.y = step(m_applied.y, desired.y, m_velocity.y);
        m_valid = true;
        return true;
    }

    [[nodiscard]] packet prepare(float degrees_per_count, clock::time_point now) const noexcept
    {
        if (!m_valid || !std::isfinite(degrees_per_count)
            || degrees_per_count < 0.0001f || degrees_per_count > 1.0f) return {};
        const auto elapsed = m_last_send == clock::time_point{}
            ? 0.004f : std::chrono::duration<float>(now - m_last_send).count();
        if (elapsed < 0.002f) return {};
        const auto limit = std::clamp(m_max_speed * std::min(elapsed, 0.020f)
            / degrees_per_count, 1.0f, 32767.0f);
        const auto quantize = [limit](float value)
        {
            return static_cast<int>(std::clamp(value, -limit, limit));
        };
        return {quantize((m_applied.y - m_sent.y) / degrees_per_count),
            quantize((m_sent.x - m_applied.x) / degrees_per_count), degrees_per_count};
    }

    void commit(const packet& movement, clock::time_point now) noexcept
    {
        if (!movement) return;
        m_sent.x -= static_cast<float>(movement.dy) * movement.degrees_per_count;
        m_sent.y += static_cast<float>(movement.dx) * movement.degrees_per_count;
        m_last_send = now;
    }

    [[nodiscard]] foundation::vec2 sent() const noexcept { return m_sent; }
    [[nodiscard]] foundation::vec2 applied() const noexcept { return m_applied; }

private:
    foundation::vec2 m_applied{};
    foundation::vec2 m_velocity{};
    foundation::vec2 m_sent{};
    clock::time_point m_last_send{};
    float m_max_speed{360.0f};
    bool m_valid{};
};

}
