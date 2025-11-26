#include "DebugHelper.h"


void drawCircle(carla::client::DebugHelper debug,
    const carla::geom::Location& center,
    float radius,
    int segments,
    float z_offset,
    float thickness,
    float life_time,  // 0 = forever
    bool persistent,
    bool fill) {

    using carla::geom::Location;
    using carla::rpc::Color;
    carla::geom::Location adjusted_center(center.x, center.y, 0.0f);
    adjusted_center = adjusted_center + carla::geom::Location{ 0.0f, 0.0f, z_offset };

    // Adjust center Z level

    for (int i = 0; i < segments; ++i) {
        float angle1 = (2 * M_PI * i) / segments;
        float angle2 = (2 * M_PI * (i + 1)) / segments;

        Location p1 = adjusted_center + Location{
            radius * std::cos(angle1),
            radius * std::sin(angle1),
            0.0f
        };
        Location p2 = adjusted_center + Location{
            radius * std::cos(angle2),
            radius * std::sin(angle2),
            0.0f
        };

        // Perimeter circle
        debug.DrawLine(p1, p2, thickness, carla::client::DebugHelper::Color(255, 0, 0), life_time, persistent);

        if (fill) {
            // Fill with "triangle fans" (radial lines)
            debug.DrawLine(adjusted_center, p1, thickness * 0.5f, carla::client::DebugHelper::Color(255, 100, 100), life_time, persistent);
        }
    }
}