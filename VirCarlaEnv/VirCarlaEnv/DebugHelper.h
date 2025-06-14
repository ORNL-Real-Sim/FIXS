#pragma once
#include <carla/client/DebugHelper.h>
#include <carla/geom/Location.h>

#define M_PI 3.14159265358979323846f
void drawCircle(carla::client::DebugHelper debug,
    const carla::geom::Location& center,
    float radius = 50.0f,
    int segments = 64,
    float z_offset = 0.5f,
    float thickness = 0.1f,
    float life_time = 0.1f,  // 0 = forever
    bool persistent = false,
    bool fill = false);