#pragma once

#include <std_msgs/msg/color_rgba.hpp>

namespace Manhattan::viz::color {

static inline std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0f)
{
    std_msgs::msg::ColorRGBA color;

    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    return color;
}

const std_msgs::msg::ColorRGBA white = color(1.0f, 1.0f, 1.0f);

const std_msgs::msg::ColorRGBA red = color(1.0f, 0.0f, 0.0f);

const std_msgs::msg::ColorRGBA green = color(0.0f, 1.0f, 0.0f);

const std_msgs::msg::ColorRGBA blue = color(0.0f, 0.0f, 1.0f);

const std_msgs::msg::ColorRGBA cyan = color(0.0f, 1.0f, 1.0f);

}
