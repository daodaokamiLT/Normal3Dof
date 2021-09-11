#pragma once

#include <iostream>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

typedef Eigen::Vector3f Vec3f;
typedef Eigen::Quaternionf Quaf;
typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Matrix<float, 3, 4> Mat34f;
typedef Eigen::Matrix4f Mat4f;

struct basic_obj_t{
    uint64_t timestamp;
    Vec3f data;
    basic_obj_t() = default;
    basic_obj_t(const uint64_t& t, const Vec3f& d): timestamp(t), data(d){}
};

typedef basic_obj_t accelecmeter_t;
typedef basic_obj_t gyroscope_t;
typedef basic_obj_t megnetometer_t;
