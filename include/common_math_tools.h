#pragma once 

#include <deque>
#include "../include/data_basic.h"

struct avgfilter{
    std::deque<basic_obj_t> vec_deque;
    Vec3f sum;
    Vec3f avg;
    int size;   
    
    avgfilter(const int sz = 8){
        size = sz;
        sum = Vec3f::Zero();
        avg = Vec3f::Zero();
    }
    avgfilter() = delete;

    void clear(){
        vec_deque.clear();
        sum = Vec3f::Zero();
        avg = Vec3f::Zero();
    }

    Vec3f GetFiltedData(){
        if(vec_deque.size() < size){
            return Vec3f::Constant(std::nan(""));
        }
        else{
            return avg;
        }
    }

    void Push(const basic_obj_t& obj){
       if(vec_deque.size() >= size){
            Vec3f data = vec_deque.front().data;
            vec_deque.pop_front();
            sum -= data;
        }
        vec_deque.push_back(obj);
        sum += obj.data;
        avg = sum / (1.0f*vec_deque.size());
    }
};

static const Vec3f kNGravityVec = Vec3f(0, 0, 9.8);

inline bool InitRotation(const Vec3f& acc, Quaf& q) {
  double accn = acc.norm();
  if (accn > 11.5 || accn < 7.5) {
    printf("acc norm is not in the init range.\n");
    return false;
  }
  Vec3f acc0 = acc.normalized();
  q = Quaf::FromTwoVectors(acc0, kNGravityVec);
  return true;
}

inline bool HorizontalLookat_fx(const Mat3f& R_ori, const Mat3f& R_wi, Mat3f& R_change){
  Mat3f R = Mat3f::Identity();
  if(std::fabs(R_wi(0,0)) == 1){
    // printf("R_wi is %f.\n", R_wi(0,0));
    if(R_wi(0,0) > 0){
      // 即 R_ori（imu） 的 x 指向 w 的 x 方向
      Vec3f vec_x = R_ori.block<3,1>(0,0);
      Vec3f vec_x0 = vec_x;

      vec_x0[2] = 0;
      vec_x0.normalize();
      Vec3f vec_x1(1,0,0);
      float theta = std::acos(vec_x0.dot(vec_x1));
      if(vec_x0[1] > 0){
        theta *= -1;
      }
      else if(vec_x0[1] < 0){
        theta *= 1;
      }
      else{
        theta = 0;
      }
      // 旋转 w坐标系，使得在x的朝向是一样的。
      R = Mat3f::Zero();
      R(2,2) = 1;
      R(0,0) = cos(theta);
      R(0,1) = -sin(theta);
      R(1,0) = sin(theta);
      R(1,1) = cos(theta);
    }else{
      Vec3f vec_x = R_ori.block<3,1>(0,0);
      Vec3f vec_x0 = vec_x;

      vec_x0[2] = 0;
      vec_x0.normalize();
      Vec3f vec_x1(-1,0,0);
      float theta = std::acos(vec_x0.dot(vec_x1));
      if(vec_x0[1] > 0){
        theta *= 1;
      }
      else if(vec_x0[1] < 0){
        theta *= -1;
      }
      else{
        theta = 0;
      }
      // 旋转 w坐标系，使得在x的朝向是一样的。
      R = Mat3f::Zero();
      R(2,2) = 1;
      R(0,0) = cos(theta);
      R(0,1) = -sin(theta);
      R(1,0) = sin(theta);
      R(1,1) = cos(theta);
    }
  }
  else if(std::fabs(R_wi(0,1)) == 1){
    if(R_wi(0,1) > 0){
      // 即 R_ori（imu） 的 y 指向 w 的 x
       // 即 R_ori（imu） 的 x 指向 w 的 x 方向
      Vec3f vec_y = R_ori.block<3,1>(0,1);
      Vec3f vec_y0 = vec_y;

      vec_y0[2] = 0;
      vec_y0.normalize();
      Vec3f vec_y1(1,0,0);
      float theta = std::acos(vec_y0.dot(vec_y1));
      if(vec_y0[1] > 0){
        theta *= -1;
      }
      else if(vec_y0[1] < 0){
        theta *= 1;
      }
      else{
        theta = 0;
      }
      // 旋转 w坐标系，使得在x的朝向是一样的。
      R = Mat3f::Zero();
      R(2,2) = 1;
      R(0,0) = cos(theta);
      R(0,1) = -sin(theta);
      R(1,0) = sin(theta);
      R(1,1) = cos(theta);
    }
    else{
      // 即 R_ori（imu） 的 y 指向 w 的 -x
      Vec3f vec_y = R_ori.block<3,1>(0,1);
      Vec3f vec_y0 = vec_y;

      vec_y0[2] = 0;
      vec_y0.normalize();
      Vec3f vec_y1(-1,0,0);
      float theta = std::acos(vec_y0.dot(vec_y1));
      if(vec_y0[1] > 0){
        theta *= 1;
      }
      else if(vec_y0[1] < 0){
        theta *= -1;
      }
      else{
        theta = 0;
      }
      // 旋转 w坐标系，使得在x的朝向是一样的。
      R = Mat3f::Zero();
      R(2,2) = 1;
      R(0,0) = cos(theta);
      R(0,1) = -sin(theta);
      R(1,0) = sin(theta);
      R(1,1) = cos(theta);
    }
  }
  else if(std::fabs(R_wi(0,2)) == 1){
    if(R_wi(0,2) > 0){
      // 即 R_ori（imu） 的 z 指向 w 的 x
      Vec3f vec_z = R_ori.block<3,1>(0,2);
      Vec3f vec_z0 = vec_z;

      vec_z0[2] = 0;
      vec_z0.normalize();
      Vec3f vec_z1(1,0,0);
      float theta = std::acos(vec_z0.dot(vec_z1));
      if(vec_z0[1] > 0){
        theta *= -1;
      }
      else if(vec_z0[1] < 0){
        theta *= 1;
      }
      else{
        theta = 0;
      }
      // 旋转 w坐标系，使得在x的朝向是一样的。
      R = Mat3f::Zero();
      R(2,2) = 1;
      R(0,0) = cos(theta);
      R(0,1) = -sin(theta);
      R(1,0) = sin(theta);
      R(1,1) = cos(theta);
    }
    else{
      // 即 R_ori（imu） 的 z 指向 w 的 -x
      Vec3f vec_z = R_ori.block<3,1>(0,2);
      Vec3f vec_z0 = vec_z;

      vec_z0[2] = 0;
      vec_z0.normalize();
      Vec3f vec_z1(-1,0,0);
      float theta = std::acos(vec_z0.dot(vec_z1));
      if(vec_z0[1] > 0){
        theta *= 1;
      }
      else if(vec_z0[1] < 0){
        theta *= -1;
      }
      else{
        theta = 0;
      }
      // 旋转 w坐标系，使得在x的朝向是一样的。
      R = Mat3f::Zero();
      R(2,2) = 1;
      R(0,0) = cos(theta);
      R(0,1) = -sin(theta);
      R(1,0) = sin(theta);
      R(1,1) = cos(theta);
    }
  }
  R_change = R.transpose() * R_ori;
  return true;
}