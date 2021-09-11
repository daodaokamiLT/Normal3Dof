#pragma once

#include "../include/common_math_tools.h"
#include "../include/data_basic.h"
#include <thread>
#include <mutex>
#include <atomic>
class MahonyCompfilter{
    private:
    // update params
        float kpw_ = 0.5;
        float kpi_ = 0.0;
        
        Vec3f gyro_bias_ = Vec3f::Zero();
        Vec3f acc_bias_ = Vec3f::Zero();
        Vec3f interfback_ = Vec3f::Zero();

        Quaf qvec_wi_ = Quaf::Identity();

        const int gyroquesize_, accquesize_;
        std::deque<gyroscope_t> gyrodatas_;
        std::deque<accelecmeter_t> accdatas_;

        std::deque<gyroscope_t> avgfilted_gyrodatas_;
        std::deque<accelecmeter_t> avgfilted_accdatas_;

        bool hasinited_ = false;

        avgfilter acc_avgfilter_;
        avgfilter gyro_avgfilter_;

        std::mutex gyro_mtx_;
        std::mutex acc_mtx_;

        std::atomic<bool> isClosed_;
    public:
        // if this is two thread deal the problem, can it sure the data is ok?
        MahonyCompfilter(const int& gyroquesize = 16, const int& accquesize = 16): gyroquesize_(gyroquesize), accquesize_(accquesize), 
                acc_avgfilter_(accquesize/2), gyro_avgfilter_(gyroquesize/2), isClosed_(false){}
        MahonyCompfilter() = delete;

        void processInGyroDatas(const gyroscope_t& datas);
        void processInAccDatas(const accelecmeter_t& datas);

        Quaf getCompfilterResult(const uint64_t& timestamp);

        bool hasInited(){return hasinited_;}

        // 这个函数必须托管到其他线程中，如何进行绑定。
        void run();

        void close(){isClosed_ = true;}
};