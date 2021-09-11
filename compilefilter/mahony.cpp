#include "mahony.h"

void MahonyCompfilter::processInGyroDatas(const gyroscope_t &obj){
    std::lock_guard<std::mutex> locker(gyro_mtx_);
    if(hasinited_){
        if(gyrodatas_.size() == gyroquesize_){
            gyrodatas_.pop_front();
        }
        gyrodatas_.push_back(obj);
        gyro_avgfilter_.Push(obj);
        Vec3f result = gyro_avgfilter_.GetFiltedData();
        if (!result.hasNaN())
        {
            if (avgfilted_gyrodatas_.size() == gyroquesize_)
            {
                avgfilted_gyrodatas_.pop_front();
            }
            avgfilted_gyrodatas_.push_back(gyroscope_t(obj.timestamp, result));
        }
    }
}

void MahonyCompfilter::processInAccDatas(const accelecmeter_t &obj){
    std::lock_guard<std::mutex> locker(acc_mtx_);
    if(!hasinited_){
        float accnorm = obj.data.norm();
        if(accnorm > 7.5 && accnorm < 11.5){
            // 取当前一小段的平稳的数据，avgfilter之后，进行初始化
            accdatas_.push_back(obj);
            acc_avgfilter_.Push(obj);
            Vec3f avgfiltedresult = acc_avgfilter_.GetFiltedData();
            if(!avgfiltedresult.hasNaN()){
                if(avgfilted_accdatas_.size() == accquesize_){
                    // select a min closed to 9.8 element. to init
                    float mindis = 1000;
                    int minindex = -1;
                    
                    for(int i=0; i<avgfilted_accdatas_.size(); ++i){
                       float dis = std::fabs(avgfilted_accdatas_[i].data.norm() - 9.8);
                       if(mindis < dis){
                           mindis = dis;
                           minindex = i;
                       } 
                    }
                    InitRotation(avgfilted_accdatas_[minindex].data, qvec_wi_);
                    hasinited_ = true;
                    acc_avgfilter_.clear();
                }
                avgfilted_accdatas_.push_back(accelecmeter_t(obj.timestamp, avgfiltedresult));
            }
        }
        else{
            accdatas_.clear();
            acc_avgfilter_.clear();
        }
        return;
    }
    else{
        if(accdatas_.size() == accquesize_){
            accdatas_.pop_front();
        }
        accdatas_.push_back(obj);
        acc_avgfilter_.Push(obj);
        Vec3f result = acc_avgfilter_.GetFiltedData();
        if(!result.hasNaN()){
            if(avgfilted_accdatas_.size() == accquesize_){
                avgfilted_accdatas_.pop_front();
            }
            avgfilted_accdatas_.push_back(accelecmeter_t(obj.timestamp, result));
        }
    }

}

Quaf MahonyCompfilter::getCompfilterResult(const uint64_t &timestamp){
    return qvec_wi_;
}


void MahonyCompfilter::run(){
    while(!isClosed_){
        std::lock_guard<std::mutex> lockgyro(gyro_mtx_);
        if(avgfilted_gyrodatas_.size()){
            
        }
        std::lock_guard<std::mutex> lockacc(acc_mtx_);    
    }
}