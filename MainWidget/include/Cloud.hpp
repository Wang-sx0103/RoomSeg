#pragma once

#include <string>
#include <memory>
#include <pcl/point_cloud.h>

//class CloudBase
//{
//public:
//    explicit CloudBase(const std::string& name): mNameID(name), mStatus(true) {}
//    const std::string getName() const
//    {
//        return this->mNameID;
//    }
//
//    void setShowStatus(const bool status)
//    {
//        this->mStatus = status;
//    }
//    const bool getShowStatus() const
//    {
//        return this->mStatus;
//    }
//    virtual void setCloudPtr() const = 0;
//    virtual void getCloudPtr() const = 0;
//    virtual ~CloudBase() {}
//
//private:
//    std::string mNameID;
//    bool mStatus;
//};

template<typename T>
class Cloud
{
public:
    Cloud(const std::string& name): mNameID(name), mStatus(true)/*CloudBase(name)*/{}

    const std::string getName() const
    {
        return this->mNameID;
    }

    void setShowStatus(const bool status)
    {
        this->mStatus = status;
    }
    const bool getShowStatus() const
    {
        return this->mStatus;
    }

    void setCloudPtr(T& cloud)
    {
        this->mpCloud = cloud;
    }

    T getCloudPtr() const
    {
        return this->mpCloud;
    }

    ~Cloud()
    {
    }

private:
    std::string mNameID;
    bool mStatus;
    T mpCloud;
};
