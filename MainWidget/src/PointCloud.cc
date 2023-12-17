#include "PointCloud.h"

PointCloud::PointCloud(const std::string& name):mNameID(name){}

void PointCloud::setShowStatus(const bool status)
{
    this->mStatus = status;
}
const bool PointCloud::getShowStatus() const
{
    return this->mStatus;
}

PointCloud::~PointCloud()
{
}
