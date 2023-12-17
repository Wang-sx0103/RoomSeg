#pragma once

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloud
{
private:
    std::string mNameID;
    bool mStatus;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr mpCloudXYZ;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr mpCloudXYZRGBI;
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr mpCloudXYZRGB;
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr mpCloudXYZRGBA;
public:
    explicit PointCloud(const std::string&);
    void setShowStatus(const bool);
    const bool getShowStatus() const;
    ~PointCloud();
};
