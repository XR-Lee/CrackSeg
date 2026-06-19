#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_map>
#include <vector>

struct RGBScore {
    float r, g, b;
    float orientationScore;
    float distanceScore;
    float finalScore;

    RGBScore(float r, float g, float b, float orientationScore, float distanceScore, float finalScore)
        : r(r), g(g), b(b), orientationScore(orientationScore), distanceScore(distanceScore), finalScore(finalScore) {}
};

struct PointData {
    std::vector<RGBScore> rgbScores;
    std::vector<int> keyframeIndices; // 记录看到这个点的关键帧索引

    void addRGBScore(const RGBScore& score, int keyframeIndex) {
        rgbScores.push_back(score);
        keyframeIndices.push_back(keyframeIndex);
        //TODO: pairwise?
    }
};

struct RGBCloud {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudNoColor;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithSmoothedColor;
    std::unordered_map<int, PointData> pointDataMap; // 用于存储每个点的多个RGB值及其分数

    RGBCloud() {
        cloudNoColor.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudWithSmoothedColor.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    }

    void addPointData(int pointIndex, const RGBScore& score, int keyframeIndex) {
        pointDataMap[pointIndex].addRGBScore(score, keyframeIndex);
    }

    PointData getPointData(int pointIndex) const {
        auto it = pointDataMap.find(pointIndex);
        if (it != pointDataMap.end()) {
            return it->second;
        } else {
            return PointData(); // 返回空的PointData
        }
    }
};
