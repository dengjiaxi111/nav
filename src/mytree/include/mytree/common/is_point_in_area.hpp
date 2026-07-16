#ifndef __ISPOINTINAREA__
#define __ISPOINTINAREA__

#include<vector>

namespace myBT{
    // 判断点是否在多边形内
    bool isPointInArea(double x, double y, const std::vector<std::pair<double, double>>& polygon) {
        bool inside = false;
        int j = polygon.size() - 1;
        
        for (int i = 0; i < polygon.size(); i++) {
            if (((polygon[i].second > y) != (polygon[j].second > y)) &&
                (x < (polygon[j].first - polygon[i].first) * (y - polygon[i].second) / 
                    (polygon[j].second - polygon[i].second) + polygon[i].first)) {
                inside = !inside;
            }
            j = i;
        }
        return inside;
    }
}
#endif