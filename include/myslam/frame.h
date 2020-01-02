/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam 
{
    
// forward declare 
class MapPoint;//前向引用申明，告诉编译器先不管MapPoint有没有对象
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;//定义指向本类的智能指针
    unsigned long                  id_;//用于记录Frame的id索引号
    double                         time_stamp_; //当前Frame的时间戳
    SE3                            T_c_w_;      //从世界到相机的变换矩阵
    Camera::Ptr                    camera_;     //指向Camera类的对象的指针
    Mat                            color_, depth_; //RGB图片和深度图片
    // std::vector<cv::KeyPoint>      keypoints_;  //图像中的关键点
    // std::vector<MapPoint*>         map_points_; //关键点对应的地图点
    bool                           is_key_frame_;  //当前Frame是否是关键帧
    
public: // data members 
    Frame();
    Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
    ~Frame();
    
    static Frame::Ptr createFrame(); 
    
    // find the depth in depth map
    double findDepth( const cv::KeyPoint& kp );
    
    // Get Camera Center
    Vector3d getCamCenter() const;
    
    void setPose( const SE3& T_c_w );
    
    // check if a point is in this frame 
    bool isInFrame( const Vector3d& pt_world );
};

}

#endif // FRAME_H
