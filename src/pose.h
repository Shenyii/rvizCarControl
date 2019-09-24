#ifndef POSE_H
#define POSE_H

#include <iostream>

class Pose
{
public:
    float x;
    float y;
    float ow;
    float oz;

    Pose()
    : x(0),y(0),ow(1),oz(0)
    {};

    Pose(float xt,float yt,float ozt,float owt)
    {
        x = xt;
        y = yt;
        ow = owt;
        oz = ozt;
    };
};

#endif
