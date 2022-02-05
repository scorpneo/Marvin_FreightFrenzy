package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MyPose2d
{
    public double x;
    public double y;
    public double heading;
    public MyPose2d(double x, double y, double heading)
    { this.x = x; this.y = y; this.heading = heading;}

    Pose2d getPose2d() { return new Pose2d(x, y, Math.toRadians(heading));}
};