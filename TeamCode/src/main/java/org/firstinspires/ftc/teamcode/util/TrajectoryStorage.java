package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class TrajectoryStorage {
   static Pose2d pose;

    public TrajectoryStorage(Pose2d pose){
        this.pose = pose;
    }

    public TrajectoryStorage(){

    }



    public static Pose2d getPose(){
        return pose;
    }


}
