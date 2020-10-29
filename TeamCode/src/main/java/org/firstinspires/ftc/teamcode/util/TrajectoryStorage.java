package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class TrajectoryStorage {
    Trajectory trajectory;

    public TrajectoryStorage(Trajectory trajectory){
        this.trajectory = trajectory;

    }

    public Pose2d end(){
        return trajectory.end();
    }
}
