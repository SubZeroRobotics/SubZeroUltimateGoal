package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config

public class GoToPoint {
    public static double voltageCompensation = 0;
    public static double headingP = 1.5;
    public static double headingD = 0;
    public static  double translateP = .11;
    public static double translateD = 0;
    public boolean isDone;
    public double xError, yError, headingError, xError_V;
    public SampleMecanumDrive drive;
    private HardwareMap hardwareMap;
    private static final double TAU = 2*Math.PI;
    FtcDashboard dashboard;
    VoltageSensor voltageSensor;
    public PIDFController headingPIDF, translateXPIDF, translateYPIDF;

    public GoToPoint(HardwareMap hwMap, SampleMecanumDrive drive) {
        this.drive = drive;
        isDone = false;
        dashboard = FtcDashboard.getInstance();
        hardwareMap = hwMap;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        headingPIDF = new PIDFController(headingP, 0, headingD, 0);
        translateXPIDF = new PIDFController(translateP, 0, translateD, 0);
        translateYPIDF = new PIDFController(translateP, 0, translateD, 0);
    }

    public void reset() {
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
    }

    public void setPose(Pose2d pose){
        drive.setPoseEstimate(pose);
    }

    private void notDone() {
        isDone = false;
    }

    public void goToPoint(Pose2d targetPose, double targetSpeed) {
        isDone = false;

        while (!isDone) {
            drive.update();

            headingError = normalizeAngleRR(Math.toRadians(targetPose.getHeading()) - (drive.getPoseEstimate().getHeading()));
            xError = targetPose.getX() - drive.getPoseEstimate().getX();
            yError = targetPose.getY() - drive.getPoseEstimate().getY();

            headingError = normalizeAngleRR(Math.toRadians(targetPose.getHeading()) - (drive.getPoseEstimate().getHeading()));
            xError = targetPose.getX() - drive.getPoseEstimate().getX();
            yError = targetPose.getY() - drive.getPoseEstimate().getY();

            double headingPID =  headingPIDF.calculate(0, headingError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double xPID = translateXPIDF.calculate(0, xError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double yPID =   translateYPIDF.calculate(0, yError) + voltageCompensation * 12 / voltageSensor.getVoltage();

            headingPID = Range.clip(headingPID, -1 * targetSpeed, 1 * targetSpeed);
            xPID = Range.clip(xPID, -1 * targetSpeed, 1 * targetSpeed);
            yPID = Range.clip(yPID, -1 * targetSpeed, 1* targetSpeed);

            Vector2d fieldCentric = new Vector2d(xPID, yPID).rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(new Pose2d(fieldCentric.getX(), fieldCentric.getY(), headingPID));

            if (Math.abs(xError) < 0.5 && Math.abs(yError) < 0.5 && Math.abs(headingError) < Math.toRadians(1)) {
                isDone = true;
            }
        }
    }

    public void turnToAbsolute(double degrees, double turnSpeed){
        goToPoint(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseVelocity().getY(), degrees), turnSpeed);
    }

    private double normalizeAngleRR(double angle) {
        while (angle > Math.PI)
            angle -= TAU;
        while (angle < -Math.PI)
            angle += TAU;
        return angle;
    }


}