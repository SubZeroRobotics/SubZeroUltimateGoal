package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    //motor members
    private static DcMotorEx motor1, motor2;
    private static ServoEx linkage;
    private static ServoEx indexer;
    private static HardwareMap hw;

    //PID constants
    private static double kp = 0;
    private static double  ki = 0;
    private static double kd = 0;
    private static double kf = 0;

    //servo position variables
    private static double indexerPush = 0;
    private static double indexerRetract = 0;
    private static double linkageUp = 0;
    private static double linkageDown = 0;


    //other constants
    private static double COUNTS_PER_REV = 7.0;
    private static double ROTATIONS_PER_MINUTE = 5800.0;

    //pid and elapsed time
    private static PIDFController pidf = new PIDFController(kp,ki,kd,kf);
    private static ElapsedTime linkageTimer = new ElapsedTime();
    private static ElapsedTime pusherTimer = new ElapsedTime();
    private static ElapsedTime shooterTimer = new ElapsedTime();



    public Shooter(HardwareMap hwmp){
        this.hw = hwmp;
        linkage =  hw.get(ServoEx.class, "linkage");
        indexer = hw.get(ServoEx.class, "indexer");
        motor1 = hw.get(DcMotorEx.class, "motor1");
        motor2 = hw.get(DcMotorEx.class, "motor2");
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    private enum State {
        POWERING_SHOOTER(100),
        RAISING_LINKAGE(100),
        SHOT1(100),
        IDLE_1(100),
        SHOT_2(100),
        IDLE_2(100),
        SHOT_3(100),
        IDLE_3(100);

        public final long timeStamp;
        private State(long timeStamp) {
            this.timeStamp = timeStamp;
        }
    }


    private void run(double power){
        State state = State.POWERING_SHOOTER;
        //shooterpower on


        /*
        Progress state is the time you are giving the previous action to actuate before calling the next one.
         */

        
        long startTime = System.currentTimeMillis();
        while(true) {
            long elapsedTime = System.currentTimeMillis() - startTime;
            //get the current time that has been elapsed
            boolean progressState = (elapsedTime >= state.timeStamp);
            //check if the elapsed time is
            switch (state){
                case POWERING_SHOOTER:
                    if(progressState){
                        setPower(power);
                        state = State.RAISING_LINKAGE;
                    }
                case RAISING_LINKAGE:
                    if (progressState){
                        linkage.setPosition(linkageUp);
                        state = State.SHOT1;
                    }
                case SHOT1:
                    if(progressState){
                        indexer.setPosition(indexerPush);
                        state = State.IDLE_1;
                    }
                case SHOT_2:
                    if(progressState){
                        indexer.setPosition(indexerPush);
                        state = State.IDLE_2;
                    }
                case SHOT_3:
                    if(progressState){
                        indexer.setPosition(indexerPush);
                        state = State.SHOT_3;
                    }
                case IDLE_1:
                    if(progressState){
                        indexer.setPosition(indexerRetract);
                        state = State.SHOT_2;
                    }
                case IDLE_2:
                    if(progressState){
                        indexer.setPosition(indexerRetract);
                        state = State.SHOT_3;
                    }
                case IDLE_3:
                    if(progressState){
                        indexer.setPosition(indexerRetract);
                        state = State.SHOT_3;
                    }
            }
        }

    }
    //stop motor lol
    public static void stopFlyWheel(){ motor1.setPower(0.0);  motor2.setPower(0.0); }

    //sets power to motor using PIDF controller
    private static void setPower(double power){
        power = power * ((ROTATIONS_PER_MINUTE / 60) * COUNTS_PER_REV);
        pidf.setSetPoint(power);
        double output = pidf.calculate(motor1.getVelocity());
        motor1.setVelocity(output);
        motor2.setVelocity(output);
    }

}
