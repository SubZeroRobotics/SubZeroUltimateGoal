package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Linkage {
    HardwareMap hardwareMap;
    SimpleServo linkage;
    SimpleServo flicker;

    private  double flickerPush = 0;
    private  double flickerPull = 0;
    private  double linkageUp = 0;
    private  double linkageDown = 0;



    public Linkage(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        linkage = new SimpleServo(hardwareMap, "linkage");
        flicker = new SimpleServo(hardwareMap, "pusher");
    }

    private enum State {
        RAISING_LINKAGE(130),
        SHOT1(150),
        IDLE_1(175),
        SHOT_2(200),
        IDLE_2(225),
        SHOT_3(100),
        IDLE_3(100);

        public final long timeStamp;
        private State(long timeStamp) {
            this.timeStamp = timeStamp;
        }
    }

    //run whole subsystem
    private void run(double power){
        State state = State.RAISING_LINKAGE;
        long startTime = System.currentTimeMillis();
        while(true) {
            long elapsedTime = System.currentTimeMillis() - startTime;
            //get the current time that has been elapsed
            boolean progressState = (elapsedTime >= state.timeStamp);
            //check if the elapsed time is
            switch (state){
                case RAISING_LINKAGE:
                    if (progressState){
                        linkage.setPosition(linkageUp);
                        state = State.SHOT1;
                    }
                    break;
                case SHOT1:
                    if(progressState){
                        flicker.setPosition(flickerPush);
                        state = State.IDLE_1;
                    }
                    break;
                case SHOT_2:
                    if(progressState){
                        flicker.setPosition(flickerPush);
                        state = State.IDLE_2;
                    }
                    break;
                case SHOT_3:
                    if(progressState){
                        flicker.setPosition(flickerPush);
                        state = State.IDLE_3;
                    }
                    break;
                case IDLE_1:
                    if(progressState){
                        flicker.setPosition(flickerPull);
                        state = State.SHOT_2;
                    }
                    break;
                case IDLE_2:
                    if(progressState){
                        flicker.setPosition(flickerPull);
                        state = State.SHOT_3;
                    }
                    break;
                case IDLE_3:
                    if(progressState){
                        flicker.setPosition(flickerPull);
                    }
                    break;
            }
        }

    }





}
