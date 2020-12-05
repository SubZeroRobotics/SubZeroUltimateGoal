package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Linkage {
    HardwareMap hardwareMap;
    Servo linkage;
    Servo flicker;

    public  double flickerPush;
    public  double flickerPull;
    private final double linkageUp;
    private final double linkageDown;

    private long millis;



    public Linkage(HardwareMap hardwareMap,double up, double down, double in, double out){
        this.hardwareMap = hardwareMap;
        flicker = hardwareMap.get(Servo.class, "pusher");
        linkage = hardwareMap.get(Servo.class, "linkage");
        linkageUp = up;
        linkageDown = down;
        flickerPush = in;
        flickerPull = out;
        this.millis = millis;
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

    public void raise(){
        linkage.setPosition(linkageUp);
    }

    public void lower(){
        linkage.setPosition(linkageDown);
    }

    public void flickerIn(){
        flicker.setPosition(flickerPush);
    }
    public void flickerOut(){
        flicker.setPosition(flickerPull);
    }




    //run whole subsystem
    public void run(){
        State state = State.SHOT1;
        long startTime = System.currentTimeMillis();
        while(true) {
            long elapsedTime = System.currentTimeMillis() - startTime;
            //get the current time that has been elapsed
            boolean progressState = (elapsedTime >= state.timeStamp);
            //check if the elapsed time is
            switch (state){
                case RAISING_LINKAGE:
                    if (progressState){
                       // linkage.setPosition(linkageUp);
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
