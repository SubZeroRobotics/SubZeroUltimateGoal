package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class NGamepad {
    public Gamepad gamepad;
    Boolean aPressed = true;
    public boolean previous = false, value = false, toggle = false;
    boolean bool;
    public NGamepad(Gamepad gamepad){
        this.gamepad = gamepad;
    }
}