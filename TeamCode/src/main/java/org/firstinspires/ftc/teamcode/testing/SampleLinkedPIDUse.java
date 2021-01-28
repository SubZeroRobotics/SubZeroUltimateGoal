package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
@Config
@TeleOp
public class SampleLinkedPIDUse extends LinearOpMode {
    // Copy your PID Coefficients here
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(.002, 0, 0);

    public static double RPM = 0;

    public static double flapPosition = .38;

    // Copy your feedforward gains here
    public static double kV = .00051;
    public static double kA = .000255;
    public static double kStatic = 0;

    // Timer for calculating desired acceleration
    // Necessary for kA to have an affect
    private final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;
    public ElapsedTime elapsedTime = new ElapsedTime();
    public ElapsedTime elapsedTime2 = new ElapsedTime();
    public ElapsedTime elapsedTime3 = new ElapsedTime();
    public ElapsedTime flapTimer = new ElapsedTime();

    Linkage linkage;


    // Our velocity controller
    private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    @Override
    public void runOpMode() throws InterruptedException {
        // SETUP MOTORS //
        // Change my id
        DcMotorEx myMotor1 = hardwareMap.get(DcMotorEx.class, "sm1");
        DcMotorEx myMotor2 = hardwareMap.get(DcMotorEx.class, "sm2");
        Shooter shooter = new Shooter(hardwareMap);
        linkage = new Linkage(hardwareMap,shooter, elapsedTime, elapsedTime2, .7,.38,.3,.52 );
        Servo angleFlap = hardwareMap.get(Servo.class, "flap");
        Servo flicker = hardwareMap.get(Servo.class, "pusher");



        // Reverse as appropriate
        // myMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        // myMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure that RUN_USING_ENCODER is not set
        myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Insert whatever other initialization stuff you do here
        ElapsedTime timer = new ElapsedTime();
        boolean servoMoving = false;
        timer.reset();

        waitForStart();


        if (isStopRequested()) return;

        // Start the veloTimer
        veloTimer.reset();

        while (!isStopRequested()) {
            ///// Run the velocity controller ////

            // Target velocity in ticks per second

            angleFlap.setPosition(flapPosition);


            if (gamepad1.y && timer.milliseconds() >= 150 && !servoMoving) {
                flicker.setPosition(.3);
                servoMoving = true;
                timer.reset();
            }

            if (timer.milliseconds() >= 150 && servoMoving) {
                flicker.setPosition(.52);
                servoMoving = false;
                timer.reset();
            }




            double targetVelo = RPM * 28.0/ 1.5 / 60.0;

            // Call necessary controller methods
            veloController.setTargetVelocity(targetVelo);
            veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
            veloTimer.reset();

            lastTargetVelo = targetVelo;

            // Get the velocity from the motor with the encoder
            double motorPos = myMotor1.getCurrentPosition();
            double motorVelo = myMotor1.getVelocity();

            // Update the controller and set the power for each motor
            double power = veloController.update(motorPos, motorVelo);
            myMotor1.setPower(power);
            myMotor2.setPower(power);

            // Do your opmode stuff
        }
    }
}