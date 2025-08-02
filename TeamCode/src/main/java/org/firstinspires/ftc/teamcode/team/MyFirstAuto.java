package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MyFirstAuto", group = "State")
public class MyFirstAuto extends DarienOpModeAuto {

    private MotorHelper tiltMotorHelper;
    private MotorHelper slideMotorHelper;
    private double tilt_gain = 1;
    private double T2 = 1000;
    private float tiltSetPoint = 0;
    private float slideSetPoint = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        initControls();
        waitForStart();

        double[] tiltMotorPID = {0, 0, 0};
        double[] slideMotorPID = {0, 0, 0};
        tiltMotorHelper = new MotorHelper(telemetry);
        slideMotorHelper = new MotorHelper(telemetry);

        while (this.opModeIsActive()) {
            // Goal: Drive from wall to pickup sample from the floor and score it in the low basket.

            // Drive from wall to 3rd spike mark
            updatePosition();
            claw.setPosition(OPEN_CLAW);
            moveToPosition(0, 12, .2);
            waitForMotors();
            claw.setPosition(CLOSED_CLAW);
            if (imageProcess.getLastResult()) {
                telemetry.addData("The block is red", "");
            } else {
                telemetry.addData("The block is yellow", "");
            }
            telemetry.update();
            sleep(250);
            autoRotate(135, .5);
            sleep(100);
            tiltSetPoint = 956;
            slideSetPoint = 3416;
            
            tiltMotorPID = tiltMotorHelper.pid(
                    tiltMotor,
                    tilt_pgain,
                    tilt_igain,
                    tiltMotorPID[2], -.5, 0.8, -0.2, 1, tiltMotorPID[1], -.5, .25,
                    tilt_gain,
                    T1,
                    T2,
                    tiltSetPoint,
                    true
            );
            tiltMotor.setPower(tiltMotorPID[0]);
            slideMotorPID = slideMotorHelper.pid(
                    slideMotor1,
                    slide_pgain,
                    slide_igain,
                    slideMotorPID[2], -.7, .6, -.7, .6, slideMotorPID[1], -1, 1,
                    slide_gain,
                    S1,
                    S2,
                    slideSetPoint,
                    false
            );
            slideMotor1.setPower(slideMotorPID[0]);
        }
    }
}