package org.firstinspires.ftc.teamcode.team.autosPedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
@Autonomous(name = "BlueAudienceSidePedro", group = "Pedro:Blues", preselectTeleOp = "TeleopFSM")
@Configurable
public class BlueAudience1 extends DarienOpModeFSM {
    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    public Follower follower;                   // Pedro Pathing follower instance
    private int pathState;                      // State machine state
    private Paths paths;                        // Paths
    private Timer pathTimer, opmodeTimer;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- ROBOT + HARDWARE INIT (from DarienOpModeFSM) ---
        initControls(); // sets up TrayServo, Elevator, Feeder, motors, AprilTag, etc.

        // --- PEDRO + TIMERS INIT ---
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Starting pose – same as your OpMode version
        follower.setStartingPose(new Pose(20.286, 124.378, Math.toRadians(54)));

        // Build all the paths once
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        telemetry.addLine("BlueGoalSidePedro: READY");
        telemetry.update();

        // --- WAIT FOR START ---
        waitForStart();
        if (isStopRequested()) return;

        opmodeTimer.resetTimer();
        setPathState(0);

        // Set the initial tray position immediately.
        setTrayPosition(TRAY_POS_1_SCORE);

        // --- MAIN AUTONOMOUS LOOP ---
        while (opModeIsActive() && !isStopRequested()) {

            // Pedro follower must be updated every loop
            follower.update();

            // Drive the state machine
            pathState = autonomousPathUpdate();

            runIntakeLifterWithColorSensor();

            /*
            // Update tray servo FSM if running
            if (trayServoFSM.isRunning()) {
                trayServoFSM.update(getRuntime());
                if (!trayServoFSM.isRunning()) {
                    // Update current tray position when done
                    currentTrayPosition = targetTrayPosition;
                }
            }

             */

            // Panels/driver telemetry
            panelsTelemetry.addData("Tray Curr", currentTrayPosition);
            //panelsTelemetry.addData("Tray Targ", targetTrayPosition);
            panelsTelemetry.addData("Path State", pathState);
            panelsTelemetry.addData("X", follower.getPose().getX());
            panelsTelemetry.addData("Y", follower.getPose().getY());
            panelsTelemetry.addData("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);

            telemetry.update();
        }
    }


    public static class Paths {

        public PathChain ShootingPosition;
        public PathChain IntakePosition;
        public PathChain Intake1;
        public PathChain Intake2;
        public PathChain Intake3;
        public PathChain ShootingPosition2;
        public PathChain Parking;

        public Paths(Follower follower) {
            ShootingPosition = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 9.000), new Pose(56.000, 18.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                    .build();

            IntakePosition = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 18.000), new Pose(42.000, 35.750))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();

            Intake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.000, 35.750), new Pose(37.000, 35.750))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Intake2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(37.000, 35.750), new Pose(32.000, 35.750))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Intake3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(32.000, 35.750), new Pose(27.000, 35.750))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ShootingPosition2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(27.000, 35.750),
                                    new Pose(45.000, 29.000),
                                    new Pose(56.000, 18.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Parking = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 18.000), new Pose(56.000, 29.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(90))
                    .build();
        }
    }




//67

    public int autonomousPathUpdate() {
        telemetry.addData("PathState", pathState);
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());

        switch (pathState) {
            case 0:
                telemetry.addLine("Case " + pathState + ": Wait for Camera");

                // Set the initial tray position
                setTrayPosition(TRAY_POS_1_SCORE);
                follower.setMaxPower(0.8); // move slowly to prevent artifacts from falling out of tray
                tagFSM.start(getRuntime());
                if ((tagFSM.isDone()) || pathTimer.getElapsedTimeSeconds() > 2.67) {
                    aprilTagDetections = tagFSM.getDetections();

                    telemetry.addLine("Case " + pathState + ": exiting");
                    follower.followPath(paths.ShootingPosition);
                    setPathState(pathState + 1);
                }
                break;

            case 1:
                //start shooting
                telemetry.addLine("Case " + pathState + ": Wait for ShootingPosition, then shoot artifact");
                        telemetry.addLine("Case " + pathState + ": start shooting");

                shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), SHOT_GUN_POWER_UP);

                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    setPathState(pathState + 1);
                }
                break;

            case 2:
                //Align to start intake
                telemetry.addLine("Case " + pathState + ": updateShooting...");
                shootPatternFSM.updateShootPattern(getRuntime());

                if (shootPatternFSM.isShootPatternDone() || pathTimer.getElapsedTimeSeconds() > 10.0) {

                    rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);

                    // now continue with next path
                    follower.followPath(paths.IntakePosition, true);
                    setPathState(pathState + 1);
                }
                break;

            case 3:
                //start intaking balls
                telemetry.addLine("Case " + pathState + ": Wait for Path3, then start Path4");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4.0) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 1p");

                    follower.setMaxPower(0.175); //slow down for pickup

                    setTrayPosition(TRAY_POS_1_INTAKE);
                    follower.followPath(paths.Intake1, true);
                    setPathState(pathState + 1);
                }
                break;

            case 4:

                telemetry.addLine("Case " + pathState + ": Wait for Path4");
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4.2) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 2p");

                    setTrayPosition(TRAY_POS_3_INTAKE);
                    follower.followPath(paths.Intake2, true);
                    setPathState(pathState + 1);
                }
                break;

            case 5:
                telemetry.addLine("Case " + pathState + ": Wait for Path5");
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.3) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 3g");

                    setTrayPosition(TRAY_POS_2_INTAKE);
                    follower.followPath(paths.Intake3, true);
                    setPathState(pathState + 1);
                }
                break;

            case 6:







            default:
                // -1 or any undefined state: do nothing, stay idle
                telemetry.addLine("Idle state (pathState = " + pathState + ")");
                break;
        }

        return pathState;
    }

    /**
     * Sets the path state and resets its timer.
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}