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

/**
 * Pedro Pathing auto using LinearOpMode via DarienOpModeFSM.
 */
@Autonomous(name = "BlueGoalSidePedro", group = "Pedro:Blues", preselectTeleOp = "Teleop")
@Configurable
public class BlueGoalSide1 extends DarienOpModeFSM {

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

        // --- MAIN AUTONOMOUS LOOP ---
        while (opModeIsActive() && !isStopRequested()) {

            // Pedro follower must be updated every loop
            follower.update();

            // Drive the state machine
            pathState = autonomousPathUpdate();

            runIntakeLifterWithColorSensor();

            // Panels/driver telemetry
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);

            telemetry.update();
        }
    }

    /**
     * Inner class defining all the Pedro paths.
     */
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        // 67
        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.286, 124.378), new Pose(47.224, 96.443))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(70))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(47.224, 96.443, Math.toRadians(70)),
                                    new Pose(47.234, 96.443, Math.toRadians(145)))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(145))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.732, 96.443), new Pose(40.739, 84.305))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(38.599, 84.305), new Pose(28.933, 84.305))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(33.555, 84.305), new Pose(23.446, 84.305))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(30.555, 84.305), new Pose(17.792, 84.305))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(17.792, 84.305),
                                    new Pose(38.910, 86.965),
                                    new Pose(47.224, 96.443)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.224, 96.443), new Pose(47.557, 122.383))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    /**
     * State machine for autonomous sequence.
     * Returns current pathState for logging.
     */
    public int autonomousPathUpdate() {

        // Helpful debug info every loop
        telemetry.addData("PathState", pathState);
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());

        switch (pathState) {
            case 0:
                telemetry.addLine("Case " + pathState + ": Start Path1");

                // Set the initial tray position
                setTrayPosition(TRAY_POS_1_SCORE);
                //servoIncremental(TrayServo, TRAY_POS_2_SCORE, currentTrayPosition, 1, 4);


                // Start first path ONCE
                follower.followPath(paths.Path1);
                setPathState(pathState + 1);
                break;

            case 1:
                telemetry.addLine("Case " + pathState + ": Wait for Path1 and camera, then start read AprilTag");

                if ((!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.0)) {

                    telemetry.addLine("Case " + pathState + ": exiting");
                    // Start AprilTag reading after path1 is done
                    tagFSM.start(getRuntime());

                    setPathState(pathState + 1);
                }
                break;

            case 2:
                telemetry.addLine("Case " + pathState + ": Read AprilTag then start Path2");

                tagFSM.update(getRuntime(), true, telemetry);

                if ((tagFSM.isDone()) || pathTimer.getElapsedTimeSeconds() > 4.0) {
                    aprilTagDetections = tagFSM.getDetections();

                    telemetry.addLine("Case " + pathState + ": exiting");
                    follower.followPath(paths.Path2);
                    setPathState(pathState + 1);
                }
                break;

            case 3:
                telemetry.addLine("Case " + pathState + ": Wait for Path2, then shoot artifact");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    telemetry.addLine("Case " + pathState + ": exiting");
                    setPathState(pathState + 1);
                }
                break;

            case 4:
                telemetry.addLine("Case " + pathState + ": start shooting");

                //startShooting();
                shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), 0.9);

                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    setPathState(pathState + 1);
                }
                break;

            case 5:
                telemetry.addLine("Case " + pathState + ": updateShooting...");
                //updateShooting(1);
                shootPatternFSM.updateShootPattern(getRuntime());

                if (shootPatternFSM.isShootPatternDone() || pathTimer.getElapsedTimeSeconds() > 12.0) {
                    //resetShooting();
                    //setBreakpoint();

                    rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                    setTrayPosition(TRAY_POS_1_INTAKE);
                    //servoIncremental(TrayServo, TRAY_POS_1_INTAKE, currentTrayPosition, 1, 4);


                    // now continue with next path
                    follower.followPath(paths.Path3, true);
                    setPathState(pathState + 1);
                }
                break;

            case 6:
                telemetry.addLine("Case " + pathState + ": Wait for Path3, then start Path4");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4.0) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact");

                    follower.setMaxPower(0.2); //slow down for pickup

                    //setBreakpoint();
                    follower.followPath(paths.Path4, true);
                    setTrayPosition(TRAY_POS_2_INTAKE);
                    setPathState(pathState + 1);
                }
                break;

            case 7:
                telemetry.addLine("Case " + pathState + ": Wait for Path3, then start Path4");
                if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact");

                    //setBreakpoint();
                    follower.followPath(paths.Path5, true);
                    setTrayPosition(TRAY_POS_3_INTAKE);
                    rubberBands.setPower(0);
                    setPathState(pathState + 1);
                }
                break;

            case 8:
                telemetry.addLine("Case " + pathState + ": Wait for Path3, then start Path4");
                if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact");

                    //setBreakpoint();
                    follower.followPath(paths.Path6, true);
                    setPathState(pathState + 1);
                }
                break;
            /*
            case 9:
                telemetry.addLine("Case " + pathState + ": Wait for Path4 to pick up artifact, then start Path5");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    telemetry.addLine("Case " + pathState + ": Moving to shooting position");

                    follower.setMaxPower(0.8); //resume normal speed

                    //setBreakpoint();
                    follower.followPath(paths.Path7, true);
                    setPathState(pathState + 1);
                }
                break;

            case 10:
                telemetry.addLine("Case " + pathState + ": Wait for Path5 to get into position, then start Path6");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    telemetry.addLine("Case " + pathState + ": Moving to Path6");
                    //setBreakpoint();
                    follower.followPath(paths.Path8, true);
                    setPathState(pathState + 1);
                }
                break;

            case 11:
                telemetry.addLine("Case " + pathState + ": Wait for Path6 to finish, then stop");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    telemetry.addLine("Case " + pathState + ": Done, setting state -1");
                    rubberBands.setPower(0);
                    setPathState(-1); // done
                }
                break;
             */

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