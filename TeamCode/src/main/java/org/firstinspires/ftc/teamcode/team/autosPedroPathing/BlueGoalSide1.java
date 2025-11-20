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
import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

/**
 * Pedro Pathing auto using LinearOpMode via DarienOpMode.
 */
@Autonomous(name = "BlueGoalSidePedro", group = "Autonomous")
@Configurable
public class BlueGoalSide1 extends DarienOpModeAuto {

    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    public Follower follower;                   // Pedro Pathing follower instance
    private int pathState;                      // State machine state
    private Paths paths;                        // Paths
    private Timer pathTimer, opmodeTimer;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- ROBOT + HARDWARE INIT (from DarienOpMode) ---
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
                            new BezierLine(new Pose(47.224, 96.443), new Pose(47.224, 96.443))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(135))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.224, 96.443), new Pose(40.739, 84.305))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.739, 84.305), new Pose(18.291, 84.305))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.291, 84.305),
                                    new Pose(39.409, 85.136),
                                    new Pose(47.224, 96.443)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.224, 96.443), new Pose(47.390, 121.552))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
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
                telemetry.addLine("Case 0: Start Path1");
                // Start first path ONCE
                follower.followPath(paths.Path1);
                setPathState(1);
                break;

            case 1:
                telemetry.addLine("Case 1: Wait for Path1, then start Path2");
                // Safety timeout in case isBusy() never goes false
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    telemetry.addLine("Case 1: Moving to Path2");
                    follower.followPath(paths.Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                telemetry.addLine("Case 2: Wait for Path2, then start Path3");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    telemetry.addLine("Case 2: Moving to Path3");
                    follower.followPath(paths.Path3, true);
                    setPathState(3);
                }
                break;

            case 3:
                telemetry.addLine("Case 3: Shoot once at scoring position");

                // Use DarienOpMode hardware directly
                TrayServo.setPosition(TRAY_POS_2_SCORE);
                currentTrayPosition = TRAY_POS_2_SCORE;

                shootArtifact(); // from DarienOpMode

                // After shooting, continue along Path4
                follower.followPath(paths.Path4, true);
                setPathState(4);
                break;

            case 4:
                telemetry.addLine("Case 4: Wait for Path4, then start Path5");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    telemetry.addLine("Case 4: Moving to Path5");
                    follower.followPath(paths.Path5, true);
                    setPathState(5);
                }
                break;

            case 5:
                telemetry.addLine("Case 5: Wait for Path5, then start Path6");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    telemetry.addLine("Case 5: Moving to Path6");
                    follower.followPath(paths.Path6, true);
                    setPathState(6);
                }
                break;

            case 6:
                telemetry.addLine("Case 6: Wait for Path6 to finish, then stop");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    telemetry.addLine("Case 6: Done, setting state -1");
                    setPathState(-1); // done
                }
                break;

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