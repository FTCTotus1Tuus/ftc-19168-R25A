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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous(name = "BlueGoalSidePedro", group = "Autonomous")
@Configurable // Panels
public class BlueGoalSide1 extends DarienOpModeAuto {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, actionTimer, opmodeTimer;
    private DarienOpModeAuto darienOpModeAuto = new DarienOpModeAuto();

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.286, 124.378, Math.toRadians(54)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

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

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine

        switch (pathState) {
            case 0:
                telemetry.addLine("Im in case 0");
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                telemetry.addLine("Im in case 1");
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                telemetry.addLine("Im in case 2");
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(paths.Path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                TrayServo.setPosition(TRAY_POS_2_SCORE);
                currentTrayPosition = TRAY_POS_2_SCORE;
                // TODO; replace sleeps with timer
                //sleep(1000);
                shootArtifact();
                //sleep(500);
                /*
                // shoot purple
                servoIncremental(TrayServo, TRAY_POS_3_SCORE, currentTrayPosition, 1, 4);
                currentTrayPosition = TRAY_POS_3_SCORE;
                sleep(1000);
                shootArtifact();
                sleep(500);
                // shoot purple
                servoIncremental(TrayServo, TRAY_POS_1_SCORE, currentTrayPosition, 1, 4);
                currentTrayPosition = TRAY_POS_1_SCORE;
                sleep(1000);
                shootArtifact();
                sleep(500);

                 */
                break;
            case 4:
                telemetry.addLine("Im in case 3");
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    setPathState(4);
                }
                break;
            case 5:
                telemetry.addLine("Im in case 4");
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    setPathState(5);
                }
                break;
            case 6:
                telemetry.addLine("Im in case 5");
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    setPathState(6);
                }
                break;
            case 7:
                telemetry.addLine("Im in case 6");
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
        return pathState;
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}