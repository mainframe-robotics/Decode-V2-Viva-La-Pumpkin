package org.firstinspires.ftc.teamcode.OpModes.Auton; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example Auto", group = "Examples")
public class Blue9Ball extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    public static Pose goalPose = new Pose(1,138);

    private final Pose startPose = new Pose(24.6,122.5,Math.toRadians(143));
    private final Pose shoot1Pose = new Pose(55.98,87.435,Math.toRadians(90));
    private final Pose intake1Pose = new Pose(30,83.4,Math.toRadians(180));
    private final Pose intake1PoseControl = new Pose(71.5,73.4);
    private final Pose shoot2Pose = new Pose(50,83.4,Math.toRadians(180));
    private final Pose intake2Pose = new Pose(28.42,58.29,Math.toRadians(180));
    private final Pose intake2PoseControl = new Pose(58,54);
    private final Pose shoot3Pose = new Pose(70,74,Math.toRadians(180));
    private final Pose leavePose = new Pose(45.4,66.7,Math.toRadians(180));

    private int pathState;
    private Path scorePreload;
    private PathChain intakeSet1, scoreSet1, intakeSet2, scoreSet2, leave;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, shoot1Pose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shoot1Pose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        intakeSet1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose,intake1Pose ))
                .setLinearHeadingInterpolation(180, intake1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSet1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, shoot2Pose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), shoot2Pose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        intakeSet2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot2Pose, intake2PoseControl,intake2Pose))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(), intake2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSet2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shoot3Pose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shoot3Pose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        leave = follower.pathBuilder()
                .addPath(new BezierLine(shoot3Pose, leavePose))
                .setLinearHeadingInterpolation(shoot3Pose.getHeading(), leavePose.getHeading())
                .build();


    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(scorePreload,true);
                    follower.turnTo(Math.toRadians(180));
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds() > 1) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(intakeSet1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() &&pathTimer.getElapsedTimeSeconds() > 1) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scoreSet1,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds() > 1) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(intakeSet2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds() > 1) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scoreSet2,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds() > 1) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(leave, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds() > 1) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}