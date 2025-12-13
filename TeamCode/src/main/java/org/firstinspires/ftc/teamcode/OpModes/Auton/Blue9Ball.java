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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Control.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Control.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Control.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;

@Autonomous(name = "Example Auto", group = "Examples")
public class Blue9Ball extends OpMode {
    private Transfer transfer;
    private Turret turret;
    private Shooter shooter;

    private Intake intake;


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
                    follower.followPath(
                            follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(follower.getPose(), scorePreload.endPose())
                                    )
                                    .setConstantHeadingInterpolation(Math.toRadians(180))
                                    .build(),
                            true
                    );

                    setPathState(20);
                }
                break;
            case 20:
                if(!follower.isBusy()){
                    state1=1;
                    setPathState(21);
                }

                break;
            case 21:
                if(state1==-1){
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds() > 1) {
                    /* Grab Sample */
                    transfer.setTargetDeg(30,opmodeTimer.getElapsedTimeSeconds());
                    intake.setPower(1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(intakeSet1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() &&pathTimer.getElapsedTimeSeconds() > 2) {
                    /* Score Sample */
                    intake.setPower(0);
                    transfer.setTargetDeg(240,opmodeTimer.getElapsedTimeSeconds());

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scoreSet1,true);
                    setPathState(-4);
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
        double sec  = opmodeTimer.getElapsedTimeSeconds();

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        turret.facePoint(goalPose,follower.getPose());
        double goalDistance = Math.hypot(goalPose.getX()-follower.getPose().getX(),goalPose.getY()-follower.getPose().getY());

        shooter.on();
        if(state1!=-1) {
            shooter.forDistance(goalDistance);
        }else{
            shooter.setTarget(0);
        }
        scoreBall(sec);
        transfer.update(sec);
        turret.update();
        shooter.update();


        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("state1: ", state1);
        telemetry.addData("cycles: ", cycles);
        telemetry.addData("input: ", inputCounter);
        telemetry.addData("shoot order: ", Arrays.toString(motif));
        telemetry.addData("test: ", test);
        telemetry.update();
    }
    int state1=-1;
    int cycles=0;
    ElapsedTime stateTimer;
    int inputCounter=0;

    String[] motif = {"PURPLE","GREEN","PURPLE"};
    boolean test = false;

    public void scoreBall(double sec){

//        transfer.spinTo(x,sec,true);
        switch (state1){
            case 0:
                if(inputCounter<3){
                    if(gamepad1.rightBumperWasPressed()){
                        motif[inputCounter]="PURPLE";
                        inputCounter++;
                    } else if (gamepad1.leftBumperWasPressed()) {
                        motif[inputCounter]="GREEN";
                        inputCounter++;
                    }
                }
                else{
                    state1=1;
                }
                break;
            case 1:
                transfer.scan(sec);
                cycles=transfer.getNumBalls();
                state1=2;
                break;
            case 2:
                if(cycles>0){
                    transfer.setTargetDeg(transfer.spinTo(motif[0],sec,true),sec);
                    stateTimer.reset();
                    state1=10;
                }
                else{
                    state1=-1;
                }
                break;
            case 10:
                if (transfer.atTarget() && shooter.atTarget() && turret.atTarget()){
                    transfer.score();
                    stateTimer.reset();
//                    test=true;
                    state1=3;
                }
                break;
            case 3:
                if (stateTimer.milliseconds()>kickDown){
                    if(cycles>1){
                        transfer.scan(sec);
                        transfer.setTargetDeg(transfer.spinTo(motif[1],sec,true),sec);
                        state1=20;
                    }
                    else {
                        state1=-1;
                    }
                }
                else if (stateTimer.milliseconds()>kickerStall){
                    transfer.retract();
                }
                break;
            case 20:
                if (transfer.atTarget()&& shooter.atTarget()&& turret.atTarget()){
                    transfer.score();
                    stateTimer.reset();
                    state1=4;
                }
                break;

            case 4:
                if (stateTimer.milliseconds()>kickDown){
                    if(cycles>2){
                        transfer.scan(sec);
                        transfer.setTargetDeg(transfer.spinTo(motif[2],sec,true),sec);
                        state1=5;
                    }
                    else {
                        state1=-1;
                    }
                }
                else if (stateTimer.milliseconds()>kickerStall){
                    transfer.retract();
                }
                break;
            case 5:
                if (transfer.atTarget()&& shooter.atTarget()&& turret.atTarget()){
                    transfer.score();
                    stateTimer.reset();
                    state1=6;
                }
                break;
            case 6:
                if (stateTimer.milliseconds()>2000){
                    transfer.retract();
                    state1=-1;
                }
                break;


//            case 4:
//                if (stateTimer.milliseconds()>kickDown){
//                    if(cycles>2){
//                        transfer.scan(sec);
//                        transfer.setTargetDeg(transfer.spinTo(motif[2],sec,true),sec);
//                        if (transfer.atTarget()&& shooter.atTarget()&& turret.atTarget()){
//                            transfer.score();
//                            stateTimer.reset();
//                            state1=-1;
//                        }
//                    }
//                    else {
//                        state1=-1;
//                    }
//                }
//                else if (stateTimer.milliseconds()>kickerStall){
//                    transfer.retract();
//                }
//                break;
//            case 5:
//

        }
    }
    public static int kickerStall=700,kickDown=1300;


    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        stateTimer=new ElapsedTime();
        opmodeTimer.resetTimer();

        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

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