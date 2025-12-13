package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Control.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Control.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Control.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Control.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;

@Config
@TeleOp
public class TestTeleOp extends LinearOpMode {
    Follower follower;

    public static double hoodAngle = Shooter.baseAngle;
    boolean state=false;

    public static int rpm= 0;

    private Transfer transfer;
    private Turret turret;
    private Shooter shooter;

    private Intake intake;
    public static Pose goalPose = new Pose(1,138);

    private ElapsedTime timer;



    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        initAprilTag(hardwareMap);
        stateTimer=new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Declare our motors
        // Make sure your ID's match your configuration

        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72,Math.toRadians(90)));
        follower.startTeleopDrive();
        follower.update();
        turret.automatic();
        shooter.setHood(hoodAngle);
        shooter.update();
        transfer.retract();

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        timer = new ElapsedTime();
        timer.reset();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            turret.on();
            double sec = timer.seconds();


            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            if(gamepad1.aWasPressed()){
                transfer.setTargetDeg(0,sec);
            }
            if(gamepad1.xWasPressed()){
                transfer.setTargetDeg(120,sec);
            }
            if(gamepad1.bWasPressed()){
                transfer.setTargetDeg(240,sec);
            }


            if(gamepad1.yWasPressed()){
                transfer.setTargetDeg(30,sec);
            }
            if(gamepad1.yWasReleased()){
                transfer.setTargetDeg(240,sec);
            }
//
            if(gamepad1.dpad_left&&state1==-1){
                transfer.score();
            }
            else if (state1==-1){
                transfer.retract();
            }

            if(gamepad1.dpadUpWasPressed()&&!state){
                state1=0;
                inputCounter=0;

                state=true;
            }
            if(gamepad1.dpadDownWasPressed()&&state){
                state1=-1;
                state=false;
            }
            if(state1!=-1){
                intake.setPower(.45);
            }
            else {
                intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

            }


            if (state){
                double goalDistance = Math.hypot(goalPose.getX()-follower.getPose().getX(),goalPose.getY()-follower.getPose().getY());
                shooter.on();
                shooter.forDistance(goalDistance);
            }else{
                shooter.on();
                shooter.setTarget(rpm);
            }
            scoreBall(sec);

            turret.facePoint(goalPose,follower.getPose());

            shooter.setHood(hoodAngle);
//            turret.facePoint(goalPose,follower.getPose());

//

            if (gamepad1.back){
                transfer.scan(sec);
            }

//            if (gamepad1.dpadRightWasPressed()){
//                transfer.scan(sec);
//                transfer.setTargetDeg(transfer.spinTo("PURPLE",sec,false),sec);
//            }
//            if (gamepad1.dpadLeftWasPressed()){
//                transfer.scan(sec);
//                transfer.setTargetDeg(transfer.spinTo("GREEN",sec,false),sec);
//            }




            shooter.update();
            turret.update();
            transfer.update(sec);





            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y ,
                   -gamepad1.left_stick_x ,
                    -gamepad1.right_stick_x , true);


            if(gamepad1.guide) {
                setRobotPoseFromCamera();
            }
            follower.update();

            telemetry.addData("turret target", turret.getTarget());
            telemetry.addData("turret pos",turret.getPosition());
            telemetry.addData("turret pow",turret.getPower());
            telemetry.addLine(transfer.getMapString());
            telemetry.addLine(transfer.getArrString());
            telemetry.addData("shooter target",shooter.getTarget());
            telemetry.addData("shooter current",shooter.getVelocity());
            telemetry.addData("shooter hood",shooter.getHood());
            telemetry.addData("turret pow",turret.getPower());
            telemetry.addData("spin curent",transfer.getPositionDeg());
            telemetry.addData("spin target",transfer.getTargetDeg());


            telemetry.addData("follower pose x:",follower.getPose().getX());
            telemetry.addData("follower pose y:",follower.getPose().getY());
            telemetry.addData("follower pose h:",Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("follower dist to goal:", Math.hypot(goalPose.getX()-follower.getPose().getX(),goalPose.getY()-follower.getPose().getY()));
            telemetry.addData("turret atTarget", turret.atTarget());
            telemetry.addData("shooter atTarget",shooter.atTarget());
            telemetry.addData("spin atTarget",transfer.atTarget());

            telemetry.addData("state1: ", state1);
            telemetry.addData("cycles: ", cycles);
            telemetry.addData("input: ", inputCounter);
            telemetry.addData("shoot order: ", Arrays.toString(motif));
            telemetry.addData("test: ", test);
            telemetry.update();
        }
    }
    int state1=-1;
    int cycles=0;
    ElapsedTime stateTimer;
    int inputCounter=0;

    String[] motif = new String[3];
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
                if (stateTimer.milliseconds()>kickerStall){
                    transfer.retract();
                    state=false;
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

    private Position cameraPosition = new Position(DistanceUnit.INCH, -.25, 4.776, 10.35, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -65, 180, 0);

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;


    private void initAprilTag(HardwareMap hardwareMap) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(907.111,907.111,661.59,343.096)
                .build();


        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(new Size(1280, 720));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
    //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
    //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates

    double myXg =0 ;
    double myYg =0;
    double myYawg =0;
    double myX =0 ;
    double myY =0;
    double myYaw =0;
    private void setRobotPoseFromCamera() {


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if (detection.id == 20 || detection.id == 24) {
                    myX = detection.robotPose.getPosition().x;
                    myY = detection.robotPose.getPosition().y;
                    myYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
//                    sleep(500);
                }
            }
        }

        Pose ftcStandard = PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH,-myY-72,myX-72,AngleUnit.DEGREES,myYaw), InvertedFTCCoordinates.INSTANCE);
//        ftcStandard=ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        if(!currentDetections.isEmpty()) {
            follower.setPose(new Pose(myY+72,72-myX, Math.toRadians(myYaw)));
        }
        myXg=follower.getPose().getX();
        myYg=follower.getPose().getY();
        myYawg=Math.toDegrees(follower.getPose().getHeading());
    }


}