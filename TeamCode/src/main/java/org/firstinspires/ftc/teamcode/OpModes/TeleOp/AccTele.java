package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import android.util.Size;

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
import org.firstinspires.ftc.teamcode.Control.Controllers.SequenceController;
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

@TeleOp
public class AccTele extends LinearOpMode {
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;

    private Intake intake;
    private Follower follower;

    private ElapsedTime timer;

    public static Pose goalPose = new Pose(3,142);
    private SequenceController sequence;

    boolean slow;
    @Override
    public void runOpMode(){

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        initAprilTag(hardwareMap);
        transfer= new Transfer(hardwareMap);
        shooter=new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72,Math.toRadians(90)));
        follower.startTeleopDrive();
        follower.update();
        timer = new ElapsedTime();
        timer.reset();
        sequence = new SequenceController(transfer,shooter,turret,intake,follower,goalPose);
        transfer.retract();
        String[] GPP = {"GREEN","PURPLE","PURPLE"};
        String[] PGP = {"PURPLE","GREEN","PURPLE"};
        String[] PPG = {"PURPLE","PURPLE","GREEN"};
        waitForStart();

        while (opModeIsActive()){
            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
            double sec = timer.seconds();
            if(!sequence.isBusy()){
                if (gamepad1.dpadLeftWasPressed()){
                    sequence.setMotifOrder(GPP);
                } else if (gamepad1.dpadDownWasPressed()) {
                    sequence.setMotifOrder(PGP);
                } else if (gamepad1.dpadRightWasPressed()) {
                    sequence.setMotifOrder(PPG);
                }
            }

            if (gamepad1.aWasPressed()){
                sequence.start();
            }
            if (gamepad1.bWasPressed()){
                sequence.end();
            }

            if(gamepad1.yWasPressed()){
                transfer.setTargetDeg(30,sec);
            }




//            transfer.update(sec);
            shooter.update();
            turret.update();
            if(gamepad1.leftBumperWasPressed()) {
                slow=false;
            }
            else if(gamepad1.rightBumperWasPressed()){
                slow=true;
            }

            if(slow){
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y*.5,
                        -gamepad1.left_stick_x*.8,
                        -gamepad1.right_stick_x*.5, true);
            }
            else {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x, true);
            }


            if(gamepad1.guide) {
                setRobotPoseFromCamera();
            }
            follower.update();
            transfer.update(sec);
            sequence.update(sec,follower);
            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            telemetry.addData("shooting state: ",sequence.getShootingState());
            telemetry.addData("follower pose x:",follower.getPose().getX());
            telemetry.addData("follower pose y:",follower.getPose().getY());
            telemetry.addData("follower pose h:",Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("follower dist to goal:", Math.hypot(goalPose.getX()-follower.getPose().getX(),goalPose.getY()-follower.getPose().getY()));
            telemetry.addLine(transfer.getArrString());
            telemetry.addLine(transfer.getMapString());

            telemetry.addLine(Arrays.toString(sequence.getMotifOrder()));
            telemetry.addData("spin curent",transfer.getPositionDeg());
            telemetry.addData("spin target",transfer.getTargetDeg());
            telemetry.addData("shooter target",shooter.getTarget());
            telemetry.addData("shooter current",shooter.getVelocity());

            telemetry.addData("turret target", turret.getTarget());
            telemetry.addData("turret pos",turret.getPosition());


            telemetry.addData("turret atTarget", turret.atTarget());
            telemetry.addData("shooter atTarget",shooter.atTarget());
            telemetry.addData("spin atTarget",transfer.atTarget());
            telemetry.addData("spin error",transfer.error());
            telemetry.update();
        }
    }
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
