package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Control.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Control.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Control.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Control.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Config
@TeleOp
public class TestTeleOp extends LinearOpMode {
    Follower follower;

    public static int rpm= 2000;

    private Transfer spin;
    private Turret turret;
    private Shooter shooter;

    private Intake intake;
    public static Pose goalPose = new Pose();

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");
        spin = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72));
        follower.startTeleopDrive();
        follower.update();
        turret.automatic();

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            turret.on();
            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
            if(gamepad1.aWasPressed()){
                spin.setTargetDeg(0);
            }
            if(gamepad1.bWasPressed()){
                spin.setTargetDeg(120);
            }
            if(gamepad1.xWasPressed()){
                spin.setTargetDeg(240);
            }
            if(gamepad1.yWasPressed()){
                spin.setTargetDeg(30);
            }
            if(gamepad1.dpadDownWasPressed()){
                spin.score();
            }
            if(gamepad1.dpadUpWasPressed()){
                spin.retract();
            }
            if(gamepad1.right_bumper){
                double goalDistance = d2d(goalPose.getX(),goalPose.getY(),follower.getPose().getX(),follower.getPose().getY());
                shooter.on();
                shooter.forDistance(goalDistance);

                turret.facePoint(goalPose,follower.getPose());
            }
            else if(gamepad1.left_bumper){
                shooter.on();
                shooter.setTarget(rpm);

                turret.facePoint(goalPose,follower.getPose());
            }
            else{
                shooter.off();
            }

            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);



            shooter.update();
            turret.update();
            spin.update();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y ,
                   -gamepad1.left_stick_x ,
                    -gamepad1.right_stick_x , true);



            follower.update();

            telemetry.addData("spin target", turret.getTarget());
            telemetry.addData("spin pos",turret.getPosition());
            telemetry.update();
        }
    }
    private double d2d(double goalX,double goalY,double robotX,double robotY){
        return Math.sqrt(
                Math.pow(goalX-robotX,2) + Math.pow(goalY-robotY,2)
        );
    }

}