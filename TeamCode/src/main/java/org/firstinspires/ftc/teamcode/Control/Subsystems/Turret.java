package org.firstinspires.ftc.teamcode.Control.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Turret {
    private DcMotorEx motor;
    private double degToTick = 0;
    private double tickToDeg = 1/degToTick;

    private PIDFController bigC,smallC;
    public static PIDFCoefficients bigCoff, smallCoff;

    private boolean on = false;
    private boolean manual = false;

    private double target =0;
    private double manualPower;

    public Turret(HardwareMap hardwareMap){
        motor=hardwareMap.get(DcMotorEx.class,"turret");
        bigCoff = new PIDFCoefficients(0,0,0,0);
        smallCoff = new PIDFCoefficients(0,0,0,0);

        bigC= new PIDFController(bigCoff);
        smallC = new PIDFController(smallCoff);


    }

    private void setTarget(double x){
        target=x;
    }
    private double getTarget(){
        return target;
    }

    private double getPosition(){
        return motor.getCurrentPosition();
    }

    public void update(){
        if (on){
            if (manual){
                motor.setPower(manualPower);
                return;
            }
            bigC.setCoefficients(bigCoff);
            smallC.setCoefficients(smallCoff);
            if (Math.abs(getTarget()-getPosition())>200){
                bigC.updateError(getTarget()-getPosition());
                motor.setPower(bigC.run());
            }
            else{
                smallC.updateError(getTarget()-getPosition());
                motor.setPower(smallC.run());
            }
        }
        else{
            motor.setPower(0);
        }
    }

    public void manual(double power) {
        manual = true;
        manualPower = power;
    }

    public void automatic() {
        manual = false;
    }

    public void on() {
        on = true;
    }

    public void off() {
        on = false;
    }

    public void setYaw(double deg) {
        deg = normalizeAngle(deg);
        setTarget(deg/degToTick);
    }

    public void resetTurret() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setTarget(0);
    }

    public static double normalizeAngle(double deg) {
        double angle = deg % 360.0;
        if (angle <= -180) angle += 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    public void facePoint(Pose targetPose, Pose robotPose) {
        double angleToTargetFromCenter = Math.toDegrees(Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX()));
        double robotAngleDiff = normalizeAngle(angleToTargetFromCenter - Math.toDegrees(robotPose.getHeading()));
        setYaw(robotAngleDiff);
    }


}
