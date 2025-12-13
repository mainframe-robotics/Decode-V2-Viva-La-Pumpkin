package org.firstinspires.ftc.teamcode.Control.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret {
    private DcMotorEx motor;
    public static double ticksPerDeg = 2.2; // 384.5 * (170/80d) (??????????)

    private PIDFController bigC,smallC;

    public static double bigKp=.013,bigKi=0,bigKd=0.0009,bigKf=0,smallKp=0.036;


    public static boolean on = true;
    private boolean manual = false;

    public static double target =0;
    private double manualPower;
    public static double lowerLimit =-170,upperLimit=90;

    private double prevMotor;



    public Turret(HardwareMap hardwareMap){
        motor=hardwareMap.get(DcMotorEx.class,"tur");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients bigCoff = new PIDFCoefficients(bigKp, bigKi, bigKd, bigKf);
        PIDFCoefficients smallCoff = new PIDFCoefficients(0, 0, 0, 0);

        bigC= new PIDFController(bigCoff);
        smallC = new PIDFController(smallCoff);


    }

    private void setTarget(double x){
        target= Range.clip(x,lowerLimit,upperLimit);
    }
    public double getTarget(){
        return target;
    }

    public double getPosition(){
        return motor.getCurrentPosition() / ticksPerDeg;
    }

    public double getNormalixedPos(){ return normalizeAngle(getPosition());}

    public double getPower(){return motor.getPower();}

    public boolean atTarget(){
        return Math.abs(getTarget()-getPosition())<2;
    }

    public void update(){
        if (on){
            if (manual){
                motor.setPower(manualPower);
                return;
            }
            PIDFCoefficients bigCoff = new PIDFCoefficients(bigKp, bigKi, bigKd, bigKf);
            PIDFCoefficients smallCoff = new PIDFCoefficients(0, 0, 0, 0);
            bigC.setCoefficients(bigCoff);
            smallC.setCoefficients(smallCoff);
            target= Range.clip(target,lowerLimit,upperLimit);
            bigC.updateError(target-getPosition());

            double powr = bigC.run();
            if(Math.abs(bigC.getError())<10){
                powr=bigC.getError()*smallKp;
            }

//            if(Math.abs(powr-prevMotor)>.05){
                motor.setPower(powr);
//                prevMotor=(powr);
//            }

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
        setTarget(deg);
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
        Pose ballPose = new Pose(robotPose.getX()-Math.cos(robotPose.getHeading()), robotPose.getY()-Math.sin(robotPose.getHeading()));


        double angleToTargetFromCenter = Math.toDegrees(Math.atan2(targetPose.getY() - ballPose.getY(), targetPose.getX() - ballPose.getX()));
        double robotAngleDiff = normalizeAngle(Math.toDegrees(robotPose.getHeading())-angleToTargetFromCenter );
        setYaw(robotAngleDiff);
    }


}
