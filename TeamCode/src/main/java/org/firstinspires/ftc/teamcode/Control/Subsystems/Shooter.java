package org.firstinspires.ftc.teamcode.Control.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Shooter {
    private DcMotorEx motor;
    private Servo hood;
    public static double ratio,veloMult=1.356;
    private static final double TICKS_PER_REV = 28.0; // Adjust for your motor encoder CPR

//    private InterpLUT gains;



    private PIDFController b, s;

    public static double t = 0,hoodT;
    private final double hoodRatio= 36.0/524.0, servoRatio=1/(360.0-30);




    public static final double baseAngle=66.2;

    /*
    target = 70
    base = 66.2
    (target-base)/hoodRatio = hoodT

    hoodT*servoRatio= servo pos



     */
    public static double bp = 0.0015, bd = 0.0, bf = 0.0002, sp = 0.0, sd = 0.000, sf = 0.0;

    public static double pSwitch = 50;

    public static double close = .1,far =.8;
    private boolean activated = true;


    public Shooter(HardwareMap hardwareMap) {
//        gains = new InterpLUT();
////        gains.add(-5 * 60.0 / TICKS_PER_REV,.00059);
////        gains.add(1000 * 60.0 / TICKS_PER_REV,.00057);
////        gains.add(2000 * 60.0 / TICKS_PER_REV,.00049);
////        gains.add(5000 * 60.0 / TICKS_PER_REV,.00043);
////        gains.add(10000 * 60.0 / TICKS_PER_REV,.0004);
//        gains.createLUT();


        b = new PIDFController(new PIDFCoefficients(bp, 0, bd, 0));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, 0));
        motor = hardwareMap.get(DcMotorEx.class, "shoot");

        hood = hardwareMap.get(Servo.class,"hood");
        //r = hardwareMap.get(DcMotorEx.class, "sr");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        hood.setDirection(Servo.Direction.REVERSE);
    }

    public double getTarget() {
        return t;
    }

    public double getVelocity() {

        double ticksPerSecond = motor.getVelocity();
        double currentRPM = ticksPerSecond * 60.0 / TICKS_PER_REV;
        return currentRPM;    }

    public void setPower(double p) {
        motor.setPower(p);
        //r.setPower(p);
    }

    public void off() {
        activated = false;
        setPower(0);
    }

    public void on() {
        activated = true;
    }

    public boolean isActivated() {
        return activated;
    }


    public void setTarget(double velocity) {
        t = velocity;
    }

    public void setHood(double x){
         x= Range.clip(x,65.5,78);
        hoodT=(x-baseAngle)/hoodRatio;
    }
    public double getHood(){
        return (hoodT*hoodRatio)+baseAngle;
    }






    public void update() {
        b.setCoefficients(new PIDFCoefficients(bp, 0, bd, 0));
        s.setCoefficients(new PIDFCoefficients(sp, 0, sd, 0));
        hood.setPosition(hoodT*servoRatio+.1);
        if (activated) {
//            if (Math.abs(getTarget() - getVelocity()) < pSwitch) {
//                s.updateError(getTarget() - getVelocity());
//                setPower(s.run());
//            } else {
                b.updateError(getTarget() - getVelocity());
                setPower(getTarget()*bf+b.run());
//            }
//            motor.setVelocity(getTarget());
        }
    }


    public boolean atTarget() {
        return Math.abs((getTarget()- getVelocity())) < 200;
    }

    public void forDistance(double distance) {
        setHood(.33*distance+52.5);

        double launchAngle = getHood();

        double goalDistance = distance;
        double goalHeight = 37;

        double g = 9.81;
        double theta = Math.toRadians(launchAngle);
        double x = goalDistance / 39.37;
        double deltaY = (goalHeight - 10.84) / 39.37;

        double launchVelocity = Math.sqrt(
                (g * x * x) /
                        (2 * Math.pow(Math.cos(theta), 2) * (x * Math.tan(theta) - deltaY))
        );

        double radPerSec = launchVelocity/0.0381;//radius


        veloMult=-0.0213775*getHood()+4.01975;
        setTarget(((60.0*radPerSec)/(2.0*Math.PI))*veloMult);


    }



}

