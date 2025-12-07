package org.firstinspires.ftc.teamcode.Control.Subsystems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    private DcMotorEx motor;
    private Servo hood;
    public static double ratio;

    private PIDFController b, s;

    private double t = 0;
    public static double bp = 0.03, bd = 0.0, bf = 0.0, sp = 0.01, sd = 0.0001, sf = 0.0;

    public static double pSwitch = 50;

    public static double close = .1,far =.8;
    private boolean activated = true;


    public Shooter(HardwareMap hardwareMap) {
        b = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
        motor = hardwareMap.get(DcMotorEx.class, "shoot");

        hood = hardwareMap.get(Servo.class,"hood");
        //r = hardwareMap.get(DcMotorEx.class, "sr");
        //r.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getTarget() {
        return t;
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

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


    public void update() {
        b.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
        s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

        if (activated) {
//            if (Math.abs(getTarget() - getVelocity()) < pSwitch) {
//                s.updateError(getTarget() - getVelocity());
//                setPower(s.run());
//            } else {
//                b.updateError(getTarget() - getVelocity());
//                setPower(b.run());
//            }
            motor.setVelocity(getTarget());
        }
    }


    public boolean atTarget() {
        return Math.abs((getTarget()- getVelocity())) < 50;
    }

    public void forDistance(double distance) {
        //setTarget((6.13992 * distance) + 858.51272);
        setTarget(2000);
    }



}

