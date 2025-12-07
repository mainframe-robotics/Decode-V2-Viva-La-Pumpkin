package org.firstinspires.ftc.teamcode.Control.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx motor;
    public Intake(HardwareMap hardwareMap){
        motor=hardwareMap.get(DcMotorEx.class,"int");

    }
    public void setPower(double x){
        motor.setPower(x);
    }
    public double getPower(){
        return motor.getPower();
    }
    public void intake(){
        setPower(1);
    }
    public void still(){
        setPower(0);
    }
    public void eject(){
        setPower(-1);
    }
}
