package org.firstinspires.ftc.teamcode.Control.Subsystems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.HashMap;
import java.util.function.Consumer;

public class Transfer {
    private DcMotorEx spin;
    private double ticksToDeg;
    private double degToTicks;
    
    private Servo kicker;

    private RevColorSensorV3 slot1;
    private RevColorSensorV3 slot2;
    private RevColorSensorV3 slot3;

    public static double kickerUp =.1,kickerDown = .4;

    private PIDFController controller;
    public static PIDFCoefficients spinCoff;
    
    public static HashMap<String,Double> spinStates;
    private boolean on;
    private double targetDeg;

    public Transfer(HardwareMap hardwareMap){
        spin=hardwareMap.get(DcMotorEx.class,"spin");
        kicker= hardwareMap.get(Servo.class,"kicker");
        slot1=hardwareMap.get(RevColorSensorV3.class,"slot1");
        slot2=hardwareMap.get(RevColorSensorV3.class,"slot2");
        slot3=hardwareMap.get(RevColorSensorV3.class,"slot3");
        
        spinCoff=new PIDFCoefficients(0,0,0,0);
        controller= new PIDFController(spinCoff);
        spinStates = new HashMap<>();
        spinStates.put("SLOT1-COLOR",60D);
        spinStates.put("SLOT2-COLOR",120D);
        spinStates.put("SLOT3-COLOR",180D);


        int ticksPerMotorRev = 0;
        double gearRatio = 1.0;
        double ticksPerTurretRev = ticksPerMotorRev * gearRatio;

        ticksToDeg = 360.0 / ticksPerTurretRev;
        degToTicks = 1.0 / ticksToDeg;
    }

    private double normalizeAngleDeg(double angleDeg) {
        double a = angleDeg % 360.0;
        if (a <= -180.0) a += 360.0;
        if (a > 180.0) a -= 360.0;
        return a;
    }

    private double wrap360(double angleDeg) {
        double a = angleDeg % 360.0;
        if (a < 0) a += 360.0;
        return a;
    }
    public void setTargetDeg(double deg) {
        targetDeg = wrap360(deg);
    }
    public double getTargetDeg() { return targetDeg; }

    public double getPositionDeg() {
        double ticks = spin.getCurrentPosition();
        double deg = ticks * ticksToDeg;

        return wrap360(deg);
    }




    public void update(){
        if (on){
            controller.setCoefficients(spinCoff);
            controller.updateError(normalizeAngleDeg(targetDeg - getPositionDeg()));
            spin.setPower(controller.run());
        }
        else{
            spin.setPower(0);
        }
    }
    //if blank,0 if green 1,if purple 2
    private int getColorID(RevColorSensorV3 sensor){




        return 0;


    }




    private String getColorString(int x){
        if (x==1){
            return "GREEN";
        } else if (x==2) {
            return "PURPLE";
        }
        else {
            return "";
        }
    }
    public void scan(){
        setTargetDeg(0);

        HashMap<String,Double> temp  =new HashMap<>();
        int slot1Green = getColorID(slot1);
        int slot2Green = getColorID(slot2);
        int slot3Green = getColorID(slot3);
        if(slot1Green>0) temp.put(getColorString(slot1Green),120D);
        if(slot2Green>0) temp.put(getColorString(slot2Green),240D);
        if(slot3Green>0) temp.put(getColorString(slot3Green),0D);

        spinStates=temp;

    }
    public void spinTo(String x){
        double closestBall =10000;
        for (String i:spinStates.keySet()){
            if (i.contains(x)){
                if( Math.min(
                        Math.abs(spinStates.get(i)-getPositionDeg()),
                        Math.abs(spinStates.get(i)-getPositionDeg()+360)
                    )<Math.min(
                        Math.abs(closestBall-getPositionDeg()),
                        Math.abs(closestBall-getPositionDeg()+360)
                )
                ){
                    closestBall=spinStates.get(i);
                }
            }
        }

        if(closestBall<1000){
            setTargetDeg(closestBall);
        }
    }
    public void spinTo(){
        double closestBall =10000;
        for (String i:spinStates.keySet()){
                if( Math.min(
                        Math.abs(spinStates.get(i)-getPositionDeg()),
                        Math.abs(spinStates.get(i)-getPositionDeg()+360)
                )<Math.min(
                        Math.abs(closestBall-getPositionDeg()),
                        Math.abs(closestBall-getPositionDeg()+360)
                )
                ) {
                    closestBall = spinStates.get(i);
                }
        }

        if(closestBall<1000){
            setTargetDeg(closestBall);
        }
    }
    public void score(){
        kicker.setPosition(kickerUp);
    }
    public void retract(){
        kicker.setPosition(kickerDown);
    }







}
