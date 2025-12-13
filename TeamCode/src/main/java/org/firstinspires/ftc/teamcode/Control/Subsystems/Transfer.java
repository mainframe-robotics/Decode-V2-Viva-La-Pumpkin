package org.firstinspires.ftc.teamcode.Control.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.HashMap;

@Config
public class Transfer {
    private MotionProfile profile;

    private DcMotorEx spin;
    private double ticksToDeg;
    private double degToTicks;
    
    private Servo kicker;

    private RevColorSensorV3 slot1;
    private RevColorSensorV3 slot2;
    private RevColorSensorV3 slot3;

    public static double mult=1;

    public static double kp=0.00905,ki=0.00001,kd=0.0002,kf=0,sp=0.03;



    public static double kickerUp =.05,kickerDown = .4;

    private PIDFController controller;
    public static PIDFCoefficients spinCoff=new PIDFCoefficients(0,0,0,0);
    
    public static HashMap<String,Double> spinStates;
    public static boolean on =true;
    public static double targetDeg;

    private ElapsedTime timer;

    public static double vMax=3400, aMax =3000;



    public Transfer(HardwareMap hardwareMap){
        profile = new MotionProfile(vMax, aMax);

        spin=hardwareMap.get(DcMotorEx.class,"spin");
        kicker= hardwareMap.get(Servo.class,"kick");
        slot1=hardwareMap.get(RevColorSensorV3.class,"slot1");
        slot2=hardwareMap.get(RevColorSensorV3.class,"slot2");
        slot3=hardwareMap.get(RevColorSensorV3.class,"slot3");
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer=new ElapsedTime();
        timer.reset();
        
        spinCoff=new PIDFCoefficients(kp,ki,kd,kf);
        controller= new PIDFController(spinCoff);
        spinStates = new HashMap<>();
        spinStates.put("SLOT1-COLOR",120D);
        spinStates.put("SLOT2-COLOR",240D);
        spinStates.put("SLOT3-COLOR",0D);


        double ticksPerMotorRev = 537.7;
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
    public void setTargetDeg(double deg, double nowTime) {
        deg = wrap360(deg);
        profile.startProfile(getPositionDeg(), deg, nowTime);
        targetDeg=deg;
    }
    public double getTargetDeg() { return targetDeg; }

    public double getPositionDeg() {
        double ticks = spin.getCurrentPosition();
        double deg = ticks * ticksToDeg;

        return wrap360(deg);
    }

    public int getNumBalls(){
        int counter=0;
        for(String i: spinStates.keySet()){
            if(i.contains("PURPLE")||i.contains("GREEN")){
                counter++;
            }
        }
        return counter;
    }





    public void update(double nowTime){
        if (on){
            spinCoff.setCoefficients(kp,ki,kd,kf);
            controller.setCoefficients(spinCoff);
            profile.maxAccel= aMax;
            profile.maxVel=vMax;

            double posCmd = profile.getPosition(nowTime);    // desired angle

            double posErrorDeg = normalizeAngleDeg(posCmd - getPositionDeg());
            double posErrorTicks = posErrorDeg * degToTicks;
            controller.updateError(posErrorTicks);
            if(Math.abs(controller.getError())>7) {
                spin.setPower(controller.run());
            }
            else {
                spin.setPower(controller.getError()*sp);
            }
        }
        else{
            spin.setPower(0);
        }
    }
    //if blank,0 if green 1,if purple 2
    private int getColorID(RevColorSensorV3 sensor){
        double alpha = sensor.alpha();
        double red = (alpha!=0)?sensor.red()/alpha:sensor.red();
        double blue = (alpha!=0)?sensor.blue()/alpha:sensor.blue();
        double green = (alpha!=0)?sensor.green()/alpha:sensor.green();

        /*
          /*
        Purple Ball:
            red:.67-.8
            blue: 1.1-1.5
            green:.75-1.17
        Green Ball:
            red: .3-.62
            blue: .95-1.23
            green: 1.2-1.6
         */

        if (inRange(red,.67,.8)&&inRange(blue,1.1,1.5)&&inRange(green,.75,1.17)){
            return 2;
        }
        else if (inRange(red,.3,.62)&&inRange(blue,.95,1.23)&&inRange(green,1.2,1.6)){
            return 1;
        }

        return 0;


    }

    private boolean inRange(double x,double min,double max){
        return x>=min&&x<=max;
    }





    private String getColorString(int x){
        if (x==1){
            return "GREEN";
        } else if (x==2) {
            return "PURPLE";
        }
        else {
            return "NOTHING";
        }
    }

    public String getMapString(){
        return spinStates.toString();
    }
    public HashMap getMap(){
        return spinStates;
    }
    private int[] numarr= new int[3];

    private double getClosest(){
        double error =100000;
        double pos=0;
        if(Math.abs(normalizeAngleDeg(getPositionDeg()-0))<error){
            pos = 0;
            error = Math.abs(normalizeAngleDeg(getPositionDeg()-0));
        }

        if (Math.abs(normalizeAngleDeg(getPositionDeg()-120))<error) {
            pos = 120;
            error = Math.abs(normalizeAngleDeg(getPositionDeg()-120));        }

        if(Math.abs(normalizeAngleDeg(getPositionDeg()-240))<error){
            pos = 240;
            error = Math.abs(normalizeAngleDeg(getPositionDeg()-240));
        }
        return pos;

    }


    public void scan(double time){
        retract();
        double scanPos= getClosest();

        setTargetDeg(scanPos,time);

        HashMap<String,Double> temp  =new HashMap<>();
        int slot1Green = getColorID(slot1);
        int slot2Green = getColorID(slot2);
        int slot3Green = getColorID(slot3);
        if(scanPos==0) {
            temp.put("Slot 1: " + getColorString(slot1Green), 120D);
            temp.put("Slot 2: " + getColorString(slot2Green), 240D);
            temp.put("Slot 3: " + getColorString(slot3Green), 0D);
        } else if(scanPos==120) {
            temp.put("Slot 1: " + getColorString(slot1Green), 240D);
            temp.put("Slot 2: " + getColorString(slot2Green), 0D);
            temp.put("Slot 3: " + getColorString(slot3Green), 120D);
        } else if(scanPos==240) {
            temp.put("Slot 1: " + getColorString(slot1Green), 0D);
            temp.put("Slot 2: " + getColorString(slot2Green), 120D);
            temp.put("Slot 3: " + getColorString(slot3Green), 240D);
        }
        numarr[0]=slot1Green;
        numarr[1]=slot2Green;
        numarr[2]=slot3Green;
        spinStates=temp;

    }
    public boolean atTarget(){
        return Math.abs(normalizeAngleDeg(getTargetDeg()-getPositionDeg()))<8;
    }

    public double error(){
        return normalizeAngleDeg(getTargetDeg()-getPositionDeg());
    }


    public String getArrString(){
        return Arrays.toString(numarr);
    }
    public int[] getArr(){
        return numarr;
    }
    public double closer(double org,String key){
        if(spinStates.containsKey(key)&&Math.min(
                Math.abs(spinStates.get(key)-getPositionDeg()),
                Math.abs(spinStates.get(key)-getPositionDeg()+360)
        )<Math.min(
                Math.abs(org-getPositionDeg()),
                Math.abs(org-getPositionDeg()+360)
        )
        ){
            org = spinStates.get(key);
        }
        return org;
    }
    public double spinTo(String x,double time,boolean remove){

        double closesetBall =10000;

        closesetBall = closer(closesetBall,"Slot 1: "+x);
        closesetBall = closer(closesetBall,"Slot 2: "+x);
        closesetBall = closer(closesetBall,"Slot 3: "+x);


        if (closesetBall ==10000){
            x=(x.equals("GREEN"))?"PURPLE":"GREEN";
            closesetBall = closer(closesetBall,"Slot 1: "+x);
            closesetBall = closer(closesetBall,"Slot 2: "+x);
            closesetBall = closer(closesetBall,"Slot 3: "+x);
        }

        if(closesetBall<1000){
            return closesetBall;
        }
        return getPositionDeg();

    }
//    public void spinTo(double time){
//        double closestBall =10000;
//        String closestBallName="";
//        for (String i:spinStates.keySet()){
//            if (!i.isEmpty()) {
//                if (Math.min(
//                        Math.abs(spinStates.get(i) - getPositionDeg()),
//                        Math.abs(spinStates.get(i) - getPositionDeg() + 360)
//                ) < Math.min(
//                        Math.abs(closestBall - getPositionDeg()),
//                        Math.abs(closestBall - getPositionDeg() + 360)
//                )
//                ) {
//                    closestBall = spinStates.get(i);
//                    closestBallName=i;
//                }
//            }
//        }
//        if(closestBall<1000){
//            setTargetDeg(closestBall,time);
//        }
//    }
    public void removeBall(String x){
        spinStates.remove(x);//temp
    }
    public void score(){
        kicker.setPosition(kickerUp);
    }
    public void retract(){
        kicker.setPosition(kickerDown);
    }






    public class MotionProfile {
        public double maxVel;    // deg/sec
        public double maxAccel;  // deg/sec^2

        private double target;
        private double start;
        private double direction;
        private double distance;

        private double tAccel, tCruise, tDecel, totalTime;
        private double vCruise;

        private double startTime;

        public MotionProfile(double maxVel, double maxAccel) {
            this.maxVel = maxVel;
            this.maxAccel = maxAccel;
        }

        public void startProfile(double currentDeg, double targetDeg, double nowTime) {
            start = currentDeg;
            target = targetDeg;

            double error = targetDeg - currentDeg;
            error = normalizeAngleDeg(error);

            direction = Math.signum(error);
            distance = Math.abs(error);

            // Compute motion phases:
            tAccel = maxVel / maxAccel;
            double distAccel = 0.5 * maxAccel * tAccel * tAccel;

            if (2 * distAccel > distance) {
                tAccel = Math.sqrt(distance / maxAccel);
                tCruise = 0;
                tDecel = tAccel;
                vCruise = maxAccel * tAccel;
            } else {
                double distCruise = distance - 2 * distAccel;
                tCruise = distCruise / maxVel;
                tDecel = tAccel;
                vCruise = maxVel;
            }

            totalTime = tAccel + tCruise + tDecel;
            startTime = nowTime;
        }

        public double getPosition(double nowTime) {
            double t = nowTime - startTime;

            if (t < 0) t = 0;
            if (t > totalTime) return target;

            if (t < tAccel) {
                return start + direction * (0.5 * maxAccel * t * t);
            }
            else if (t < tAccel + tCruise) {
                double t2 = t - tAccel;
                return start +
                        direction * (0.5 * maxAccel * tAccel * tAccel + vCruise * t2);
            }
            else {
                double t2 = t - (tAccel + tCruise);
                double distBeforeDecel =
                        0.5 * maxAccel * tAccel * tAccel + vCruise * tCruise;
                double decelDist = vCruise * t2 - 0.5 * maxAccel * t2 * t2;
                return start + direction * (distBeforeDecel + decelDist);
            }
        }

        public double getVelocity(double nowTime) {
            double t = nowTime - startTime;

            if (t < 0) return 0;
            if (t > totalTime) return 0;

            if (t < tAccel)
                return direction * (maxAccel * t);
            else if (t < tAccel + tCruise)
                return direction * vCruise;
            else {
                double t2 = t - (tAccel + tCruise);
                return direction * (vCruise - maxAccel * t2);
            }
        }
    }

}
