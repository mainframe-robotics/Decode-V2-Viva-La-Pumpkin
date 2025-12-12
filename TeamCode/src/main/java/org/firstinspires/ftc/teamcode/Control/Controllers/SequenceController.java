package org.firstinspires.ftc.teamcode.Control.Controllers;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.Control.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Control.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Control.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Control.Subsystems.Turret;

import java.util.concurrent.TimeUnit;

public class SequenceController {
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;

    private Intake intake;
    private Follower follower;

    private NanoTimer timer;

    public int shootingState = -1;

    private String[] motifOrder ={"PURPLE","GREEN","PURPLE"} ;

    private Pose goalPose;

    public SequenceController(Transfer transfer,Shooter shooter, Turret turret,Intake intake,Follower follower,Pose goalPose){
        this.transfer = transfer;
        this.shooter = shooter;
        this.turret = turret;
        this.intake = intake;
        this.follower = follower;
        this.goalPose = goalPose;

        timer = new NanoTimer();


    }

    public boolean isBusy(){
        return shootingState!=-1;
    }

    public void setMotifOrder(String[] order){
        motifOrder=order;
    }
    public String[] getMotifOrder(){
        return motifOrder;
    }

    public void start(){
        shootingState=0;
    }
    public void end(){
        shootingState=-1;
    }

    public int getShootingState(){
        return shootingState;
    }



    public void update(double sec,Follower follower){

        this.follower=follower;
        if (shootingState!=-1){

            turret.on();
            intake.setPower(.45);
            double goalDistance = Math.hypot(goalPose.getX()-follower.getPose().getX(),goalPose.getY()-follower.getPose().getY());
            shooter.on();
            shooter.forDistance(goalDistance);
            turret.facePoint(goalPose,follower.getPose());
        }
        switch (shootingState){
            case -1:
                shooter.off();
                turret.off();
                break;
            case 0:
                transfer.scan(sec);
                timer.resetTimer();
                shootingState=30;
                break;
            case 30:
                if(timer.getElapsedTime(TimeUnit.MILLISECONDS)>100){
                    transfer.spinTo(motifOrder[0],sec,true);
                    shootingState=1;
                }
                break;
            case  1:
                if(shooter.atTarget()&& turret.atTarget()&& transfer.atTarget()){
                    transfer.score();
                    timer.resetTimer();
                    shootingState=2;
                }
                break;
            case 2:
                if (timer.getElapsedTime(TimeUnit.MILLISECONDS)>1800){
                    transfer.spinTo(motifOrder[1],sec,true);
                    timer.resetTimer();
                    shootingState=3;
                } else if (timer.getElapsedTime(TimeUnit.MILLISECONDS)>700) {
                    transfer.retract();

                }
                break;
            case  3:
                if(shooter.atTarget()&& turret.atTarget()&& transfer.atTarget()){
                    timer.resetTimer();
                    shootingState=10;
                }
                break;
            case 10:
                if(timer.getElapsedTime(TimeUnit.MILLISECONDS)>300){
                    transfer.score();
                    timer.resetTimer();
                    shootingState=4;
                }
                break;
            case 4:
                if (timer.getElapsedTime(TimeUnit.MILLISECONDS)>1800){
                    transfer.spinTo(motifOrder[2],sec,true);
                    shootingState=5;
                } else if (timer.getElapsedTime(TimeUnit.MILLISECONDS)>700) {
                    transfer.retract();
                }
                break;
            case  5:
                if(shooter.atTarget()&& turret.atTarget()&& transfer.atTarget()){
                    timer.resetTimer();
                    shootingState=20;
                }
                break;
            case 20:
                if(timer.getElapsedTime(TimeUnit.MILLISECONDS)>300){
                    transfer.score();
                    timer.resetTimer();
                    shootingState=6;
                }
            break;
            case 6:
                if (timer.getElapsedTime(TimeUnit.MILLISECONDS)>1800){
//                    transfer.setTargetDeg(0,sec);
                    shooter.off();
                    turret.setYaw(0);
                    shootingState=-1;
                } else if (timer.getElapsedTime(TimeUnit.MILLISECONDS)>700) {
                    transfer.retract();
                }
                break;

        }

        transfer.update(sec);
        shooter.update();
        turret.update();
        follower.update();
    }






    /*
    all possible states:
    PPP
    PPG
    PGG
    GGG
    PPB
    PGB
    GGB
    PBB
    GBB

    if PBB or GBB
        use motif[0]
        finish
    if PPB PGB GGB
        use motif[0]
        use motif[1]
        finish
    if PPP,PPG,PGG,GGG,PPB
        use motif[0]
        use motif[1]
        use motif[2]
        finish
     */







}
