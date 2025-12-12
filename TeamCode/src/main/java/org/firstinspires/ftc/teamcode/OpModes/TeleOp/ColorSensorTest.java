package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Config
@TeleOp
public class ColorSensorTest extends LinearOpMode {

    RevColorSensorV3 sensor1,sensor2,sensor3;
    public static int slot=1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        sensor1=hardwareMap.get(RevColorSensorV3.class,"slot1");
        sensor2=hardwareMap.get(RevColorSensorV3.class,"slot2");
        sensor3=hardwareMap.get(RevColorSensorV3.class,"slot3");
        waitForStart();
        while (opModeIsActive()){
            RevColorSensorV3 sensor;
            if(slot==1)sensor=sensor1;
            else if(slot==2)sensor=sensor2;
            else if(slot==3)sensor=sensor3;
            else sensor =sensor1;

            double alpha = sensor.alpha();
            double red = (alpha!=0)?sensor.red()/alpha:sensor.red();
            double blue = (alpha!=0)?sensor.blue()/alpha:sensor.blue();
            double green = (alpha!=0)?sensor.green()/alpha:sensor.green();

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
            String ball="";
            if (inRange(red,.67,.8)&&inRange(blue,1.1,1.5)&&inRange(green,.75,1.17)){
                ball="Purple";
            }
            else if (inRange(red,.3,.62)&&inRange(blue,.95,1.23)&&inRange(green,1.2,1.6)){
                ball="Green";
            }
            else {
                ball="Nothing";
            }

            telemetry.addData("red: ",red);
            telemetry.addData("blue: ",blue);
            telemetry.addData("green: ",green);
            telemetry.addData("ball: ",ball);
            telemetry.update();


        }

    }
    private boolean inRange(double x,double min,double max){
        return x>=min&&x<=max;
    }
}
