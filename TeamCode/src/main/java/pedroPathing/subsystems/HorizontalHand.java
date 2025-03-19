package pedroPathing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class HorizontalHand {
    private final Servo wrist;
    private final Servo hand;
    private final Servo finger;
    public static double par = .79;
    public static double perp = .46;
    public static double open = .25;
    public static double closed = .6;
    public static double down = 0.03;
    public static double up = .68;
    public static double limit = 0.20;

    public HorizontalHand(HardwareMap hardwareMap, boolean isAuto){
        this.wrist = hardwareMap.get(Servo.class, "wrist");
        this.hand = hardwareMap.get(Servo.class, "clawServo");
        this.finger = hardwareMap.get(Servo.class, "paulFinger");
        if (isAuto){
            open = 0.15;
        }
    }

    public void openClaw(){
        finger.setPosition(open);
    }

    public void closeClaw(){
        finger.setPosition(closed);
    }

    public void handPar(){
        hand.setPosition(par);
    }

    public void wristTransfer(){
        wrist.setPosition(up);
    }

    public void wristPickup(){
        wrist.setPosition(down);
    }


    //input will be -90 to 80 with 0 being perp
    public void setPositionByCamera(double input){
        //perp and par = 90
        //.79-.46 = servo per 90
        double servoPerDeg = 0.0036666666666667;
        double output = perp - servoPerDeg*input;
        if (output < limit) {
            output = limit;
        }
        hand.setPosition(output);
    }

    public void setPosition(){
        hand.setPosition(par);
    }
}
