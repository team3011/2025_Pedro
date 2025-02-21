package pedroPathing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Odometry {
    private Servo odoServo;
    public static double odoUpPos = .1;
    public static double odoDownPos = .7;

    public Odometry(HardwareMap hardwareMap){
        this.odoServo = hardwareMap.get(Servo.class, "odoServo");
    }

    public void odoUp(){
        odoServo.setPosition(odoUpPos);
    }

    public void odoDown(){
        odoServo.setPosition(odoDownPos);
    }
}
