package pedroPathing.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class VerticalGripper {
    private ServoImplEx gripper;
    private double min = -135;
    private double max = 135;
    public static double holdPos = -50;
    public static double releasePos = 20;

    public VerticalGripper(@NonNull HardwareMap hardwareMap){
        this.gripper = hardwareMap.get(ServoImplEx.class,"vgripper");
        this.gripper.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void goToHold(){
        setPosition(holdPos);
    }

    public void goToRelease(){
        setPosition(releasePos);
    }

    private double setPosition(double input){
        input = map(input,min,max,0,1);
        this.gripper.setPosition(input);
        return input;
    }

    private double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
