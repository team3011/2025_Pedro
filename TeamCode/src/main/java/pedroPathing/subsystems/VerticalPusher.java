package pedroPathing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class VerticalPusher {
    DcMotorEx max;
    public static double maxPower = .2;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static int outTicks = 100;
    private int targetPosition = 0;
    //private final PIDController controller;

    public VerticalPusher(HardwareMap hardwareMap){
        //max = hardwareMap.get(DcMotorEx.class, "vpush");
        //max.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //max.setDirection(DcMotorSimple.Direction.REVERSE);
        //max.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //max.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //controller = new PIDController(kP, kI, kD);
        //max.setPower(0);
    }

    public void goOut(){
        targetPosition = outTicks;
    }

    public void goIn(){
        targetPosition = 0;
    }

    public void update(){
        //calc the speed the motor should move
        //controller.setPID(kP,kI,kD);
        //double pid = this.controller.calculate(max.getCurrentPosition(), this.targetPosition);
        //pid = limiter(pid, maxPower);
        //max.setPower(pid);
        //050980669 - dont use this number Fabio
    }

    private double limiter(double input, double limiter){
        if (input > limiter) {
            input = limiter;
        } else if (input < -limiter) {
            input = -limiter;
        }
        return input;
    }
}
