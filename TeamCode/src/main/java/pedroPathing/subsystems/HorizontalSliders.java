package pedroPathing.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class HorizontalSliders {
    Telemetry dashboardTelemetry;
    private final DcMotorEx leftMotor;
    private int lastPosition = 0;
    private int targetPosition = 0;
    private final PIDController controller;
    public static double kP = 0.04;
    public static double kI = 0;
    public static double kD = 0;
    public static float convertTicksToMillimeters = .4054f; // 225mm/1000ticks = .225
    private boolean resetFlag = false;
    public static int maximumMilliamps = 3500;
    public static double maxPower = .8;
    public static double minimumSpeed = 0.1;
    private boolean goingUp = false;
    private boolean holdingPosition = true;
    private int targetPositionMM;
    boolean manualMove = false;

    public HorizontalSliders(@NonNull HardwareMap hardwareMap, Telemetry db){
        this.leftMotor = hardwareMap.get(DcMotorEx.class, "horLeft");
        this.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.controller = new PIDController(kP, kI, kD);
        dashboardTelemetry = db;

    }

    public void manualInput(double input){
        manualMove = true;
        if (getPositionMM() > 400 && input > 0) {
            this.leftMotor.setPower(0);
        } else {
            this.leftMotor.setPower(input);
        }

        targetPositionMM = 999;
    }

    public int getPosition(){
        return this.leftMotor.getCurrentPosition();
    }

    public int getPositionMM(){
        return Math.round(leftMotor.getCurrentPosition()*convertTicksToMillimeters);
    }

    public void setPosition(int mm){
        manualMove = false;
        //are we trying to move?
        if (this.targetPositionMM != mm) {
            this.holdingPosition = false;
            //when we go down we need to reset
            if (mm > 0) {
                this.resetFlag = true;
            }

            if (mm - this.targetPosition > 0) {
                this.goingUp = true;
            } else {
                this.goingUp = false;
            }

            if (mm > 400) {
                mm = 400;
            }

            this.targetPositionMM = mm;
            // mm divided by ratio = mm multiplied by ratio^-1
            // ^ for my sanity
            float notInteger = mm / convertTicksToMillimeters;
            mm = Math.round(notInteger);
            this.targetPosition = mm;
        }
    }

    /**
     * this will limit the input to a range of -limiter to limiter
     * @param input the value to be limited
     * @param limiter the max value the input can be
     * @return the limited input
     */
    private double limiter(double input, double limiter){
        if (input > limiter) {
            input = limiter;
        } else if (input < -limiter) {
            input = -limiter;
        }
        return input;
    }

    public double getCurrent(){
        return Math.round(this.leftMotor.getCurrent(CurrentUnit.MILLIAMPS)/10)*10;
    }

    public void update(){
        if (!this.holdingPosition && !manualMove) {
            double pid;
            //get the current position of the sliders
            int currentPosition = getPosition();

            //calc the speed the motor should move
            this.controller.setPID(kP,kI,kD);
            pid = this.controller.calculate(currentPosition, this.targetPosition);
            pid = limiter(pid, maxPower);
            lastPosition = currentPosition;


            if (targetPosition != 0 && Math.abs(pid)< minimumSpeed && Math.abs(currentPosition-lastPosition)<10) {
                this.holdingPosition = true;
                pid = 0;
            }


            if (this.resetFlag && !this.goingUp && currentPosition < 100) {
                if (this.getCurrent() > maximumMilliamps){
                    this.resetFlag = false;
                    this.holdingPosition = true;
                    pid = 0;
                    this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    this.targetPosition = 0;
                } else {
                    pid = -0.4;
                }
            }

            this.leftMotor.setPower(pid);

            //dashboardTelemetry.addData("hor-current position in ticks", currentPosition);
            //dashboardTelemetry.addData("hor-pid output", pid);
        }

//        dashboardTelemetry.addData("where am i?",getPositionMM());
//        dashboardTelemetry.addData("target?",targetPositionMM);
//        dashboardTelemetry.addData("hor-reset Flag", resetFlag);
//        dashboardTelemetry.addData("hor-going out", goingUp);
//        dashboardTelemetry.addData("hor-milliamps", this.getCurrent());
//        dashboardTelemetry.addData("hor-holding pos", this.holdingPosition);
//        dashboardTelemetry.addData("hor-target position in ticks", this.targetPosition);
//        dashboardTelemetry.update();
    }
}
