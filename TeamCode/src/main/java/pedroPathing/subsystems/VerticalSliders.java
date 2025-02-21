package pedroPathing.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//read this to learn about PID and feedforward
//https://www.ctrlaltftc.com/the-pid-controller

@Config
public class VerticalSliders {
    private DcMotorEx rightMotor;
    private DcMotorEx leftMotor;
    private int lastPosition = 0;
    private int targetPosition = 0;
    private PIDController controller;
    public static double kP = 0.015;
    public static double kI = 0.001;
    public static double kD = 0.0003;
    public static double kG = 0.015;
    public static float convertTicksToMillimeters = .225f; // 225mm/1000ticks = .225
    private boolean resetFlag = false;
    public static int maximumMilliamps = 1750;
    public static double maxPower = 1;
    public static double minimumSpeed = 0.1;
    private boolean goingUp = false;
    private boolean holdingPosition = true;
    private int targetPositionMM;
    Telemetry dashboardTelemetry;
    boolean isLifting = false;

    public VerticalSliders(@NonNull HardwareMap hardwareMap, Telemetry db){
        this.rightMotor = hardwareMap.get(DcMotorEx.class,"vertiRight");
        this.rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        dashboardTelemetry = db;

        this.leftMotor = hardwareMap.get(DcMotorEx.class,"vertiLeft");
        this.leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        this.controller = new PIDController(kP, kI, kD);
    }

    public void setKG(double input){
        kG = input;
    }

    public double getCurrent(int motor){
        if (motor == 0) {
            return Math.round(this.leftMotor.getCurrent(CurrentUnit.MILLIAMPS)/10)*10;
        }
        return Math.round(this.rightMotor.getCurrent(CurrentUnit.MILLIAMPS)/10)*10;
    }

    //this should return the current position in mm above starting position
    public int getPositionMillimeter(){
        //creating the result as a float needs to be done
        //since we dont care about precision enough we just use a float :thumbsup:
        float result = this.rightMotor.getCurrentPosition() * convertTicksToMillimeters;
        return Math.round(result);
    }

    public void manualInput(double input){
        this.leftMotor.setPower(input);
        this.rightMotor.setPower(input);
    }

    //returns position in TICKS
    public int getPosition(){
        return this.rightMotor.getCurrentPosition();
    }

    public int getTargetPosition() {return this.targetPosition;}

    //mm is the distance in millimeters you want the sliders to move
    public void setPosition(int mm){
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

            if(mm == -99 ){
                this.resetFlag = true;
                this.holdingPosition = false;
            }

            if (mm > 940) {
                mm = 940;
            }

            this.targetPositionMM = mm;
            // mm divided by ratio = mm multiplied by ratio^-1
            // ^ for my sanity
            float notInteger = mm / convertTicksToMillimeters;
            mm = Math.round(notInteger);
            this.targetPosition = mm;
        }
    }

    public void update(){
        if (!this.holdingPosition || isLifting) {
            double pid = 0;
            //get the current position of the sliders
            int currentPosition = getPosition();

            //calc the speed the motor should move
            this.controller.setPID(kP,kI,kD);
            pid = this.controller.calculate(currentPosition, this.targetPosition);
            pid = limiter(pid, maxPower);
            pid += VerticalSliders.kG;
            lastPosition = currentPosition;

            if (!isLifting) {
                if (targetPosition != 0 && Math.abs(pid) < minimumSpeed && Math.abs(currentPosition - lastPosition) < 10) {
                    this.holdingPosition = true;
                    if (targetPosition > 100) {
                        pid = VerticalSliders.kG;
                    }
                }

                if (this.resetFlag && !this.goingUp && currentPosition < 400) {
                    if (this.getCurrent(1) > maximumMilliamps) {
                        this.resetFlag = false;
                        this.holdingPosition = true;
                        pid = 0;
                        this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        this.targetPosition = 0;
                    } else {
                        pid = -0.4;
                    }
                }
            }

            this.rightMotor.setPower(pid);
            this.leftMotor.setPower(pid);

            dashboardTelemetry.addData("vert-current position in ticks", currentPosition);
            dashboardTelemetry.addData("vert-last position in ticks", lastPosition);
            dashboardTelemetry.addData("vert-pid output", pid);
        }

//        dashboardTelemetry.addData("vert-reset Flag", resetFlag);
        dashboardTelemetry.addData("vert-going up", goingUp);
//        dashboardTelemetry.addData("vert-milliamps", this.getCurrent(1));
//        dashboardTelemetry.addData("vert-holding pos", this.holdingPosition);
        dashboardTelemetry.addData("vert-target position in ticks", this.targetPosition);
//        dashboardTelemetry.addData("check1", check1);
//        dashboardTelemetry.addData("check2", check2);
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

    public void addPowerForLift(boolean b) {
        isLifting = b;
    }
}
