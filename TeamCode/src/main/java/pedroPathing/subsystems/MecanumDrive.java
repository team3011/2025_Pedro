package pedroPathing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import javax.annotation.Nonnull;

// referenced https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

@Config
public class MecanumDrive {
    public static double correctionMultiplier = 1;
    public static double ANGULAR_TOLERANCE_DEGREES = 2;
    public static double rotMulti = 1.1;
    private final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public GoBildaPinpointDriver pinpoint;
    private double headingToMaintain = 0;
    //limits how fast human can rotate robot, decrease to slow down rotation
    public double rotation_multi = 0.5;
    //acceptable angle tolerance in radians
    public double ANGULAR_TOLERANCE = Math.PI/90;

    public MecanumDrive(@Nonnull HardwareMap hardwareMap){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // TODO: reverse motor directions if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    //***************************************************
    //pulled from 2024 drive code to integrate Pinpoint
    //***************************************************

    //returns the current heading of the robot in RAD
    public double calcYaw() {
        pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        return Math.round(pinpoint.getHeading()*10)/10.0;
    }

    //determines the shortest path to desired angle in degrees
    public double figureOutWhatIsShorter(double reading) {
        double result;
        double oppositeButEqualReading;

        if (reading > 0) {
            oppositeButEqualReading = reading - 360;
        } else {
            oppositeButEqualReading = reading + 360;
        }

        double normalReadingDifference = Math.abs(this.headingToMaintain - reading);
        double oppositeReadingDifference = Math.abs(this.headingToMaintain - oppositeButEqualReading);
        boolean isOppositeReadingShorter =
                normalReadingDifference > oppositeReadingDifference;

        if (isOppositeReadingShorter) {
            result = this.headingToMaintain - oppositeButEqualReading;
        } else {
            result = this.headingToMaintain - reading;
        }
        return -result;
    }

    //return a value that is below or equal to the limit
    private double limiter(double input, double lim) {
        //this will limit the pid to a range of -1 to 1
        if (input > lim) {
            input = lim;
        } else if (input < -lim) {
            input = -lim;
        }
        return input;
    }

    public void setHeadingToMaintain(double input){
        this.headingToMaintain = input;
    }


    public void drive2(double x, double y, double rx){
        x = -x;
        y = -y;
        double robotHeadingRAD = calcYaw();
        double robotHeadingDEG = Math.toDegrees(robotHeadingRAD);

        if(rx == 0){
            //we're trying to maintain our current heading
            //calc the shortest deviation to target heading in degrees
            double shorter = this.figureOutWhatIsShorter(robotHeadingDEG);
            //check if we are within tolerance
            boolean isWithinAngularTolerance =
                    Math.abs(shorter) < ANGULAR_TOLERANCE_DEGREES;

            //we turn if we're not within a tolerance
            if(!isWithinAngularTolerance){
                //this means we are moving
                if (Math.abs(y) > 0 || Math.abs(x) > 0) {
                    double rotSpeed = Math.abs(shorter);
                    if (rotSpeed > 20) {
                        rotSpeed = 1;
                    } else {
                        rotSpeed = correctionMultiplier * rotSpeed * rotSpeed / 800.0;
                    }
                    rx = limiter(shorter, rotSpeed);
                } else {
                    //this means we are not moving but not pointing in the right direction
                    double rotSpeed = Math.abs(shorter);
                    if (rotSpeed > 20) {
                        rotSpeed = 1;
                    } else {
                        rotSpeed = 2 * correctionMultiplier * rotSpeed * rotSpeed / 800.0;
                    }
                    rx = limiter(shorter, rotSpeed);
                }
            }
        }else{
            //we're going to maintain our new heading once we've stopped turning.
            //not before we've turned
            this.headingToMaintain = robotHeadingDEG;
        }
        //triangle """magic"""
        double rotX = x * Math.cos(-robotHeadingRAD) - y * Math.sin(-robotHeadingRAD);
        double rotY = x * Math.sin(-robotHeadingRAD) + y * Math.cos(-robotHeadingRAD);
        rotX = rotX * rotMulti;

        //double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        //double frontLeftPower = (y+x+rx) / denominator;
        //double backLeftPower = (y - x + rx) / denominator;
        //double frontRightPower = (y-x-rx) / denominator;
        //double backRightPower = (y + x - rx) / denominator;

        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx)  / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX + rx)   / denominator;
        double backRightPower = (rotY + rotX - rx)  / denominator;

        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
    }

    private double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public double getHeadingToMaintain(){
        return headingToMaintain;
    }

}
