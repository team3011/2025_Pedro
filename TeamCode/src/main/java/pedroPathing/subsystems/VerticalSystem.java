package pedroPathing.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class VerticalSystem {
    //@Config variables
    public static int Height_clip = 450;
    public static int Height_clipClip = 290;
    public static int Height_transfer = 40;
    public static int Height_prepToLift = 175;
    public static int Height_basket = 940;
    public static int Height_stow = 300;
    public static int Height_lift = 100;
    public static int gripPause = 500;
    static int gripperState = -1; //0 means closed, 1 means open
    static int gripperTarget = -1;
    static ElapsedTime runtime = new ElapsedTime();
    boolean waitForClip = false;
    boolean autoWait= false;


    static VerticalSliders verticalSliders;
    static VerticalFlipper verticalFlipper;
    static VerticalGripper verticalGripper;

    Telemetry dashboardTelemetry;


    public VerticalSystem (@NonNull HardwareMap hardwareMap, Telemetry db) {
        verticalSliders = new VerticalSliders(hardwareMap, db);
        verticalFlipper = new VerticalFlipper(hardwareMap);
        verticalGripper = new VerticalGripper(hardwareMap);
        dashboardTelemetry = db;
    }

    public void prepToBasket(){
        verticalSliders.setPosition(Height_basket);
        verticalFlipper.goToDropOffBasket();
        gripperTarget = 0;
    }

    public void prepToStow(){
        verticalSliders.setPosition(Height_stow);
        gripperTarget = 0;
    }

    public void prepToTransfer(){
        verticalSliders.setPosition(Height_transfer);
        verticalFlipper.goToPickUp();
        gripperTarget = 1;
    }

    public void goHome(){
        verticalSliders.setPosition(-99);
        verticalFlipper.goToPickUp();
        gripperTarget = 1;
        verticalSliders.setKG(.015);
        verticalSliders.addPowerForLift(false);
    }

    public void prepToLift(){
        verticalSliders.setPosition(Height_prepToLift);
        verticalFlipper.goToPickUp();
    }

    public void prepToClip(){
        verticalSliders.setPosition(Height_clip);
        verticalFlipper.goToDropOffClip();
        gripperTarget = 0;
    }

    public void setAutoWait(boolean b){
        autoWait = b;
    }

    public void clipClip(){
        verticalSliders.setPosition(Height_clipClip);
        waitForClip = true;
    }

    public void lift(){
        verticalSliders.addPowerForLift(true);
        verticalSliders.setPosition(Height_lift);
    }

    public void openGripper(){
        gripperTarget = 1;
    }

    public void closeGripper(){
        gripperTarget = 0;
    }

    public void update(){
        if (waitForClip && Math.abs(verticalSliders.getPositionMillimeter()-Height_clipClip) < 10) {
            gripperTarget = 1;
            waitForClip = false;
        }

        if (gripperState != gripperTarget){
            runtime.reset();
            if (gripperTarget == 1) {
                verticalGripper.goToRelease();
                gripperState = 1;
            } else {
                verticalGripper.goToHold();
                gripperState = 0;
            }
        }
        if (runtime.milliseconds() > gripPause || autoWait) {
            verticalSliders.update();
        }
        //dashboardTelemetry.addData("gripper state", gripperState);
    }
}
