package pedroPathing.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.subsystems.SuperSystem;
import pedroPathing.subsystems.VerticalGripper;

//@Config       //if you want config
@TeleOp       //if this is a teleop
//@Autonomous   //if this is an auto
public class Subsystem_Test extends OpMode{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    SuperSystem superSystem;
    DcMotorSimple bottomSlider;


    VerticalGripper verticalGripper;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        dashboardTelemetry.addData("Status", "Initialized");
        superSystem = new SuperSystem(hardwareMap,dashboardTelemetry, 0);
        //bottomSlider = hardwareMap.get(DcMotorSimple.class, "bottomSlider");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step
        verticalGripper = new VerticalGripper(hardwareMap);
        superSystem.setToggleState(0);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        superSystem.start();
        superSystem.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
//        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
//        verticalGripper.goToHold();
//        ElapsedTime timer = new ElapsedTime();
//        while (timer.milliseconds() < 1000) {}
//
//        verticalGripper.goToRelease();
//        timer.reset();
//        while (timer.milliseconds() < 1000) {}

        superSystem.headlightsOn();
//        superSystem.prepToDropOff();
        superSystem.update();
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}