package pedroPathing.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.subsystems.Odometry;
import pedroPathing.subsystems.SuperSystem;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Config
@Autonomous
public class AutoRedClip3 extends OpMode {
    Odometry odometry;
    SuperSystem superSystem;
    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    ElapsedTime clipTimer = new ElapsedTime();
    public static int clipTime = 350;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static double humanX = 123;
    public static double humanY = 120.861;
    public static double yOffset = 1;
    public static double xOffset = 5;
    public static double clipX = 106.5;
    public static double clipY = 81;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     */



    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreload, move1, move2, move3, scorePickup1, scoreDropOff1, scorePickup2, scoreDropOff2, scorePickup3, scoreDropOff3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                // Line 1
                .addPath(
                        new BezierLine(
                                new Point(134.5, 81, Point.CARTESIAN),
                                new Point(clipX, clipY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        move1 = follower.pathBuilder()
                // Line 2
                .addPath(
                        new BezierCurve(
                                new Point(106.500, 81.000, Point.CARTESIAN),
                                new Point(126.400, 95.111, Point.CARTESIAN),
                                new Point(128.178, 104.356, Point.CARTESIAN),
                                new Point(105.067, 108.267, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                //Line 3
                .addPath(
                        new BezierCurve(
                                new Point(105.067, 108.267, Point.CARTESIAN),
                                new Point(76.978, 106.489, Point.CARTESIAN),
                                new Point(81.778, 115.733, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                // Line 4
                .addPath(
                        new BezierCurve(
                                new Point(81.778, 115.733, Point.CARTESIAN),
                                new Point(81.067, 120.533, Point.CARTESIAN),
                                new Point(124.800-xOffset, 120.711, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                // Line 5
                .addPath(
                        new BezierCurve(
                                new Point(124.800-xOffset, 120.711, Point.CARTESIAN),
                                new Point(85.156, 112.356, Point.CARTESIAN),
                                new Point(82.133, 125.333, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                // Line 6
                .addPath(
                        new BezierCurve(
                                new Point(82.133, 125.333, Point.CARTESIAN),
                                new Point(90.311, 134.400, Point.CARTESIAN),
                                new Point(124.800-xOffset, 128.533, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                // Line 7
//                .addPath(
//                        new BezierCurve(
//                                new Point(124.800-xOffset, 128.533, Point.CARTESIAN),
//                                new Point(88.711, 122.489, Point.CARTESIAN),
//                                new Point(85.156, 134.578-yOffset, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                // Line 8
//                .addPath(
//                        new BezierCurve(
//                                new Point(85.156, 134.578-yOffset, Point.CARTESIAN),
//                                new Point(103.822, 134.400-yOffset, Point.CARTESIAN),
//                                new Point(124.800-xOffset, 134.578-yOffset, Point.CARTESIAN)
//                        )
//                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                // Line 9
                .addPath(
                        new BezierLine(
                                new Point(124.800-xOffset, 128.533, Point.CARTESIAN),
                                new Point(humanX, humanY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        move2 = follower.pathBuilder()
                .addPath(
                        /*
                        new BezierCurve(
                                new Point(130.000, 120.028, Point.CARTESIAN),
                                new Point(70.086, 94.557+yOffset, Point.CARTESIAN),
                                new Point(68.587, 150.000+yOffset, Point.CARTESIAN),
                                new Point(104.879, 125.688+yOffset, Point.CARTESIAN),
                                new Point(130.183, 129.850+yOffset, Point.CARTESIAN)
                        ) */
                        new BezierCurve(
                                new Point(129.778, 119.644, Point.CARTESIAN),
                                new Point(39.822, 104.533, Point.CARTESIAN),
                                new Point(115.022, 139.556, Point.CARTESIAN),
                                new Point(52.622, 130.667, Point.CARTESIAN),
                                new Point(131.556, 131.733, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        move3 = follower.pathBuilder()
                .addPath(
                        /*
                        new BezierCurve(
                                new Point(130.183, 129.850+yOffset, Point.CARTESIAN),
                                new Point(78.000, 118.000+yOffset, Point.CARTESIAN),
                                new Point(76.245, 132.180+yOffset, Point.CARTESIAN),
                                new Point(65.000, 139.000+yOffset, Point.CARTESIAN),
                                new Point(130.682, 133.679+yOffset, Point.CARTESIAN)
                        ) */
                        new BezierCurve(
                                new Point(131.556, 131.733, Point.CARTESIAN),
                                new Point(72.711, 97.244, Point.CARTESIAN),
                                new Point(59.911, 140.622, Point.CARTESIAN),
                                new Point(93.333, 132.089, Point.CARTESIAN),
                                new Point(60.800, 136.711, Point.CARTESIAN),
                                new Point(130.682, 133.679, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(130.682, 133.679, Point.CARTESIAN),
                                new Point(103.214, 119.195, Point.CARTESIAN),
                                new Point(humanX, humanY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        scoreDropOff1 = follower.pathBuilder()
                // Line 10
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(humanX, humanY, Point.CARTESIAN),
                                new Point(131.514, 86.899, Point.CARTESIAN),
                                new Point(clipX, clipY-3, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        scorePickup2 = follower.pathBuilder()
                // Line 11
                .addPath(
                        new BezierCurve(
                                new Point(clipX, clipY-3, Point.CARTESIAN),
                                new Point(131.514, 86.899, Point.CARTESIAN),
                                new Point(humanX, humanY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        scoreDropOff2 = follower.pathBuilder()
                .addPath(
                        // Line 12
                        new BezierCurve(
                                new Point(humanX, humanY, Point.CARTESIAN),
                                new Point(131.514, 86.899, Point.CARTESIAN),
                                new Point(clipX-1, clipY-5, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        scorePickup3 = follower.pathBuilder()
                // Line 13
                .addPath(
                        new BezierCurve(
                                new Point(clipX-1, clipY-5, Point.CARTESIAN),
                                new Point(131.514, 86.899, Point.CARTESIAN),
                                new Point(humanX, humanY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        scoreDropOff3 = follower.pathBuilder()
                // Line 14
                .addPath(
                        new BezierCurve(
                                new Point(humanX, humanY, Point.CARTESIAN),
                                new Point(131.514, 86.899, Point.CARTESIAN),
                                new Point(clipX, clipY-7, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                //move to clip dropoff and prep to clip
                follower.followPath(scorePreload,true);
                superSystem.prepToDropOff();
                setPathState(1);
                break;
            case 1:
                //clip it
                if(!follower.isBusy()) {
                    superSystem.dropOff();
                    clipTimer.reset();
                    setPathState(2);
                }
                break;
            case 2:
                //move block 1
                if (clipTimer.milliseconds() > clipTime) {
                    follower.followPath(move1,true);
                    superSystem.reset();
                    setPathState(6);
                    superSystem.setAutoWait(false);
                }
                break;
            case 3:
                //move block 2
                if(!follower.isBusy()) {
                    follower.followPath(move2,.8, false);
                    setPathState(4);
                }
                break;
            case 4:
                //move block 3
                if(!follower.isBusy()) {
                    follower.followPath(move3,.8, false);
                    setPathState(5);
                }
                break;
            case 5:
                //move to pick up block 1
                if(!follower.isBusy()) {
                    superSystem.setAutoWait(false);
                    follower.followPath(scorePickup1, true);
                    setPathState(66);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    //look and pickup block 1
                    superSystem.scan(0);
                    superSystem.setXRdy();
                    setPathState(7);
                }
                break;
            case 7:
                //move to dropoff 1
                if (superSystem.isRdyToMove()) {
                    follower.followPath(scoreDropOff1,.85, true);
                    setPathState(8);

                }
                break;
            case 8:
                //clip it 1
                if(!follower.isBusy()) {
                    superSystem.setAutoWait(true);
                    superSystem.dropOff();
                    clipTimer.reset();
                    setPathState(9);
                }
                break;
            case 9:
                //move to pick up block 2
                if (clipTimer.milliseconds() > clipTime && !follower.isBusy()) {
                    superSystem.setAutoWait(false);
                    superSystem.resetWithArmOut();
                    follower.followPath(scorePickup2, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    //look and pickup block 2
                    superSystem.scan(0);
                    superSystem.setXRdy();
                    setPathState(11);
                }
                break;
            case 11:
                //move to dropoff 2
                if (superSystem.isRdyToMove()) {
                    follower.followPath(scoreDropOff2,.85, true);
                    setPathState(12);
                }
                break;
            case 12:
                //clip it 2
                if(!follower.isBusy()) {
                    superSystem.dropOff();
                    clipTimer.reset();
                    setPathState(13);
                }
                break;
            case 13:
                //move to pick up block 3
                if (clipTimer.milliseconds() > clipTime) {

                    follower.followPath(scorePickup3, true);
                    superSystem.reset();
                    setPathState(140);
                }
                break;
            case 14:
                //look and pickup block 3
                if(!follower.isBusy()) {
                    superSystem.scan(0);
                    superSystem.setXRdy();
                    setPathState(15);
                }
                break;
            case 15:
                //move to dropoff 3
                if (superSystem.isRdyToMove()) {
                    follower.followPath(scoreDropOff3, .85  ,true);
                    setPathState(16);
                }
                break;
            case 16:
                //clip it 3
                if(!follower.isBusy()) {
                    superSystem.dropOff();
                    clipTimer.reset();
                    setPathState(17);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        superSystem = new SuperSystem(hardwareMap,dashboardTelemetry, 1);
        superSystem.setAlliance(false);
        superSystem.setToggleStateAuto();
        odometry = new Odometry(hardwareMap);
        odometry.odoDown();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        superSystem.closeVGripper();
        superSystem.setAutoWait(true);
        superSystem.setIsAutoScan(true);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(134.5,81,Math.toRadians(180)));
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        clipTimer.reset();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        superSystem.update();
        follower.update();
        autonomousPathUpdate();
        dashboardTelemetry.addData("transfer",superSystem.getIsAutoTransferDone());

        // Feedback to Driver Hub
        dashboardTelemetry.addData("path state", pathState);
        dashboardTelemetry.addData("x", follower.getPose().getX());
        dashboardTelemetry.addData("y", follower.getPose().getY());
        dashboardTelemetry.addData("heading", follower.getPose().getHeading());
        dashboardTelemetry.update();
    }



    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

