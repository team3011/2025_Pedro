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
public class AutoRedClip extends OpMode {
    SuperSystem superSystem;
    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    ElapsedTime clipTimer = new ElapsedTime();
    public static int clipTime = 1000;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(134.5, 81, Math.toRadians(180));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(110, 81, Math.toRadians(180));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain move1, move2, move3, scorePickup1, scoreDropOff1, scorePickup2, scoreDropOff2, scorePickup3, scoreDropOff3;

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
        scorePreload = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());


        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        move1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(110.000, 81.000, Point.CARTESIAN),
                                new Point(129.350, 141.669, Point.CARTESIAN),
                                new Point(74.414, 82.072, Point.CARTESIAN),
                                new Point(57.267, 123.690, Point.CARTESIAN),
                                new Point(87.066, 120.694, Point.CARTESIAN),
                                new Point(130.000, 120.028, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        move2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(130.000, 120.028, Point.CARTESIAN),
                                new Point(70.086, 94.557, Point.CARTESIAN),
                                new Point(68.587, 150.000, Point.CARTESIAN),
                                new Point(104.879, 125.688, Point.CARTESIAN),
                                new Point(130.183, 129.850, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        move3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(130.183, 129.850, Point.CARTESIAN),
                                new Point(78.000, 118.000, Point.CARTESIAN),
                                new Point(76.245, 132.180, Point.CARTESIAN),
                                new Point(65.000, 139.000, Point.CARTESIAN),
                                new Point(130.682, 133.679, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(130.682, 133.679, Point.CARTESIAN),
                                new Point(121.193, 120.361, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        scoreDropOff1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(121.193, 120.361, Point.CARTESIAN),
                                new Point(110.000, 79.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(110, 79, Point.CARTESIAN),
                                new Point(121.193, 120.361, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        scoreDropOff2 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(121.193, 120.361, Point.CARTESIAN),
                                new Point(110.000, 77.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(110, 77, Point.CARTESIAN),
                                new Point(121.193, 120.361, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        scoreDropOff3 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(121.193, 120.361, Point.CARTESIAN),
                                new Point(110.000, 75.000, Point.CARTESIAN)
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
                superSystem.prepToDropOff();
                follower.followPath(scorePreload);
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
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(move1,false);
                    setPathState(3);
                }
                break;
            case 3:
                //move block 2
                if(!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(move2, false);
                    setPathState(4);
                }
                break;
            case 4:
                //move block 3
                if(!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(move3, false);
                    setPathState(5);
                }
                break;
            case 5:
                //move to pick up block 1
                if(!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup1, false);
                    setPathState(6);
                }
                break;
            case 6:
                //look and pickup block 1
                superSystem.scan(0);
                superSystem.setXRdy();
                setPathState(7);
                break;
            case 7:
                //move to dropoff 1
                if (superSystem.isRdyToMove()) {
                    follower.followPath(scoreDropOff1, false);
                    setPathState(8);
                }
                break;
            case 8:
                //clip it 1
                if(!follower.isBusy()) {
                    superSystem.dropOff();
                    clipTimer.reset();
                    setPathState(9);
                }
                break;
            case 9:
                //move to pick up block 2
                if(!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup2, false);
                    setPathState(10);
                }
                break;
            case 10:
                //look and pickup block 2
                superSystem.scan(0);
                superSystem.setXRdy();
                setPathState(11);
                break;
            case 11:
                //move to dropoff 2
                if (superSystem.isRdyToMove()) {
                    follower.followPath(scoreDropOff2, false);
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
                if(!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup3, false);
                    setPathState(14);
                }
                break;
            case 14:
                //look and pickup block 3
                superSystem.scan(0);
                superSystem.setXRdy();
                setPathState(15);
                break;
            case 15:
                //move to dropoff 3
                if (superSystem.isRdyToMove()) {
                    follower.followPath(scoreDropOff3, false);
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
        superSystem = new SuperSystem(hardwareMap,dashboardTelemetry);
        superSystem.setAlliance(false);
        superSystem.setToggleStateAuto();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
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
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        superSystem.update();

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }



    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

