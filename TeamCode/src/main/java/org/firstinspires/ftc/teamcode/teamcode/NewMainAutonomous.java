package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teamcode.drive.CatMecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.teamcode.trajectorysequence.TrajectorySequence;


/**
 * MainAutonomous.java
 *
 *
 * A Linear OpMode class to be an autonomous method for both Blue & Red alliance sides where we pick
 * which side of the alliance bridge we start off at with gamepad1 as well as selecting alliance
 * color and whether we park under the alliance bridge closer or further from the game field wall.
 * Also will detect the position and deliver the skystone using machine vision and move the
 * foundation.
 *
 * Mec_Odo_AutonomousLevel6_Statey is written to use machine vision and SkyStone delivery to our
 * autonomous route with the help intake jaws that suck in a stone at any orientation using a
 * "touch it-own it" approach.  A servo and two motors make up TC-73/Bucky's arm and stack stones as
 * well as our team marker.

 * This autonomous is used for our State Championship(February 7-8, 2020).
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back.
 */

@Autonomous(name="New Auto", group="CatAuto")

public class NewMainAutonomous extends LinearOpMode {

    /* Declare OpMode members. */

    CatHW_Async robot = new CatHW_Async();    // All the hardware classes init here.
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;

    private ElapsedTime runningTime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        robot.init(hardwareMap, this);




        /*
        Init Delay Option Select:
         */

        // After init is pushed but before Start we can change the delay using dpad up/down //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive() && !isStopRequested()) {
            if (this.isStopRequested()) {
                // Leave the loop if STOP is pressed
                return;
            }
            if (gamepad1.dpad_up && (delayTimer.seconds() > 0.8)) {
                // Increases the amount of time we wait
                timeDelay += 1;
                delayTimer.reset();
            }
            if (gamepad1.dpad_down && (delayTimer.seconds() > 0.8)) {
                // Decreases the amount of time we wait
                if (timeDelay > 0) {
                    // No such thing as negative time
                    timeDelay -= 1;
                }
                delayTimer.reset();
            }
            if (((gamepad1.x) && delayTimer.seconds() > 0.5)) {
                // Changes Alliance Sides
                if (robot.isRedAlliance && !robot.isLeftAlliance) {

                    robot.isRedAlliance = true;
                    robot.isLeftAlliance = true;

                } else if (robot.isRedAlliance && robot.isLeftAlliance) {

                    robot.isLeftAlliance = true;
                    robot.isRedAlliance = false;
                } else if (!robot.isRedAlliance && robot.isLeftAlliance) {

                    robot.isLeftAlliance = false;
                    robot.isRedAlliance = false;
                } else if (!robot.isRedAlliance && !robot.isLeftAlliance) {

                    robot.isLeftAlliance = false;
                    robot.isRedAlliance = true;
                }
                delayTimer.reset();
            }




            /*
             * Telemetry while waiting for PLAY:
             */
            //telemetry.addData("Pos","%.3f %.3f %.3f",robot.drive.realSense.getXPos(),robot.drive.realSense.getYPos(), robot.drive.realSense.getRotation());
            dashboardTelemetry.addData("Analysis Left Red", robot.eyes.pipeline.avgLeftRed);
            dashboardTelemetry.addData("Analysis Middle Red", robot.eyes.pipeline.avgMiddleRed);
            dashboardTelemetry.addData("Analysis Right Red", robot.eyes.pipeline.avgRightRed);
            dashboardTelemetry.addData("Analysis Left Blue", robot.eyes.pipeline.avgLeftBlue);
            dashboardTelemetry.addData("Analysis Middle Blue", robot.eyes.pipeline.avgMiddleBlue);
            dashboardTelemetry.addData("Analysis Right Blue", robot.eyes.pipeline.avgRightBlue);
            //dashboardTelemetry.addData("Position", robot.eyes.pipeline.avgValue);
            dashboardTelemetry.addData("Position", robot.eyes.getConePos());
            telemetry.addData("Position", robot.eyes.getConePos());
            //telemetry.addData("POS ","Is Left:%s", robot.isLeftAlliance);
            if(robot.isLeftAlliance && robot.isRedAlliance){
                telemetry.addData("Alliance","Red, Left");
            }else if(!robot.isLeftAlliance && robot.isRedAlliance){
                telemetry.addData("Alliance","Red, Right");
            }else if(robot.isLeftAlliance && !robot.isRedAlliance){
                telemetry.addData("Alliance","Blue, Left");
            }else if(!robot.isLeftAlliance && !robot.isRedAlliance){
                telemetry.addData("Alliance","Blue, Right");
            }
            //telemetry.addData("Distance", robot.jaws.intakeDistance.getDistance(DistanceUnit.INCH));

            dashboardTelemetry.update();
            telemetry.update();


            /*
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             */


        }



        /*
         * Runs after hit start:
         * DO STUFF FOR the OPMODE!!!
         */

        robot.robotWait(timeDelay);

        robot.prowl.driveto(0,10,0,0.3,5);



        if(isStopRequested()) return;




    }
    public void redLeft(){
        CatHW_Vision.UltimateGoalPipeline.conePosition conePos = robot.eyes.getConePos();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        CatMecanumDrive drive = robot.drive;
        drive.setPoseEstimate(startPose);
        TrajectorySequence leftPixel = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(22,6))
                .lineToLinearHeading(new Pose2d(8,5,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(5, -65,Math.toRadians(-80)),
                        CatMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(44, -75,Math.toRadians(99)),
                        CatMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .build();


        TrajectorySequence rightPixel = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(25, -3),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(new Pose2d(30,-8,Math.toRadians(-100)),
                        CatMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToConstantHeading(new Vector2d(30, 0),
                        CatMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(8,0,Math.toRadians(-80)))
                .lineToLinearHeading(new Pose2d(22, -65,Math.toRadians(-70)),
                        CatMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(50, -75,Math.toRadians(99)),
                        CatMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))

                .build();
        TrajectorySequence middlePixel = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(30, -2))
                .lineToLinearHeading(new Pose2d(8,0,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(15, -65,Math.toRadians(-80)),
                        CatMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(46, -75,Math.toRadians(99)),
                        CatMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .build();
        Trajectory parkLeft = drive.trajectoryBuilder(leftPixel.end(),Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(15,-85,Math.toRadians(90)))
                .build();


        Trajectory parkMiddle = drive.trajectoryBuilder(middlePixel.end(),Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(20,-85,Math.toRadians(90)))
                .build();
        Trajectory parkRight = drive.trajectoryBuilder(rightPixel.end(),Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(33,-85,Math.toRadians(90)))
                .build();


        robot.jaws.rotateIntake();

        switch(conePos) {
            case NONE:
            case RIGHT:
                drive.followTrajectorySequence(rightPixel);
                //robot.jaws.setIntakePower(-.33);
                robot.robotWait(.5);
                runningTime.reset();
                while(runningTime.seconds() < 6) {
                    robot.drive.scoreHexAutonomous();
                    robot.drive.updateDistance();
                }
                robot.jaws.setHexLiftMiddle();
                robot.robotWait(1);
                robot.jaws.dispence();
                robot.robotWait(2);
                robot.jaws.zeroPos();
                robot.robotWait(.25);
                robot.jaws.autoSetHexZero();
                robot.robotWait(.5);
                drive.followTrajectory(parkRight);
                break;
            case MIDDLE:
                drive.followTrajectorySequence(middlePixel);
                runningTime.reset();
                while(runningTime.seconds() < 6) {
                    robot.drive.scoreHexAutonomous();
                    robot.drive.updateDistance();
                }
                robot.drive.setMotorPowers(0, 0, 0, 0);

                robot.jaws.setHexLiftMiddle();
                robot.robotWait(1);
                robot.jaws.dispence();
                robot.robotWait(1);
                robot.jaws.zeroPos();
                robot.robotWait(.25);
                robot.jaws.autoSetHexZero();
                robot.robotWait(.5);
                drive.followTrajectory(parkMiddle);

                break;
            case LEFT:
                drive.followTrajectorySequence(leftPixel);
                runningTime.reset();
                while(runningTime.seconds() < 6) {
                    robot.drive.scoreHexAutonomous();
                    robot.drive.updateDistance();
                }
                robot.jaws.setHexLiftMiddle();
                robot.robotWait(1);
                robot.jaws.dispence();
                robot.robotWait(2);
                robot.jaws.zeroPos();
                robot.robotWait(.25);
                robot.jaws.autoSetHexZero();
                robot.robotWait(.5);
                drive.followTrajectory(parkLeft);
                break;
        }



    }

    public void redRight(){
        //CatHW_Vision.UltimateGoalPipeline.conePosition conePos = robot.eyes.getConePos();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        CatMecanumDrive drive = robot.drive;
        drive.setPoseEstimate(startPose);
        TrajectorySequence rightPixel = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(20, -13))
                .lineToConstantHeading(new Vector2d(15, -13))

                .lineToLinearHeading(new Pose2d(33,-50,Math.toRadians(90)))
                .build();
        TrajectorySequence middlePixel = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(10, -5))
                .lineToConstantHeading(new Vector2d(30, -5))
                .lineToConstantHeading(new Vector2d(26, -5))

                .lineToLinearHeading(new Pose2d(36,-50,Math.toRadians(90)))
                .build();
        TrajectorySequence leftPixel = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(10, -5))
                .lineToConstantHeading(new Vector2d(20, -5))

                .lineToLinearHeading(new Pose2d(36,-10,Math.toRadians(90)),
                        CatMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(25))

                .lineToLinearHeading(new Pose2d(40,-50,Math.toRadians(90)))
                .build();

            robot.jaws.rotateIntake();
            drive.followTrajectorySequence(leftPixel);






        /*switch(conePos) {
            case NONE:
            case RIGHT:
                drive.followTrajectorySequence(rightPixel);
                runningTime.reset();
                while(runningTime.seconds() < 3) {
                    robot.drive.scoreHexAutonomous();
                    robot.drive.updateDistance();
                }
                robot.drive.setMotorPowers(0, 0, 0, 0);

                robot.jaws.setHexLiftMiddle();
                robot.robotWait(1);
                robot.jaws.dispenceAuto();
                robot.robotWait(2);
                robot.jaws.zeroPos();
                robot.robotWait(1);
                robot.jaws.autoSetHexZero();
                robot.robotWait(1);
                break;
            case MIDDLE:
                robot.jaws.rotateIntake();
                drive.followTrajectorySequence(middlePixel);
                runningTime.reset();
                while(runningTime.seconds() < 3) {
                    robot.drive.scoreHexAutonomous();
                    robot.drive.updateDistance();
                }
                robot.drive.setMotorPowers(0, 0, 0, 0);
                robot.jaws.setHexLiftMiddle();
                robot.robotWait(1);
                robot.jaws.dispence();
                robot.robotWait(1);
                robot.jaws.zeroPos();
                robot.jaws.autoSetHexZero();
                robot.robotWait(.5);
                break;
            case LEFT:
                robot.jaws.hexLift.setTargetPosition(80);
                robot.jaws.hexLift.setPower(1);
                drive.followTrajectory(moveToMiddle);
                drive.followTrajectory(leftPixel);
                //robot.jaws.setIntakePower(-.25);
                robot.robotWait(1);
                //robot.jaws.setIntakePower(0);

                drive.followTrajectory(dropLeftPixel);
                runningTime.reset();
                while(runningTime.seconds() < 3) {
                    robot.drive.scoreHexAutonomous();
                    robot.drive.updateDistance();
                }
                robot.drive.setMotorPowers(0, 0, 0, 0);

                robot.jaws.setHexLiftMiddle();
                robot.robotWait(2);
                robot.jaws.dispence();
                robot.robotWait(2);
                robot.jaws.zeroPos();
                robot.robotWait(1);
                robot.jaws.autoSetHexZero();
                robot.robotWait(1);
                drive.followTrajectory(leftPark);
                drive.followTrajectory(leftParkFinal);
                break;
        }*/
    }

    public void blueLeft(){
        CatHW_Vision.UltimateGoalPipeline.conePosition conePos = robot.eyes.getConePos();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        CatMecanumDrive drive = robot.drive;
        drive.setPoseEstimate(startPose);
        TrajectorySequence leftPixel = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(23,5),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToConstantHeading(new Vector2d(19,10),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(30, 28,Math.toRadians(-120)),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))

                .build();
        Trajectory leftPark = drive.trajectoryBuilder(leftPixel.end(),Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(12, 40,Math.toRadians(-90)),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))


                .build();
        TrajectorySequence middlePixel = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(28,-3))
                .lineToConstantHeading(new Vector2d(22,-3))
                .lineToLinearHeading(new Pose2d(35.5, 28,Math.toRadians(-120)))
                .build();
        Trajectory middlePark = drive.trajectoryBuilder(middlePixel.end(),Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(12, 40,Math.toRadians(-90)),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))


                .build();

        TrajectorySequence rightPixel = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(28,0),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(new Pose2d(30,-5,Math.toRadians(-100)),
                        CatMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(30,-8,Math.toRadians(-100)),
                        CatMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(33, 30,Math.toRadians(-100)),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))

                .build();
        Trajectory rightPark = drive.trajectoryBuilder(rightPixel.end(),Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(9, 35,Math.toRadians(-90)),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))


                .build();



        robot.jaws.rotateIntake();

        switch(conePos) {
            case NONE:
            case RIGHT:
                drive.followTrajectorySequence(rightPixel);
                runningTime.reset();
                while(runningTime.seconds() < 3) {
                    robot.drive.scoreHexAutonomous();
                    robot.drive.updateDistance();
                }
                robot.drive.setMotorPowers(0, 0, 0, 0);
                robot.jaws.setHexLiftMiddle();
                robot.robotWait(1);
                robot.jaws.dispence();
                robot.robotWait(1);
                robot.jaws.zeroPos();
                robot.robotWait(1);
                robot.jaws.autoSetHexZero();
                robot.robotWait(.5);
                drive.followTrajectory(rightPark);
                break;
            case MIDDLE:
                drive.followTrajectorySequence(middlePixel);
                runningTime.reset();
                while(runningTime.seconds() < 3) {
                    robot.drive.scoreHexAutonomous();
                    robot.drive.updateDistance();
                }
                robot.drive.setMotorPowers(0, 0, 0, 0);
                robot.jaws.setHexLiftMiddle();
                robot.robotWait(1);
                robot.jaws.dispence();
                robot.robotWait(1);
                robot.jaws.zeroPos();
                robot.robotWait(1);
                robot.jaws.autoSetHexZero();
                robot.robotWait(.5);
                drive.followTrajectory(middlePark);
                break;
            case LEFT:
                drive.followTrajectorySequence(leftPixel);
                runningTime.reset();
                while(runningTime.seconds() < 3) {
                    robot.drive.scoreHexAutonomous();
                    robot.drive.updateDistance();
                }
                robot.drive.setMotorPowers(0, 0, 0, 0);
                robot.jaws.setHexLiftMiddle();
                robot.robotWait(1);
                robot.jaws.dispence();
                robot.robotWait(1);
                robot.jaws.zeroPos();
                robot.robotWait(1);
                robot.jaws.autoSetHexZero();
                robot.robotWait(.5);
                drive.followTrajectory(leftPark);
                break;


        }
    }
    public void blueRight() {
        CatHW_Vision.UltimateGoalPipeline.conePosition conePos = robot.eyes.getConePos();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        CatMecanumDrive drive = robot.drive;
        TrajectorySequence leftPixel = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(24, -6),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(50))
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(100)),
                        CatMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToConstantHeading(new Vector2d(30, -10),
                        CatMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToConstantHeading(new Vector2d(8, -10),
                        CatMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(8, -5, Math.toRadians(80)),
                        CatMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(19, 60, Math.toRadians(65)),
                        CatMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(46, 80, Math.toRadians(240)),
                        CatMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))

                .build();

        Trajectory leftPark = drive.trajectoryBuilder(leftPixel.end(), -90)
                .lineToLinearHeading(new Pose2d(25, 80, Math.toRadians(-90)),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence middlePixel = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(29, -6),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(4, -6, Math.toRadians(90)),
                        CatMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(4, 5, Math.toRadians(80)),
                        CatMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(19, 60, Math.toRadians(65)),
                        CatMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(49, 75, Math.toRadians(250)),
                        CatMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))

                .build();
        Trajectory middlePark = drive.trajectoryBuilder(middlePixel.end(), -90)
                .lineToLinearHeading(new Pose2d(25, 80, Math.toRadians(-90)),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence rightPixel = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(27, -15),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(4, -6, Math.toRadians(90)),
                        CatMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(4, 5, Math.toRadians(90)),
                        CatMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(13, 60, Math.toRadians(75)),
                        CatMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(41, 78, Math.toRadians(-90)),
                        CatMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(60))

                .build();
        Trajectory rightPark = drive.trajectoryBuilder(rightPixel.end(), -90)
                .lineToLinearHeading(new Pose2d(9, 85, Math.toRadians(-90)),
                        CatMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        CatMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        robot.jaws.rotateIntake();

        switch (conePos) {
            case NONE:
            case RIGHT:
                drive.followTrajectorySequence(rightPixel);
                runningTime.reset();
                while (runningTime.seconds() < 5) {
                    robot.drive.scoreHexAutonomous();
                    robot.drive.updateDistance();
                }
                robot.drive.setMotorPowers(0, 0, 0, 0);
                robot.jaws.setHexLiftMiddle();
                robot.robotWait(1);
                robot.jaws.dispence();
                robot.robotWait(1);
                robot.jaws.zeroPos();
                robot.robotWait(1);
                robot.jaws.autoSetHexZero();
                robot.robotWait(.5);
                drive.followTrajectory(rightPark);
                break;
            case MIDDLE:
                drive.followTrajectorySequence(middlePixel);
                runningTime.reset();
                while (runningTime.seconds() < 6) {
                    robot.drive.scoreHexAutonomous();
                    robot.drive.updateDistance();
                }
                robot.drive.setMotorPowers(0, 0, 0, 0);
                robot.jaws.setHexLiftMiddle();
                robot.robotWait(1);
                robot.jaws.dispence();
                robot.robotWait(1);
                robot.jaws.zeroPos();
                robot.robotWait(1);
                robot.jaws.autoSetHexZero();
                robot.robotWait(.5);
                drive.followTrajectory(middlePark);
                break;
            case LEFT:
                drive.followTrajectorySequence(leftPixel);
                runningTime.reset();
                while (runningTime.seconds() < 6) {
                    robot.drive.scoreHexAutonomous();
                    robot.drive.updateDistance();
                }
                robot.drive.setMotorPowers(0, 0, 0, 0);
                robot.jaws.setHexLiftMiddle();
                robot.robotWait(1);
                robot.jaws.dispence();
                robot.robotWait(1);
                robot.jaws.zeroPos();
                robot.robotWait(1);
                robot.jaws.autoSetHexZero();
                robot.robotWait(.25);
                drive.followTrajectory(leftPark);
                break;
        }
    }

}

