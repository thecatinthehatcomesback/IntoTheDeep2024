package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

/**
 * MainTeleOp.java
 *
 *
 * A Linear opMode class that is used as our TeleOp method for the driver controlled period.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
@TeleOp(name = "Cat Teleop", group = "CatTeleOp")
public class CatTeleop extends LinearOpMode
{
    /* Declare OpMode members. */
    private ElapsedTime elapsedGameTime = new ElapsedTime();


    /* Declare OpMode members. */
    CatHW_Async robot;  // Use our new mecanum async hardware


    /* Constructor */
    public CatTeleop() {
        robot = new CatHW_Async();
    }

    SparkFunOTOS myOtos;

    @Override
    public void   runOpMode() throws InterruptedException {

        // Informs driver the robot is trying to init
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        // Initialize the hardware
        robot.init(hardwareMap, this);
        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

        // Finished!  Now tell the driver...
        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        ElapsedTime delayTimer = new ElapsedTime();

        while (!opModeIsActive() && !isStopRequested()) {
            if (((gamepad1.x) && delayTimer.seconds() > 0.8)) {
                delayTimer.reset();

                // Changes Alliance Sides
                CatHW_Async.isRedAlliance = !CatHW_Async.isRedAlliance;
            }
            telemetry.addData("Alliance", "%s", CatHW_Async.isRedAlliance ? "Red" : "Blue");
            telemetry.update();

            if (CatHW_Async.isRedAlliance) {
                //robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else {
                //robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }
            dashboardTelemetry.update();

        }


        // Go! (Presses PLAY)
        elapsedGameTime.time(TimeUnit.SECONDS);
        elapsedGameTime.reset();
        double driveSpeed;
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double SF;
        boolean endGame = false;
        boolean under10Sec = false;
        boolean alignMode = false;


        ElapsedTime buttontime = new ElapsedTime();
        buttontime.reset();


        double lastTime = elapsedGameTime.milliseconds();
        double avgLoopTime = 0;
        double avgT1 = 0;
        double avgT2 = 0;
        double avgT3 = 0;

        // Run infinitely until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Measure loop time
            double currentTime = elapsedGameTime.milliseconds();
            double loopTime = currentTime - lastTime;
            lastTime = currentTime;
            avgLoopTime = loopTime * 0.1 + avgLoopTime * 0.9;

            //--------------------------------------------------------------------------------------
            // Driver 1 Controls:
            //-----------------------------------------------------------------------------------

            if (elapsedGameTime.time() > 90 && CatHW_Async.isRedAlliance && !endGame) {
                //robot.lights.blink(15, RevBlinkinLedDriver.BlinkinPattern.RED,1000 );
                endGame = true;

            } else if (elapsedGameTime.time() > 90 && !CatHW_Async.isRedAlliance && !endGame) {
                //robot.lights.blink(15, RevBlinkinLedDriver.BlinkinPattern.BLUE,1000 );
                endGame = true;
            }

            // Drive train speed control:

            if (gamepad1.right_trigger > .1 || gamepad1.left_trigger > .1) {
                driveSpeed = 1.00;
            } else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                driveSpeed = 0.50;
            } else {
                driveSpeed = 0.9;
            }

            double forward = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y);
            //forward = forward - (gamepad1.left_bumper? 1.0 : 0) * 0.3 + (gamepad1.right_bumper? 1.0:0) *.3;
            double strafe = ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x);
            if (gamepad1.dpad_left) {
                strafe = strafe - 0.5;
            } else if (gamepad1.dpad_right) {
                strafe = strafe + 0.5;
            } else if (gamepad1.dpad_up) {
                forward = forward + .4;
            } else if (gamepad1.dpad_down) {
                forward = forward - 0.4;
            }
            double turn = gamepad1.left_stick_x;
            if (gamepad2.dpad_up){
                double power = robot.jaws.armMotor.getPower();
                robot.jaws.armMotor.setPower(power+0.001);
            }
            if (gamepad2.dpad_down){
                double power = robot.jaws.armMotor.getPower();
                robot.jaws.armMotor.setPower(power-0.001);
            }

            // Input for setDrivePowers train and sets the dead-zones:
            leftFront = forward + strafe + turn;
            rightFront = forward - strafe - turn;
            leftBack = forward - strafe + turn;
            rightBack = forward + strafe - turn;

            // Calculate the scale factor:
            SF = robot.prowl.findScalor(leftFront, rightFront, leftBack, rightBack);
            // Set powers to each setDrivePowers motor:
            leftFront = leftFront * SF * driveSpeed;
            rightFront = rightFront * SF * driveSpeed;
            leftBack = leftBack * SF * driveSpeed;
            rightBack = rightBack * SF * driveSpeed;
            robot.prowl.setDrivePowers(leftFront,rightFront,leftBack,rightBack);
            currentTime = elapsedGameTime.milliseconds();
            avgT1 = avgT1 * 0.9 + (currentTime - lastTime) * 0.1;


            //if(gamepad1.right_bumper){
            //    robot.jaws.setArmAngle(30);
           // } else if (gamepad1.left_bumper){
            //    robot.jaws.setArmAngle(100);
            //}
            // DRIVE!!!

            currentTime = elapsedGameTime.milliseconds();
            avgT2 = avgT2 * 0.9 + (currentTime - lastTime) * 0.1;

            currentTime = elapsedGameTime.milliseconds();
            avgT3 = avgT3 * 0.9 + (currentTime - lastTime) * 0.1;

            //--------------------------------------------------------------------------------------
            // Telemetry Data:
            //--------------------------------------------------------------------------------------
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            telemetry.addData("Game Timer", "%.2f", elapsedGameTime.time());
            telemetry.addData("Trigger", "Left %.2f right %.2f", gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.addData("Loop Time", "%3.0f ms  %3.0f/%3.0f/%3.0f", avgLoopTime, avgT1, avgT2, avgT3);
            telemetry.addData("X/Y/Theta", "%3.1f %3.1f %3.1f",pos.x,pos.y,pos.h);
            telemetry.addData("armPower","%5.3f  pos %d",robot.jaws.armMotor.getPower(),robot.jaws.armMotor.getCurrentPosition());

            telemetry.update();
            dashboardTelemetry.update();
        }
    }


    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0,(0));
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

}