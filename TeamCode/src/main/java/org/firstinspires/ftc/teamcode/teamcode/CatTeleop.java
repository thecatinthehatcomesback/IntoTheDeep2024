package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    @Override
    public void   runOpMode() throws InterruptedException {

        // Informs driver the robot is trying to init
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // Initialize the hardware
        robot.init(hardwareMap, this);

        // All the configuration for the OTOS is done in this helper method, check it ou
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
                driveSpeed = 0.3;
            } else {
                driveSpeed = 0.75;
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


            if (gamepad2.dpad_up){
                robot.jaws.setArmAngle(100);
            }
            if (gamepad2.dpad_left){
                robot.jaws.setArmAngle(20);
            }
            if (gamepad2.dpad_down){
                robot.jaws.setArmAngle(0);
            }
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                double cur=robot.jaws.getArmAngle();
                cur=cur-gamepad2.left_stick_y;
                robot.jaws.setArmAngle(cur);
            }
            if (gamepad2.left_bumper){
                robot.jaws.openGripper();
            }
            if (gamepad2.right_bumper){
                robot.jaws.closeGripper();
            }
            if (gamepad2.left_trigger>0.1){
                robot.jaws.gripper.setPosition(robot.jaws.gripper.getPosition()-0.003);
            }
            if  (gamepad2.right_trigger>0.1){
                robot.jaws.gripper.setPosition(robot.jaws.gripper.getPosition()+0.003);

            }
            if (gamepad2.right_stick_x>0.3) {
                robot.jaws.wrist.setPosition(robot.jaws.wrist.getPosition()+0.003);
            }
            if (gamepad2.right_stick_x<-0.3) {
                robot.jaws.wrist.setPosition(robot.jaws.wrist.getPosition()-0.003);
            }
            if (gamepad2.cross){
                robot.jaws.setExtendShort();
            }
            if (gamepad2.circle){
                robot.jaws.setExtendMedium();
            }
            if (gamepad2.triangle){
                robot.jaws.setExtendLong();
            }
            if (Math.abs(gamepad2.right_stick_y) > 0.1) {
                if (gamepad2.right_stick_y<0) {
                    robot.jaws.bumpExtend(20);
                }
                else{
                    robot.jaws.bumpExtend(-20);
                }
            }
            // DRIVE!!!

            currentTime = elapsedGameTime.milliseconds();
            avgT2 = avgT2 * 0.9 + (currentTime - lastTime) * 0.1;

            currentTime = elapsedGameTime.milliseconds();
            avgT3 = avgT3 * 0.9 + (currentTime - lastTime) * 0.1;

            //--------------------------------------------------------------------------------------
            // Telemetry Data:
            //--------------------------------------------------------------------------------------
            SparkFunOTOS.Pose2D pos = robot.prowl.myOtos.getPosition();
            telemetry.addData("Game Timer", "%.2f", elapsedGameTime.time());
            telemetry.addData("Loop Time", "%3.0f ms  %3.0f/%3.0f/%3.0f", avgLoopTime, avgT1, avgT2, avgT3);
            telemetry.addData("X/Y/Theta", "%3.1f %3.1f %3.1f",pos.x,pos.y,pos.h);
            telemetry.addData("armPower","%5.3f  angle %.0f pos %d tar %d" ,robot.jaws.armMotor.getPower(),
                    robot.jaws.getArmCurAngle(),robot.jaws.armMotor.getCurrentPosition(),robot.jaws.target);
            telemetry.addData("gripper","%.2f",robot.jaws.gripper.getPosition());
            telemetry.addData("wrist","%.2f",robot.jaws.wrist.getPosition());

            telemetry.addData("extend","cur %d target %d",robot.jaws.armExtend.getCurrentPosition(),robot.jaws.armExtend.getTargetPosition());
            telemetry.update();
            dashboardTelemetry.update();
        }
        robot.jaws.ourThread.pleaseStop();
    }
}