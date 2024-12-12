package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


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

public class   NewMainAutonomous extends LinearOpMode {

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
        robot.jaws.closeGripper();



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
                if (robot.isLeftAlliance) {
                    robot.isLeftAlliance = false;
                } else {
                    robot.isLeftAlliance = true;
                }
                delayTimer.reset();
            }




            /*
             * Telemetry while waiting for PLAY:
             */
            telemetry.addData("Time Delay ","%.0f  seconds",timeDelay);
            telemetry.addData("Position ","%s",robot.isLeftAlliance ? "left":"right");
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
        if (robot.isLeftAlliance) {
            left();
        } else {
            right();
        }

        robot.jaws.ourThread.pleaseStop();

        if(isStopRequested()) return;





    }
    private void right(){
        robot.prowl.driveto(0,15,0,0.4,5);
        robot.jaws.setArmAngle(80);
        robot.robotWait(1);
        robot.jaws.wrist.setPosition(.7);
        robot.robotWait(.5);
        robot.prowl.driveto(0,24,0,0.4,5);
        robot.jaws.setExtendAuto();
        robot.robotWait(.5);
        robot.jaws.setArmAngle(60);
        robot.robotWait(1);
        robot.prowl.driveto(0,5,0,0.6,2);
        robot.robotWait(.5);
        robot.jaws.openGripper();
        robot.robotWait(.5);
        robot.prowl.driveto(45,5,0,0.4,5);

        robot.robotWait(1);
    }
    private void left() {


        robot.prowl.driveto(4, 15, 0, 0.4, 5);
        robot.robotWait(.5);
        robot.jaws.setArmAngle(84);
        robot.robotWait(.5);
        robot.jaws.wrist.setPosition(.7);
        robot.robotWait(.5);
        robot.jaws.setExtendLong();
        robot.robotWait(.5);
        robot.jaws.openGripper();
        robot.robotWait(.5);
        robot.jaws.setArmAngle(90);
        robot.jaws.setExtendMedium();
        robot.robotWait(.5);
        robot.prowl.driveto(8, 8, -110, 0.6, 2);
        robot.robotWait(1);
        robot.jaws.setArmAngle(0);
        robot.robotWait(1);
        robot.jaws.closeGripper();
        robot.robotWait(.5);
       /* robot.prowl.driveto(48, -3, 180, 0.6, 2);
        robot.prowl.driveto(48, 5, 180, 0.6, 2);
        robot.prowl.driveto(3, 10, 170, 0.6, 2.5);
        robot.prowl.driveto(48, 6, 180, 0.8, 5);
        robot.prowl.driveto(48, 16, 180, 0.6, 5);
        robot.prowl.driveto(3, 16, 180, 0.6, 2.5);
        robot.prowl.driveto(48, 22, 180, 0.8, 5);
        robot.prowl.driveto(3, 22, 180, 0.6, 2.5);
        robot.prowl.driveto(48, 16, 180, 0.8, 5);
        robot.prowl.driveto(48, -8, 180, 0.6, 5);
        robot.jaws.setExtendLong();
        robot.jaws.setArmAngle(60);
       */ robot.robotWait(.5);

    }
}

