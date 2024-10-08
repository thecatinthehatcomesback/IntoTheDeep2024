package org.firstinspires.ftc.teamcode.teamcode;

import static java.lang.Math.sqrt;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class CatProwl extends CatHW_Subsystem {
    public CatProwl(CatHW_Async mainHardware){
        super(mainHardware);
    }
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;

    boolean isDone;

    SparkFunOTOS myOtos;
    Pose2d targetPos;
    double driveSpeed;
    public void init()throws InterruptedException{

        leftFrontMotor = hwMap.dcMotor.get("leftFront");
        rightFrontMotor = hwMap.dcMotor.get("rightFront");
        leftRearMotor = hwMap.dcMotor.get("leftRear");
        rightRearMotor = hwMap.dcMotor.get("rightRear");

        // Define motor directions: //
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        myOtos = hwMap.get(SparkFunOTOS.class, "sensor_otos");
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        configureOtos();
    }
    private void configureOtos() {
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
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

    }
    public void driveto(double x,double y,double theta,double speed,double timeout){
        targetPos = new Pose2d(x,y,Math.toRadians(theta));
        driveSpeed=speed;
        isDone = false;
        while (!isDone){
            doDrive();
        }
    }
    public void doDrive(){
        SparkFunOTOS.Pose2D currentPos = myOtos.getPosition();
        double dist = Math.sqrt( Math.pow(targetPos.getX() - currentPos.x , 2) + Math.pow(targetPos.getY() - currentPos.y,2) );
        double theta=Math.atan2(targetPos.getY() - currentPos.y,targetPos.getX() - currentPos.x);
        double rotDiff=targetPos.getHeading()-currentPos.h;
        double ypow= Math.sin(theta)*driveSpeed;
        double xpow= Math.cos(theta)*driveSpeed;
        double rpow= Math.max(-1,Math.min(rotDiff/2,1));


        double denominator = Math.max(Math.abs(ypow) + Math.abs(xpow) + Math.abs(rpow), 1);
        double frontLeftPower = (ypow + xpow + rpow) / denominator;
        double backLeftPower = (ypow - xpow + rpow) / denominator;
        double frontRightPower = (ypow - xpow - rpow) / denominator;
        double backRightPower = (ypow + xpow - rpow) / denominator;

        leftFrontMotor.setPower(frontLeftPower);
        leftRearMotor.setPower(backLeftPower);
        rightFrontMotor.setPower(frontRightPower);
        rightRearMotor.setPower(backRightPower);
        Log.d("catbot",String.format("LF %.2f LR %.2f RF %.2f RR %.2f dist %.2f theta %.2f xpow %.2f ypow %.2f rpow %.2f " ,
               frontLeftPower, backLeftPower, frontRightPower, backRightPower, dist, theta, xpow, ypow, rpow ));
        if(dist<1.0){
            setDrivePowers(0,0,0,0);
            isDone=true;


        }
    }
    public void driveto(double x,double y,double theta){
        driveto(x,y,theta,0.5,2.0);
    }

    /**
     * Sets powers to the four drive train motors.
     *
     * @param leftFront  motor's power.
     * @param rightFront motor's power.
     * @param leftBack   motor's power.
     * @param rightBack  motor's power.
     */
    public void setDrivePowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        leftFrontMotor.setPower(leftFront);
        rightFrontMotor.setPower(rightFront);
        leftRearMotor.setPower(leftBack);
        rightRearMotor.setPower(rightBack);
    }
}
