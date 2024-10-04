package org.firstinspires.ftc.teamcode.teamcode;

import static java.lang.Math.sqrt;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

    SparkFunOTOS myOtos;
    Pose2d targetPos;
    double driveSpeed;
    public void init()throws InterruptedException{

        leftFrontMotor = hwMap.dcMotor.get("LFront");
        rightFrontMotor = hwMap.dcMotor.get("RFront");
        leftRearMotor = hwMap.dcMotor.get("LRear");
        rightRearMotor = hwMap.dcMotor.get("RRear");

        // Define motor directions: //
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        myOtos = hwMap.get(SparkFunOTOS.class, "sensor_otos");
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
    }

    public void driveto(double x,double y,double theta,double speed,double timeout){
        targetPos = new Pose2d(x,y,Math.toRadians(theta));
        driveSpeed=speed;
        while(!pos.equals(targetPos) ){
            if(targetPos.getX()<0){
                leftFrontMotor.setPower(-speed);
                rightFrontMotor.setPower(-speed);
                leftRearMotor.setPower(-speed);
                rightRearMotor.setPower(-speed);
            }
        }
    }
    public void dodrive(){
        SparkFunOTOS.Pose2D currentPos = myOtos.getPosition();
        double dist = Math.sqrt( Math.pow(targetPos.getX() - currentPos.x , 2) + Math.pow(targetPos.getY() - currentPos.y,2) );
        double theta=Math.atan2(targetPos.getY() - currentPos.y,targetPos.getX() - currentPos.x);
        double rotDiff=targetPos.getHeading()-currentPos.h;
        double ypow= Math.sin(theta)*driveSpeed;
        double xpow= Math.cos(theta)*driveSpeed;
        double rpow= Math.max(-1,Math.min(rotDiff/2,1));
        double frontLeft = ypow + xpow;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
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
