package org.firstinspires.ftc.teamcode.teamcode;

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
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        Pose2d targetPos = new Pose2d(x,y,theta);
        while(!pos.equals(targetPos) ){
            if(targetPos.getX()<0){
                leftFrontMotor.setPower(-speed);
                rightFrontMotor.setPower(-speed);
                leftRearMotor.setPower(-speed);
                rightRearMotor.setPower(-speed);
            }
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
