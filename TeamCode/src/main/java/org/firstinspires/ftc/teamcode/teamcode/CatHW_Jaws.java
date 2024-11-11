package org.firstinspires.ftc.teamcode.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * CatHW_Jaws.java
 *
 *
 * This class containing common code accessing hardware specific to the movement of the jaws/intake.
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Jaws extends CatHW_Subsystem
{

    // Motors: //

    public DcMotor armMotor = null;
    public DcMotor armExtend = null;

    public Servo gripper = null;

    public ElapsedTime liftTime = null;
    public ElapsedTime pidTimer = null;
    public int target;
    //values for pid
    double kP = 0.008;
    double kI = 0.0;
    double kD = 0.0;
    double feedForword = 0.2;

    double lastError;
    double lastTime;

    public Update_PID ourThread;
    private static final double ticksPerRev = (3.61*5.23*28);
    private static final double ticksPerDegree = ticksPerRev/360;
    private static final double maxPower= 0.4;

    // Timers: //

    /* Constructor */
    public CatHW_Jaws(CatHW_Async mainHardware) {
        super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init() {

        // Define and initialize motors: /armMotor/

        armMotor = hwMap.dcMotor.get("armMotorNew");
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armExtend = hwMap.dcMotor.get("armExtend");
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtend.setDirection(DcMotorSimple.Direction.REVERSE);

        gripper = hwMap.servo.get("gripper");

        liftTime = new ElapsedTime();
        pidTimer = new ElapsedTime();

        ourThread = new Update_PID(this);
        Thread th = new Thread(ourThread,"update pid");
        th.start();
    }



    //----------------------------------------------------------------------------------------------
    // Jaw Methods:
    //----------------------------------------------------------------------------------------------

    public void setArmAngle (double degree){
        target=((int)(degree*ticksPerDegree));
    }
    public void updatePID(){
        int current = 0;//armMotor.getCurrentPosition();
        double error = current-target;
        double angle = current/ticksPerDegree-30;
        double derivative = (error - lastError) / (pidTimer.seconds()-lastTime);
        double power = error * kP + derivative*kD + Math.cos(Math.toRadians(angle))*feedForword;
        power = Math.min(power,maxPower);
        power = Math.max(power,-maxPower);
        //armMotor.setPower(power);

        lastError=error;
        lastTime = pidTimer.seconds();
    }

    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {
        boolean result = false;
        return result;
    }
}