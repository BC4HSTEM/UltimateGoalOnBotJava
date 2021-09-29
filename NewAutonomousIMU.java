package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.lang.Math;
import java.util.Set;
import java.util.List;
import java.lang.annotation.Target;

@Autonomous(name="NewAutonomousIMU", group="2021 Ultimate Goal")
public class NewAutonomousIMU extends LinearOpMode
{
    // Initialize Timers
    private ElapsedTime  runtime = new ElapsedTime();

    // Declare the hardware (motors, servos, sensors, IMU, etc)
    private DcMotor right;
    private DcMotor left;
    private DcMotor center;
    private BNO055IMU imu;
    
    // Declare constants for Encoders, Gear Ratios and Robot Measurements
    private static final int ENCODER_CLICKS = 1680;
    private static final double WHEEL_DIAM = 10.0;
    private static final double WHEEL_CIRC = WHEEL_DIAM * Math.PI;
    private static final double DRIVE_GEAR_RATIO = 2/3;
    private static final double CLICKS_PER_CM = ENCODER_CLICKS / WHEEL_CIRC;
 
    // Declare IMU 
    BNO055IMU.Parameters IMU_Parameters;
    ElapsedTime ElapsedTime2;
    double Left_Power;
    double Right_Power;
    double Original_Power;
    float Yaw_Angle = 0;   
    

    @Override
    public void runOpMode()
    {
        
        // Telemetry to show OpMode is initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. 
        left  = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        center = hardwareMap.get(DcMotor.class, "center");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        // Initialize motor direction
        right.setDirection(DcMotor.Direction.FORWARD);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        center.setDirection(DcMotor.Direction.REVERSE);
        
        // Reset motor encoders
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Initialize motor encoder modes
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set motor to brake and hold at encoder count
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize IMU 
        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(IMU_Parameters);        
            
        telemetry.addData("Status", "IMU initialized, calibration started.");
        telemetry.update();
    
        telemetry.addData("status","init" );
        telemetry.update();
        
        // Calibrate IMU       
        while (!IMU_Calibrated()) {
          telemetry.addData("If calibration ", "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
          telemetry.update();
          // Wait one second before checking calibration status again.
          sleep(1000);
        }
           
        telemetry.addData("Status", "Calibration Complete");
        telemetry.addData("Action needed:", "Please press the start triangle");
        telemetry.update();
    
    
        waitForStart();
        // @TODO: OpMode needs waitForStart(); and while (opModeIsActive()) {...}

        telemetry.addData("status","op mode started" );
        telemetry.update();
        
            
        /*
            @TODO: We need to turn our Yaw Angle code into a "turnBot" method after we get it working!
            @TODO: Use the runtime variable already defined and being used by our driveBot method. 
                    Do a search for runtime to see how runtime is used
        */

    // END runOpmode()
    
    
    /**********************************************
     * BEGIN METHODS driveDistance, driveBot
     *********************************************/
    
    /**
    * Drive Distance Method
    * Calculate distance from CM to encoder counts
    */  
        setPower(0.4);
        driveStraight(2,1);
        turnRight();
        turnLeft();
    }
    public static double driveDistance(double distance)
    {
        double drive  = (ENCODER_CLICKS/ WHEEL_CIRC);
        int outputClicks= (int)Math.floor(drive * distance);
        return outputClicks;
    }
    // END driveDistance() method



    // @TODO: Add a new method for turning by Yaw Angle
    /**
    * Turn by Yaw Angle Method
    * Left & Right wheels travel distance in CM +/-, power to both wheels, timeout in seconds
    */
    
    

    // @TODO: Add IMU for driving straight but ONLY when driving straight! 
    /**
    * Drive by Encoder Method
    * Left & Right wheels travel distance in CM +/-, power to both wheels, timeout in seconds
    */
    public void driveStraight(double distance, double speed)
    {
        ElapsedTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        // Set motor powers to the variable values.
        Left_Power = Original_Power;
        Right_Power = Original_Power;
        // Set motor powers to the variable values.
        left.setPower(Left_Power);
        right.setPower(Right_Power);
        while (!(ElapsedTime2.milliseconds() >= distance*1000 || isStopRequested())) {
            // save yaw angle
            Yaw_Angle = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle * (float)(Math.PI/180));
            // print yaw angle
            telemetry.addData("Yaw angle", Yaw_Angle);
            //make sure the robot is driving straight within 5 degrees
            if (Yaw_Angle < -5) {
              // Turn left
              Left_Power = Original_Power - (Original_Power * 0.16);
              Right_Power = Original_Power + (Original_Power * 0.16);
            } else if (Yaw_Angle > 5) {
              // Turn right.
              Left_Power = Original_Power + (Original_Power * 0.16);
              Right_Power = Original_Power - (Original_Power * 0.16);
            } else {
              // Continue straight
              Left_Power = Original_Power;
              Right_Power = Original_Power;
            }
            // Report the new power levels to the Driver Station.
            telemetry.addData("Left Motor Power", Left_Power);
            telemetry.addData("Right Motor Power", Right_Power);
            // Update the motors to the new power levels.
            left.setPower(Left_Power);
            right.setPower(Right_Power);
            telemetry.update();
            // Wait 1/5 second before checking again.
            sleep(200);
        }
    }
    public void turnRight()
    {
        //MOVE RIGHT
        left.setPower(0.2);
        right.setPower(-0.2);
        // Continue until robot yaws right by 90 degrees or stop is pressed on Driver Station.
        sleep(1000);
        //turns to the right; 90, 180, negative, -90, 0
        while ( !(Yaw_Angle >= 85 || isStopRequested()) ) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", Yaw_Angle);
            telemetry.update();
        }
        // We're done. Turn off motors
        left.setPower(0);
        right.setPower(0);
        // Pause so final telemetry is displayed.
        sleep(1000);
    }
    public void turnLeft( )
    {
        //MOVE RIGHT
        left.setPower(-0.2);
        right.setPower(0.2);
        // Continue until robot yaws right by 90 degrees or stop is pressed on Driver Station.
        sleep(1000);
        //turns to the right; 90, 180, negative, -90, 0
        while ( !(Yaw_Angle <= -85 || isStopRequested()) ) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", Yaw_Angle);
            telemetry.update();
        }
        // We're done. Turn off motors
        left.setPower(0);
        right.setPower(0);
        // Pause so final telemetry is displayed.
        sleep(1000);
    }
        public void turn(boolean dir, int degrees)
    {
            if(!dir)
            {
                //MOVE LEFT
                 left.setPower(-0.2);
                right.setPower(0.2);
                degrees = (int)(degrees * 0.94444444444);
                //turns to the right; 90, 180, negative, -90, 0
                while ( !(Yaw_Angle <= (-1*degrees) || isStopRequested()) ) {
                    // Update Yaw-Angle variable with current yaw.
                    Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                    // Report yaw orientation to Driver Station.
                    telemetry.addData("Yaw value", Yaw_Angle);
                    telemetry.update();
                }
            }
            else
            {
                //MOVE RIGHT
            left.setPower(0.2);
            right.setPower(-0.2);
            degrees = (int)(degrees * 0.94444444444);
            while ( !(Yaw_Angle >= degrees || isStopRequested()) ) {
                // Update Yaw-Angle variable with current yaw.
                Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                // Report yaw orientation to Driver Station.
                telemetry.addData("Yaw value", Yaw_Angle);
                telemetry.update();
            }
        }
        // We're done. Turn off motors
        left.setPower(0);
        right.setPower(0);
        // Pause so final telemetry is displayed.
        sleep(1000);
    }
    public void setPower(double c)
    {
        Original_Power = c;
    }
    public void driveBot(double distanceInCMleft, double distanceInCMright, double power, double timeoutS) 
    {
        telemetry.addData("status","encoder reset");
        telemetry.update();
        
        int rightTarget;
        int leftTarget;

        if(opModeIsActive()) 
        {
            telemetry.addData("status","getEncoderClicks");
            telemetry.update();
            
            rightTarget = (int) driveDistance(distanceInCMright);
            leftTarget = (int) driveDistance(distanceInCMleft);

            right.setTargetPosition(rightTarget);
            left.setTargetPosition(leftTarget);

            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            runtime.reset();

            right.setPower(power);
            left.setPower(power);
        
            while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (left.isBusy() && right.isBusy()))
                {
                    telemetry.addData("driveBot count", "leftTarget, rightTarget" );
                    telemetry.update();
                }
                
            left.setPower(0);
            right.setPower(0);
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }   
    }
    // END driveBot() method
    
 
    /**
    * IMU Calibration Method
    * Checks IMU calibration and returns telementry
    * If IMU is NOT calibrated, run the calibration opMode 
    */ 
    private boolean IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", imu.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", imu.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Status", imu.getSystemStatus().toString());
        return imu.isGyroCalibrated();
    }
    // END IMU_Calibrated() method
    
}