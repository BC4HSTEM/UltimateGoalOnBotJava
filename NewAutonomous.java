package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;

@Autonomous(name="AutoBox2", group="Skystone Test")
public class NewAutonomous extends LinearOpMode
{
    private ElapsedTime  runtime = new ElapsedTime();

    //servo port 2
    private DcMotor right;
    private DcMotor left;
    private DcMotor center;
    private DcMotor ramp;
    private CRServo launcher;
    private CRServo intake;
    private Servo claw;
    private Servo arm;
    
    //encoder clicks are originally 1680
    //multiply that by two thirds   
     private static final int ENCODER_CLICKS = 1680;
     //private static final int DRIVE_ENCODER_CLICKS = 1680;
     //private static final double ROBOT_DIAM = 40 ;
     //private static final double ROBOT_CIRC = ROBOT_DIAM * Math.PI;
     private static final double WHEEL_DIAM = 10.0;
     private static final double WHEEL_CIRC = WHEEL_DIAM * Math.PI;
     private static final double DRIVE_GEAR_RATIO = 2/3;
     private static final double CLICKS_PER_CM = ENCODER_CLICKS / WHEEL_CIRC;
    
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
        ramp = hardwareMap.get(DcMotor.class, "ramp");
        launcher = hardwareMap.get(CRServo.class, "launcher");
        intake = hardwareMap.get(CRServo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");
        
        // Initialize motor direction
        right.setDirection(DcMotor.Direction.FORWARD);
        left.setDirection(DcMotor.Direction.REVERSE);
        
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Initialize motor encoder modes
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        

        center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        center.setDirection(DcMotor.Direction.REVERSE);
        
        ramp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        

        telemetry.addData("status","init" );
        telemetry.update();

        // OpMode needs waitForStart(); and while (opModeIsActive()) {...}
        waitForStart();

        telemetry.addData("status","op mode started" );
        telemetry.update();
        
        // Strafe Right
        center.setTargetPosition(840);
        center.setPower(0.45);
        sleep(1000);
        center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        center.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        center.setPower(0);
        

        // Drive Forward
        driveBot(91.4,91.4,0.4,5.0);
        telemetry.addData("status","first run to position called" );
        telemetry.addData("status", left.getMode() );
        telemetry.addData("status","left motor,  %7d", left.getCurrentPosition() );
        telemetry.addData("status","right motor,  %7d", right.getCurrentPosition() );
        telemetry.update();
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 /*       
        left.setPower(0.2);
        right.setPower(0.2); */
        sleep(1000);
        
        
        // Strafe Left
        center.setTargetPosition(840);
        center.setPower(-0.45);
        sleep(1000);
        center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        center.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        center.setPower(0);
        
        sleep(1000);

        launcher.setPower(0.78);
        sleep(1000);
        
        

/*
        
        driveBot(19.0,-19.0,00.3,5.0);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        sleep(1000);
        
        
        driveBot(10.0,10.0,00.3,5.0);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        sleep(1000);
        
        
        driveBot(-17.0,17.0,00.3,5.0);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        sleep(1000);
*/       

        // Strafe Left
        center.setTargetPosition(840);
        center.setPower(-0.45);
        sleep(1000);
        center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        center.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        center.setPower(0);

        //Drive Forward
        driveBot(30.0,30.0,00.3,5.0);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        sleep(1000);
        
        //Drive Back
        driveBot(-25.0,-25.0,00.3,5.0);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        sleep(1000);
        
        
        
    }
     public static double driveDistance(double distance)
        {
            double drive  = (ENCODER_CLICKS/ WHEEL_CIRC);
            int outputClicks= (int)Math.floor(drive * distance);
            return outputClicks;
        }

    /* Drive method */
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
        
            // rightTarget = right.getCurrentPosition()+ (int)(distanceInCM * CLICKS_PER_CM);
            /// leftTarget = left.getCurrentPosition()+ (int)(distanceInCM * CLICKS_PER_CM);
            
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
                    telemetry.addData("Path1", "leftTarget, rightTarget" );
                    telemetry.update();
                }
            left.setPower(0);
            right.setPower(0);
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }   
    }
}
