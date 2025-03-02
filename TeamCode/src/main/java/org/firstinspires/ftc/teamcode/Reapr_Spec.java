package org.firstinspires.ftc.teamcode;

import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

@Autonomous(name="Hawktuahnomous")
public class Reapr_Spec extends LinearOpMode {
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    
    private Servo wrist;
    private DcMotor arm;
    private Servo claw;
    private DcMotor fourBarCH;
    private DcMotor fourBarEH;

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = (1 + (46.0 / 17.0)) * (1 + (46.0 / 11.0)) * 28 ;    // GoBilda 5202 Series 312 RPM Planetary Gear Motor
    static final double     DRIVE_GEAR_REDUCTION    = 7.0/3.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 1;
    
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        
        DcMotor motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        // Reverse left motors 
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        motorFrontLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        
        Servo claw = hardwareMap.get(Servo.class, "claw"); 
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm"); 
        DcMotor fourBarCH = hardwareMap.get(DcMotor.class, "fourBarCH");
        DcMotor fourBarEH = hardwareMap.get(DcMotor.class, "fourBarEH");
        DcMotor hang = hardwareMap.get(DcMotor.class, "hang");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        
        fourBarEH.setDirection(DcMotorSimple.Direction.REVERSE);
        fourBarCH.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fourBarEH.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        
        waitForStart();
        
        //arm moves up a little bit
        arm.setPower(-1);
        wrist.setPosition(0);
        claw.setPosition(0);
        sleep(1000);
        arm.setPower(0);
        
        //moves forwards up to the thing
        encoderDrive(DRIVE_SPEED, -15, -15, -15, -15, 200.0);
        
        //wrist goes up and specimen gets hooked on
        wrist.setPosition(1);
        sleep(1000);
        encoderDrive(DRIVE_SPEED, 4, 4, 4, 4, 200.0);
        
        //open claw and move to the pickup point, turning along the way
        claw.setPosition(0.8);
        sleep(500);
        
        encoderDrive(DRIVE_SPEED, -15, 15, 15, -15, 200.0);
        sleep(2000);
        
        encoderDrive(DRIVE_SPEED, 28.25, -28.25, 28.25, -28.25, 200.0);
        sleep(2000);
        
        encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 200.0);
        sleep(2000);
        
        //bring the arm down just a little bit
        wrist.setPosition(0);
        arm.setPower(1);
        sleep(450);
        arm.setPower(0);
        
        //grab the thing and go back up
        // claw.setPosition(0);
        // arm.setPower(-1);
        // sleep(700);
        // arm.setPower(0);
        
        // //move forwards
        // encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 200.0);
        
        sleep(15000);
    }
    
    public void encoderDrive(double speed, double leftInches, double rightInches, double leftBackInches, double rightBackInches, double timeoutS) {
    
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Meccanum Drivetrain
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorFrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motorFrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = motorBackLeft.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = motorBackRight.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            motorFrontLeft.setTargetPosition(newLeftTarget);
            motorFrontRight.setTargetPosition(newRightTarget);
            motorBackLeft.setTargetPosition(newLeftBackTarget);
            motorBackRight.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            motorFrontLeft.setPower(Math.abs(speed));
            motorFrontRight.setPower(Math.abs(speed));
            motorBackLeft.setPower(Math.abs(speed));
            motorBackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                  (runtime.seconds() < timeoutS) &&
                  (motorFrontLeft.isBusy() && motorFrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            motorFrontLeft.getCurrentPosition(),
                                            motorFrontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
