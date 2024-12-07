// Base code from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import java.util.List;

@TeleOp(name = "TeleOP")
public class Reapr_ITD_TeleOP extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Declare our motors
        
        // Meccanum Drivetrain
        DcMotor motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Claw Control
        Servo claw = hardwareMap.get(Servo.class, "claw");
        CRServo arm = hardwareMap.get(CRServo.class, "arm");
        DcMotor slide = hardwareMap.get(DcMotor.class, "slide");
        
        waitForStart();

       if (isStopRequested()) return;

        boolean isSlowMode = false;
        double dividePower = 1.0;

        while (opModeIsActive()) {
            //! Control Speed
            if (isSlowMode) {
                dividePower = 1.5;
            } else {
                dividePower = 1.0;
            }

            if (gamepad1.left_stick_button || gamepad1.dpad_right) {
                if (isSlowMode) {
                    isSlowMode = false;
                    sleep(500);
                } else {
                    isSlowMode = true;
                    sleep(500);
                }
            }

            //! Mecccanum controls
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x =  gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            motorFrontLeft.setPower((y + x + rx) / denominator / dividePower);//Positive rotation results in forward & right motion
            motorBackLeft.setPower((y - x + rx) / denominator / dividePower);//Positive rotation results in forward & left motion
            motorFrontRight.setPower((y - x - rx) / denominator / dividePower);//Positive rotation results in forward & left motion
            motorBackRight.setPower((y + x - rx) / denominator / dividePower);//Positive rotation results in forward & right motion

            // arm control stuff            
            slide.setPower(gamepad2.left_stick_y / dividePower);
            
            // move claw open on y if not already at lowest position.
            if (gamepad2.y){
                claw.setPosition(0);
            }

            // move claw closed on a if not already at the highest position.
            if (gamepad2.a){
                claw.setPosition(1);
            }
        
            if (gamepad2.x) {
                arm.setPower(1);
            } else if (gamepad2.b) {
                arm.setPower(-1);
            }
            arm.setPower(0);
        }
    }
}