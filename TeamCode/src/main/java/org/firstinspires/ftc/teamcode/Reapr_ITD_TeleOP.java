package org.firstinspires.ftc.teamcode;

import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

@TeleOp(name = "Robot Centric")  // Defines this class as a TeleOp mode
public class Reapr_ITD_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Gamepad state tracking for current and previous states
        Gamepad currPad1 = new Gamepad();
        Gamepad currPad2 = new Gamepad();
        Gamepad prevPad1 = new Gamepad();
        Gamepad prevPad2 = new Gamepad();

        // Get all Lynx hardware hubs and set bulk caching mode for performance optimization
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize drive motors
        DcMotor motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        // Reverse left motors to ensure proper driving direction
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all drive motors to brake mode when power is zero
        motorFrontLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Initialize additional mechanisms
        Servo claw = hardwareMap.get(Servo.class, "claw"); 
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        DcMotor fourBarCH = hardwareMap.get(DcMotor.class, "fourBarCH");
        DcMotor fourBarEH = hardwareMap.get(DcMotor.class, "fourBarEH");
        DcMotor hang = hardwareMap.get(DcMotor.class, "hang");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");

        // Initialize servo for additional arm movement
        Servo arm1 = hardwareMap.get(Servo.class, "arm1");
        arm1.setDirection(Servo.Direction.REVERSE);

        // Configure behaviors for additional motors
        arm.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fourBarEH.setDirection(DcMotorSimple.Direction.REVERSE);
        fourBarCH.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fourBarEH.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        waitForStart();  // Waits for the driver to press the start button

        if (isStopRequested()) return;

        double dividePower = 1;  // Power scaling factor for speed control

        while (opModeIsActive()) {
            // Update previous gamepad states before reading new inputs
            prevPad1.copy(currPad1);
            prevPad2.copy(currPad2);
            
            currPad1.copy(gamepad1);
            currPad2.copy(gamepad2);

            // Toggle speed reduction mode
            if (gamepad1.left_stick_button || gamepad1.dpad_right) {
                dividePower = Math.abs(dividePower - 1.0) < 0.1 ? 1.5 : 1.0;
                sleep(500);  // Prevents rapid toggling
            }

            // Read joystick values for driving
            double y = -gamepad1.left_stick_y;       // Forward/backward (inverted)
            double x =  gamepad1.left_stick_x * 1.1; // Strafing (scaled to counteract imperfections)
            double rx = gamepad1.right_stick_x;      // Rotation

            // Normalize motor powers to prevent values outside the range [-1,1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            motorFrontLeft.setPower((y + x + rx) / denominator / dividePower);
            motorBackLeft.setPower((y - x + rx) / denominator / dividePower);
            motorFrontRight.setPower((y - x - rx) / denominator / dividePower);
            motorBackRight.setPower((y + x - rx) / denominator / dividePower);

            // Control four-bar mechanism
            fourBarCH.setPower(-gamepad2.left_stick_y);
            fourBarEH.setPower(-gamepad2.left_stick_y);

            // Claw control using triggers
            if (gamepad2.left_trigger > 0.1 || gamepad1.left_trigger > 0.1) {
                claw.setPosition(0.2);  // Close claw
            }
            if (gamepad2.right_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                claw.setPosition(0.8);  // Open claw
            }

            // Arm control
            arm.setPower(gamepad2.right_stick_y);

            // Hang mechanism control
            if (gamepad1.y) {
                hang.setPower(1);
                hang.setPower(0);
            }
            if (gamepad1.x) {
                hang.setPower(-1);
                sleep(10000);
                hang.setPower(0);
            }

            // Arm1 position control (incremental)
            if (currPad2.a && !prevPad2.a) {
                if (arm1.getPosition() < 1) 
                    arm1.setPosition(arm1.getPosition() + 0.15);
            }
            if (currPad2.y && !prevPad2.y) {
                if (arm1.getPosition() > 0) 
                    arm1.setPosition(arm1.getPosition() - 0.15);
            }

            // Wrist control using D-pad
            if (gamepad2.dpad_right) {
                wrist.setPosition(0);   // Set wrist to default position
            }
            if (gamepad2.dpad_up) {
                wrist.setPosition(0.17);  // Adjust wrist slightly upwards
            }
            if (gamepad2.dpad_down) {
                wrist.setPosition(0.5);   // Move wrist further down
            }
        }
    }
}