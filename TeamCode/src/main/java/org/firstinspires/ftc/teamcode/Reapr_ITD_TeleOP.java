package org.firstinspires.ftc.teamcode;
import java.util.List;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;


enum State {
    REST,
    HIGH,
    MID,
    LOW
}

@TeleOp(name = "TeleOP")
public class Reapr_ITD_TeleOp_Copy extends LinearOpMode {
    final double top = 0.7;
    final double chill = 0.85;
    final double bott = 1.0;
    
    State state = State.REST;
    
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currPad1 = new Gamepad();
        Gamepad currPad2 = new Gamepad();
        Gamepad prevPad1 = new Gamepad();
        Gamepad prevPad2 = new Gamepad();
        
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
        
        boolean hangActive = false;
        
        Servo arm1 = hardwareMap.get(Servo.class, "arm1");
        arm1.setDirection(Servo.Direction.REVERSE);
        
        fourBarEH.setDirection(DcMotorSimple.Direction.REVERSE);
        fourBarCH.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fourBarEH.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        
        waitForStart();

       if (isStopRequested()) return;
       
        double dividePower = 1.0;

        while (opModeIsActive()) {
            prevPad1.copy(currPad1);
            prevPad2.copy(currPad2);
            
            currPad1.copy(gamepad1);
            currPad2.copy(gamepad2);
            
            if (gamepad1.left_stick_button || gamepad1.dpad_right) {
                dividePower = Math.abs(dividePower - 1.0) < 0.1 ? 1.5 : 1.0;
                sleep(500);
            }

            double y = -gamepad1.left_stick_y;       // Remember, this is reversed!
            double x =  gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            motorFrontLeft.setPower((y + x + rx) / denominator / dividePower);
            motorBackLeft.setPower((y - x + rx) / denominator / dividePower);
            motorFrontRight.setPower((y - x - rx) / denominator / dividePower);
            motorBackRight.setPower((y + x - rx) / denominator / dividePower);

            fourBarCH.setPower(-gamepad2.left_stick_y * 0.8);
            fourBarEH.setPower(-gamepad2.left_stick_y * 0.8);
            
            if (gamepad2.left_trigger > 0.1 || gamepad1.left_trigger > 0.1){
                claw.setPosition(0);
            }

            if (gamepad2.right_trigger > 0.1 || gamepad1.right_trigger > 0.1){
                claw.setPosition(0.8);
            }
            
            arm.setPower(-gamepad2.right_stick_y);
            
            if (gamepad1.y) {
                hang.setPower(1);
                hang.setPower(0);
            }
            if (gamepad1.x) {
                hang.setPower(-1);
                sleep(3000);
                hang.setPower(0);
            }
            
            //this part is currently under revision, don't mess with it avyay
            if (currPad2.y && !prevPad2.y) {
                State temp = state;
                switch(temp) {
                    case HIGH:
                        break;
                    case MID:
                        arm1.setPosition(top);
                        state = State.HIGH;
                        break;
                    case LOW:
                        arm1.setPosition(chill);
                        state = State.MID;
                        break;
                    default:
                        break;
                }
            }
            
            if (currPad2.a && !prevPad2.a) {
                State temp = state;
                switch(temp) {
                    case REST:
                        arm1.setPosition(top);
                        state = State.HIGH;
                        break;
                    case HIGH:
                        arm1.setPosition(chill);
                        state = State.MID;
                        break;
                    case LOW:
                        break;
                    case MID:
                        arm1.setPosition(bott);
                        state = State.LOW;
                        break;
                    default:
                        break;
                }
            }
            
            if (gamepad2.dpad_right) {
                wrist.setPosition(0.28);
            }
            
            if (gamepad2.dpad_up) {
                wrist.setPosition(0);
            }
            
            if (gamepad2.dpad_down) {
                wrist.setPosition(1);
            }
        }
    }
}