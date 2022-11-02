package org.firstinspires.ftc.teamcode.ftc_2021_2022;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Adapted from code at
// https://github.com/KNO3Robotics/FTC/blob/master/FtcRobotController/src/main/java/com/qualcomm/ftcrobotcontroller/season/resq2015/atlas/v4/robot/Drive.java#L34


@TeleOp(name = "Arcade Drive", group = "Iterative Opmode")
//@Disabled
public class Drive extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftTop = null;
    private DcMotor leftBottom = null;
    private DcMotor rightTop = null;
    private DcMotor rightBottom = null;
    private DcMotor arm = null;
    private Servo claw = null;
    private CRServo duckWheel1 = null;
    private CRServo duckWheel2 = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftTop = hardwareMap.get(DcMotor.class, "leftTop");
        leftBottom = hardwareMap.get(DcMotor.class, "leftBottom");
        rightTop = hardwareMap.get(DcMotor.class, "rightTop");
        rightBottom = hardwareMap.get(DcMotor.class, "rightBottom");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        duckWheel1 = hardwareMap.get(CRServo.class, "duckWheel1");
        duckWheel2 = hardwareMap.get(CRServo.class, "duckWheel2");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftTop.setDirection(DcMotor.Direction.REVERSE);
        leftBottom.setDirection(DcMotor.Direction.REVERSE);
        rightTop.setDirection(DcMotor.Direction.FORWARD);
        rightBottom.setDirection(DcMotor.Direction.FORWARD);

        // 1. Set the encoder value to 0
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        drive(0, 0);
    }


    boolean reverse = false;

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double speed_variant = 0.7;
        double speed = gamepad1.right_trigger * speed_variant - gamepad1.left_trigger * speed_variant;
        double direction = -gamepad1.left_stick_x;
        double armSpeed = gamepad2.left_trigger * 0.5 - gamepad2.right_trigger * 0.5;
        if (armSpeed != 0) {
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        armRun(armSpeed);
        double duckSpeed = 1.0;
        duckRun1(duckSpeed);
        duckRun2(duckSpeed);


        if (gamepad1.x) {
            reverse = false;
        } else if (gamepad1.y) {
            reverse = true;
        }

        DriveDir driveDir = getDir(direction);

        if (Math.abs(speed) < 5 && driveDir != DriveDir.STRAIGHT) {
            drive(-direction, direction);
        } else {
            if (reverse) {
                drive(driveDir.getLeft(reverse) * speed, driveDir.getRight(reverse) * speed);
            } else {
                drive(driveDir.getRight(reverse) * speed, driveDir.getLeft(reverse) * speed);
            }
        }

        if(gamepad1.dpad_right) {
            moveRight(0.75);
        } else if(gamepad1.dpad_left) {
            moveLeft(0.75);
        } else {
            moveLeft(0);
        }
        if (gamepad2.left_bumper) {
            claw.setPosition(1.0 );
        } else if(gamepad2.right_bumper) {
            claw.setPosition(0.0);
        } else if (gamepad2.x) {
            claw.setPosition(0.5);
        } else if (gamepad2.a) {
            claw.setPosition(1.0);
        }

        if (gamepad2.b) {
            // 2. Tell the motor that we were using the encoder
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // 3. Determine the new position (this takes the current position and adds 20 to it)
            int newPosition = arm.getCurrentPosition() + 100;

            // 4. Set the motor to use the new position
            arm.setTargetPosition(newPosition);


            // 5. Tell motor to RUN_TO_POSITION
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            arm.setPower(0.5);

            while(arm.isBusy()) {
                // do nothing
            }
        }

        telemetry.addData("Drive - Dat - speed", speed);
        telemetry.addData("Drive - Dat - direction", driveDir.name());
        telemetry.addData("Drive - Set - leftTop", leftTop.getPower());
        telemetry.addData("Drive - Set - leftBottom", leftBottom.getPower());
        telemetry.addData("Drive - Set - rightTop", rightTop.getPower());
        telemetry.addData("Drive - Set - rightBottom", rightBottom.getPower());
        telemetry.addData("Drive - Enc - Left", leftTop.getCurrentPosition());
        telemetry.addData("Drive - Enc - Right", rightTop.getCurrentPosition());
        telemetry.addData("Arm - Enc", arm.getCurrentPosition());
        telemetry.update();

    }

    public void moveLeft(double speed) {
        telemetry.addData("Strafe speed", speed);


        leftTop.setPower(-speed * 0.75);
        leftBottom.setPower(speed * 0.75);
        rightTop.setPower(speed * 0.75);
        rightBottom.setPower(-speed * 0.75);
    }

    public void moveRight(double speed) {
        telemetry.addData("moveRight speed", speed);


        moveLeft(-speed);
    }

    public enum DriveDir {
        HARD_LEFT(-1, 1),
        MEDIUM_LEFT(0, 1),
        EASY_LEFT(0.5, 1),
        STRAIGHT(1, 1),
        EASY_RIGHT(1, 0.5),
        MEDIUM_RIGHT(1, 0),
        HARD_RIGHT(1, -1);

        private double left, right;

        DriveDir(double left, double right) {
            this.left = left;
            this.right = right;
        }

        public double getLeft(boolean reverse) {
            return reverse ? -left : left;
        }

        public double getRight(boolean reverse) {
            return reverse ? -right : right;
        }
    }

    private void left(double power) {
        try {
            leftTop.setPower(power);
            leftBottom.setPower(power);
        } catch (Exception ex) {
        }
    }

    private void right(double power) {
        try {
            rightTop.setPower(power);
            rightBottom.setPower(power * 1.2);
        } catch (Exception ex) {
        }
    }

    public void drive(double left, double right) {
        left(left);
        right(right);
    }

    private void armRun(double armSpeed) {
        try {
            double armVarient = 1.0;
            if (gamepad2.dpad_left) {
                armVarient = 0.5;
            }
            arm.setPower(armSpeed * armVarient);
        } catch (Exception ex) {
        }
    }

    public void duckRun1(double duckSpeed) {
        if (gamepad2.dpad_left) {
            duckWheel1.setPower(duckSpeed);
        }
        else if (gamepad2.dpad_right) {
            duckWheel1.setPower(-duckSpeed);
        }
        else {
            duckWheel1.setPower(0);
        }
    }

    public void duckRun2(double duckSpeed) {
        if (gamepad2.dpad_up) {
            duckWheel2.setPower(duckSpeed);
        }

        else if (gamepad2.dpad_down) {
            duckWheel2.setPower(-duckSpeed);
        }
        else {
            duckWheel2.setPower(0);
        }
    }



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    private Drive.DriveDir getDir(double dir) {
        if (dir <= -0.7) {
            return Drive.DriveDir.HARD_LEFT;
        } else if (dir <= -0.4) {
            return Drive.DriveDir.MEDIUM_LEFT;
        } else if (dir <= -0.1) {
            return Drive.DriveDir.EASY_LEFT;
        } else if (dir >= 0.7) {
            return Drive.DriveDir.HARD_RIGHT;
        } else if (dir >= 0.4) {
            return Drive.DriveDir.MEDIUM_RIGHT;
        } else if (dir >= 0.1) {
            return Drive.DriveDir.EASY_RIGHT;
        } else {
            return Drive.DriveDir.STRAIGHT;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}