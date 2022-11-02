package org.firstinspires.ftc.teamcode.ftc_2021_2022.archive.trish_sandbox;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Adapted from code at
// https://github.com/KNO3Robotics/FTC/blob/master/FtcRobotController/src/main/java/com/qualcomm/ftcrobotcontroller/season/resq2015/atlas/v4/robot/Drive.java#L34


@TeleOp(name = "Trish - Arcade Drive", group = "Iterative Opmode")
//@Disabled
public class DriveTrish extends OpMode {
    // Declare OpMode members.
    RobotConfigTrish robot   = new RobotConfigTrish();   // Use a Pushbot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        robot.init(hardwareMap, telemetry);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    boolean reverse = false;

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        arcadeDrive();

        int armDistanceUp = (int)(gamepad2.left_trigger * 50);
        robot.raiseArmToPosition(armDistanceUp);

        int armDistanceDown = (int)(gamepad2.right_trigger * 50);
        robot.raiseArmToPosition(-armDistanceDown);

        double duckSpeed = 1.0;
        if (gamepad2.dpad_left) {
            robot.ducks(duckSpeed);
        } else if (gamepad2.dpad_right) {
            robot.ducks(-duckSpeed);
        }
        else {
            robot.ducks(0);
        }

        duckRunLeft(duckSpeed);

        if(gamepad1.dpad_right) {
            robot.strafeRight(0.75);
        } else if(gamepad1.dpad_left) {
            robot.strafeLeft(0.75);
        } else {
            robot.drive(0, 0);
        }

        if (gamepad2.left_bumper) {
            robot.setClawClosed();
        } else if(gamepad2.right_bumper) {
            robot.setClawOpen();
        } else if (gamepad2.x) {
            robot.setClawMid();
        } else if (gamepad2.a) {
            robot.setClawClosed();
        }

        if (gamepad2.b) {
            int distance = 100;
            robot.raiseArmToPosition(distance);
        }
    }

    public void arcadeDrive() {
        double speed_variant = 0.7;
        double speed = gamepad1.right_trigger * speed_variant - gamepad1.left_trigger * speed_variant;
        double direction = -gamepad1.left_stick_x;

        if (gamepad1.x) {
            reverse = false;
        } else if (gamepad1.y) {
            reverse = true;
        }

        DriveDir driveDir = getDir(direction);

        if (Math.abs(speed) < 5 && driveDir != DriveDir.STRAIGHT) {
            robot.drive(-direction, direction);
        } else {
            if (reverse) {
                robot.drive(driveDir.getLeft(reverse) * speed, driveDir.getRight(reverse) * speed);
            } else {
                robot.drive(driveDir.getRight(reverse) * speed, driveDir.getLeft(reverse) * speed);
            }
        }
    }

    public void duckRunLeft(double duckSpeed) {
        if (gamepad2.dpad_up) {
            robot.duckWheelLeft.setPower(duckSpeed);
        }

        else if (gamepad2.dpad_down) {
            robot.duckWheelLeft.setPower(-duckSpeed);
        }
        else {
            robot.duckWheelLeft.setPower(0);
        }
    }

    private DriveDir getDir(double dir) {
        if (dir <= -0.7) {
            return DriveDir.HARD_LEFT;
        } else if (dir <= -0.4) {
            return DriveDir.MEDIUM_LEFT;
        } else if (dir <= -0.1) {
            return DriveDir.EASY_LEFT;
        } else if (dir >= 0.7) {
            return DriveDir.HARD_RIGHT;
        } else if (dir >= 0.4) {
            return DriveDir.MEDIUM_RIGHT;
        } else if (dir >= 0.1) {
            return DriveDir.EASY_RIGHT;
        } else {
            return DriveDir.STRAIGHT;
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
        robot.runtime.reset();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
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

}