/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.ftc_2022_2023;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Drive with Distance Sensor", group="Linear Opmode")
//@Disabled
public class DriveWithDistanceSensor extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private Servo claw;
    private DistanceSensor sensorRange;
    private double currentHeight;
    private double finalPosition;


    // junction heights
    private static final int HIGH_JUCTION=780;
    private static final int MID_JUCTION=582;
    private static final int SMALL_JUCTION=347;
    private static final int NO_JUCTION=22;

    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftTop");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBottom");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightTop");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBottom");

        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");

        // you can use this as a regular DistanceSensor.
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int armMax = 0;
        int armMin = -1250;

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        claw.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        telemetry.addData("Target", finalPosition);
//        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Claw", claw.getPosition());
//            telemetry.update();
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = -gamepad1.left_stick_x * 0.8;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = (axial + lateral + yaw) * 0.3;
            double rightFrontPower = (axial - lateral - yaw) * 0.3;
            double leftBackPower   = (axial - lateral + yaw) * 0.3;
            double rightBackPower  = (axial + lateral - yaw) * 0.3;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // ARM STUFF
            double armSpeed = (gamepad2.left_trigger * 0.8) - (gamepad2.right_trigger * 0.8);

////                //Arm control with bumpers
                if (gamepad2.right_bumper) {
                    arm.setPower(-0.5);
                } else if (gamepad2.left_bumper) {
                    arm.setPower(0.5);
                } else {
                    if (arm.getCurrentPosition() >= armMin && arm.getCurrentPosition() <= armMax) {
                        arm.setPower(armSpeed);
                    } else {
                        arm.setPower(0);
                    }
                }

            double autoArmSpeed = 0.8;
            currentHeight = sensorRange.getDistance(DistanceUnit.MM);
            if (gamepad2.y) {
                finalPosition = HIGH_JUCTION;
                telemetry.addData("Target", finalPosition);

                if (currentHeight > finalPosition) {
                    while(opModeIsActive() && currentHeight > finalPosition) {
                        telemetry.addData("Action", "Lowering Height");
                        telemetry.addData("Current height", currentHeight);
                        telemetry.addData("Target Height", finalPosition);
                        telemetry.update();
                        currentHeight = sensorRange.getDistance(DistanceUnit.MM);
                        arm.setPower(autoArmSpeed);
                    }
                    arm.setPower(0);
                } else if (currentHeight < finalPosition) {
                    while(opModeIsActive() && currentHeight < finalPosition) {
                        telemetry.addData("Action", "Raising Height");
                        telemetry.addData("Current height", currentHeight);
                        telemetry.addData("Target Height", finalPosition);
                        telemetry.update();
                        currentHeight = sensorRange.getDistance(DistanceUnit.MM);
                        arm.setPower(-autoArmSpeed);
                    }
                    arm.setPower(0);
                }
            } else if (gamepad2.b) {
                finalPosition = MID_JUCTION;
                telemetry.addData("Target", finalPosition);

                if (currentHeight > finalPosition) {
                    while(opModeIsActive() && currentHeight > finalPosition) {
                        telemetry.addData("Action", "Lowering Height");
                        telemetry.addData("Current height", currentHeight);
                        telemetry.addData("Target Height", finalPosition);
                        telemetry.update();
                        currentHeight = sensorRange.getDistance(DistanceUnit.MM);
                        arm.setPower(autoArmSpeed);
                    }
                    arm.setPower(0);
                } else if (currentHeight < finalPosition) {
                    while(opModeIsActive() && currentHeight < finalPosition) {
                        telemetry.addData("Action", "Raising Height");
                        telemetry.addData("Current height", currentHeight);
                        telemetry.addData("Target Height", finalPosition);
                        telemetry.update();
                        currentHeight = sensorRange.getDistance(DistanceUnit.MM);
                        arm.setPower(-autoArmSpeed);
                    }
                    arm.setPower(0);
                }

            } else if (gamepad2.a) {
                finalPosition = SMALL_JUCTION;
                telemetry.addData("Target", finalPosition);

                if (currentHeight > finalPosition) {
                    while(opModeIsActive() && currentHeight > finalPosition) {
                        telemetry.addData("Action", "Lowering Height");
                        telemetry.addData("Current height", currentHeight);
                        telemetry.addData("Target Height", finalPosition);
                        telemetry.update();
                        currentHeight = sensorRange.getDistance(DistanceUnit.MM);
                        arm.setPower(autoArmSpeed);
                    }
                    arm.setPower(0);
                } else if (currentHeight < finalPosition) {
                    while(opModeIsActive() && currentHeight < finalPosition) {
                        telemetry.addData("Action", "Raising Height");
                        telemetry.addData("Current height", currentHeight);
                        telemetry.addData("Target Height", finalPosition);
                        telemetry.update();
                        currentHeight = sensorRange.getDistance(DistanceUnit.MM);
                        arm.setPower(-autoArmSpeed);
                    }
                    arm.setPower(0);
                }
            }
            else if (gamepad2.x) {
                finalPosition = NO_JUCTION;
                telemetry.addData("Target", finalPosition);

                if (currentHeight > finalPosition) {
                    while (opModeIsActive() && currentHeight > finalPosition) {
                        telemetry.addData("Action", "Lowering Height");
                        telemetry.addData("Current height", currentHeight);
                        telemetry.addData("Target Height", finalPosition);
                        telemetry.update();
                        currentHeight = sensorRange.getDistance(DistanceUnit.MM);
                        arm.setPower(autoArmSpeed);
                    }
                    arm.setPower(0);
                } else if (currentHeight < finalPosition) {
                    while (opModeIsActive() && currentHeight < finalPosition) {
                        telemetry.addData("Action", "Raising Height");
                        telemetry.addData("Current height", currentHeight);
                        telemetry.addData("Target Height", finalPosition);
                        telemetry.update();
                        currentHeight = sensorRange.getDistance(DistanceUnit.MM);
                        arm.setPower(-autoArmSpeed);
                    }
                    arm.setPower(0);
                }
            }

                if (gamepad2.dpad_left) {
                // then do this stuff
                claw.setPosition(0.8);
            } else {
                // do something else
                claw.setPosition(0);
            }
        }
    }


    public void backwardDrive(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    public void forwardDrive(double power) {
        backwardDrive(-power);
    }

    public void rightStrafe(double power) {
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(-power);
    }

    public void leftStrafe(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(power);
    }
}
