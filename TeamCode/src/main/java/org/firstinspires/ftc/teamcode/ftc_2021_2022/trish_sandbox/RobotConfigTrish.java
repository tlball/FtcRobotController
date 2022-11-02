/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.ftc_2021_2022.trish_sandbox;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class RobotConfigTrish
{
    /* Public OpMode members. */
    public DcMotor leftTop = null;
    public DcMotor leftBottom = null;
    public DcMotor rightTop = null;
    public DcMotor rightBottom = null;
    public CRServo duckWheelRight = null;
    public CRServo duckWheelLeft = null;
    public DcMotor arm = null;
    public Servo claw = null;

    public static final double CLAW_OPEN    =  0 ;
    public static final double CLAW_CLOSED  =  1 ;
    public static final double CLAW_MID     =  0.5 ;

    //    public static final double ARM_UP_POWER    =  0.45 ;
    //    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hardwareMap  =  null;
    Telemetry telemetry      = null;

    public ElapsedTime runtime  = new ElapsedTime();

    /* Constructor */
    public RobotConfigTrish() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry tltry) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        // Set up telemetry
        telemetry = tltry;

                // Define and Initialize Motors
        leftTop = hardwareMap.get(DcMotor.class, "leftTop");
        leftBottom = hardwareMap.get(DcMotor.class, "leftBottom");
        rightTop = hardwareMap.get(DcMotor.class, "rightTop");
        rightBottom = hardwareMap.get(DcMotor.class, "rightBottom");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        duckWheelRight = hardwareMap.get(CRServo.class, "duckWheel1");
        duckWheelLeft = hardwareMap.get(CRServo.class, "duckWheel2");

        leftTop.setDirection(DcMotor.Direction.REVERSE);
        leftBottom.setDirection(DcMotor.Direction.REVERSE);
        rightTop.setDirection(DcMotor.Direction.FORWARD);
        rightBottom.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        drive(0, 0);

        // Reset all encoders.
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initialize ALL installed servos.
        claw.setPosition(CLAW_CLOSED);
    }

    public void driveToPosition(int drivePosition) {
        int distance = leftBottom.getCurrentPosition() + drivePosition;
        telemetry.addData("distance", drivePosition);
        telemetry.addData("leftBottom Current position", leftBottom.getCurrentPosition());
        telemetry.addData("leftBottom New position", distance);
        leftBottom.setTargetPosition(distance);
        leftTop.setTargetPosition(distance);
        rightBottom.setTargetPosition(distance);
        rightTop.setTargetPosition(distance);
        telemetry.update();

//        sleep(2000);


        leftBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.update();
        drive(0.5, 0.5);

        while (leftBottom.isBusy()) {
            telemetry.addData("leftBottom Actual position", leftBottom.getCurrentPosition());
            telemetry.update();

            // do nothing
//            telemetry.addData("Arm position", arm.getCurrentPosition());    //
//            telemetry.update();

        }
//        sleep(2000);

        // 6. Turn off the motor
        drive(0, 0);
    }

    public void strafeToPosition(int strafePosition) {
        leftTop.setTargetPosition(leftTop.getCurrentPosition() - strafePosition);
        leftBottom.setTargetPosition(leftBottom.getCurrentPosition() + strafePosition);
        rightTop.setTargetPosition(rightTop.getCurrentPosition() + strafePosition);
        rightBottom.setTargetPosition(rightBottom.getCurrentPosition() - strafePosition);

        leftBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(0.5, 0.5);

        while (leftBottom.isBusy()) {
            // do nothing
        }

        // 6. Turn off the motor
        drive(0, 0);
    }

    public void left(double power) {
        try {
            leftBottom.setPower(power);
            leftTop.setPower(power);
        } catch (Exception ex) {
        }
    }

    public void right(double power) {
        try {
            rightBottom.setPower(power);
            rightTop.setPower(power);
        } catch (Exception ex) {
        }
    }

    public void drive(double left, double right) {
        left(left);
        right(right);
    }


    public void strafeLeft(double speed) {
        telemetry.addData("Strafe speed", speed);


        leftTop.setPower(-speed * 0.75);
        leftBottom.setPower(speed * 0.75);
        rightTop.setPower(speed * 0.75);
        rightBottom.setPower(-speed * 0.75);
    }

    public void strafeRight(double speed) {
        telemetry.addData("moveRight speed", speed);


        strafeLeft(-speed);
    }

//    public void controlArm(double speed) {
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        arm.setPower(speed);
//    }

    public void raiseArmToPosition(int position) {
        if (position == 0) {
            return; // do nothing
        }

//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //       arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Distance", position);    //
        telemetry.addData("Starting Arm position", arm.getCurrentPosition());    //

        int armPosition = arm.getCurrentPosition() + position;
        telemetry.addData("New Arm position", armPosition);    //

        if (armPosition < 0) {
            return; // do nothing
        }

        arm.setTargetPosition(armPosition);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        telemetry.addData("Arm position", arm.getCurrentPosition());    //
//        telemetry.update();
        arm.setPower(0.5);
        while (arm.isBusy()) {
            // do nothing
        }
    }

    public void ducks(double duckSpeed) {
        duckWheelRight.setPower(duckSpeed);
    }

    public void setClawOpen() {
      claw.setPosition(CLAW_OPEN);
    }

    public void setClawClosed() {
        claw.setPosition(CLAW_CLOSED);
    }

    public void setClawMid() {
        claw.setPosition(CLAW_MID);
    }
 }

