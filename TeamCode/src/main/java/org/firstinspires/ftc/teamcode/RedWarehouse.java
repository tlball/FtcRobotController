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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Red Warehouse", group="Autonomous")
//@Disabled
public class RedWarehouse extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftTop;
    private DcMotor leftBottom;
    private DcMotor rightTop;
    private DcMotor rightBottom;
    private DcMotor arm;
    private Servo claw;

    public void setup() {

        leftTop = hardwareMap.get(DcMotor.class, "leftTop");
        leftBottom = hardwareMap.get(DcMotor.class, "leftBottom");
        rightTop = hardwareMap.get(DcMotor.class, "rightTop");
        rightBottom = hardwareMap.get(DcMotor.class, "rightBottom");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");

        leftTop.setDirection(DcMotor.Direction.REVERSE);
        leftBottom.setDirection(DcMotor.Direction.REVERSE);
        rightTop.setDirection(DcMotor.Direction.FORWARD);
        rightBottom.setDirection(DcMotor.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initiated");    //

        telemetry.update();
    }

    @Override
    public void runOpMode() {

        setup();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        claw.setPosition(1.0);
        sleep(1000);

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        raiseArm(150);

        strafeToPosition(890);

        raiseArm(-145);

        // Step 1:  Drive forward for 3 seconds
        /*arm.setPower(0.3);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        strafeRight(0.3);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 4.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Stop
        strafeRight(0); //Stops motors
        arm.setPower(0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);*/
    }


    private void driveToPosition(int drivePosition) {
        int distance = leftBottom.getCurrentPosition() + drivePosition;
        leftBottom.setTargetPosition(distance);
        leftTop.setTargetPosition(distance);
        rightBottom.setTargetPosition(distance);
        rightTop.setTargetPosition(distance);

        leftBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(0.5, 0.5);

        while (leftBottom.isBusy()) {
            // do nothing
//            telemetry.addData("Arm position", arm.getCurrentPosition());    //
//            telemetry.update();

        }
        // 6. Turn off the motor
        drive(0, 0);
    }

    private void strafeToPosition(int strafePosition) {
        leftTop.setTargetPosition(leftTop.getCurrentPosition() + strafePosition);
        leftBottom.setTargetPosition(leftBottom.getCurrentPosition() - strafePosition);
        rightTop.setTargetPosition(rightTop.getCurrentPosition() - strafePosition);
        rightBottom.setTargetPosition(rightBottom.getCurrentPosition() + strafePosition);

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
            rightBottom.setPower(power);
        } catch (Exception ex) {
        }
    }

    public void drive(double left, double right) {
        left(left);
        right(right);
    }


    private void raiseArm(int position) {
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //       arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Distance", position);    //
        telemetry.addData("Starting Arm position", arm.getCurrentPosition());    //
        int armPosition = arm.getCurrentPosition() + position;
        telemetry.addData("New Arm position", armPosition);    //

        arm.setTargetPosition(armPosition);


        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        telemetry.addData("Arm position", arm.getCurrentPosition());    //
//        telemetry.update();
        arm.setPower(0.5);
        while (arm.isBusy()) {
            // do nothing
        }
        telemetry.addData("Actual Arm position", arm.getCurrentPosition());    //
//        arm.setPower(0);
        sleep(2000);
        telemetry.addData("Fallen Arm position", arm.getCurrentPosition());    //
        telemetry.update();
        sleep(1000);
    }


}
