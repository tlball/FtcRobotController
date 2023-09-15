package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



@TeleOp(name="FieldCentricDrive", group="Sample")
public class FieldCentricDriveChatGPT extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeftDrive  = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        backLeftDrive   = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive  = hardwareMap.dcMotor.get("backRightDrive");

        // Initialize the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;  // Reverse this to change the direction of forward movement
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Get the current robot orientation
            Orientation angles = imu.getAngularOrientation(AxesOrder.ZYX, AngleUnit.DEGREES);
            double robotAngle = Math.toRadians(angles.firstAngle);  // Convert to radians

            // Field-centric calculations
            double temp = drive * Math.cos(robotAngle) - strafe * Math.sin(robotAngle);
            strafe = drive * Math.sin(robotAngle) + strafe * Math.cos(robotAngle);
            drive = temp;

            // Calculate powers for each wheel
            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower = drive - strafe + rotate;
            double backRightPower = drive + strafe - rotate;

            // Normalize powers so no value exceeds 1.0
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
        }
    }
}