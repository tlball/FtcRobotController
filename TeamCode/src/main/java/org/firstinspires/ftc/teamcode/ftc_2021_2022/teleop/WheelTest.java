package org.firstinspires.ftc.teamcode.ftc_2021_2022.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Adapted from code at
// https://github.com/KNO3Robotics/FTC/blob/master/FtcRobotController/src/main/java/com/qualcomm/ftcrobotcontroller/season/resq2015/atlas/v4/robot/Drive.java#L34


@TeleOp(name = "Wheel Test", group = "Iterative Opmode")
//@Disabled
public class WheelTest extends OpMode {
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

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    boolean reverse = false;

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double speed = 0.75;

        if(gamepad1.left_bumper) {
            speed = -speed;
        }

        if (gamepad1.x) {
            leftTop.setPower(speed);
        } else {
            leftTop.setPower(0);
        }
        if (gamepad1.y) {
            leftBottom.setPower(speed);
        } else {
            leftBottom.setPower(0);
        }
        if (gamepad1.a) {
            rightTop.setPower(speed);
        } else {
            rightTop.setPower(0);
        }
        if (gamepad1.b) {
            rightBottom.setPower(speed);
        } else {
            rightBottom.setPower(0);
        }

        if(gamepad1.dpad_left) {
            leftTop.setPower(-speed);
            leftBottom.setPower(speed);
            rightTop.setPower(speed);
            rightBottom.setPower(-speed);
        } else if (gamepad1.dpad_right) {
            leftTop.setPower(speed);
            leftBottom.setPower(-speed);
            rightTop.setPower(-speed);
            rightBottom.setPower(speed);
        } else {
            leftTop.setPower(0);
            leftBottom.setPower(0);
            rightTop.setPower(0);
            rightBottom.setPower(0);
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

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}