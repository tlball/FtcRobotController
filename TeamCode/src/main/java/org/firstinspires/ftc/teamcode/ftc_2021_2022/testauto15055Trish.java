package org.firstinspires.ftc.teamcode.ftc_2021_2022;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by katie on 10/10/2019.
 */


//@Disabled
@Autonomous(name = "testauto15055-Trish", group = "Autonomus")
public class testauto15055Trish extends OpMode {
    int d = 0;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorLB;//ENCODER
    //private DcMotor motorLF;
    //private DcMotor motorRF;
    private DcMotor motorRB;//ENCODER
    private DcMotor motorL;
    //private Servo servoB;
    private DcMotor motorR;
    private DcMotor motorC;

    double startPosition;
    double liftStartPosition;
    double driveStartPosition;
    double clawStartPosition;
    double rototo;
    double currentPosition;
    double distanceTravelled;
    int v_state = 0;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        motorLB = hardwareMap.dcMotor.get("LB");//ENCODER
        motorRB = hardwareMap.dcMotor.get("RB");//ENCODER
        motorL = hardwareMap.dcMotor.get("lift");
        motorR = hardwareMap.dcMotor.get("duck");
        motorC = hardwareMap.dcMotor.get("succ");
        //
        // motorRF.setDirection(DcMotor.Direction.REVERSE);
        motorLB.setDirection(DcMotor.Direction.REVERSE);//ENCODER
        motorC.setDirection(DcMotor.Direction.REVERSE);
        motorL.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        telemetry.addData("Status", "Running: " + runtime.toString());

        // case 0
        telemetry.addData("Case 0", startPosition);
        telemetry.addData("Case 0", liftStartPosition);
        telemetry.addData("Case 0", rototo);
        startPosition = motorLB.getCurrentPosition();
        liftStartPosition = Math.abs(motorL.getCurrentPosition());
        clawStartPosition = Math.abs(motorC.getCurrentPosition());
        liftStartPosition = Math.abs(motorR.getCurrentPosition());


        telemetry.addData("Case 1", startPosition);
        telemetry.addData("Case 1 - Distance Travelled", distanceTravelled);
        telemetry.update();

        // case 1
        currentPosition = motorLB.getCurrentPosition();
        distanceTravelled = currentPosition - startPosition;
        telemetry.addData("Case 1 - Distance Travelled", distanceTravelled);
        telemetry.update();

        while (distanceTravelled <= 1700) {
            motorLB.setPower(0.5);
            motorRB.setPower(0.5);
        }
        motorL.setPower(0.0);
        telemetry.addData("Case 1 Done - Distance Travelled", distanceTravelled);
        telemetry.update();

        motorLB.setPower(0.0);
        motorRB.setPower(0.0);


        // case 2
        startPosition = motorLB.getCurrentPosition();
        telemetry.addData("Case 2", liftStartPosition);
        currentPosition = Math.abs(motorL.getCurrentPosition());
        telemetry.addData("Case 2", currentPosition);
        distanceTravelled = currentPosition - liftStartPosition;

        while (distanceTravelled <= 1150) {
            telemetry.addData("Case 2 - Distance Travelled", distanceTravelled);
            telemetry.update();
            motorL.setPower(0.5);
        }
        telemetry.addData("Case 2 Done- Distance Travelled", distanceTravelled);
        telemetry.update();
        motorL.setPower(0.00);
        liftStartPosition = motorL.getCurrentPosition();

        // case 3
        telemetry.addData("Case 3", clawStartPosition);
        currentPosition = Math.abs(motorC.getCurrentPosition());
        distanceTravelled = currentPosition - clawStartPosition;
        telemetry.addData("Case 3 - Distance Travelled", distanceTravelled);
        while (distanceTravelled <= 1000) {
            motorC.setPower(-0.5);
        }
        telemetry.addData("Case 3 Done - Distance Travelled", distanceTravelled);
        motorC.setPower(0.0);
        clawStartPosition = motorC.getCurrentPosition();

        // case 4
        telemetry.addData("Case 4", startPosition);
        currentPosition = Math.abs(motorL.getCurrentPosition());
        distanceTravelled = currentPosition - startPosition;
        telemetry.addData("Case 4 - Distance Travelled", distanceTravelled);
        while (distanceTravelled <= 2150) {
            motorL.setPower(-0.5);
            telemetry.addData("Case 4 - Distance Travelled", distanceTravelled);

        }
        telemetry.addData("Case 4 - Distance Travelled", distanceTravelled);
        motorL.setPower(0.00);
        startPosition = motorL.getCurrentPosition();

          /*  case 4: //lift down
                telemetry.addData("Case 4", liftStartPosition);
                currentPosition = Math.abs(motorL.getCurrentPosition());
                distanceTravelled = currentPosition - liftStartPosition;
                if (distanceTravelled >= 1000) {
                    motorL.setPower(-0.5);
                    if (distanceTravelled < 1000){
                        motorL.setPower(0.0);
                    }

                }
                else {
                    motorL.setPower(0.0);
                    liftStartPosition = motorL.getCurrentPosition();
                    v_state++;
                }*/

            /* case 5: //rotate 90 degrees counterclockwise
                telemetry.addData("Case 5", liftStartPosition);
                currentPosition = Math.abs(motorC.getCurrentPosition());
                distanceTravelled = currentPosition - liftStartPosition;
                telemetry.addData("Case 5 - Distance Travelled", distanceTravelled);
                if (distanceTravelled <= 1000) {
                    motorLB.setPower(-0.5);
                    // motorLF.setPower(0.5);
                    motorRB.setPower(0.5);
                    // motorRF.setPower(0.5);
                }
                else {
                    motorLB.setPower(0.0);
                    // motorLF.setPower(0.0);
                    motorRB.setPower(0.0);
                    // motorRF.setPower(0.0);
                    startPosition = motorLB.getCurrentPosition();
                    v_state++;
                }
                break;

            case 6: //drive forward to duck
                telemetry.addData("Case 6", startPosition);
                currentPosition = motorLB.getCurrentPosition();
                distanceTravelled = currentPosition - startPosition;
                if (distanceTravelled <= 2500) {
                    motorLB.setPower(0.5);
                    // motorLF.setPower(0.5);
                    motorRB.setPower(0.5);
                    // motorRF.setPower(0.5);
                }
                else {
                    motorLB.setPower(0.0);
                    // motorLF.setPower(0.0);
                    motorRB.setPower(0.0);
                    // motorRF.setPower(0.0);
                    startPosition = motorLB.getCurrentPosition();
                    v_state++;
                }
                break;

            case 7: //rotates toward carousel
                telemetry.addData("Case 7", startPosition);
                currentPosition = motorLB.getCurrentPosition();
                distanceTravelled = currentPosition - startPosition;
                if (distanceTravelled <= 1000) {
                    motorLB.setPower(0.5);
                    // motorLF.setPower(0.5);
                    motorRB.setPower(-0.5);
                    // motorRF.setPower(0.5);
                }
                else {
                    motorLB.setPower(0.0);
                    // motorLF.setPower(0.0);
                    motorRB.setPower(0.0);
                    // motorRF.setPower(0.0);
                    startPosition = motorLB.getCurrentPosition();
                    v_state++;
                }
                break;

            case 8: //drive to carousel
                telemetry.addData("Case 8", startPosition);
                currentPosition = motorLB.getCurrentPosition();
                distanceTravelled = currentPosition - startPosition;
                if (distanceTravelled <= 1500) {
                    motorLB.setPower(0.5);
                    // motorLF.setPower(0.5);
                    motorRB.setPower(0.5);
                    // motorRF.setPower(0.5);
                }
                else {
                    motorLB.setPower(0.0);
                    // motorLF.setPower(0.0);
                    motorRB.setPower(0.0);
                    // motorRF.setPower(0.0);
                    startPosition = motorLB.getCurrentPosition();
                    v_state++;
                }
                break;

            case 9: //spin carousel
                telemetry.addData("Case 9", startPosition);
                currentPosition = motorR.getCurrentPosition();
                distanceTravelled = currentPosition - startPosition;
                if (distanceTravelled <= 3000) {
                    motorR.setPower(0.4);
                }
                else {
                    motorR.setPower(0.0);
                    startPosition = motorLB.getCurrentPosition();
                    v_state++;
                }
                break;

            case 10: //backup
                telemetry.addData("Case 10", startPosition);
                currentPosition = motorLB.getCurrentPosition();
                distanceTravelled = currentPosition - startPosition;
                if (distanceTravelled <= 3000) {
                    motorLB.setPower(-0.5);
                    // motorLF.setPower(0.5);
                    motorRB.setPower(-0.5);
                    // motorRF.setPower(0.5);
                }
                else {
                    motorLB.setPower(0.0);
                    // motorLF.setPower(0.0);
                    motorRB.setPower(0.0);
                    // motorRF.setPower(0.0);
                    startPosition = motorLB.getCurrentPosition();
                    v_state++;
                }
                break;

            case 11: //rotate to park
                telemetry.addData("Case 11", startPosition);
                currentPosition = motorLB.getCurrentPosition();
                distanceTravelled = currentPosition - startPosition;
                if (distanceTravelled <= 1500) {
                    motorLB.setPower(-0.5);
                    // motorLF.setPower(0.5);
                    motorRB.setPower(0.5);
                    // motorRF.setPower(0.5);
                }
                else {
                    motorLB.setPower(0.0);
                    // motorLF.setPower(0.0);
                    motorRB.setPower(0.0);
                    // motorRF.setPower(0.0);
                    startPosition = motorLB.getCurrentPosition();
                    v_state++;
                }
                break;

            case 12: //drive forward and park
                telemetry.addData("Case 12", startPosition);
                currentPosition = motorLB.getCurrentPosition();
                distanceTravelled = currentPosition - startPosition;
                if (distanceTravelled <= 1500) {
                    motorLB.setPower(0.5);
                    // motorLF.setPower(0.5);
                    motorRB.setPower(0.5);
                    // motorRF.setPower(0.5);
                }
                else {
                    motorLB.setPower(0.0);
                    // motorLF.setPower(0.0);
                    motorRB.setPower(0.0);
                    // motorRF.setPower(0.0);
                    startPosition = motorLB.getCurrentPosition();
                    v_state++;
                }
                break; */

    }
}