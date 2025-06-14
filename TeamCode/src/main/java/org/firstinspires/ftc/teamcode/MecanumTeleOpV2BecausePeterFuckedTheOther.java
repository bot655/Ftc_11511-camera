package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="MecanumTeleOp")
public class MecanumTeleOpV2BecausePeterFuckedTheOtherOne extends LinearOpMode {
    private CRServo crServo;

    // Arm limits
    private final int ARM_MIN_POSITION = 0;
    private final int ARM_MAX_POSITION = 1600;

    @Override
    public void runOpMode() throws InterruptedException {
        // Motors
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftRear = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightRear = hardwareMap.dcMotor.get("rightBack");
        DcMotor big_arm = hardwareMap.dcMotor.get("arm");

        // Servo
        crServo = hardwareMap.get(CRServo.class, "claw");

        // Reverse right side motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Arm setup
        big_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        big_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        big_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int armTargetPosition = 0;
        double maxArmSpeed = 0.5;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double max;

            // Mecanum drive input
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Drive
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftRear.setPower(leftBackPower);
            rightRear.setPower(rightBackPower);

            // Arm control with limits
            double armInput = gamepad1.right_trigger - gamepad1.left_trigger;
            int currentPos = big_arm.getCurrentPosition();

            if (Math.abs(armInput) > 0.05) {
                boolean movingUp = armInput > 0;
                boolean movingDown = armInput < 0;

                // Check if within limits
                if ((movingUp && currentPos < ARM_MAX_POSITION) ||
                        (movingDown && currentPos > ARM_MIN_POSITION)) {

                    big_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    big_arm.setPower(armInput * maxArmSpeed);
                    armTargetPosition = big_arm.getCurrentPosition();
                } else {
                    // At limit, hold position
                    big_arm.setTargetPosition(armTargetPosition);
                    big_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    big_arm.setPower(0.2);
                }
            } else {
                // Not moving input — hold current target
                armTargetPosition = Math.max(ARM_MIN_POSITION, Math.min(ARM_MAX_POSITION, armTargetPosition));
                big_arm.setTargetPosition(armTargetPosition);
                big_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                big_arm.setPower(0.2);
            }

            // CRServo control
            if (gamepad2.right_trigger > 0.1) {
                crServo.setPower(1.0);
            } else if (gamepad2.left_trigger > 0.1) {
                crServo.setPower(-1.0);
            } else {
                crServo.setPower(0.0);
            }

            // Telemetry
            telemetry.addData("Arm Pos", currentPos);
            telemetry.addData("Arm Target", armTargetPosition);
            telemetry.update();
        }
    }
}
