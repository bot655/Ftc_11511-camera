package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="MecanumTeleOp")
public class MecanumTeleOp extends LinearOpMode {
    private CRServo crServo;

    // Arm limits
    private final int ARM_MIN_POSITION = 0;
    private final int ARM_MAX_POSITION = 533; // 1440 counts/rev * 120 deg / 360 deg


    @Override
    public void runOpMode() throws InterruptedException {
        // Motors
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftRear = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightRear = hardwareMap.dcMotor.get("rightBack");
        DcMotor big_arm = hardwareMap.dcMotor.get("arm");
        DcMotor slide = hardwareMap.dcMotor.get("slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            // Arm control - direct power from gamepad2 left stick
            double armPower = -gamepad2.left_stick_y;
            big_arm.setPower(armPower);

            // CRServo control (updated to use gamepad2 bumpers)
            if (gamepad2.right_bumper) {
                crServo.setPower(1.0);
            } else if (gamepad2.left_bumper) {
                crServo.setPower(-1.0);
            } else {
                crServo.setPower(0.0);
            }

            // Slide control
            if (gamepad1.dpad_up) {
                slide.setPower(1.0);
            } else if (gamepad1.dpad_down) {
                slide.setPower(-1.0);
            } else {
                slide.setPower(0.0);
            }

            // Telemetry
            telemetry.addData("Arm Pos", big_arm.getCurrentPosition());
            telemetry.addData("LF Power", leftFrontPower);
            telemetry.update();
        }
    }
}
