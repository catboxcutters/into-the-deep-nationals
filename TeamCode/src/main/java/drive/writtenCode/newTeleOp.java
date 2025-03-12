package drive.writtenCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import drive.writtenCode.controllers.*;

@TeleOp(name = "TeleOpCodeNEW", group = "Linear OpMode")
public class newTeleOp extends LinearOpMode {

    // Gamepad variables
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // Timers
    private final ElapsedTime globalTimer = new ElapsedTime();
    private final ElapsedTime beamTimer = new ElapsedTime();
    private final ElapsedTime slidesTimer = new ElapsedTime();
    private final ElapsedTime releaseTimer = new ElapsedTime();

    // Constants and State Variables
    private static final double TIME_TO_SLIDES = 0.35;
    private static final double TIME_TO_START_BEAM = 0.05;

    private double clawRotateTarget = 0;
    private double driveRate = 1.0;
    private boolean isSlidesTransition = false;
    private boolean isRungTransition = false;

    private RobotMap robot;
    private CameraController cameraController;

    // Controllers
    private LinkageController linkageController;
    private SlidesController slidesController;
    private FourbarController fourbarController;
    private ClawController clawController;
    private ClawPositionController clawPositionController;
    private ClawRotateController clawRotateController;
    private ScoreSystemController scoreSystemController;
    private LinkageSlidesController linkageSlidesController;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        globalTimer.reset();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            // Update gamepad states
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Handle inputs
            handleGamepadInputs(currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2);

            // Update robot controllers
            updateControllers();

            // Update telemetry
            updateTelemetry();
        }


        cameraController.stopCamera();
    }

    private void initializeHardware() {
        // Initialize robot hardware and controllers
        robot = new RobotMap(hardwareMap);
        cameraController = new CameraController(hardwareMap, "Webcam 1");

        linkageController = new LinkageController(robot);
        slidesController = new SlidesController(robot);
        fourbarController = new FourbarController(robot);
        clawController = new ClawController(robot);
        clawPositionController = new ClawPositionController(robot);
        clawRotateController = new ClawRotateController(robot);
        scoreSystemController = new ScoreSystemController(clawController, clawRotateController, fourbarController, clawPositionController,linkageController);
        linkageSlidesController = new LinkageSlidesController(linkageController, slidesController);

        configureMotors();
        cameraController.startCamera();
    }

    private void configureMotors() {
        // Configure drive motors
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        MotorConfigurationType[] motorConfigs = {
                leftFront.getMotorType().clone(),
                leftBack.getMotorType().clone(),
                rightFront.getMotorType().clone(),
                rightBack.getMotorType().clone()
        };

        for (MotorConfigurationType config : motorConfigs) {
            config.setAchieveableMaxRPMFraction(1.0);
        }

        setMotorMode(leftFront, leftBack, rightFront, rightBack, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorZeroPowerBehavior(leftFront, leftBack, rightFront, rightBack, DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    private void setMotorMode(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    private void setMotorZeroPowerBehavior(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }
    private void setMotorPowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    private void handleDriving() {
        double y = -gamepad1.left_stick_y;
        double x = (-gamepad1.left_trigger + gamepad1.right_trigger);
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator * driveRate;
        double leftBackPower = (y - x + rx) / denominator * driveRate;
        double rightFrontPower = (y - x - rx) / denominator * driveRate;
        double rightBackPower = (y + x - rx) / denominator * driveRate;

        setMotorPowers(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    private void handleGamepadInputs(Gamepad currentGamepad1, Gamepad currentGamepad2, Gamepad previousGamepad1, Gamepad previousGamepad2) {
        // Camera Toggle
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            cameraController.toggleDetectionColor();
            if (cameraController.getCurrentDetectionColor().equals("Blue")) {
                gamepad2.rumbleBlips(1);
                gamepad1.rumbleBlips(1);
            } else {
                gamepad2.rumbleBlips(2);
                gamepad1.rumbleBlips(2);
            }
        }

        // LED Color Setting Based on Detection
        if (cameraController.getCurrentDetectionColor().equals("Yellow")) {
            gamepad1.setLedColor(255, 255, 0, 1000);
            gamepad2.setLedColor(255, 255, 0, 100);
        } else if (cameraController.getCurrentDetectionColor().equals("Blue")) {
            gamepad1.setLedColor(0, 0, 255, 1000);
            gamepad2.setLedColor(0, 0, 255, 100);
        }

        // Linkage and Slide Control: Right Bumper
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            switch (linkageSlidesController.currentStatus) {
                case INIT:
                    clawController.currentStatus = ClawController.ClawStatus.OPEN;
                    linkageSlidesController.currentStatus = LinkageSlidesController.LinkageSlidesStatus.LOWER_LINKAGE;
                    scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                    driveRate = 0.4;
                    break;
                case EXTEND_SLIDES:
                    scoreSystemController.timer.reset();
                    slidesTimer.reset();
                    isSlidesTransition = true;
                    clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.AUTO_ALIGN;
                    scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR_SUB_AUTO_ALIGN;
                    driveRate = 1.0;
                    break;
            }
        }

        // Linkage and Slide Control: Left Bumper
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            switch (linkageSlidesController.currentStatus) {
                case INIT:
                    clawController.currentStatus = ClawController.ClawStatus.OPEN;
                    linkageSlidesController.currentStatus = LinkageSlidesController.LinkageSlidesStatus.LOWER_LINKAGE;
                    scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                    driveRate = 0.4;
                    break;
                case EXTEND_SLIDES:
                    scoreSystemController.timer.reset();
                    slidesTimer.reset();
                    isRungTransition = true;
                    clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.AUTO_ALIGN;
                    scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR_SUB_AUTO_ALIGN_RUNG;
                    driveRate = 1.0;
                    break;
            }
        }

        // Score System Controls
        if (currentGamepad2.y && !previousGamepad2.y) {
            if (scoreSystemController.currentStatus == ScoreSystemController.ScoreSystemStatus.INIT) {
                scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
            }
        }

        if (currentGamepad2.a && !previousGamepad2.a) {
            switch (linkageSlidesController.currentStatus) {
                case INIT:
                    linkageSlidesController.currentStatus = LinkageSlidesController.LinkageSlidesStatus.BSK_HIGH;
                    scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.SCORE;
                    break;
                default:
                    linkageSlidesController.currentStatus = LinkageSlidesController.LinkageSlidesStatus.INIT;
                    scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.INIT;
                    clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.INIT;
                    break;
            }
        }

        if (currentGamepad2.b && !previousGamepad2.b) {
            switch (linkageSlidesController.currentStatus) {
                case INIT:
                    linkageSlidesController.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                    scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                    break;
                case HIGH_RUNG:
                    linkageSlidesController.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                    scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                    releaseTimer.reset();
                    break;
            }
        }

        // Claw Rotate Control with D-Pad Right
        if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
            switch (clawRotateController.currentStatus) {
                case INIT:
                    clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.MINUS;
                    break;
                case MINUS:
                    clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.MINUS2;
                    break;
                case PLUS:
                    clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.INIT;
                    break;
                case PLUS2:
                    clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.PLUS;
                    break;
            }
        }

// Claw Rotate Control with D-Pad Left
        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
            switch (clawRotateController.currentStatus) {
                case INIT:
                    clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.PLUS;
                    break;
                case PLUS:
                    clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.PLUS2;
                    break;
                case MINUS:
                    clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.INIT;
                    break;
                case MINUS2:
                    clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.MINUS;
                    break;
            }
        }

        // Claw Open/Close Toggle
        if (currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button) {
            switch (clawController.currentStatus) {
                case OPEN:
                    clawController.currentStatus = ClawController.ClawStatus.CLOSE;
                    break;
                case CLOSE:
                    clawController.currentStatus = ClawController.ClawStatus.OPEN;
                    break;
            }
        }

        // Fourbar Rung Control
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            switch (fourbarController.currentStatus) {
                case RUNG:
                    scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG_PARA;
                    break;
                case RUNG_PARA:
                    scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                    break;
            }
        }
    }


    private void updateControllers() {
        linkageController.update(robot.linkage.getCurrentPosition(), 0);
        slidesController.update(robot.linkage.getCurrentPosition(), 0);
        clawController.update();
        fourbarController.update();
        clawPositionController.update();
        clawRotateController.update(clawRotateTarget, cameraController.getOrientationAngle());
        scoreSystemController.update(robot.encoderClaw.getVoltage());
        linkageSlidesController.update();
    }

    private void updateTelemetry() {
        telemetry.addData("Claw Status", clawController.currentStatus);
        telemetry.addData("Fourbar Status", fourbarController.currentStatus);
        telemetry.addData("Score System Status", scoreSystemController.currentStatus);
        telemetry.addData("Linkage Slides Status", linkageSlidesController.currentStatus);
        telemetry.addData("Camera Detection Color", cameraController.getCurrentDetectionColor());
        telemetry.update();
    }
}
