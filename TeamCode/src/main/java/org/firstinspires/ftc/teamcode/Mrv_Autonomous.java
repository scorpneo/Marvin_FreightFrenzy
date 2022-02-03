package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;
import java.util.concurrent.TimeUnit;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

/*
 * This is an example of a more complex path to really test the tuning.
 */


@Config
@Autonomous(group = "Autonomous")
public class Mrv_Autonomous extends LinearOpMode {

    enum MrvAllianceField {
        RED,
        BLUE
    }

    Mrv_Robot marvyn = new Mrv_Robot();

    private static final String[] TFOD_MODEL_LABELS =
            {
                    "Aztechs_TSE"
            };

    private static FtcDashboard mrvDashboard;
    private static VuforiaLocalizer mrvVuforia;
    private static TFObjectDetector mrvTfod;
    private static Telemetry mrvDashboardTelemetry;
    private static TelemetryPacket mrvDashboardTelemetryPacket = new TelemetryPacket();
    private static int iTeleCt = 1;

    // Tensorflow camera settings
    private static int TFodResolution = 320;
    private static double TFodZoomFactor = 1;

    // VUFORIA Key
    public static final String VUFORIA_LICENSE_KEY = "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_AztechsTSE1.tflite";

    // Field Dimensions
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;
    private OpenGLMatrix mrvLastLocation = new OpenGLMatrix();

    public TrajectorySequence mrvRed1;
    public TrajectorySequence mrvRed2;
    public TrajectorySequence mrvBlue1;
    public TrajectorySequence mrvBlue2;
    public TrajectorySequence mrvBluePickupTraj;
    public TrajectorySequence mrvBlueDropoffTraj;

    // Warehouse drop off level
    private static int mrvWarehouseLevel;


    // TODO: Trajectory positions: Fine tune and move to Mrv_Robot
    Pose2d blue_1_pose_estimate = new Pose2d(-33.75, 62.625, Math.toRadians(-90));
    Pose2d red_1_pose_estimate = new Pose2d(-37, -62.625, Math.toRadians(90));

    Pose2d blue_2_pose_estimate = new Pose2d(13, 62.625, Math.toRadians(-90));
    Pose2d red_2_pose_estimate = new Pose2d(9.75, -62.625, Math.toRadians(90));

    Pose2d blue_1_shipping_hub_pos = new Pose2d(-21.5, 45.75, Math.toRadians(-70));
    Pose2d red1_shipping_hub_pos = new Pose2d(-21.5, -45.75, Math.toRadians(70));

    Pose2d blue_2_shipping_hub_pos = new Pose2d(-3, 46.5, Math.toRadians(-110));
    Pose2d red2_shipping_hub_pos = new Pose2d(-1.625, -45.75, Math.toRadians(110));

    Pose2d blue_duck_wheel_pos = new Pose2d(-59.75, 55, Math.toRadians(0));
    Pose2d red_duck_wheel_pos = new Pose2d(-59.75, -55, Math.toRadians(0));

    Pose2d blue_warehouse_enter_pos = new Pose2d(11, 64.75, Math.toRadians(0));
    Pose2d red_warehouse_enter_pos = new Pose2d(11, -64.75, Math.toRadians(0));

    Pose2d blue_warehouse_pos = new Pose2d(42.75, 64.75, Math.toRadians(0));
    Pose2d red_warehouse_pos = new Pose2d(36.75, -64.75, Math.toRadians(0));

    Pose2d blue_storage_pos = new Pose2d(-59.75, 35.5, Math.toRadians(0));
    Pose2d red_storage_pos = new Pose2d(-59.75, -35.5, Math.toRadians(0));

    Pose2d blue_park_pos = new Pose2d(40, 36, Math.toRadians(45));
    Pose2d red_park_pos = new Pose2d(64, -36, Math.toRadians(-90));


    // Dashboard config variables
    // TODO: Wrist Positions: Fine tune and move to Mrv_Robot
    public static double Wrist_Chute_Pos = 0.85;
    public static double Wrist_Pickup_Pos = 0.3;
    public static double Wrist_Dropoff_Pos = 0.4;

    // Speed control variables
    public static double slower_speed = 25;
    public static double slower_accel = 25;

    // Drop of Warehouse
    // TODO: DaWinchi positions: Fine tune and move to Mrv_Robot
    public static int overrideWarehouseDropoffLevel = 0;

    public static double DaWinchi_Level0_Dropoff_pos = 0;
    public static double DaWinchi_Level1_Dropoff_pos = 0.22;
    public static double DaWinchi_Level2_Dropoff_Pos = 0.62;
    public static int Dawinchi_dropoff_ticks = 0;
    public static double DaWinchi_pickup_pos = 0.285;
    public static double DaWinchi_Power = 0.5;
    public static double DaWinch_Neutral = 0;
    public static int mrvDawinchiCalculatedDropOffPos = 0;
    public static int mrvLinacCalcualtedDropOffPos = 0;

    // TODO: LinAc Positions: Fine tune and move to Mrv_Robot
    public static double Linac_Dropoff_Revs = 1.0;
    public static double Linac_Pickup_Revs = 1.0;
    public static double Linac_Neutral = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        marvyn.init(hardwareMap);

        iTeleCt = 1;

        telemetry.clearAll();
        telemetry.update();
        telemetry.setAutoClear(false);

        // init Dashboard
        mrvDashboard = FtcDashboard.getInstance();
        mrvDashboardTelemetryPacket = new TelemetryPacket();

        telemetry.addLine(String.format("%d. Marvin Initialized!", iTeleCt++));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Marvin Initialized!", iTeleCt++));

        double volts = getBatteryVoltage();
        telemetry.addLine(String.format("%d. Battery voltage: %.1f volts", iTeleCt++, volts));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Battery voltage: %.1f volts", iTeleCt++, volts));

        // init VUFORIA
        VuforiaLocalizer.Parameters vuParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuParams.cameraName = marvyn.eyeOfSauron;
        vuParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuParams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuParams.useExtendedTracking = false;
        mrvVuforia = ClassFactory.getInstance().createVuforia(vuParams);

        // init Tensorflow
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.4f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = TFodResolution;
        mrvTfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, mrvVuforia);
        mrvTfod.loadModelFromAsset(TFOD_MODEL_ASSET, TFOD_MODEL_LABELS);

        telemetry.addLine(String.format("%d. Tensorflow assets loaded", iTeleCt++));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Tensorflow assets loaded", iTeleCt++));

        // Startup camera
        mrvDashboard.startCameraStream(mrvTfod, 0); // start streaming camera
        telemetry.addLine(String.format("%d. TFod Camera Stream started", iTeleCt++));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. TFod Camera Stream started", iTeleCt++));

        if (mrvTfod != null) {
            mrvTfod.activate();
            mrvTfod.setZoom(TFodZoomFactor, 16.0 / 9.0);
        }
        sleep(1000);
        telemetry.addLine(String.format("%d. Tfod activated! Ready to Start!", iTeleCt++));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Tfod activated! Ready to Start!", iTeleCt++));

        telemetry.update();
        mrvDashboard.sendTelemetryPacket(mrvDashboardTelemetryPacket);

        waitForStart();
        mrvDashboardTelemetryPacket.clearLines();

        marvyn.Wristy.setPosition(Wrist_Dropoff_Pos);
        mrvWarehouseLevel = MrvGetWarehouseLevel(MrvAllianceField.BLUE);

        telemetry.addLine(String.format("%d. Detected Warehouse Level: %d", iTeleCt++, mrvWarehouseLevel));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Detected Warehouse Level: %d", iTeleCt++, mrvWarehouseLevel));
        telemetry.update();

        initMotorsAndServos();
        // Extend Linac if not extended
        int iLinacDropoffPos = (int) (Linac_Dropoff_Revs * marvyn.Linac_Ticks_Per_Rev);
        double dLinacPower = 1;
        marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, dLinacPower);
        while(opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.LIN_AC) < iLinacDropoffPos ) {
            idle();
        }
        marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 0);

        AssumeDropoffPositions(2);
//        sleep(1000);
//        AssumePickupPositions();
//        sleep(1000);
//        AssumeDropoffPositions(2);
//        sleep(1000);
//        AssumePickupPositions();
//        sleep(1000);
//        AssumeDropoffPositions(2);
//        sleep(1000);
//        AssumePickupPositions();
//        sleep(1000);
//        AssumeDropoffPositions(2);
//        sleep(1000);
//        AssumePickupPositions();
//        sleep(1000);

        //buildTrajectories();
        buildFreightDropoffTrajectories();

        Pose2d startPose = blue_2_pose_estimate;
        marvyn.mecanumDrive.setPoseEstimate(startPose);

//        telemetry.addLine(String.format("%d. Initial Pose Estimate: (x: %.3f, y: %.3f, Heading: %.3f)", iTeleCt++, startPose.getX(), startPose.getY(), startPose.getHeading()));
//        mrvDashboardTelemetryPacket.addLine(String.format("%d. Initial Pose Estimate: (x: %.3f, y: %.3f, Heading: %.3f)", iTeleCt++, startPose.getX(), startPose.getY(), startPose.getHeading()));

        ElapsedTime trajectoryTimer = new ElapsedTime(MILLISECONDS);
        trajectoryTimer.reset();
        trajectoryTimer.startTime();
        marvyn.mecanumDrive.followTrajectorySequence(mrvBlue2);

//        telemetry.addLine(String.format("%d. Total time to complete Trajectory Sequence: %.3f ", iTeleCt++, trajectoryTimer.time(TimeUnit.SECONDS)));
//        mrvDashboardTelemetryPacket.addLine(String.format("%d. Total time to complete Trajectory Sequence: %.3f ", iTeleCt++, trajectoryTimer.time(TimeUnit.SECONDS)));

        telemetry.update();
        mrvDashboard.sendTelemetryPacket(mrvDashboardTelemetryPacket);
    }

    int MrvGetWarehouseLevel(MrvAllianceField field) {
        int PyraPos = 0;
        if (mrvTfod != null && opModeIsActive()) {
            boolean bDuckFound = false;
            float left = 0.0f;
            float top = 0.0f;
            float bottom = 0.0f;
            float right = 0.0f;
            float mdpt = 0.0f;
            List<Recognition> updatedRecognition = mrvTfod.getUpdatedRecognitions();
            if (updatedRecognition != null) {
                telemetry.addLine(String.format("%d. # Objects detected: %d", iTeleCt++, updatedRecognition.size()));
                mrvDashboardTelemetryPacket.addLine(String.format("%d. # Objects detected: %d", iTeleCt++, updatedRecognition.size()));
                int i = 0;
                for (Recognition recognition : updatedRecognition) {
                    left = recognition.getLeft();
                    right = recognition.getRight();
                    top = recognition.getTop();
                    bottom = recognition.getBottom();
                    mdpt = (left + right) / 2;

                    //if (recognition.getLabel()== "Duck") {
                    if (recognition.getLabel() == "Aztechs_TSE" && recognition.getConfidence() > 0.8) {
                        if (marvyn.FirstPosMin <= mdpt && mdpt <= marvyn.FirstPosMax) {
                            PyraPos = 1;
                        } else if (marvyn.SecPosMin <= mdpt && mdpt <= marvyn.SecPosMax) {
                            PyraPos = 2;

                        }

                        bDuckFound = true;
                        telemetry.addLine(String.format("%d. Object (%d:) ", iTeleCt++, i) + recognition.getLabel());
                        telemetry.addLine(String.format("%d. Confidence: %.3f", iTeleCt++, recognition.getConfidence()));
                        telemetry.addLine(String.format("%d. BBox (%d), (%.03f, %.03f)  (%.03f, %.03f)", iTeleCt++, i, left, top, right, bottom));
                        telemetry.addLine(String.format("%d. BBox midpt (%d),  (%.03f)", iTeleCt++, i, mdpt));
                        telemetry.addLine(String.format("%d. Image Size (%d), Width: %d; Height: %d ", iTeleCt++, i, recognition.getImageWidth(), recognition.getImageHeight()));
                        telemetry.update();

                        mrvDashboardTelemetryPacket.addLine(String.format("%d. Object (%d:) ", iTeleCt++, i) + recognition.getLabel());
                        mrvDashboardTelemetryPacket.addLine(String.format("%d. Confidence: %.3f", iTeleCt++, recognition.getConfidence()));
                        mrvDashboardTelemetryPacket.addLine(String.format("%d. BBox (%d), (%.03f, %.03f)  (%.03f, %.03f)", iTeleCt++, i, left, top, right, bottom));
                        mrvDashboardTelemetryPacket.addLine(String.format("%d. BBox midpt (%d),  (%.03f)", iTeleCt++, i, mdpt));
                        mrvDashboardTelemetryPacket.addLine(String.format("%d. Image Size (%d), Width: %d; Height: %d ", iTeleCt++, i, recognition.getImageWidth(), recognition.getImageHeight()));
                        i++;
                        break;
                    }
                }
            } else {
                telemetry.addLine(String.format("%d. Objects Detected: None!", iTeleCt++));
                mrvDashboardTelemetryPacket.addLine(String.format("%d. Objects Detected: None!", iTeleCt++));
                telemetry.update();
                PyraPos = 0;
            }
        }
        telemetry.addLine(String.format("%d. PyraPos: %d", iTeleCt++, PyraPos));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. PyraPos: %d", iTeleCt++, PyraPos));

        telemetry.update();

        return PyraPos;
    }

    void PreLoadFreightDropoff(int level) {
        int iLinacDropoffPosition = (int) (Linac_Dropoff_Revs * marvyn.Linac_Ticks_Per_Rev) * -1;
        int iDawinchiDropoffPosition = 0;
        switch (level) {
            case 0: // BOTTOM
                iDawinchiDropoffPosition = (int) (marvyn.Dawinchi_Ticks_Per_Rev * DaWinchi_Level0_Dropoff_pos);

                // Set winch to 0 level
                mrvDashboardTelemetryPacket.addLine(String.format("%d. Setting Dawinchi to Level: %d", iTeleCt++, level));
                mrvDashboardTelemetryPacket.addLine(String.format("%d. Calculated Dawinchi dropoff pos in ticks = %d", iTeleCt++, iDawinchiDropoffPosition));
                marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, STOP_AND_RESET_ENCODER);
                marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, RUN_WITHOUT_ENCODER);
                marvyn.setTargetPosition(Mrv_Robot.MrvMotors.DA_WINCHI, iDawinchiDropoffPosition);
                marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 1);
                while (opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.DA_WINCHI) < iDawinchiDropoffPosition) {
                    idle();
                }
                marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);


                // set Linac to drop off position
                int iPos = -1 * marvyn.Linac_Ticks_Per_Rev;
                marvyn.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, STOP_AND_RESET_ENCODER);
                marvyn.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, RUN_WITHOUT_ENCODER);
                marvyn.setTargetPosition(Mrv_Robot.MrvMotors.LIN_AC, iPos);
                marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, -1);
                while (opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.LIN_AC) > iLinacDropoffPosition) {
                    idle();
                }
                marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 0);


                break;
            case 1: // MIDDLE
                iDawinchiDropoffPosition = (int) (marvyn.Dawinchi_Ticks_Per_Rev * DaWinchi_Level1_Dropoff_pos);

                mrvDashboardTelemetryPacket.addLine(String.format("%d. Setting Dawinchi to Level: %d", iTeleCt++, level));
                mrvDashboardTelemetryPacket.addLine(String.format("%d. Calculated Dawinchi dropoff pos in ticks = %d", iTeleCt++, iDawinchiDropoffPosition));
                // Set winch to 2 level
                marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, STOP_AND_RESET_ENCODER);
                marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, RUN_WITHOUT_ENCODER);
                marvyn.setTargetPosition(Mrv_Robot.MrvMotors.DA_WINCHI, iDawinchiDropoffPosition);
                marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 1);
                while (opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.DA_WINCHI) < iDawinchiDropoffPosition) {
                    idle();
                }
                marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);

                // set Linac to drop off position
                marvyn.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, STOP_AND_RESET_ENCODER);
                marvyn.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, RUN_WITHOUT_ENCODER);
                marvyn.setTargetPosition(Mrv_Robot.MrvMotors.LIN_AC, iLinacDropoffPosition);
                marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, -1);
                while (opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.LIN_AC) > iLinacDropoffPosition) {
                    idle();
                }
                marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 0);
                break;
            case 2: // TOP
                iDawinchiDropoffPosition = (int) (marvyn.Dawinchi_Ticks_Per_Rev * DaWinchi_Level2_Dropoff_Pos);
                iDawinchiDropoffPosition = -1 * iDawinchiDropoffPosition;

                mrvDashboardTelemetryPacket.addLine(String.format("%d. Setting Dawinchi to Level: %d", iTeleCt++, level));
                mrvDashboardTelemetryPacket.addLine(String.format("%d. Calculated Dawinchi dropoff pos in ticks = %d", iTeleCt++, iDawinchiDropoffPosition));

                int iPos2 = (int) (-1.3 * marvyn.Linac_Ticks_Per_Rev);
                // Set winch to 2 level
                marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, STOP_AND_RESET_ENCODER);
                marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, RUN_WITHOUT_ENCODER);
                marvyn.setTargetPosition(Mrv_Robot.MrvMotors.DA_WINCHI, iDawinchiDropoffPosition);
                marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, -1);
                while (opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.DA_WINCHI) > iDawinchiDropoffPosition) {
                    idle();
                }
                marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);

                // set Linac to drop off position
                marvyn.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, STOP_AND_RESET_ENCODER);
                marvyn.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, RUN_WITHOUT_ENCODER);
                marvyn.setTargetPosition(Mrv_Robot.MrvMotors.LIN_AC, iLinacDropoffPosition);
                marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, -1);
                while (opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.LIN_AC) > iLinacDropoffPosition) {
                    idle();
                }
                marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 0);
                break;
        }
    }

    void CalculateWarehouseDropOffPos(int level) {
        switch (level) {
            case 0: // BOTTOM
                mrvDawinchiCalculatedDropOffPos = -1 * (int) (marvyn.Dawinchi_Ticks_Per_Rev * DaWinchi_Level0_Dropoff_pos);
                break;
            case 1: // MIDDLE
                mrvDawinchiCalculatedDropOffPos = -1 * (int) (marvyn.Dawinchi_Ticks_Per_Rev * DaWinchi_Level1_Dropoff_pos);
                break;
            case 2: // TOP
                mrvDawinchiCalculatedDropOffPos = -1 * (int) (marvyn.Dawinchi_Ticks_Per_Rev * DaWinchi_Level2_Dropoff_Pos * -1);
                break;
        }
    }


    void buildTrajectories() {
        telemetry.addLine("Building Trajectories");
        mrvDashboardTelemetryPacket.addLine("Building Trajectories");

        mrvBlue1 = marvyn.mecanumDrive.trajectorySequenceBuilder(marvyn.blue_1_pose_estimate)
                //drive to hub
                .lineToLinearHeading(marvyn.blue_1_shipping_hub_pos)
                .addTemporalMarker(() -> {
                    PreLoadFreightDropoff(mrvWarehouseLevel);
                })
                //Release the Claw
                .addTemporalMarker(() -> {
                    //marvyn.The_Claw.setPosition(Claw_Open_Pos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    // retract Linac
                    marvyn.setTargetPosition(Mrv_Robot.MrvMotors.LIN_AC, 0);
                    marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 1);
                    while (opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.LIN_AC) < 0) {
                        idle();
                    }
                    marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 0);
                })
                .lineToLinearHeading(marvyn.blue_duck_wheel_pos)
                //do duck
                .addTemporalMarker(() -> {
                    marvyn.setPower(Mrv_Robot.MrvMotors.DUCK_WHEELS, 0.25 * -1);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    marvyn.setPower(Mrv_Robot.MrvMotors.DUCK_WHEELS, 0);
                })
                //enter warehouse
                .lineToLinearHeading(marvyn.blue_storage_pos)

                .build()
        ;
        return;
    }

    void SpinDuckWheel() {
        sleep(1000);
        marvyn.setPower(Mrv_Robot.MrvMotors.DUCK_WHEELS, 0.25 * -1);
        sleep(2500);
        marvyn.setPower(Mrv_Robot.MrvMotors.DUCK_WHEELS, 0);
        return;
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    void buildFreightDropoffTrajectories() {

        telemetry.addLine(String.format("%d. Figure Skating", iTeleCt++));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Figure Skating", iTeleCt++));
        mrvBlue2 = marvyn.mecanumDrive.trajectorySequenceBuilder(blue_2_pose_estimate)
                //drive to hub
                .lineToLinearHeading(blue_2_shipping_hub_pos)
                .addTemporalMarker(() -> {
                    marvyn.Claw_Left.setPower(-0.5);
                    marvyn.Claw_Right.setPower(-0.5);
                })
                .waitSeconds(0.3)
                .addTemporalMarker( ()-> {
                    marvyn.Claw_Left.setPower(-0.5);
                    marvyn.Claw_Right.setPower(-0.5);
                })
                .lineToLinearHeading(blue_warehouse_enter_pos)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    marvyn.Wristy.setPosition(Wrist_Pickup_Pos);
                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0.5);
                })
                .waitSeconds(0.8)
                .addTemporalMarker(() -> {
                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);
                })
                //go to warehouse entrance
                .lineToLinearHeading(blue_warehouse_pos, marvyn.mecanumDrive.getVelocityConstraint(slower_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), marvyn.mecanumDrive.getAccelerationConstraint(slower_accel))
//
//                //.waitSeconds(0.5)
////                .addTemporalMarker(() -> {
////                    Freight_Neutral();
////                })
////                .waitSeconds(0.5)
//
//                .lineToLinearHeading(blue_warehouse_enter_pos, marvyn.mecanumDrive.getVelocityConstraint(slower_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), marvyn.mecanumDrive.getAccelerationConstraint(slower_accel))
//                .lineToLinearHeading(blue_2_shipping_hub_pos)
//
//                .addTemporalMarker(() -> {
//                    //freight pickup
//                    //move winch and linac so wheels are parallel to ground
//                    Freight_Dropoff(2);
//                })
//                .waitSeconds(0.5)
////                .addTemporalMarker(() -> {
////                    Freight_Neutral();
////                })
////                .waitSeconds(0.5)
////
////
//                .lineToLinearHeading(blue_warehouse_enter_pos)
//                .lineToLinearHeading(blue_warehouse_pos, marvyn.mecanumDrive.getVelocityConstraint(slower_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), marvyn.mecanumDrive.getAccelerationConstraint(slower_accel))
//
//
//                .addTemporalMarker(() -> {
//                    //freight pickup
//                    //move winch and linac so wheels are parallel to ground
//                    Freight_Pickup();
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    Freight_Neutral();
//                })
//                .waitSeconds(0.5)
//
//
//                .lineToLinearHeading(blue_warehouse_enter_pos, marvyn.mecanumDrive.getVelocityConstraint(slower_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), marvyn.mecanumDrive.getAccelerationConstraint(slower_accel))
//                .lineToLinearHeading(blue_2_shipping_hub_pos)
//
//
//                .addTemporalMarker(() -> {
//                    //freight pickup
//                    //move winch and linac so wheels are parallel to ground
//                    Freight_Dropoff(2);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    Freight_Neutral();
//                })
//                .waitSeconds(0.5)
//
//
//                .lineToLinearHeading(blue_warehouse_enter_pos)
//                .lineToLinearHeading(blue_warehouse_pos, marvyn.mecanumDrive.getVelocityConstraint(slower_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), marvyn.mecanumDrive.getAccelerationConstraint(slower_accel))
//                .lineToSplineHeading(blue_park_pos)

                .build()
        ;
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Trjectory Duration after build: %.3f", iTeleCt++, mrvBlue2.duration()));
        return;

    }


    void initMotorsAndServos()
    {
        // Rest Dawinchi - current position becomes 0
        marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, STOP_AND_RESET_ENCODER);
        marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, RUN_WITHOUT_ENCODER);

        // Reset Linac - current position becomes 0
        marvyn.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, STOP_AND_RESET_ENCODER);
        marvyn.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, RUN_WITHOUT_ENCODER);

        marvyn.Wristy.setPosition(Wrist_Pickup_Pos);
    }

    void AssumePickupPositions() {
        //
        marvyn.Wristy.setPosition(Wrist_Pickup_Pos);

        // Lower Dawinchi
        int iDawinchiPickupPos = (int) (DaWinchi_pickup_pos*marvyn.Dawinchi_Ticks_Per_Rev);
        double dDawinchiPower = 1*DaWinchi_Power;

        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, dDawinchiPower);
        while(opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.DA_WINCHI) < iDawinchiPickupPos) {
            idle();
        }
        double timeTaken = timer.seconds();
        int iCurrentPos = marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.DA_WINCHI);
        marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Lower Winch to Pickup position: %d, Current Position: %d Time taken to  %.3f s", iTeleCt++, iDawinchiPickupPos, iCurrentPos, timeTaken));

//        // Extend Linac if not extended
//        int iLinacPickupOos = (int) (Linac_Pickup_Revs * marvyn.Linac_Ticks_Per_Rev);
//        double dLinacPower = 1;
//        marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, dLinacPower);
//        while(opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.LIN_AC) < iLinacPickupOos ) {
//            idle();
//        }
//        marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 0);

    }

    void AssumeDropoffPositions(int iLevel )
    {
        if(iLevel == 0)
        {
            Dawinchi_dropoff_ticks = (int) (DaWinchi_Level0_Dropoff_pos*marvyn.Dawinchi_Ticks_Per_Rev);
        }
        else if(iLevel == 1)
        {
            Dawinchi_dropoff_ticks = (int) (DaWinchi_Level1_Dropoff_pos*marvyn.Dawinchi_Ticks_Per_Rev);
        }
        else if(iLevel == 2)
        {
            Dawinchi_dropoff_ticks = (int) (DaWinchi_Level2_Dropoff_Pos*marvyn.Dawinchi_Ticks_Per_Rev);
        }

        marvyn.Wristy.setPosition(Wrist_Dropoff_Pos);

        // Lower Dawinchi
        int iDawinchiDropoffPos = -1*Dawinchi_dropoff_ticks;
        double dDawinchiPower = -1*DaWinchi_Power;
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, dDawinchiPower);
        while(opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.DA_WINCHI) > iDawinchiDropoffPos) {
            idle();
        }
        double timeTaken = timer.seconds();
        int iCurrentPos = marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.DA_WINCHI);
        marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);
        marvyn.duck_wheel.setPower(0);
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Raise Winch to level: %d, position %d: iCurrentPos %d, Time taken: %.3f s", iTeleCt++, iLevel, iDawinchiDropoffPos, iCurrentPos, timeTaken));
    }


    void Freight_Dropoff(int iLevel)
    {


        if( iLevel == 2)
        {
            marvyn.Wristy.setPosition(Wrist_Dropoff_Pos);
            // Lower Dawinchi
            int iDawinchiDropoffPos = (int) (DaWinchi_Level2_Dropoff_Pos*marvyn.Dawinchi_Ticks_Per_Rev);
            iDawinchiDropoffPos = -1*iDawinchiDropoffPos;
            double dDawinchiPower = -1*DaWinchi_Power;
            marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, dDawinchiPower);
            while(opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.DA_WINCHI) > iDawinchiDropoffPos) {
                idle();
            }
            marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);

            // Extend Linac if not extended
            int iLinacDropoffPos = (int) (Linac_Dropoff_Revs * marvyn.Linac_Ticks_Per_Rev);
            double dLinacPower = 1;
            marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, dLinacPower);
            while(opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.LIN_AC) < iLinacDropoffPos ) {
                idle();
            }
            marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 0);

            //spin intake
            marvyn.Claw_Left.setPower(-0.5);
            marvyn.Claw_Right.setPower(-0.5);
        }
    }

    void Freight_Pickup() {
        marvyn.Wristy.setPosition(Wrist_Pickup_Pos);

        // Lower Dawinchi
        int iDawinchiPickupPos = (int) (DaWinchi_pickup_pos*marvyn.Dawinchi_Ticks_Per_Rev);
        double dDawinchiPower = 1*DaWinchi_Power;
        marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, dDawinchiPower);
        while(opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.DA_WINCHI) < iDawinchiPickupPos) {
            idle();
        }
        marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);

        // Extend Linac if not extended
        int iLinacPickupOos = (int) (Linac_Pickup_Revs * marvyn.Linac_Ticks_Per_Rev);
        double dLinacPower = 1;
        marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, dLinacPower);
        while(opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.LIN_AC) < iLinacPickupOos ) {
            idle();
        }
        marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 0);

        marvyn.Claw_Left.setPower(0.5);
        marvyn.Claw_Right.setPower(0.5);
    }

    void Freight_Neutral() {
        marvyn.Claw_Left.setPower(0);
        marvyn.Claw_Right.setPower(0);
//        marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 1);
//        while (opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.LIN_AC) < Linac_Neutral) {
//            idle();
//        }
//        marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 0);
//
//        marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 1);
//        while (opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.DA_WINCHI) < DaWinch_Neutral) {
//            idle();
//        }
//        marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);
//        marvyn.Wristy.setPosition(Wrist_Chute_Pos);
//    }
    }


}