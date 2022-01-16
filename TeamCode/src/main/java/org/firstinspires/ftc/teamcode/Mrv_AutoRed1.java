package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.jetbrains.annotations.NotNull;

import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

/*
 * This is an example of a more complex path to really test the tuning.
 */


@Config
@Autonomous(group = "drive")
public class Mrv_AutoRed1 extends LinearOpMode {

    enum MrvAllianceField
    {
        RED,
        BLUE
    }

    Mrv_Robot marvyn = new Mrv_Robot();

    private static final String[] TFOD_MODEL_LABELS =
            {
                    "Ball",
                    "Cube",
                    "Duck",
                    "Marker",
                    "Aztechs_TSE"
            };

//    private static final String[] TFOD_MODEL_LABELS =
//            {
//                    "Aztechs_TSE"
//            };

    private static FtcDashboard mrvDashboard;
    private static VuforiaLocalizer mrvVuforia;
    private static TFObjectDetector mrvTfod;
    private static MultipleTelemetry mrvTelemetry;
    private static TelemetryPacket mrvDashboardTelemetryPacket = new TelemetryPacket();
    private static int iTeleCt = 1;

    // VUFORIA Key
    public static final String VUFORIA_LICENSE_KEY = "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    //    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_Aztechs.tflite";
    // Field Dimensions
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;
    private OpenGLMatrix mrvLastLocation   = new OpenGLMatrix();

    public TrajectorySequence mrvRed1;
    public TrajectorySequence mrvRed2;
    public TrajectorySequence mrvBlue1;
    public TrajectorySequence mrvBlue2;

//    public static double starting_pos_x = 12;

    // Dashboard config variables

    // Claw position
    public static double Claw_Open_Pos = 0.4;
    public static double Claw_Close_Pos = 0.0;

    // Wrist position
    public static double Wrist_Parallel_to_Linac = 0.425; // Parallel to arm
    public static double Wrist_chute_dropoff = 0.85; // Perpendicular to Arm at top
    public static double Wrist_Start_Pos = 0.0; // Perpendicular to Arm at bottom


    // Tensor flow - Warehouse level detection
    public static int TFodResolution = 320;
    public static double TFodZoomFactor = 1;
    public static int sleepyTime = 5000;
    public static double FirstPosLeftMin = -5;
    public static double FirstPosLeftMax = 115;
    public static double FirstPosRightMin = 60;
    public static double FirstPosRightMax = 230;
    public static double SecPosLeftMin = 315;
    public static double SecPosLeftMax = 530;
    public static double SecPosRightMin = 420;
    public static double SecPosRightMax = 640;

    // Trajectory sequencing
    public static Pose2d blue_1_pose_estimate      = new Pose2d(-33.75, 62.625, Math.toRadians(-90));
    public static Pose2d red_1_pose_estimate       = new Pose2d(-37, -62.625, Math.toRadians(90));

    public static Pose2d blue_2_pose_estimate      = new Pose2d( 13, 62.625, Math.toRadians(-90));
    public static Pose2d red_2_pose_estimate       = new Pose2d( 9.75, -62.625, Math.toRadians(90));

    public static Pose2d blue_1_shipping_hub_pos = new Pose2d(-21.5,  45.75, Math.toRadians(-70));
    public static Pose2d red1_shipping_hub_pos      = new Pose2d(-21.5, -45.75, Math.toRadians(70));

    public static Pose2d blue_2_shipping_hub_pos     = new Pose2d(-1.625,  45.75, Math.toRadians(-110));
    public static Pose2d red2_shipping_hub_pos      = new Pose2d(-1.625, -45.75, Math.toRadians(110));

    public static Pose2d blue_duck_wheel_pos       = new Pose2d(-59.75, 55, Math.toRadians(0));
    public static Pose2d red_duck_wheel_pos        = new Pose2d(-59.75, -55, Math.toRadians(0));

    public static Pose2d blue_warehouse_enter_pos  = new Pose2d( 11, 64.75, Math.toRadians(0));
    public static Pose2d red_warehouse_enter_pos   = new Pose2d( 11, -64.75, Math.toRadians(0));

    public static Pose2d blue_warehouse_pos        = new Pose2d( 36.75, 64.75, Math.toRadians(0 ));
    public static Pose2d red_warehouse_pos         = new Pose2d( 36.75, -64.75, Math.toRadians(0));

    public static Pose2d blue_storage_pos          = new Pose2d( -59.75, 35.5, Math.toRadians(0));
    public static Pose2d red_storage_pos          = new Pose2d( -59.75, -35.5, Math.toRadians(0));

    public static Pose2d blue_park_pos             = new Pose2d( 64, 36, Math.toRadians(90));
    public static Pose2d red_park_pos              = new Pose2d( 64, -36, Math.toRadians(-90));

    public static double slower_speed = 25;
    public static double slower_accel = 25;

    // Drop of Warehouse
    public static int mrvWarehouseLevel;
    public static double DaWinchi_Level0_Dropoff = 0.7;
    public static double DaWinchi_Level1_Dropoff = 0.15;
    public static double DaWinchi_Level2_Dropoff = 0.55;
    public static int Dawinchi_Ticks_Per_Rev = 1120; // From REV Robotics HD HEX 40:1
    public static int dropOffLevel = 0;

    public static double Linac_Dropoff_Revs = 1.2;
    public static int Linac_Ticks_Per_Rev = 288; // From REV Robotics Core HEX

    // Goal 1: Drop off freight on all 3 levels
    // Goal 2: Duck wheel spin
    // Goal 3: Recognize TSE for correct level
    // Goal 4: Trajectory for Blue 1
    // Goal 5: Individual programs for each of the other positions

    @Override
    public void runOpMode() throws InterruptedException {
        marvyn.init(hardwareMap);

        // init Dashboard
        mrvDashboard = FtcDashboard.getInstance();
        mrvTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry() );
        mrvTelemetry.clearAll();
        mrvTelemetry.update();
        mrvTelemetry.setAutoClear(false);
        FtcDashboard.getInstance().getTelemetry().setAutoClear(false);

        mrvTelemetry.addLine(String.format("%d. Marvin Initialized!", iTeleCt++));
        double volts = getBatteryVoltage();
        mrvTelemetry.addData(String.format("%d. Battery voltage", iTeleCt++),  String.format("%.1f volts",volts) );

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

        mrvTelemetry.addLine(String.format("%d. Tensorflow assets loaded", iTeleCt++));
        mrvTelemetry.update();
        FtcDashboard.getInstance().sendTelemetryPacket(mrvDashboardTelemetryPacket);

        // Set the claw to hold the preloaded element
        marvyn.The_Claw.setPosition(Claw_Close_Pos);
        sleep(2000);


        // Startup camera
        mrvDashboard.startCameraStream(mrvTfod, 0); // start streaming camera
        mrvTelemetry.addLine(String.format("%d. TFod Camera Stream started", iTeleCt++));
        mrvTelemetry.update();

        if (mrvTfod != null) {
            mrvTfod.activate();
            mrvTfod.setZoom(TFodZoomFactor, 16.0 / 9.0);
        }
        sleep(2000);
        mrvTelemetry.addLine("Tfod activated! Ready to Start!");
        mrvTelemetry.update();

        waitForStart();
        marvyn.Wristy.setPosition(Wrist_Parallel_to_Linac);
        mrvWarehouseLevel = MrvGetWarehouseLevel(MrvAllianceField.BLUE);
        mrvTelemetry.addData("Warehouse Level", mrvWarehouseLevel);
        mrvTelemetry.update();

        buildTrajectories();

        marvyn.mecanumDrive.setPoseEstimate(red_1_pose_estimate);

        ElapsedTime time = new ElapsedTime(MILLISECONDS);
        time.reset();
        time.startTime();
        marvyn.mecanumDrive.followTrajectorySequence(mrvRed1);
        mrvTelemetry.addData("time", time);
        mrvTelemetry.update();
    }

    int MrvGetWarehouseLevel(MrvAllianceField field )
    {
        int PyraPos = 0;
        if(mrvTfod != null && opModeIsActive())
        {
            boolean bDuckFound = false;
            float left   = 0.0f;
            float top    = 0.0f;
            float bottom = 0.0f;
            float right  = 0.0f;

            List<Recognition> updatedRecognition = mrvTfod.getUpdatedRecognitions();
            if(updatedRecognition != null) {
                mrvTelemetry.addData("# Objects detected: ", updatedRecognition.size());
                mrvDashboardTelemetryPacket.put("# Objects detected: ", updatedRecognition.size());
                int i = 0;
                for(Recognition recognition : updatedRecognition)
                {
                    left = recognition.getLeft();
                    right = recognition.getRight();
                    top = recognition.getTop();
                    bottom = recognition.getBottom();
                    if (recognition.getLabel()== "Duck") {
                        if ((FirstPosLeftMin <= left && left <= FirstPosLeftMax) && (FirstPosRightMin <= right && right <= FirstPosRightMax)) {
                            PyraPos = 1;
                        } else if ((SecPosLeftMin <= left && left <= SecPosLeftMax) && (SecPosRightMin <= right && right <= SecPosRightMax)) {
                            PyraPos = 2;

                        }

                        bDuckFound = true;
                        mrvTelemetry.addData(String.format("Object (%d:) ", i), recognition.getLabel());
                        mrvTelemetry.addData("Confidence: ", recognition.getConfidence());
                        mrvTelemetry.addData("(left, top), (right,bottom): ", String.format(" (%.03f, %.03f)  (%.03f, %.03f) ", left, top, right, bottom));
//                        mrvTelemetry.addData("Warehouse Level", PyraPos);
                        mrvTelemetry.update();

                        mrvDashboardTelemetryPacket.put(String.format("Object (%d:) ", i), recognition.getLabel());
                        mrvDashboardTelemetryPacket.put(String.format("Confidence (%d) ", i), recognition.getConfidence());
                        mrvDashboardTelemetryPacket.put(String.format("BBox (%d),", i), String.format(" (%.03f, %.03f)  (%.03f, %.03f) ", left, top, right, bottom));
                        mrvDashboardTelemetryPacket.put(String.format("Image Size (%d),", i), String.format(" Width: %d; Height: %d ", recognition.getImageWidth(), recognition.getImageHeight()));
                        i++;
                        break;
                    }
                }
            }
            else
            {
                mrvTelemetry.addData("Objects Detected","No Objects detected!");
                mrvTelemetry.update();
                PyraPos = 0;
            }
        }
        mrvTelemetry.addData("Warehouse Level", PyraPos);
        mrvTelemetry.update();

        return PyraPos;
    }

    // TODO: Autonomous Freight Drop off [Avi/Diya]
    void FreightDropOff(int level)
    {
        int iLinacDropoffPosition = (int)(Linac_Dropoff_Revs*Linac_Ticks_Per_Rev) *-1;
        int iDawinchiDropoffPosition = 0;
        switch(level)
        {
            case 0: // BOTTOM
                iDawinchiDropoffPosition = (int) (Dawinchi_Ticks_Per_Rev * DaWinchi_Level0_Dropoff);

                // Set winch to 0 level
                marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, STOP_AND_RESET_ENCODER);
                marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, RUN_WITHOUT_ENCODER);
                marvyn.setTargetPosition(Mrv_Robot.MrvMotors.DA_WINCHI, iDawinchiDropoffPosition);
                marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 1);
                while (opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.DA_WINCHI) < iDawinchiDropoffPosition) {
                    idle();
                }
                marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);

                // set Linac to drop off position
                int iPos = -1*Linac_Ticks_Per_Rev;
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
                iDawinchiDropoffPosition = (int) (Dawinchi_Ticks_Per_Rev * DaWinchi_Level1_Dropoff);

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
                iDawinchiDropoffPosition = (int) (Dawinchi_Ticks_Per_Rev * DaWinchi_Level2_Dropoff);
                iDawinchiDropoffPosition = -1*iDawinchiDropoffPosition;

                int iPos2 = (int)(-1.3*Linac_Ticks_Per_Rev);
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


    void buildTrajectories()
    {
//
        mrvRed1 = marvyn.mecanumDrive.trajectorySequenceBuilder(red_1_pose_estimate)
                //drive to hub
                .lineToLinearHeading(red1_shipping_hub_pos)
                .addTemporalMarker(() -> {
                    FreightDropOff(mrvWarehouseLevel);
                })
                //Release the Claw
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    marvyn.The_Claw.setPosition(Claw_Open_Pos);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    // retract Linac
                    marvyn.setTargetPosition(Mrv_Robot.MrvMotors.LIN_AC, 0);
                    marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 1);
                    while (opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.LIN_AC) < 0) {
                        idle();
                    }
                    marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 0);
                })
                .waitSeconds(2)
                .lineToLinearHeading(red_duck_wheel_pos)
                //do duck
                .addTemporalMarker(() -> {
                    marvyn.setPower(Mrv_Robot.MrvMotors.DUCK_WHEELS, 0.25);
                })
                .waitSeconds(4)
                .addTemporalMarker(() ->{
                    marvyn.setPower(Mrv_Robot.MrvMotors.DUCK_WHEELS, 0);
                })
                //enter warehouse
                .lineToLinearHeading(red_storage_pos)

                .build()
        ;
        return;
    }

    // TODO: Autonomous Freight Pickup [Lavanya]
    void FreightPickUp()
    {

    }

    // TODO: Autonomous Spin Duck Wheel [Avi/Diya]
    void SpinDuckWheel()
    {
        sleep(1000);
        marvyn.setPower(Mrv_Robot.MrvMotors.DUCK_WHEELS, 0.25*-1);
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



}


