package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

/*
 * This is an example of a more complex path to really test the tuning.
 */


@Config
@Autonomous(group = "Autonomous")
public class Mrv_AutoBlue2 extends LinearOpMode {

    enum MrvAllianceField
    {
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
    private static MultipleTelemetry mrvTelemetry;
    private static TelemetryPacket mrvDashboardTelemetryPacket = new TelemetryPacket();
    private static int iTeleCt = 1;

    // VUFORIA Key
    public static final String VUFORIA_LICENSE_KEY = "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";
    //private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
      private static final String TFOD_MODEL_ASSET = "FreightFrenzy_AztechsTSE1.tflite";
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
    public static double Wrist_Parallel_to_ground = 0.52; // Parallel to ground for pickup
    public static double Wrist_chute_dropoff = 0.85; // Perpendicular to Arm at top
    public static double Wrist_Start_Pos = 0.0; // Perpendicular to Arm at bottom


    // Tensor flow - Warehouse level detection
    public static int TFodResolution = 320;
    public static double TFodZoomFactor = 1;
    public static int sleepyTime = 5000;


 
    public static double slower_speed = 25;
    public static double slower_accel = 25;

    // Drop of Warehouse
    public static int mrvWarehouseLevel;
    public static double DaWinchi_Level0_Dropoff = 0.5;
    public static double DaWinchi_Level1_Dropoff = 0.10;
    public static double DaWinchi_Level2_Dropoff = 0.45;
    public static double DaWinchi_pickup_Revs = 0.82;

    public static double Linac_Dropoff_Revs = 1.3;
    public static double Linac_Pickup_Revs = 2.3;

    public static int mrvDawinchiCalculatedDropOffPos = 0;
    public static int mrvLinacCalcualtedDropOffPos = 0;

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
//        marvyn.The_Claw.setPosition(Claw_Close_Pos);

        // Startup camera
        mrvDashboard.startCameraStream(mrvTfod, 0); // start streaming camera
        mrvTelemetry.addLine(String.format("%d. TFod Camera Stream started", iTeleCt++));
        mrvTelemetry.update();

        if (mrvTfod != null) {
            mrvTfod.activate();
            mrvTfod.setZoom(TFodZoomFactor, 16.0 / 9.0);
        }
        sleep(1000);
        mrvTelemetry.addLine("Tfod activated! Ready to Start!");
        mrvTelemetry.update();

        waitForStart();
        marvyn.Wristy.setPosition(Wrist_Parallel_to_Linac);
        mrvWarehouseLevel = MrvGetWarehouseLevel(MrvAllianceField.BLUE);
        FtcDashboard.getInstance().sendTelemetryPacket(mrvDashboardTelemetryPacket);
        mrvDashboardTelemetryPacket = new TelemetryPacket();

        mrvTelemetry.addData("Warehouse Level", mrvWarehouseLevel);
        mrvTelemetry.update();

        buildTrajectories();
        //figureSkating();
        //FreightPickUp();
        marvyn.mecanumDrive.setPoseEstimate(marvyn.blue_2_pose_estimate);

        ElapsedTime time = new ElapsedTime(MILLISECONDS);
        time.reset();
        time.startTime();
        marvyn.mecanumDrive.followTrajectorySequence(mrvBlue2);
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
            float mdpt = 0.0f;
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
                    mdpt = (left+right)/2;

                    //if (recognition.getLabel()== "Duck") {
                    if (recognition.getLabel()== "Aztechs_TSE" && recognition.getConfidence() > 0.8) {
                            if (marvyn.FirstPosMin <= mdpt && mdpt <= marvyn.FirstPosMax ) {
                            PyraPos = 1;
                        } else if (marvyn.SecPosMin <= mdpt && mdpt <= marvyn.SecPosMax) {
                            PyraPos = 2;

                        }

                        bDuckFound = true;
                        mrvTelemetry.addData(String.format("Object (%d:) ", i), recognition.getLabel());
                        mrvTelemetry.addData("Confidence: ", recognition.getConfidence());
                        mrvTelemetry.addData("(left, top), (right,bottom): ", String.format(" (%.03f, %.03f)  (%.03f, %.03f) ", left, top, right, bottom));

                        mrvTelemetry.update();

                        mrvDashboardTelemetryPacket.put(String.format("Object (%d:) ", i), recognition.getLabel());
                        mrvDashboardTelemetryPacket.put(String.format("Confidence (%d) ", i), recognition.getConfidence());
                        mrvDashboardTelemetryPacket.put(String.format("BBox (%d),", i), String.format(" (%.03f, %.03f)  (%.03f, %.03f) ", left, top, right, bottom));
                        mrvDashboardTelemetryPacket.put(String.format("BBox midpt (%d)", i), String.format(" (%.03f)", mdpt));
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
        mrvDashboardTelemetryPacket.put(String.format("Warehouse Level:"), String.format("%d",PyraPos));

        mrvTelemetry.update();

        return PyraPos;
    }

    void FreightDropOff(int level)
    {
        int iLinacDropoffPosition = (int)(Linac_Dropoff_Revs*marvyn.Linac_Ticks_Per_Rev) *-1;
        int iDawinchiDropoffPosition = 0;
        switch(level)
        {
            case 0: // BOTTOM
                iDawinchiDropoffPosition = (int) (marvyn.Dawinchi_Ticks_Per_Rev * DaWinchi_Level0_Dropoff);

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
                int iPos = -1*marvyn.Linac_Ticks_Per_Rev;
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
                iDawinchiDropoffPosition = (int) (marvyn.Dawinchi_Ticks_Per_Rev * DaWinchi_Level1_Dropoff);

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
                iDawinchiDropoffPosition = (int) (marvyn.Dawinchi_Ticks_Per_Rev * DaWinchi_Level2_Dropoff);
                iDawinchiDropoffPosition = -1*iDawinchiDropoffPosition;

                int iPos2 = (int)(-1.3*marvyn.Linac_Ticks_Per_Rev);
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

        mrvBlue2 = marvyn.mecanumDrive.trajectorySequenceBuilder(marvyn.blue_2_pose_estimate)
                //drive to hub
                .lineToLinearHeading(marvyn.blue_2_shipping_hub_pos)
                .addTemporalMarker(() -> {
                    FreightDropOff(mrvWarehouseLevel);
                })
                //Release the Claw
//                .addTemporalMarker(()->{
//                    marvyn.The_Claw.setPosition(Claw_Open_Pos);
//                })
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
                //enter warehouse
                .lineToLinearHeading(marvyn.blue_warehouse_enter_pos)
                .lineToLinearHeading(marvyn.blue_warehouse_pos)
                .lineToLinearHeading(marvyn.blue_park_pos)
                .build()
        ;
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


