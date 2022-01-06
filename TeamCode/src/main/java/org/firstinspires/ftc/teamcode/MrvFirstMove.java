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

// Robot Core

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Java Utils
import java.util.ArrayList;
import java.util.List;


// FTC Dashboard
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

// Vuforia
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

// Tensor Flow
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;


// Roadrunner
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name="Mrv_MyFirstMove", group="Manual mode")
@Config
public class MrvFirstMove extends LinearOpMode {

    enum MrvAllianceFieldPos
    {
        RED_1,
        RED_2,
        BLUE_1,
        BLUE_2
    }
    private static final String[] TFOD_MODEL_LABELS =
    {
        "Ball",
        "Cube",
        "Duck",
        "Marker",
        "AztechTSE"
    };


    private static FtcDashboard         mrvDashboard;
    private static VuforiaLocalizer     mrvVuforia;
    private static VuforiaTrackables    mrvVuTrackables = null;
    private TFObjectDetector            mrvTfod;
    Mrv_Robot                           marvyn = new Mrv_Robot();
    private static MultipleTelemetry    mrvTelemetry;
    private static TelemetryPacket     mrvDashboardTelemetryPacket = new TelemetryPacket();


    // VUFORIA Key
    public static final String VUFORIA_LICENSE_KEY = "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";

    // Field Dimensions
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;
    private OpenGLMatrix mrvLastLocation   = new OpenGLMatrix();

    private static Pose2d blueShippingHubPos;
    private static Pose2d redShippingHubPos;
    private static Pose2d mrvStartingPos2d;
    private static int iTeleCt = 1;

    // Dashboard configurable variables
    public static double xPos = 30;
    public static double yPos = 30;
    public static double turnPos = 0;
    public static double speed = 15;
    public static int TFodResolution = 320;
    public static double TFodZoomFactor = 2.5;
    public static int sleepyTime = 5000;

    public static double FirstPosLeftMin = -5;
    public static double FirstPosLeftMax = 115;
    public static double FirstPosRightMin = 60;
    public static double FirstPosRightMax = 230;

    public static double SecPosLeftMin = 315;
    public static double SecPosLeftMax = 530;
    public static double SecPosRightMin = 420;
    public static double SecPosRightMax = 640;

    @Override
    public void runOpMode() throws InterruptedException {

        // TODO: Roadrunner tuning [Atiksh/Anshul]
        // TODO: Update drive constants based on new parameters [Atiksh/Anshul]

        // init Mecanum Drive
        marvyn.init(hardwareMap);
        iTeleCt = 1;

        // init Dashboard
        mrvDashboard = FtcDashboard.getInstance();
        mrvTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry() );
        mrvTelemetry.clearAll();
        mrvTelemetry.update();
        mrvTelemetry.setAutoClear(false);
        FtcDashboard.getInstance().getTelemetry().setAutoClear(false);

        mrvTelemetry.addLine(String.format("%d. Marvin Initialized!", iTeleCt++));
        //        mrvTelemetry.update();
        double volts = getBatteryVoltage();
        mrvTelemetry.addData(String.format("%d. Battery voltage", iTeleCt++),  String.format("%.1f volts",volts) );
//        mrvTelemetry.update();

        // init VUFORIA
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuParams.cameraName = marvyn.eyeOfSauron;
        vuParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuParams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        // TODO: Do we need to tweak useExtendedTracking for Vuforia? [Lavanya]
        vuParams.useExtendedTracking = false;

        mrvVuforia = ClassFactory.getInstance().createVuforia(vuParams);
        mrvVuTrackables = mrvVuforia.loadTrackablesFromAsset("FreightFrenzy");

        // Name and locate each target on the Field
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,    halfField,     mmTargetHeight, 90, 0, 0 );
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,    halfField,     mmTargetHeight, 90, 0, 180);
        mrvTelemetry.addData(String.format("%d. Vuforia Trackers identified", iTeleCt++), mrvVuTrackables.size());
//        mrvTelemetry.update();

        // Calculate camera position
        // TODO: Identify the correct Camera position and update these variables. [Lavanya]
        final float CAMERA_FORWARD_DISPLACEMENT  = 7.0f * mmPerInch; // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch; // ex: Camera is 8 inches above the ground
        final float CAMERA_LEFT_DISPLACEMENT     = -6.0f* mmPerInch; // eg: Enter the left distance from the center of the robot to the camera lens

        // Tell vuforia where the camera is mounted on the robot
        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT )
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        // vuForia Trackables seem to need robot camera position
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(mrvVuTrackables);
        for(VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(vuParams.cameraName, cameraLocationOnRobot);
        }
        mrvTelemetry.addLine(String.format("%d. Trackables configured!", iTeleCt++));
//        mrvTelemetry.update();

        // Startup camera
        mrvDashboard.startCameraStream(mrvVuforia, 0); // start streaming camera
        mrvTelemetry.addLine(String.format("%d. Vuforia Camera Stream started", iTeleCt++));
//        mrvTelemetry.update();

        // Steps for Robot Autonomous execution
        // 1. Use Vuforia to detect robot's starting position (1 of 4 possible positions)
        // mrvVuTrackables.activate();
        MrvAllianceFieldPos startingPos= MrvAllianceFieldPos.BLUE_1;
        mrvVuTrackables.activate();
        sleep(sleepyTime);
        boolean TargetVisible = MrvGetRobotPosition(startingPos, mrvLastLocation);
        if (TargetVisible) {
            mrvTelemetry.addData(String.format("%d. StartingPos", iTeleCt++), startingPos);
            // Get initial pose 2D from starting pos
            VectorF translation = mrvLastLocation.getTranslation();
            Orientation rotation = Orientation.getOrientation(mrvLastLocation, EXTRINSIC, XYZ, RADIANS);
            mrvTelemetry.addData(String.format("%d. mrvLoc Pos (inches)", iTeleCt++), "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            mrvTelemetry.addData(String.format("%d. mrvLoc Rot (deg)", iTeleCt++), "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            mrvStartingPos2d = new Pose2d(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, rotation.thirdAngle);
            DashboardUtil dUtil = new DashboardUtil();
            dUtil.drawRobot(mrvDashboardTelemetryPacket.fieldOverlay(), mrvStartingPos2d);

            mrvTelemetry.addLine(String.format("%d. Starting pos updated on field overlay", iTeleCt++));
        }

        // init Tensorflow
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.4f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = TFodResolution;
//        tfodParameters
        mrvTfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, mrvVuforia);
        mrvTfod.loadModelFromAsset(TFOD_MODEL_ASSET, TFOD_MODEL_LABELS);

        mrvTelemetry.addLine(String.format("%d. Tensorflow assets loaded", iTeleCt++));
        mrvTelemetry.update();
        FtcDashboard.getInstance().sendTelemetryPacket(mrvDashboardTelemetryPacket);

        /* Get initial pose 2D from starting pos
        VectorF translation  = mrvLastLocation.getTranslation();
        Orientation rotation = Orientation.getOrientation(mrvLastLocation, EXTRINSIC,XYZ, RADIANS);
        mrvStartingPos2d = new Pose2d(translation.get(0)/mmPerInch, translation.get(1)/mmPerInch, rotation.thirdAngle);
        */
        // 2. Map Vuforia given field coordinates to Tensorflow screen coordinates <- hmm... maybe not.
        if (mrvTfod != null) {
            mrvTfod.activate();
            mrvTfod.setZoom(TFodZoomFactor, 16.0 / 9.0);
        }
        sleep(2000);
        mrvTelemetry.addLine("Tfod activated! Ready for Init!");
        mrvTelemetry.update();

        waitForStart();
        mrvDashboard.startCameraStream(mrvTfod, 0); // start streaming camera
        mrvTelemetry.addLine("Started TFod camera streaming");

        // 3. Use Tensorflow to read barcode and identify duck position
         // Give TFD a second to complete recognition processing
        mrvDashboardTelemetryPacket.put("Voltage: ", getBatteryVoltage());
        mrvDashboardTelemetryPacket.put("Recognition #", 0);
        int iLevel = MrvGetWarehouseLevel(startingPos);
        mrvDashboard.sendTelemetryPacket(mrvDashboardTelemetryPacket);
        mrvDashboardTelemetryPacket = new TelemetryPacket();

        // 4. Generate trajectory to drop off freight drop off
        if( startingPos == MrvAllianceFieldPos.BLUE_1 || startingPos == MrvAllianceFieldPos.BLUE_2) {
            // Move to BLue Shipping hub
            // Trajectory moveTo(
            marvyn.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-32, 60, Math.toRadians(90)))
                    //drive to hub
                    .lineToLinearHeading(new Pose2d(-12, -48, Math.toRadians(90)))
                    //TODO: set exact end pose for duck spline
                    .splineTo(new Vector2d(-48,-56),Math.toRadians(-135))
                    //do duck
                    .waitSeconds(3)
                    //go to warehouse entrance
                    //.lineToLinearHeading(new Pose2d(18,-64,Math.toRadians(0)))
                    //enter warehouse
                    //.forward(36)
                    //wait to not cause the weird Roadrunner heading issue
                    //.waitSeconds(0.5)
                    //move over
                    //.lineToLinearHeading(new Pose2d(56,-40,Math.toRadians(-90)))
                    //.strafeLeft(24)

                    .build()
                ;

        }
        else if (startingPos == MrvAllianceFieldPos.RED_1 || startingPos == MrvAllianceFieldPos.RED_2)
        {
            // Move to Red Shipping hub pos
        }

        // 5. Follow trajectory
        // 6. Freight drop off
        // 7. Generate trajectory to go to duck wheel (red/blue)
        // 8. Follow trajectory
        // 9. Spin duck wheel
        // 10. Generate trajectory to park in warehouse
        // 11. Follow trajectory to park in warehouse

        // Alternately, if we are not doing duck wheel
        //  7. Generate trajectory to go to warehouse
        //  8. Use Tensorflow to detect freight element in warehouse
        //  9. Follow trajectory to freight element
        //  10. pick up freight element
        //  11. Generate trajectory to return to freight drop off
        //  12. Follow trajectory
        //  13. Freight Drop off
        //  14. Generate trajectory to park in Warehouse
        //  15. Follow trajectory to park in Warehouse

        if (isStopRequested()) return;

       // Building the trajectory
       /*
        Trajectory Red = marvyn.mecanumDrive.trajectoryBuilder(mrvStartingPos2d)
                .splineTo(new Vector2d(-12,-48),Math.toRadians(90),
                    SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        marvyn.mecanumDrive.followTrajectory(Red);

        sleep(2000);

        marvyn.mecanumDrive.followTrajectory(marvyn.mecanumDrive.trajectoryBuilder(Red.end(), true)
        .splineTo(new Vector2d(0, 0), Math.toRadians(180))

        .build());

        // Building the trajectory
        Trajectory Blue = marvyn.mecanumDrive.trajectoryBuilder(mrvStartingPos2d)
                .splineTo(new Vector2d(-12,48),Math.toRadians(-90),
                        SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        marvyn.mecanumDrive.followTrajectory(Red);

        sleep(2000);

        marvyn.mecanumDrive.followTrajectory(marvyn.mecanumDrive.trajectoryBuilder(Red.end(), true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build());
*/

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

    //TODO: Autonomous MrvGetStartingPosition [Lavanya]
    boolean MrvGetRobotPosition(MrvAllianceFieldPos fieldPos, OpenGLMatrix lastLocation)
    {
        boolean bTargetVisible = false;
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(mrvVuTrackables);
        int iCount = 0;
        for (VuforiaTrackable trackable : allTrackables) {
            //mrvTelemetry.addLine(String.format("Checking tracker: %d  ", iCount++) + trackable.getName());
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                mrvTelemetry.addData(String.format("%d.  Vuforia Visibile Target", iTeleCt++) , trackable.getName());
                bTargetVisible = true;
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation.multiply(robotLocationTransform);
                    VectorF translation = robotLocationTransform.getTranslation();
                    Orientation rotation = Orientation.getOrientation(mrvLastLocation, EXTRINSIC, XYZ, RADIANS);
                    mrvTelemetry.addData(String.format("%d. RobLoc Pos (inches)", iTeleCt++), "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                    mrvTelemetry.addData(String.format("%d. RobLoc Rot (deg)", iTeleCt++), "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                }
                break;
            }
            else
            {
                mrvTelemetry.addLine(String.format("%d. ",iTeleCt++) + trackable.getName() + String.format(" Not Visible"));
            }

        }
        if (!bTargetVisible) {
            mrvTelemetry.addLine(String.format("%d. Vuforia Visible Target: None!", iTeleCt++));
        }

        return bTargetVisible;
    }

    // TODO: Autonmous MrvGetWarehouseLevel [Lavanya]
    int MrvGetWarehouseLevel(MrvAllianceFieldPos fieldPos)
    {
        if(mrvTfod != null && opModeIsActive())
        {
            boolean bDuckFound = false;
            int duckPos = 0;
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
                    //if(recognition.getLabel() == "Duck") {
                    if (recognition.getLabel()== "Duck") {
                        if ((FirstPosLeftMin <= left && left <= FirstPosLeftMax) && (FirstPosRightMin <= right && right <= FirstPosRightMax)) {
                            duckPos = 1;
                        } else if ((SecPosLeftMin <= left && left <= SecPosLeftMax) && (SecPosRightMin <= right && right <= SecPosRightMax)) {
                            duckPos = 2;

                        } else {
                            duckPos = 0;
                        }

                    } else {
                        duckPos = 3;
                    }

                        bDuckFound = true;
                        mrvTelemetry.addData(String.format("Object (%d:) ", i), recognition.getLabel());
                        mrvTelemetry.addData("Confidence: ", recognition.getConfidence());
                        mrvTelemetry.addData("(left, top), (right,bottom): ", String.format(" (%.03f, %.03f)  (%.03f, %.03f) ",  left, top, right, bottom) );
                        mrvTelemetry.addData("Warehouse Level", duckPos);
                        mrvTelemetry.update();

                        mrvDashboardTelemetryPacket.put(String.format("Object (%d:) ", i), recognition.getLabel());
                        mrvDashboardTelemetryPacket.put(String.format("Confidence (%d) ", i), recognition.getConfidence());
                        mrvDashboardTelemetryPacket.put(String.format("BBox (%d),", i), String.format(" (%.03f, %.03f)  (%.03f, %.03f) ", left, top, right, bottom) );
                        mrvDashboardTelemetryPacket.put(String.format("Image Size (%d),", i), String.format(" Width: %d; Height: %d ", recognition.getImageWidth(), recognition.getImageHeight()) );




                        i++;
                        //break;
                    //}
                }
            }
            else
            {
                mrvTelemetry.addData("Objects Detected","No Objects detected!");
                mrvTelemetry.update();
            }
        }
        mrvTelemetry.update();


        // Based on ducks detected - how do we map to the level for each of these positions?
        // Hint: Could we figure out the image rectangles for each of these positions and then go from there?
        //  Would we find ducks that are from the opposing Alliance's bar codes as well?
        switch(fieldPos)
        {
            case RED_1:
                break;
            case RED_2:
                break;
            case BLUE_1:
                break;
            case BLUE_2:
                break;
        }
        return 0;
    }

    // TODO: Autonomous Freight Drop off [Avi/Diya]
    void FreightDropOff(int level)
    {


    }

    // TODO: Autonomous Freight Pickup [Lavanya]

    void FreightPickUp()
    {

    }

    // TODO: Autonomous Spin Duck Wheel [Avi/Diya]
    void SpinDuckWheel()
    {
    }

    /*
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */


    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = mrvVuTrackables.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }


}


