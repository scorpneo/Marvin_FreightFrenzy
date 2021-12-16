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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Java Utils
import java.util.ArrayList;
import java.util.List;


// FTC Dashboard
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

// Vuforia
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

// Tensor Flow
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


// Roadrunner
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous
public class MrvFirstMove extends LinearOpMode {

    enum MrvAllianceFieldPos
    {
        RED_1,
        RED_2,
        BLUE_1,
        BLUE_2
    }
    private static final String[] TFOD_MODEL_LABELS
    {
        "Ball"",
        "Cube",
        "Duck"",
        "Marker",
        "AztechTSE"
    };

    public static double xPos = 30;
    public static double yPos = 30;
    public static double turnPos = 0;
    public static double speed = 15;

    private static FtcDashboard         mrvDashboard;
    private static VuforiaLocalizer     mrvVuforia;
    private static VuforiaTrackables    mrvVuParams = null;
    private TFObjectDetector            mrvTfod;
    private static Mrv_Robot            marvyn;

    // VUFORIA Key
    public static final String VUFORIA_LICENSE_KEY = "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";

    // Field Dimensions
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;
    private OpenGLMatrix mrvLastLocation   = null;
    public static Pose2d blueShippingHubPos;
    public static Pose2d redShippingHubPos;

    @Override
    public void runOpMode() throws InterruptedException {

        // init Mecanum Drive
        marvyn.init(hardwareMap);

        // init Dashboard
        mrvDashboard = new FtcDashboard();

        // init VUFORIA
        VuforiaLocalizer.Parameters vuParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuParams.cameraName = marvyn.eyeOfSauron;
        vuParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuParams.useExtendedTracking = false;

        mrvVuforia = ClassFactory.getInstance().createVuforia(vuParams);
        mrvVuParams = mrvVuforia.loadTrackablesFromAsset("FreightFrenzy");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(mrvVuParams);

        // Name and locate each target on the Field
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,    halfField,     mmTargetHeight, 90, 0, 0 );
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,    halfField,     mmTargetHeight, 90, 0, 180);

        // Calculate camera position
        final float CAMERA_FORWARD_DISPLACEMENT  = 7.0f * mmPerInch; // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch; // ex: Camera is 8 inches above the ground
        final float CAMERA_LEFT_DISPLACEMENT     = -6.0f* mmPerInch; // eg: Enter the left distance from the center of the robot to the camera lens

        // Tell vuforia where the camera is mounted on the robot
        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT )
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        // vuForia Trackables seem to need robot camera position
        for(VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(vuParams.cameraName, cameraLocationOnRobot)
        }

        // init Tensorflow
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        mrvTfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, mrvVuforia);
        mrvTfod.loadModelFromAsset(TFOD_MODEL_ASSET, TFOD_MODEL_LABELS);
        if(mrvTfod != null) {
            mrvTfod.activate();
            mrvTfod.setZoom(2.5, 16.0/9.0);
        }

        mrvDashboard.startCameraStream(mrvVuforia, 0); // start streaming camera

        // Steps for Robot Autonomous execution
        // 1. Use Vuforia to detect robot's starting position (1 of 4 possible positions)
        MrvAllianceFieldPos startingPos = MrvGetStartingPosition(mrvVuParams, mrvLastLocation);

        // 2. Map Vuforia given field coordinates to Tensorflow screen coordinates <- hmm... maybe not.

        waitForStart();

        // 3. Use Tensorflow to read barcode and identify duck position
        int iLevel = MrvGetWarehouseLevel(startingPos);

        // 4. Generate trajectory to drop off freight drop off
        if(startingPos = MrvAllianceFieldPos.BLUE_1 || MrvAllianceFieldPos.BLUE_2 ) {
            // Move to BLue Shipping hub

        }
        else if (startingPos == MrvAllianceFieldPos.RED_1 || MrvAllianceFieldPos.RED_2)
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

        //Building the trajectory
        Trajectory MrvFirstMove = marvyn.mecanumDrive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(xPos,yPos),Math.toRadians(turnPos),
                    SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        marvyn.mecanumDrive.followTrajectory(MrvFirstMove);

        sleep(2000);

        marvyn.mecanumDrive.followTrajectory(marvyn.mecanumDrive.trajectoryBuilder(MrvFirstMove.end(), true)
        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
        .build());
    };

    MrvAllianceFieldPos MrvGetStartingPosition(VuforiaTrackables vuTargets, OpenGLMatrix lastLocation)
    {
        return MrvAllianceFieldPos.RED_1;
    }

    int MrvGetWarehouseLevel(MrvAllianceFieldPos fieldPos)
    {
        if(mrvTfod != null && opModeIsActive())
        {
            List<Recognition> updatedRecognition = mrvTfod.getUpdatedRecognitions();
            if(updatedRecognition != null) {
                telemetry.addData("# Objects detected", updatedRecognition.size());
                for(Recognition recognition : updatedRecognition)
                {
                    if(recognition.getLabel() == "Duck")
                        telemetry.addData(String.format("duck detected @"));
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
            }
            else
            {
                telemetry.addData("No Objects detected!");
            }
        }

        // TODO: Based on ducks detected - how do we map to the level for each of these positions?
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

    void FreightDropOff(int level)
    {

    }

    void FreightPickUp()
    {

    }

    void SpinDuckWheel()
    {
    }

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }
    }


