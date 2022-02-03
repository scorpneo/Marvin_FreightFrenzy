package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    public static void main(String args[]) {
        MeepMeep mm = new MeepMeep(1000)
                .setBackgroundAlpha(1f)
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL);

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

        Pose2d blue_warehouse_pos1 = new Pose2d(42.75, 64.75, Math.toRadians(0));
        Pose2d blue_warehouse_pos2 = new Pose2d(45.75, 64.75, Math.toRadians(0));
        Pose2d red_warehouse_pos = new Pose2d(36.75, -64.75, Math.toRadians(0));

        Pose2d blue_storage_pos = new Pose2d(-59.75, 35.5, Math.toRadians(0));
        Pose2d red_storage_pos = new Pose2d(-59.75, -35.5, Math.toRadians(0));

        Pose2d blue_warehouse_park_pos = new Pose2d(42.75, 38, Math.toRadians(15));
        Pose2d red_park_pos = new Pose2d(64, -36, Math.toRadians(-90));


        double dFromLevel0ToPickup = 0.3;    // 1. Time to lower from level 0 -> Pickup
        double dFromLevel1ToPickup = 0.3;    // 2. Time to lower from level 1 -> Pickup
        double dFromLevel2ToPickup = 0.3;    // 3. Time to lower from level 2 -> Pickup
        double dRaiseToLevel2      = 0.3;    // 4. Time to raise from Pickup -> level2
        double dEjectFreight       = 0.3;    // 5. Time to drop off freight element
        double dIntakeFreight      = 0.3;    // 6. Time to pick up freight element
        double offsetEjectFreight  = 0.3;    // 7. Offset to eject freight after reaching shipping hub pos
        double offsetPickupFreight = 0.3;    // 8. Offset to pickup freight before reaching warehouse pos
        double offsetLowerToPickup = 0.3;  // 9. Offset to lower to pickup position (Start to do this after retracting from drop-off position)
        double offsetRaiseToDropOff = 0.3; // 10. Offset to raise to drop-off position (Start to do this after retracting from pickup position)

        double dWarehouseLevelToPickup = dFromLevel0ToPickup;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(54, 54, Math.toRadians(180), Math.toRadians(180), 11.8)
                .followTrajectorySequence(drive ->
                        //red duck
                        drive.trajectorySequenceBuilder(blue_2_pose_estimate)
                                // Step 1: Drop off pre-loaded Freight and return to Warehouse Enter pos
                                .lineToLinearHeading(blue_2_shipping_hub_pos)
                                .UNSTABLE_addTemporalMarkerOffset(offsetEjectFreight,() -> {  // Runs 0.1 seconds into the WaitSeconds call and keeps the servos on for 0.4s
//                                    marvyn.Claw_Left.setPower(-0.5);
//                                    marvyn.Claw_Right.setPower(-0.5);
                                })
                                .waitSeconds(dEjectFreight)
                                .addTemporalMarker( ()-> {
//                                    marvyn.Claw_Left.setPower(0);
//                                    marvyn.Claw_Right.setPower(0);
                                })
                                .lineToLinearHeading(blue_warehouse_enter_pos)
                                .UNSTABLE_addTemporalMarkerOffset(offsetLowerToPickup, () -> {  // Begin Lowering DaWinchi 0.5 seconds before reaching warehouse enter pos and keep it running for 0.5 s after (total time = 1s)
//                                    marvyn.Wristy.setPosition(Wrist_Pickup_Pos);
//                                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0.5);
                                })
                                .waitSeconds(dWarehouseLevelToPickup)
                                .addTemporalMarker(() -> {
//                                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);
                                })
                                // Step2: Enter Warehouse, Pickup freight, drop it off and return to Warehouse enter pos
                                .lineToLinearHeading(blue_warehouse_pos1)
                                .UNSTABLE_addTemporalMarkerOffset(offsetPickupFreight, () -> { // Start intake 0.3 seconds before reahing warehouse_pos and keep running them for 0.5 seconds after.
//                                    marvyn.Claw_Left.setPower(0.5);
//                                    marvyn.Claw_Right.setPower(0.5);
                                })
                                .waitSeconds(dIntakeFreight)
                                .addTemporalMarker(()-> {
//                                    marvyn.Claw_Left.setPower(0);
//                                    marvyn.Claw_Right.setPower(0);
                                })
                                .lineToLinearHeading(blue_warehouse_enter_pos)//, marvyn.mecanumDrive.getVelocityConstraint(slower_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), marvyn.mecanumDrive.getAccelerationConstraint(slower_accel))
                                .UNSTABLE_addTemporalMarkerOffset(offsetRaiseToDropOff, () -> { // Begin Raising DaWinchi 0.5 seconds before reaching warehouse enter pos and keep it running for 0.5 s after (total time = 1s)
//                                    marvyn.Wristy.setPosition(Wrist_Dropoff_Pos);
//                                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, -0.5);
                                })
                                .waitSeconds(dRaiseToLevel2)
                                .addTemporalMarker(()->{
//                                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);
                                })
                                .lineToLinearHeading(blue_2_shipping_hub_pos)
                                .UNSTABLE_addTemporalMarkerOffset(offsetEjectFreight,() -> {  // Runs 0.1 seconds into the WaitSeconds call and keeps the servos on for 0.4s
//                                    marvyn.Claw_Left.setPower(-0.5);
//                                    marvyn.Claw_Right.setPower(-0.5);
                                })
                                .waitSeconds(dEjectFreight)
                                .addTemporalMarker(() -> {
//                                    marvyn.Claw_Left.setPower(0);
//                                    marvyn.Claw_Right.setPower(0);
                                })
                                .lineToLinearHeading(blue_warehouse_enter_pos)
                                .UNSTABLE_addTemporalMarkerOffset(offsetLowerToPickup, () -> {  // Begin Lowering DaWinchi 0.5 seconds before reaching warehouse enter pos and keep it running for 0.5 s after (total time = 1s)
//                                    marvyn.Wristy.setPosition(Wrist_Pickup_Pos);
//                                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0.5);
                                })
                                .waitSeconds(dWarehouseLevelToPickup)
                                .addTemporalMarker(() -> {
//                                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);
                                })

                                // Step 3: Enter Warehouse, Pickup freight, drop it off and return to Warehouse enter pos
                                .lineToLinearHeading(blue_warehouse_pos2)//, marvyn.mecanumDrive.getVelocityConstraint(slower_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), marvyn.mecanumDrive.getAccelerationConstraint(slower_accel))
                                .UNSTABLE_addTemporalMarkerOffset(offsetPickupFreight, () -> { // Start intake 0.3 seconds before reahing warehouse_pos and keep running them for 0.5 seconds after.
//                                    marvyn.Claw_Left.setPower(0.5);
//                                    marvyn.Claw_Right.setPower(0.5);
                                })
                                .waitSeconds(dIntakeFreight)
                                .addTemporalMarker(()-> {
//                                    marvyn.Claw_Left.setPower(0);
//                                    marvyn.Claw_Right.setPower(0);
                                })
                                .lineToLinearHeading(blue_warehouse_enter_pos)//, marvyn.mecanumDrive.getVelocityConstraint(slower_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), marvyn.mecanumDrive.getAccelerationConstraint(slower_accel))
                                .UNSTABLE_addTemporalMarkerOffset(offsetRaiseToDropOff, () -> { // Begin Raising DaWinchi 0.5 seconds before reaching warehouse enter pos and keep it running for 0.5 s after (total time = 1s)
//                                    marvyn.Wristy.setPosition(Wrist_Dropoff_Pos);
//                                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, -0.5);
                                })
                                .waitSeconds(dRaiseToLevel2)
                                .addTemporalMarker(()->{
//                                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);
                                })
                                .lineToLinearHeading(blue_2_shipping_hub_pos)
                                .UNSTABLE_addTemporalMarkerOffset(offsetEjectFreight,() -> {  // Runs 0.1 seconds into the WaitSeconds call and keeps the servos on for 0.4s
//                                    marvyn.Claw_Left.setPower(-0.5);
//                                    marvyn.Claw_Right.setPower(-0.5);
                                })
                                .waitSeconds(dEjectFreight)
                                .addTemporalMarker(() -> {
//                                    marvyn.Claw_Left.setPower(0);
//                                    marvyn.Claw_Right.setPower(0);
                                })
                                .lineToLinearHeading(blue_warehouse_enter_pos)
                                .UNSTABLE_addTemporalMarkerOffset(offsetLowerToPickup, () -> {  // Begin Lowering DaWinchi 0.5 seconds before reaching warehouse enter pos and keep it running for 0.5 s after (total time = 1s)
//                                    marvyn.Wristy.setPosition(Wrist_Pickup_Pos);
//                                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0.5);
                                })
                                .waitSeconds(dWarehouseLevelToPickup)
                                .addTemporalMarker(() -> {
//                                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);
                                })

                                // Step 4: Go to park position
                                .lineToLinearHeading(blue_warehouse_pos1)
                                .lineToLinearHeading(blue_warehouse_park_pos)
                                //enter warehouse
                                .build()
                );

        mm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}