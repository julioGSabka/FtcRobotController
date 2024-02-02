package org.firstinspires.ftc.teamcode.RoadRunnerScripts.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

public class ApriltagLocalizer implements Localizer {


    @NonNull
    @Override
    public Pose2d getPoseEstimate() {




        return null;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {

    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {

    }
}
