// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    // Properties are consistent with an Arducam OV9281 running on an Orange Pi 5
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(79.0));
    cameraProperties.setCalibError(0.25, 0.08);
    cameraProperties.setFPS(30.0);
    cameraProperties.setAvgLatencyMs(20.0);
    cameraProperties.setLatencyStdDevMs(5.0);
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);

    // Simulated camera frames can be enabled during development
    // They are very compute-intensive and should be disabled for normal robot operation
    // View these cameras starting at localhost:1181 for camera0 and going up
    cameraSim.enableRawStream(VisionConstants.enableSimRawStream);
    cameraSim.enableProcessedStream(VisionConstants.enableSimProcessedStream);
    cameraSim.enableDrawWireframe(VisionConstants.enableSimWireframe);

    // attach the simulated camera to the vision simulation
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
