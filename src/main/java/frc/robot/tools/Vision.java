/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot.tools;

 //import static frc.robot.Constants.Vision.*;
 import static frc.robot.Constants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
 import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.Robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
 import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.targeting.PhotonTrackedTarget;
 
 public class Vision {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;
    private GenericEntry photonVisionEnableCam1;
    private GenericEntry photonVisionEnableCam2;
    private boolean cam1Enabled = false;
    private Transform3d cameraToRobot = null; 
    GenericEntry entryCameraXOffset = null;
    GenericEntry entryCameraYOffset = null;
    GenericEntry entryCameraHeight = null;
    GenericEntry entryCameraRotation = null;
    private boolean cam2Enabled = false;
    /*private Transform3d camera2ToRobot = Constants.PhotonVisionConstants.camera2ToRobot;
    GenericEntry entryCamera2XOffset = null;
    GenericEntry entryCamera2YOffset = null;
    GenericEntry entryCamera2Height = null;*/

    private AprilTagFieldLayout aprilTagFieldLayout = null;
 
     // Simulation
     private PhotonCameraSim cameraSim;
     private VisionSystemSim visionSim;

     private EstimatedRobotPose estimatedRobotPose;
     private boolean canSeeTag = false;

     public enum CameraEnum { Camera1, Camera2 }

     private CameraEnum cameraEnum;
    List<Pose3d> allTagPoses = new ArrayList<>();
 
     /**
      * @param estConsumer Lamba that will accept a pose estimate and pass it to your desired {@link
      *     edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
      */
    public Vision(EstimateConsumer estConsumer, CameraEnum cameraEnum) {
         this.estConsumer = estConsumer;
         this.cameraEnum = cameraEnum;
         String cameraName = "";
         cam1Enabled = Constants.kEnablePhotonVisionCamera1;
         cam2Enabled = Constants.kEnablePhotonVisionCamera2;

        ShuffleboardTab photonVisionTab = Shuffleboard.getTab("PhotonVision");
        if(cameraEnum == CameraEnum.Camera1) {

            photonVisionEnableCam1 = photonVisionTab.add("PhotonVisionEnableCam1", cam1Enabled)
            .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
            photonVisionEnableCam1.setBoolean(cam1Enabled);

        } else if(cameraEnum == CameraEnum.Camera2) {

            photonVisionEnableCam2 = photonVisionTab.add("PhotonVisionEnableCam2", cam2Enabled)
            .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
            photonVisionEnableCam2.setBoolean(cam2Enabled);

        }
        
        try {
            aprilTagFieldLayout = AprilTagFieldLayout
						.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark
						.m_resourceFile);
        } catch (IOException e) {
            System.out.println(e.toString());
        }

        if(cameraEnum == CameraEnum.Camera1) {
            cameraName = PhotonVisionConstants.CameraName;
            photonEstimator =
                 new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PhotonVisionConstants.cameraToRobot);
        } else {
            cameraName = PhotonVisionConstants.Camera2Name;
            photonEstimator =
                 new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PhotonVisionConstants.camera2ToRobot);
        }


        //camera = new PhotonCamera(PhotonVisionConstants.CameraName);
        if(cameraEnum == CameraEnum.Camera1) {
            camera = new PhotonCamera("cam1");
        } else {
            camera = new PhotonCamera("cam2");
        }

         
 
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
 
        // ----- Simulation
        if (Robot.isSimulation()) {
             // Create the vision system simulation which handles cameras and targets on the field.
             visionSim = new VisionSystemSim("main");
             // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
             visionSim.addAprilTags(aprilTagFieldLayout);
             // Create simulated camera properties. These can be set to mimic your actual camera.
             var cameraProp = new SimCameraProperties();
             cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
             cameraProp.setCalibError(0.35, 0.10);
             cameraProp.setFPS(15);
             cameraProp.setAvgLatencyMs(50);
             cameraProp.setLatencyStdDevMs(15);
             // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
             // targets.
             cameraSim = new PhotonCameraSim(camera, cameraProp);
             // Add the simulated camera to view the targets on this simulated field.

             if(cameraEnum == CameraEnum.Camera1) {
                visionSim.addCamera(cameraSim, PhotonVisionConstants.cameraToRobot);
             } else {
                visionSim.addCamera(cameraSim, PhotonVisionConstants.camera2ToRobot);
             }
 
             cameraSim.enableDrawWireframe(true);
        }

        if(Constants.PhotonVisionConstants.debugPhotonVision) {
            String cameraOffsetName = "camera1";
            cameraToRobot = Constants.PhotonVisionConstants.cameraToRobot;

            if(cameraEnum == CameraEnum.Camera2) {
                cameraOffsetName = "camera2";
                cameraToRobot = Constants.PhotonVisionConstants.camera2ToRobot;
            }

            entryCameraXOffset = photonVisionTab.add(cameraOffsetName + "XOffset", cameraToRobot.getX())
            .withWidget(BuiltInWidgets.kTextView)
			.getEntry();

            entryCameraYOffset = photonVisionTab.add(cameraOffsetName + "YOffset", cameraToRobot.getY())
            .withWidget(BuiltInWidgets.kTextView)
			.getEntry();

            entryCameraHeight = photonVisionTab.add(cameraOffsetName + "Height", cameraToRobot.getZ())
            .withWidget(BuiltInWidgets.kTextView)
			.getEntry();

            entryCameraRotation = photonVisionTab.add(cameraOffsetName + "Rotation", cameraToRobot.getRotation().getAngle())
            .withWidget(BuiltInWidgets.kTextView)
			.getEntry();
        }
    }
 
     public void periodic() {

        if(Constants.PhotonVisionConstants.debugPhotonVision) {
            if(
                entryCameraXOffset.getDouble(0.0) != cameraToRobot.getX()
                || entryCameraYOffset.getDouble(0.0) != cameraToRobot.getY()
                || entryCameraHeight.getDouble(0.0) != cameraToRobot.getZ()
                || entryCameraRotation.getDouble(0.0) != cameraToRobot.getRotation().getAngle()
            ) {


                cameraToRobot = new Transform3d(
                    new Translation3d(
                        entryCameraXOffset.getDouble(0.0), 
                        entryCameraYOffset.getDouble(0.0), 
                        entryCameraHeight.getDouble(0.0)
                    ),
                    new Rotation3d(
						0,
						entryCameraRotation.getDouble(0.0),
						0
					)
                );

            }
        }

        allTagPoses.clear();

        if(cameraEnum == CameraEnum.Camera1) {
            cam1Enabled = photonVisionEnableCam1.getBoolean(true);
        } else if(cameraEnum == CameraEnum.Camera2) {
            cam2Enabled = photonVisionEnableCam2.getBoolean(true);
        }

         Optional<EstimatedRobotPose> visionEst = Optional.empty();
         for (var change : camera.getAllUnreadResults()) {
             visionEst = photonEstimator.update(change);
             updateEstimationStdDevs(visionEst, change.getTargets());
 
             if (Robot.isSimulation()) {
                 visionEst.ifPresentOrElse(
                         est ->
                                 getSimDebugField()
                                         .getObject("VisionEstimation")
                                         .setPose(est.estimatedPose.toPose2d()),
                         () -> {
                             getSimDebugField().getObject("VisionEstimation").setPoses();
                         });
             }
 
             visionEst.ifPresent(
                    est -> {
                         // Change our trust in the measurement based on the tags we can see
                         var estStdDevs = getEstimationStdDevs();
 
                         estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

                         if(cameraEnum == CameraEnum.Camera1) {
                            Logger.recordOutput(
								"PhotonVisionEstimator1/Robot",
								est.estimatedPose.toPose2d());
                         } else if(cameraEnum == CameraEnum.Camera2) {
                            Logger.recordOutput(
								"PhotonVisionEstimator2/Robot",
								est.estimatedPose.toPose2d());
                         }
                        
                        this.estimatedRobotPose = est;
                        canSeeTag = true;

                        if (Constants.kDebugPhotonVision) {

			        	    for (PhotonTrackedTarget target : est.targetsUsed) {
					            allTagPoses.add(
						            aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
					            );
				            }

                            if(cameraEnum == CameraEnum.Camera1) {
                                Logger.recordOutput(
			 		            "PhotonVision/TargetsUsed1",
			 		                allTagPoses.toArray(new Pose3d[allTagPoses.size()])
				                );
                            } else {
                                Logger.recordOutput(
                                    "PhotonVision/TargetsUsed2",
                                    allTagPoses.toArray(new Pose3d[allTagPoses.size()])
                                );
                            }
				            
			            }

                    }
                );

            if(visionEst.isEmpty()) {
                canSeeTag = false;
                if (Constants.kDebugPhotonVision) {
                    if(cameraEnum == CameraEnum.Camera1) {
                        Logger.recordOutput(
                        "PhotonVision/TargetsUsed1",
                            new Pose3d[0]
                        );
                    } else {
                        Logger.recordOutput(
                            "PhotonVision/TargetsUsed2",
                            new Pose3d[0]
                        );
                    }
                }
            }
         }

     }
 
     /**
      * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
      * deviations based on number of tags, estimation strategy, and distance from the tags.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      * @param targets All targets in this camera frame
      */
     private void updateEstimationStdDevs(
             Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
         if (estimatedPose.isEmpty()) {
             // No pose input. Default to single-tag std devs
             curStdDevs = PhotonVisionConstants.kSingleTagStdDevs;
 
         } else {
             // Pose present. Start running Heuristic
             var estStdDevs = PhotonVisionConstants.kSingleTagStdDevs;
             int numTags = 0;
             double avgDist = 0;
 
             // Precalculation - see how many tags we found, and calculate an average-distance metric
             for (var tgt : targets) {
                 var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                 if (tagPose.isEmpty()) continue;
                 numTags++;
                 avgDist +=
                         tagPose
                                 .get()
                                 .toPose2d()
                                 .getTranslation()
                                 .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
             }
 
             if (numTags == 0) {
                 // No tags visible. Default to single-tag std devs
                 curStdDevs = PhotonVisionConstants.kSingleTagStdDevs;
             } else {
                 // One or more tags visible, run the full heuristic.
                 avgDist /= numTags;
                 // Decrease std devs if multiple targets are visible
                 if (numTags > 1) estStdDevs = PhotonVisionConstants.kMultiTagStdDevs;
                 // Increase std devs based on (average) distance
                 if (numTags == 1 && avgDist > 4)
                     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                 else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                 curStdDevs = estStdDevs;
             }
         }
     }
    
    
 
     /**
      * Returns the latest standard deviations of the estimated pose from {@link
      * #getEstimatedGlobalPose()}, for use with {@link
      * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
      * only be used when there are targets visible.
      */
     public Matrix<N3, N1> getEstimationStdDevs() {
         return curStdDevs;
     }
 
     // ----- Simulation
 
     public void simulationPeriodic(Pose2d robotSimPose) {
         visionSim.update(robotSimPose);
     }
 
     /** Reset pose history of the robot in the vision system simulation. */
     public void resetSimPose(Pose2d pose) {
         if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
     }
 
     /** A Field2d for visualizing our robot and objects on the field. */
     public Field2d getSimDebugField() {
         if (!Robot.isSimulation()) return null;
         return visionSim.getDebugField();
     }
 
     @FunctionalInterface
     public static interface EstimateConsumer {
         public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
     }

    public boolean isVisionEstAvailable() {
        return canSeeTag;
    }

    public EstimatedRobotPose getEstimatedRobotPose() {
        return estimatedRobotPose;
    }

    public boolean getCamera1Enabled() {
        return cam1Enabled;
    }

    public boolean getCamera2Enabled() {
        return cam2Enabled;
    }
 }