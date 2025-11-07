package frc.robot.tools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.*;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.PhotonVisionConstants;
import edu.wpi.first.wpilibj.RobotBase;

//import edu.wpi.first.cscore.VideoMode.PixelFormat;

public class PhotonVision {
	private PhotonCamera _camera;
	private PhotonCamera _camera2;
	
	// simulation variables
	private VisionSystemSim _visionSystemSim = null;
	private Pose3d _sim_farTargetPose = null;
	private double _sim_targetWidth = 0.0;
	private double _sim_targetHeight = 0.0;
	private AprilTagFieldLayout _aprilTagFieldLayout = null;
	private PhotonPoseEstimator _photonPoseEstimator = null;
	private PhotonPoseEstimator _photonPoseEstimator2 = null;
	private ShuffleboardTab photonVisionTab = null;
	private EstimatedRobotPose _estimatedRobotPose1 = null;
	private EstimatedRobotPose _estimatedRobotPose2 = null;
	//private int[] _targetsUsed = new int[0];
	//private int _speakerTarget = 0;
	//PhotonPipelineResult result = null;
	//List<PhotonTrackedTarget> targets;
	private Pose2d prevEstimatedRobotPose = null;
	//private Pose2d prevPhotonEstimatedPose = null;
	List<Pose3d> allTagPoses = new ArrayList<>();
	List<Pose3d> allTagPoses2 = new ArrayList<>();
	private Optional<EstimatedRobotPose> estimatedRobotPose1 = null;
	private Optional<EstimatedRobotPose> estimatedRobotPose2 = null;

	//private double[] poseArray = new double[3];

	//private Pose3d camPose = new Pose3d();
	//private Pose3d _lastPhotonPoseEstimatorPose = new Pose3d();
	//private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

	private boolean isSim = false;
	//private Optional<EstimatedRobotPose> o = null;
	
	public PhotonVision() {

		// Is PhotonVision Enabled?
		if (Constants.kEnablePhotonVision) {

			// Is this a simulation?
			if (RobotBase.isReal()) {
				isSim = false;
			} else {
				isSim = true;
			}

			if(Constants.PhotonVisionConstants.kEnableCamera1) {
				_camera = new PhotonCamera(PhotonVisionConstants.CameraName);
			}
			
			if(Constants.PhotonVisionConstants.kEnableCamera2) {
				_camera2 = new PhotonCamera(PhotonVisionConstants.Camera2Name);
			}

			try {
				// This sets up the field with the correct Apriltags in the proper places
				// These values are used later for aiming and localization
				_aprilTagFieldLayout = AprilTagFieldLayout
						.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark
						.m_resourceFile);

				// Set if we are blue or red
				// Set the origin accordingly
				if (DriverStation.getAlliance().isPresent()) {

					if (DriverStation.getAlliance().get() == Alliance.Blue) {
						_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
					} else {
						_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
					}
				} else {
					_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
				}

			} catch (IOException e) {
				System.out.println("PhotonVision::PhotonVision() - error:" + e.toString());
				return;
			}

			// If we are in a simulator, set it
			if(isSim) {
				System.out.println("running setupSimulation()");
				setupSimulation(new Pose3d());
			}

			// If the field layout is created and valid and the camera is created and connected
			// create the photon pose estimator.  It is where photonvision thinks the robot is.
			if (_aprilTagFieldLayout != null) {
				createPhotonPoseEstimators();
			}

			// Create the elements in Shuffleboard for debugging if debugPhotonVision is true 
			if (Constants.PhotonVisionConstants.kEnableCamera1 
				&& _camera.isConnected() 
				&& Constants.kDebugPhotonVision == true) {

				photonVisionTab = Shuffleboard.getTab("PhotonVision");

				//photonVisionTab.addDouble("Target Distance", this::getTargetDistance);
				photonVisionTab.addBoolean("Connection", this::isConnected);
				//photonVisionTab.addBoolean("Has Target", this::hasTarget);
				//photonVisionTab.addString("Targets Used", this::targetsUsed);
			}

			if (Constants.PhotonVisionConstants.kEnableCamera2 
				&& _camera2.isConnected() 
				&& Constants.kDebugPhotonVision == true) {

				photonVisionTab = Shuffleboard.getTab("PhotonVision");
				photonVisionTab.addBoolean("Cam2", this::isConnected2);
			
			}

			// Start a new thread that runs faster ~10 milliseconds instead of ~20 milliseconds
			// Thread thread = new Thread() {
			// 	public void run() {

			// 		while (true) {

			// 			if (prevEstimatedRobotPose == null) {
			// 				System.out.println("PhonVision::getPose() - prevEstimatedRobotPose is null");
			// 				prevEstimatedRobotPose = new Pose2d();
			// 			}

			// 			if (isSim) {
			// 				// Update PhotonVision based on our new robot position.
			// 				_visionSystemSim.update(prevEstimatedRobotPose);

			// 				//vision.getSimDebugField()
							
			// 			}

			// 			try {
			// 				result = _camera.getLatestResult();
			// 				targets = result.getTargets();
			// 				_targetsUsed = new int[targets.size()];

			// 				// List<Pose3d> allTagPoses = new ArrayList<>();
			// 				allTagPoses.clear();

			// 				int i = 0;
			// 				for (PhotonTrackedTarget target : targets) {
			// 					allTagPoses.add(
			// 							_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get());

			// 					_targetsUsed[i] = target.getFiducialId();
			// 					i++;
			// 				}

			// 				///////////////

			// 				if(allTagPoses != null) {

			// 					Logger.recordOutput(
			// 						"AprilTagVision/TargetsUsed",
			// 						allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
			// 				}

			// 				/////////

			// 				Optional<EstimatedRobotPose> o = getPhotonPose(prevEstimatedRobotPose);

			// 				if (o.isPresent()) {

			// 					_estimatedRobotPose = o.get();

			// 					for (PhotonTrackedTarget target : _estimatedRobotPose.targetsUsed) {
			// 						allTagPoses.add(
			// 								_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get());
			// 					}

			// 					if(Constants.debugPhotonVision) {
			// 						RobotContainer.field.getObject("PhotonEstimatedRobot").setPose(_estimatedRobotPose.estimatedPose.toPose2d());
			// 					}

			// 					/*poseArray[0] = _estimatedRobotPose.estimatedPose.getX();
			// 					poseArray[1] = _estimatedRobotPose.estimatedPose.getY();
			// 					poseArray[2] = _estimatedRobotPose.estimatedPose.getRotation().getAngle();*/
						
			// 					/*Logger.recordOutput(
			// 						"PhotonVisionEstimator/position",
			// 						poseArray
			// 					);*/

			// 					//networkTableInstance.getEntry("/photonvision/PhotonVisionEstimator/position").setDoubleArray(poseArray);

			// 				} else {
			// 					// System.out.println("PhonVision::getPose() - I don't see any tags");
			// 					// Since we do not have any tags that we can see, blank out the list
			// 					// System.out.println("we do not have a pose");
			// 					Logger.recordOutput(
			// 							"AprilTagVision/TagPoses",
			// 							allTagPoses.toArray(new Pose3d[allTagPoses.size()]));

			// 					_estimatedRobotPose = null;
			// 				}

			// 				/*Logger.recordOutput(
			// 					"PhotonVisionEstimator/Robot",
			// 					_lastPhotonPoseEstimatorPose
			// 				);*/

			// 				/*poseArray[0] = _lastPhotonPoseEstimatorPose.getX();
			// 				poseArray[1] = _lastPhotonPoseEstimatorPose.getY();
			// 				poseArray[2] = _lastPhotonPoseEstimatorPose.getRotation().getAngle();*/
						
			// 				/*Logger.recordOutput(
			// 					"PhotonVisionEstimator/position",
			// 					poseArray
			// 				);*/
			// 				//networkTableInstance.getEntry("/photonvision/PhotonVisionEstimator/position").setDoubleArray(poseArray);

			// 				///////////////

			// 				Thread.sleep(5);

			// 			} catch (Exception e) {
			// 				e.printStackTrace();
			// 				System.out.println(e);
			// 			}
			// 		}
			// 	}
			// };

			// thread.start();
		}
	}

	// Is the camera1 connected?
	public boolean isConnected() {
		if(Constants.kEnablePhotonVision && Constants.PhotonVisionConstants.kEnableCamera1) {

			if(_camera != null) {
				return _camera.isConnected();
			} else {
				return false;
			}
		} else {
			return false;
		}
	}

	// Is the camera2 connected?
	public boolean isConnected2() {
		if(Constants.kEnablePhotonVision && Constants.PhotonVisionConstants.kEnableCamera2) {

			if(_camera2 != null) {
				return _camera2.isConnected();
			} else {
				return false;
			}
		} else {
			return false;
		}
	}


	// Get the pose/location on the field that photonvision thinks the robot is at
	public EstimatedRobotPose getPose1(Pose2d prevEstimatedRobotPose) {

		this.prevEstimatedRobotPose = prevEstimatedRobotPose;

		// Get the estimated robot pose
		estimatedRobotPose1 = getPhotonPose1(prevEstimatedRobotPose);

		if (isSim) {
			// Update PhotonVision based on our new robot position.
			_visionSystemSim.update(prevEstimatedRobotPose);
		}

		allTagPoses.clear();

		if (estimatedRobotPose1.isPresent()) {

			_estimatedRobotPose1 = estimatedRobotPose1.get();

			if (Constants.kDebugPhotonVision) {
				for (PhotonTrackedTarget target : _estimatedRobotPose1.targetsUsed) {
					allTagPoses.add(
						_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
					);
				}

				Logger.recordOutput(
			 		"PhotonVision/TargetsUsed1",
			 		allTagPoses.toArray(new Pose3d[allTagPoses.size()])
				);
			}

			//if (Constants.debugPhotonVision) {
			//	RobotContainer.field.getObject("PhotonEstimatedRobot")
			//		.setPose(_estimatedRobotPose.estimatedPose.toPose2d());
			//}

			//Logger.recordOutput(
			// 	"AprilTagVision/TargetsUsed",
			// 	allTagPoses.toArray(new Pose3d[allTagPoses.size()]));

			/*if(Constants.enableLogger) {

				Logger.recordOutput(
			 		"AprilTagVision/TargetsUsed",
			 		allTagPoses.toArray(new Pose3d[allTagPoses.size()])
				);
			}*/
			

		} else {
			// System.out.println("PhonVision::getPose() - I don't see any tags");
			// Since we do not have any tags that we can see, blank out the list
			// System.out.println("we do not have a pose");

			/*if(Constants.enableLogger) {
				Logger.recordOutput(
					"AprilTagVision/TagPoses",
					allTagPoses.toArray(new Pose3d[allTagPoses.size()]));

				_estimatedRobotPose = null;
			}*/

			/*if (Constants.kDebugPhotonVision) {
				Logger.recordOutput(
					"PhotonVision/TagPoses",
					allTagPoses.toArray(new Pose3d[allTagPoses.size()]));

				_estimatedRobotPose = null;
			}*/

			if (Constants.kDebugPhotonVision) {
				Logger.recordOutput(
					"PhotonVision/TargetsUsed1",
					allTagPoses.toArray(new Pose3d[allTagPoses.size()]));

				_estimatedRobotPose1 = null;
			}
		}

		return _estimatedRobotPose1;
	}

	public EstimatedRobotPose getPose2(Pose2d prevEstimatedRobotPose) {

		this.prevEstimatedRobotPose = prevEstimatedRobotPose;

		// Get the estimated robot pose
		estimatedRobotPose2 = getPhotonPose2(prevEstimatedRobotPose);

		if (isSim) {
			// Update PhotonVision based on our new robot position.
			_visionSystemSim.update(prevEstimatedRobotPose);
		}

		// Clear out the tags and let it be filled in if necessary
		allTagPoses2.clear();

		if (estimatedRobotPose2.isPresent()) {

			_estimatedRobotPose2 = estimatedRobotPose2.get();

			if (Constants.kDebugPhotonVision) {
				for (PhotonTrackedTarget target : _estimatedRobotPose2.targetsUsed) {
					allTagPoses2.add(
						_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
					);
				}

				Logger.recordOutput(
			 		"PhotonVision/TargetsUsed2",
			 		allTagPoses2.toArray(new Pose3d[allTagPoses2.size()])
				);
			}

		} else {

			if (Constants.kDebugPhotonVision) {
				Logger.recordOutput(
					"PhotonVision/TargetsUsed2",
					allTagPoses2.toArray(new Pose3d[allTagPoses2.size()]));

				_estimatedRobotPose2 = null;
			}
		}

		return _estimatedRobotPose2;
	}

	// Set the reference 2D (X,Y) pose/position for PhotonVision to use
	public void setReferencePose2d(Pose2d referencePose) {
		
		// If camera1 is enabled and the estimator is not null
		if(Constants.PhotonVisionConstants.kEnableCamera1 && _photonPoseEstimator != null) {
			_photonPoseEstimator.setReferencePose(referencePose);
		}

		// If camera2 is enabled and the estimator is not null
		if(Constants.PhotonVisionConstants.kEnableCamera1 && _photonPoseEstimator2 != null) {
			_photonPoseEstimator2.setReferencePose(referencePose);
		}
	}

	// Set the reference 3D (X,Y,Z) pose/position for PhotonVision to use
	public void setReferencePose3d(Pose3d referencePose) {

		if(Constants.PhotonVisionConstants.kEnableCamera1 && _photonPoseEstimator != null) {
			_photonPoseEstimator.setReferencePose(referencePose);
		}

		if(Constants.PhotonVisionConstants.kEnableCamera2 && _photonPoseEstimator2 != null) {
			_photonPoseEstimator2.setReferencePose(referencePose);
		}
	}

	// Does Photonvision have a target?
	/*public boolean hasTarget() {		

		if(_estimatedRobotPose != null && !_estimatedRobotPose.targetsUsed.isEmpty()) {
			return true;
		}

		return false;
	}*/

	// How far is the specified target
	/*public double targetDistance(int targetNumber) {
		if(_estimatedRobotPose != null) {
			return PhotonUtils.getDistanceToPose(
				_estimatedRobotPose.estimatedPose.toPose2d(),
				_aprilTagFieldLayout.getTagPose(targetNumber).get().toPose2d()
			);
		}

		return -1;
	}*/

	/*public double getTargetDistance() {
		double distance = 0.0;

		if(_estimatedRobotPose != null) {
			for(PhotonTrackedTarget target : _estimatedRobotPose.targetsUsed) {
				if(_speakerTarget == 0) {
					if(target.getFiducialId() == 7) {
						if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
							distance = targetDistance(7);
							_speakerTarget = 7;
							Logger.recordOutput("AprilTagVision/Target", distance);
							return distance;
						}
					} else if(target.getFiducialId() == 4) {
						if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
							distance = targetDistance(4);
							_speakerTarget = 4;
							Logger.recordOutput("AprilTagVision/Target", distance);
							return distance;
						}
					}
				} else {
					// we have already set the speaker target so just set the distance
					distance = targetDistance(_speakerTarget);
					Logger.recordOutput("AprilTagVision/Target", distance);
					return distance;
				}
			}
		} else {
			//System.out.println("getTargetDistance() - estimatedRobotPose is null");
		}
		
		return distance;
	}*/

	// Aim at the specified target
	public double aimAtTarget(int targetNumber) {

		PhotonPipelineResult result = _camera.getLatestResult();
		List<PhotonTrackedTarget> targets = result.getTargets();

		for(PhotonTrackedTarget target: targets) {
			if(target.getFiducialId() == targetNumber) {
				return target.getYaw();
			}
		}

		return 0.0;
	}

	// Can photon vision see the specified target?
	public boolean canSeeTarget(int targetNumber) {

		//PhotonTrackedTarget targetToAimAt = null;
		PhotonPipelineResult result = _camera.getLatestResult();
		PhotonPipelineResult result2 = _camera2.getLatestResult();
		if(result.hasTargets() == false || result2.hasTargets() == false) {
			//System.out.println("Don't see any targets");
			return false;
		}

		// if we get here, we can see some targets, just might not be the correct one

		List<PhotonTrackedTarget> targets = result.getTargets();
		List<PhotonTrackedTarget> targets2 = result.getTargets();

		for(PhotonTrackedTarget target: targets) {
			if(target.getFiducialId() == targetNumber) {
				//System.out.println("target: " + target.getFiducialId());
				return true;
			}
		}
		for(PhotonTrackedTarget target: targets2) {
			if(target.getFiducialId() == targetNumber) {
				//System.out.println("target: " + target.getFiducialId());
				return true;
			}
		}

		return false;
	}

	// Need to look this over
	// This is a private method used only internally
	private Optional<EstimatedRobotPose> getPhotonPose1(Pose2d prevEstimatedRobotPose) {

		try {

			if (Constants.PhotonVisionConstants.kEnableCamera1 && _photonPoseEstimator != null) {

				// Check if we are in simulation and the previousEstimatedRobotPose is not null
				// and we are not connected to the camera
				// Change this for testing
				if (prevEstimatedRobotPose != null) {
					_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
				} else {
					System.out.println("PhotonVision::getPhotonPose1() - prevEstimatedRobotPose is null");
					prevEstimatedRobotPose = new Pose2d();
				}

				estimatedRobotPose1 = _photonPoseEstimator.update(_camera.getLatestResult());
				

				/*allTagPoses.clear();

				if (estimatedRobotPose.isPresent()) {
					_lastPhotonPoseEstimatorPose = estimatedRobotPose.get().estimatedPose;

					try {

						for (PhotonTrackedTarget target : estimatedRobotPose.get().targetsUsed) {
							if (target.getFiducialId() >= 1 && target.getFiducialId() <= 8) {
								allTagPoses.add(_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get());
							}
						}

						if(Constants.enableLogger) {

							// use _lastPhotonPoseEstimatorPose since we assigned it earlier
							Logger.recordOutput(
								"PhotonVisionEstimator/Robot",
								_lastPhotonPoseEstimatorPose);
						}

						

					} catch (Exception e) {
						System.out.println(e.toString());
					}
				} else {

					if(Constants.enableLogger) {
						Logger.recordOutput(
							"PhotonVisionEstimator/Robot",
							_lastPhotonPoseEstimatorPose);
					}

					return Optional.empty();
				}*/

				return estimatedRobotPose1;
			} else {
				createPhotonPoseEstimators();
			}

			return Optional.empty();
		} catch (Exception e) {
			return Optional.empty();
		}
	}

	private Optional<EstimatedRobotPose> getPhotonPose2(Pose2d prevEstimatedRobotPose) {

		try {

			if (Constants.PhotonVisionConstants.kEnableCamera2 && _photonPoseEstimator2 != null) {

				// Check if we are in simulation and the previousEstimatedRobotPose is not null
				// and we are not connected to the camera
				// Change this for testing
				if (prevEstimatedRobotPose != null) {
					_photonPoseEstimator2.setReferencePose(prevEstimatedRobotPose);
				} else {
					System.out.println("PhotonVision::getPhotonPose2() - prevEstimatedRobotPose is null");
					prevEstimatedRobotPose = new Pose2d();
				}

				estimatedRobotPose2 = _photonPoseEstimator2.update(_camera2.getLatestResult());				

				return estimatedRobotPose2;
			} else {
				createPhotonPoseEstimators();
			}

			return Optional.empty();
		} catch (Exception e) {
			return Optional.empty();
		}
	}

	// setup the tags and set the origin to how to show the tags
	public void setupAprilTagFieldLayoutSim() {

		_visionSystemSim.clearVisionTargets();

		if(DriverStation.getAlliance().isPresent()) {
			if(DriverStation.getAlliance().get() == Alliance.Blue) {
				_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
			} else {
				_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
			}
		} else {
			_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
		}

		if(_aprilTagFieldLayout != null) {
			_visionSystemSim.addAprilTags(_aprilTagFieldLayout);
		}
	}

	private void setupSimulation(Pose3d aprilTagFieldLayoutOrigin) {

		_visionSystemSim = new VisionSystemSim("main");

		if(Constants.PhotonVisionConstants.kEnableCamera1) {
			PhotonCameraSim cameraSim1 = new PhotonCameraSim(_camera);
			cameraSim1.enableDrawWireframe(true);
			_visionSystemSim.addCamera(cameraSim1, Constants.PhotonVisionConstants.cameraToRobot);
		}

		if(Constants.PhotonVisionConstants.kEnableCamera2) {
			PhotonCameraSim cameraSim2 = new PhotonCameraSim(_camera2);
			cameraSim2.enableDrawWireframe(true);
			_visionSystemSim.addCamera(cameraSim2, Constants.PhotonVisionConstants.camera2ToRobot);
		}
			
		
		// See
    	// https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    	// page 208
    	_sim_targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
    	// See
    	// https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    	// page 197
    	_sim_targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
    	// See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    	// pages 4 and 5
    	//double tgtXPos = Units.feetToMeters(54);
		double tgtXPos = 5;
    	//double tgtYPos =
            //Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
		double tgtYPos = 5;
    	_sim_farTargetPose =
            new Pose3d(
                    new Translation3d(tgtXPos, tgtYPos, 0.5),
                    new Rotation3d(0.0, 0.0, 0.0));		
		
		setupAprilTagFieldLayoutSim();
	}

	public void createPhotonPoseEstimators() {
		if (_camera != null) {
			if (_camera.isConnected()) {
				_photonPoseEstimator = new PhotonPoseEstimator(_aprilTagFieldLayout, 
					Constants.PhotonVisionConstants.poseStrategy, 
					Constants.PhotonVisionConstants.cameraToRobot
				);
			} else {
				System.out.println("PhotonVision::PhotonVision() - camera1 is not connected");
			}
		}

		if (_camera2 != null) {
			if (_camera2.isConnected()) {
				_photonPoseEstimator2 = new PhotonPoseEstimator(_aprilTagFieldLayout, 
					Constants.PhotonVisionConstants.poseStrategy, 
					Constants.PhotonVisionConstants.camera2ToRobot
				);
			} else {
				System.out.println("PhotonVision::PhotonVision() - camera2 is not connected");
			}
		}
	}

	/*public String targetsUsed() {

		if(_targetsUsed.length == 0) {
			return "";
		}

		String targets = "";

		for(int target : _targetsUsed) {
			targets += " " + target;
		}

		return targets;
	}*/

	/*public double getTargetUsed() {
		if(this._targetsUsed.length > 0) {
			return this._targetsUsed[0];
		}

		return 0;
	}*/

	/*public double getSpeakerTarget() {
		return _speakerTarget;
	}*/
}
