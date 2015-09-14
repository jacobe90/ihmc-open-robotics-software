package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.LinkedHashMap;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.tools.FormattingTools;
import us.ihmc.utilities.io.printing.PrintTools;

public class TaskspaceToJointspaceHandForcefeedbackControlState extends TrajectoryBasedTaskspaceHandControlState
{
	private final static boolean TRAJECTORY_FORCEFEEDBACK = true; //DO NOT SET TO TRUE! NOT ENTIRELY TESTED YET!! T.Meier If false: make sure trajectory time from behavior is set big enough
	private final static boolean SENSOR_NOISE = true;
	private boolean isDone = false;
	private final BooleanYoVariable forcefeedback;
	private final Boolean doPositionControlOnJoints;
	private enum JointControlMode
	{
		FORCE_CONTROL, POSITION_CONTROL
	};
	private JointControlMode jointControlMode;

	private final HumanoidReferenceFrames referenceFrames;
	private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
	private final ReferenceFrame handControlFrame;
	private PoseTrajectoryGenerator poseTrajectoryGenerator;

	private final FramePose desiredPose = new FramePose();

	private final FramePoint desiredPosition = new FramePoint(worldFrame);
	private final FrameVector desiredVelocity = new FrameVector(worldFrame);
	private final FrameVector desiredAcceleration = new FrameVector(worldFrame);

	private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
	private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
	private final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);

	private TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator;

	private final double dtControl;
	private final OneDoFJoint[] oneDoFJoints;
	private final boolean[] doIntegrateDesiredAccelerations;
	private final LinkedHashMap<OneDoFJoint, PIDController> jointPIDControllers;
	private final LinkedHashMap<OneDoFJoint, RateLimitedYoVariable> jointRateLimitedAccelerations;
	private final DoubleYoVariable jointMaxAcceleration;

	private final DoubleYoVariable currentTimeInState;

	// Force measurement
	private final String sensorPrefix;
	private ForceSensorDataReadOnly wristSensor;
	private Wrench wristSensorWrench = new Wrench();
	private final DoubleYoVariable fxRaw, fyRaw, fzRaw;
	private final AlphaFilteredYoVariable fxFiltered, fyFiltered, fzFiltered;
	private final double alpha;
	private DoubleYoVariable massHandandDrill;
	private static final double GRAVITY = 9.81;
	private static final double MASSDRILL = 1.442;
	private final BooleanYoVariable compensateRadialForce;
	private Vector3d circularTrajectoryOmega;
	private Vector3d circularTrajectoryRadius;
	private Point3d circularTrajectoryOrigin;
	private final IntegerYoVariable trajectoryNumber;

	//MBC
	private final DoubleYoVariable scaledTimeVariable;
	private final DoubleYoVariable timeParameterScaleFactor;
	private final static double MAX_TIME_SCALE = 0.5;
	private final DoubleYoVariable coeffC1, coeffC2;
	private final DoubleYoVariable currentTangentialForce;
	private final DoubleYoVariable currentTangentialForceModel;
	private final DoubleYoVariable currentTangentialVelocity;
	private final DoubleYoVariable mpcVelocity;
	private final DoubleYoVariable desiredTangentialForce;
	private final DoubleYoVariable pCutControl;

	//MPA
	private final DoubleYoVariable epsilon;
	private final DoubleYoVariable modelError;
	private final DoubleYoVariable referenceError;
	private final DoubleYoVariable w1;
	private final DoubleYoVariable w2;
	private final double v0_ramp = 0.001;
	private final double v1_ramp = 0.005;
	private final double w1_0 = 0.15;//old 0.15
	private final double w2_0 = 50.0;//old 5.0

	private final double referenceForce = -10.0;
	private final double pressForce = -5.0;

	private final FramePoint preScalingTrajectoryPosition = new FramePoint(worldFrame);
	private final FrameVector preScalingTrajectoryVelocity = new FrameVector(worldFrame);
	private final FrameVector preScalingTrajectoryAcceleration = new FrameVector(worldFrame);
	private double preScalingTrajectoryVelocityDouble;

	private Vector3d tempVector = new Vector3d();
	private Vector3d forceVectorInWorld = new Vector3d();
	private Vector3d normalVectorInWorld = new Vector3d();
	private Vector3d tangentTrajectoryVectorInWorld = new Vector3d();
	private Vector3d lastTangentTrajectoryVectorInWorld = new Vector3d();

	private final FramePoint currentHandPos = new FramePoint();
	private final FramePoint lastHandPos = new FramePoint(worldFrame);

	// Sensor noise for simulation
	private final long random = System.currentTimeMillis();
	private Random randomNumberGenerator;

	public TaskspaceToJointspaceHandForcefeedbackControlState(String namePrefix, HandControlState stateEnum, RobotSide robotSide,
			MomentumBasedController momentumBasedController, int jacobianId, RigidBody base, RigidBody endEffector,
			boolean doPositionControlOnJoints, YoPIDGains gains, YoVariableRegistry parentRegistry)
	{
		super(namePrefix, stateEnum, momentumBasedController, jacobianId, base, endEffector, parentRegistry);
		referenceFrames = new HumanoidReferenceFrames(momentumBasedController.getFullRobotModel());
		handControlFrame = referenceFrames.getHandFrame(robotSide);
		currentTimeInState = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "CurrentTimeInCutForceControlState",
				registry);
		dtControl = momentumBasedController.getControlDT();
		oneDoFJoints = ScrewTools.createOneDoFJointPath(base, endEffector);
		doIntegrateDesiredAccelerations = new boolean[oneDoFJoints.length];
		this.doPositionControlOnJoints = doPositionControlOnJoints;

		forcefeedback = new BooleanYoVariable(robotSide.getShortLowerCaseName() + "_forceFeedback", parentRegistry);
		forcefeedback.set(TRAJECTORY_FORCEFEEDBACK);

		// TODO: Add mass of drill.
		massHandandDrill = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_massHandAndDrill", parentRegistry);
		massHandandDrill.set(TotalMassCalculator.computeSubTreeMass(
				momentumBasedController.getFullRobotModel().getHand(robotSide))/*+ MASSDRILL*/);

		for (int i = 0; i < oneDoFJoints.length; i++)
		{
			doIntegrateDesiredAccelerations[i] = oneDoFJoints[i].getIntegrateDesiredAccelerations();
		}

		/**
		 * Position Control on real robot.
		 * ForceControl in simulation.
		 */
		if (this.doPositionControlOnJoints)
		{
			jointControlMode = JointControlMode.POSITION_CONTROL;
			jointPIDControllers = null;
			jointRateLimitedAccelerations = null;
			jointMaxAcceleration = null;
		}
		else
		{
			jointControlMode = JointControlMode.FORCE_CONTROL;
			jointPIDControllers = new LinkedHashMap<OneDoFJoint, PIDController>();
			jointRateLimitedAccelerations = new LinkedHashMap<OneDoFJoint, RateLimitedYoVariable>();
			jointMaxAcceleration = gains.getYoMaximumAcceleration();

			for (OneDoFJoint joint : oneDoFJoints)
			{
				String suffix = StringUtils.uncapitalize(joint.getName());
				PIDController pidController = new PIDController(gains.getYoKp(), gains.getYoKi(), gains.getYoKd(),
						gains.getYoMaxIntegralError(), suffix, registry);
				jointPIDControllers.put(joint, pidController);

				RateLimitedYoVariable rateLimitedAcceleration = new RateLimitedYoVariable(suffix + "Acceleration",
						registry, gains.getYoMaximumJerk(), dtControl);
				jointRateLimitedAccelerations.put(joint, rateLimitedAcceleration);
			}
		}

		PrintTools.warn(this, "Joint control Mode: " + jointControlMode, true);	

		wristSensor = momentumBasedController.getWristForceSensor(robotSide);
		sensorPrefix = robotSide.getShortLowerCaseName() + "_wristSensor";

		alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(5.0, dtControl);
		fxRaw = new DoubleYoVariable(sensorPrefix + "_Fx_raw", registry);
		fyRaw = new DoubleYoVariable(sensorPrefix + "_Fy_raw", registry);
		fzRaw = new DoubleYoVariable(sensorPrefix + "_Fz_raw", registry);
		fxFiltered = new AlphaFilteredYoVariable(sensorPrefix + "_Fx_filtered", registry, alpha, fxRaw);
		fyFiltered = new AlphaFilteredYoVariable(sensorPrefix + "_Fy_filtered", registry, alpha, fyRaw);
		fzFiltered = new AlphaFilteredYoVariable(sensorPrefix + "_Fz_filtered", registry, alpha, fzRaw);
		trajectoryNumber = new IntegerYoVariable(sensorPrefix + "_trajectory_number", registry);
		trajectoryNumber.set(0);

		lastHandPos.setToZero(worldFrame);
		currentTangentialForce = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_F_tangential", registry);
		currentTangentialForceModel = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_F_tangential_model", registry);
		currentTangentialVelocity = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_v_tangential", registry);
		scaledTimeVariable = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_scaled_time", registry);
		timeParameterScaleFactor = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_scale_factor", registry);
		mpcVelocity = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_v_MPC", registry);
		compensateRadialForce = new BooleanYoVariable(robotSide.getShortLowerCaseName() + "_compensate_RadialForce",
				parentRegistry);
		compensateRadialForce.set(false);

		desiredTangentialForce = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_F_tangential_desired", registry);
		pCutControl = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_P_Control", registry);
		pCutControl.set(0.05);
		desiredTangentialForce.set(referenceForce);

		coeffC1 = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_C1", registry);
		coeffC2 = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_C2", registry);
		epsilon = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_epsilon", registry);
		modelError = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_model_error", registry);
		referenceError = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_reference_error", registry);

		//TODO: find reasonable initial coeffs
		setModelCoeffs(100.0, 0.1);//90.0, 1.0
		epsilon.set(0.0);

		w1 = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_w1", registry);
		w2 = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_w2", registry);
		w1.set(w1_0);
		w2.set(w2_0);

		if (SENSOR_NOISE && TRAJECTORY_FORCEFEEDBACK)
		{
			randomNumberGenerator = new Random(random);
		}
		else
		{
			randomNumberGenerator = null;
		}
	}

	public void setModelCoeffs(double C1, double C2)
	{
		coeffC1.set(C1);
		coeffC2.set(C2);
	}

	private void resetFilterVariables(){
		fxFiltered.reset();
		fyFiltered.reset();
		fzFiltered.reset();
	}

	private void mpaMpcControl()
	{
		/**
		 * Pre-calculate velocity if scale factor was 1.0:
		 */
		poseTrajectoryGenerator.compute(scaledTimeVariable.getDoubleValue() + dtControl);
		poseTrajectoryGenerator.packLinearData(preScalingTrajectoryPosition, preScalingTrajectoryVelocity,
				preScalingTrajectoryAcceleration);
		preScalingTrajectoryVelocityDouble = preScalingTrajectoryVelocity.length();

		/**
		 *  Calculate current velocity in desired trajectory plane by finite differences:
		 */
		currentHandPos.setToZero(handControlFrame);
		currentHandPos.changeFrame(worldFrame);
		lastHandPos.changeFrame(worldFrame);
		tempVector.set(currentHandPos.getPoint());
		tempVector.sub(lastHandPos.getPoint());
		tempVector.setX(0.0);
		tangentTrajectoryVectorInWorld.set(tempVector);

		lastHandPos.setIncludingFrame(currentHandPos);

		double currentVelocity = (Math.abs(lastHandPos.getX()) > 0.0) ? tempVector.length() * 1.0 / dtControl : 0.0;
		currentTangentialVelocity.set(currentVelocity);


		wristSensor.packWrench(wristSensorWrench);
		CutForceControlHelper.wristSensorUpdate(wristSensorWrench, worldFrame, fxRaw, fyRaw, fzRaw,
				fxFiltered, fyFiltered, fzFiltered, massHandandDrill.getDoubleValue(), SENSOR_NOISE, randomNumberGenerator, currentTimeInState.getDoubleValue());

		CutForceControlHelper.getTangentForce(forceVectorInWorld, currentTangentialForce, tangentTrajectoryVectorInWorld,
				lastTangentTrajectoryVectorInWorld, fxFiltered.getDoubleValue(), fyFiltered.getDoubleValue(),
				fzFiltered.getDoubleValue(),false, currentTimeInState.getDoubleValue());

		mpcVelocity.set(0.0);

		/**
		 * Control:
		 */
		currentTangentialForceModel.set(CutForceControlHelper.exponentialForceModel(currentTangentialVelocity.getDoubleValue(),
				coeffC1.getDoubleValue(), coeffC2.getDoubleValue()));
		referenceError.set(-desiredTangentialForce.getDoubleValue() + currentTangentialForce.getDoubleValue());
		modelError.set(currentTangentialForce.getDoubleValue() + currentTangentialForceModel.getDoubleValue());

		if(!poseTrajectoryGenerator.isDone())
		{
			CutForceControlHelper.adaptW(currentTangentialVelocity.getDoubleValue(), v0_ramp, v1_ramp, w1_0, w2_0, w1, w2);
			CutForceControlHelper.modelParameterAdaption(currentTangentialForce.getDoubleValue(),
					currentTangentialForceModel.getDoubleValue(), currentTangentialVelocity.getDoubleValue(), epsilon,
					coeffC1, coeffC2, w1.getDoubleValue(), w2.getDoubleValue());
		}
		double mpcVelocitydouble = (poseTrajectoryGenerator.isDone()) ?
				0.0 : 1.0 / coeffC2.getDoubleValue() * Math.log(Math.abs(desiredTangentialForce.getDoubleValue()) /
						coeffC1.getDoubleValue() + 1.0);
		mpcVelocity.set(mpcVelocitydouble);

		double scaleFactor = (preScalingTrajectoryVelocityDouble == 0.0) ? 
				0.0 : mpcVelocity.getDoubleValue() / preScalingTrajectoryVelocityDouble;

		// P-term
		scaleFactor += Math.abs(referenceError.getDoubleValue()) < 0.5 ?
				referenceError.getDoubleValue() * pCutControl.getDoubleValue() : 0.0;
		scaleFactor = (scaleFactor > MAX_TIME_SCALE) ? MAX_TIME_SCALE : scaleFactor;
		timeParameterScaleFactor.set(scaleFactor);

		if (timeParameterScaleFactor.getDoubleValue() == MAX_TIME_SCALE)
		{
			PrintTools.warn(this, "Scale factor saturation.");
		}
		// DEBUG: SET TO 1 anyway
		//    timeParameterScaleFactor.set(1.0);

		/**
		 *  Evaluate the trajectory at new parameter value
		 */
		scaledTimeVariable.add(dtControl * timeParameterScaleFactor.getDoubleValue());
		poseTrajectoryGenerator.compute(scaledTimeVariable.getDoubleValue());
		poseTrajectoryGenerator.get(desiredPose);
		poseTrajectoryGenerator.packLinearData(desiredPosition, desiredVelocity, desiredAcceleration);

		// TODO:
		/**
		 * PID Control in direction of wall
		 */
		//				desiredPosition.checkReferenceFrameMatch(worldFrame);
		//				
		//				forceVectorInWorld.scale(1.0);
		//				forceVectorInWorld.dot(normalVectorInWorld);
		//				
		//				tempVector.set(-0.01*(forceVectorInWorld.length() - Math.abs(pressForce)), 0.0, 0.0);
		//				desiredPosition.add(tempVector);

		desiredVelocity.scale(timeParameterScaleFactor.getDoubleValue());
		desiredAcceleration.scale(timeParameterScaleFactor.getDoubleValue() * timeParameterScaleFactor.getDoubleValue());
		poseTrajectoryGenerator.packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
		desiredAngularVelocity.scale(timeParameterScaleFactor.getDoubleValue());
		desiredAngularAcceleration.scale(timeParameterScaleFactor.getDoubleValue() * timeParameterScaleFactor.getDoubleValue());

		ReferenceFrame controlFrame = taskspaceToJointspaceCalculator.getControlFrame();
		desiredVelocity.changeFrame(controlFrame);
		desiredAngularVelocity.changeFrame(controlFrame);

		taskspaceToJointspaceCalculator.compute(desiredPosition, desiredOrientation, desiredVelocity, desiredAngularVelocity);
		taskspaceToJointspaceCalculator.packDesiredJointAnglesIntoOneDoFJoints(oneDoFJoints);
		taskspaceToJointspaceCalculator.packDesiredJointVelocitiesIntoOneDoFJoints(oneDoFJoints);
		taskspaceToJointspaceCalculator.packDesiredJointAccelerationsIntoOneDoFJoints(oneDoFJoints);

	}

	private void trajectoryPositionControl()
	{
		scaledTimeVariable.set(currentTimeInState.getDoubleValue());

		wristSensor.packWrench(wristSensorWrench);
		CutForceControlHelper.wristSensorUpdate(wristSensorWrench, worldFrame, fxRaw, fyRaw, fzRaw, fxFiltered, fyFiltered,
				fzFiltered, massHandandDrill.getDoubleValue(), false, randomNumberGenerator, 0.0);
		CutForceControlHelper.getTangentForce(forceVectorInWorld, currentTangentialForce, tangentTrajectoryVectorInWorld,
				lastTangentTrajectoryVectorInWorld, fxFiltered.getDoubleValue(), fyFiltered.getDoubleValue(),
				fzFiltered.getDoubleValue(), false, currentTimeInState.getDoubleValue());

//		poseTrajectoryGenerator.compute(currentTimeInState.getDoubleValue());
//		poseTrajectoryGenerator.get(desiredPose);
//		poseTrajectoryGenerator.packLinearData(desiredPosition, desiredVelocity, desiredAcceleration);
//		poseTrajectoryGenerator.packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
//
//		ReferenceFrame controlFrame = taskspaceToJointspaceCalculator.getControlFrame();
//
//		desiredVelocity.changeFrame(controlFrame);
//		desiredAngularVelocity.changeFrame(controlFrame);
//
//		desiredPosition.changeFrame(controlFrame);
//		desiredAcceleration.changeFrame(controlFrame);
//		desiredOrientation.changeFrame(controlFrame);
//		desiredAngularAcceleration.changeFrame(controlFrame);
//
//		taskspaceToJointspaceCalculator.compute(desiredPosition, desiredOrientation, desiredVelocity, desiredAngularVelocity);
//		taskspaceToJointspaceCalculator.packDesiredJointAnglesIntoOneDoFJoints(oneDoFJoints);
//		taskspaceToJointspaceCalculator.packDesiredJointVelocitiesIntoOneDoFJoints(oneDoFJoints);
//		taskspaceToJointspaceCalculator.packDesiredJointAccelerationsIntoOneDoFJoints(oneDoFJoints);
	}


	@Override
	public boolean isDone() {

		//TODO: change
		return false;//isDone;
	};

	@Override
	public void setTrajectory(PoseTrajectoryGenerator poseTrajectoryGenerator)
	{
		this.poseTrajectoryGenerator = poseTrajectoryGenerator;
	}

	@Override
	public void setHoldPositionDuration(double holdPositionDuration)
	{
		// TODO Auto-generated method stub

	}

	@Override
	public FramePose getDesiredPose()
	{
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public ReferenceFrame getReferenceFrame()
	{
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void setControlModuleForForceControl(RigidBodySpatialAccelerationControlModule
			handRigidBodySpatialAccelerationControlModule)
	{
		// TODO Auto-generated method stub

	}

	@Override
	public void setControlModuleForPositionControl(TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator)
	{
		this.taskspaceToJointspaceCalculator = taskspaceToJointspaceCalculator;

	}

	public void initialize(Point3d rotationAxisOriginInWorld, Vector3d rotationAxisInWorld)
	{
		if (rotationAxisOriginInWorld != null)
		{
			normalVectorInWorld.set(rotationAxisInWorld);
			normalVectorInWorld.normalize();
			circularTrajectoryOrigin = new Point3d(rotationAxisOriginInWorld);
			circularTrajectoryOmega = new Vector3d(rotationAxisInWorld);
			circularTrajectoryRadius = new Vector3d();
		}
		else
		{
			normalVectorInWorld.set(rotationAxisInWorld);
			normalVectorInWorld.normalize();
			circularTrajectoryOrigin = null;
			circularTrajectoryOmega = null;
			circularTrajectoryRadius = null;
		}
	}

	@Override
	public void doTransitionIntoAction()
	{
		trajectoryNumber.add(1);
		resetFilterVariables();
		currentTimeInState.set(0.0);
		scaledTimeVariable.set(0.0);

		poseTrajectoryGenerator.showVisualization();
		poseTrajectoryGenerator.initialize();
		taskspaceToJointspaceCalculator.setSelectionMatrix(selectionMatrix);

		currentHandPos.setToZero(handControlFrame);
		lastHandPos.setToZero(handControlFrame);

		if (jointControlMode == JointControlMode.POSITION_CONTROL)
		{
			enableJointPositionControl();
		}
	}

	@Override
	public void doTransitionOutOfAction()
	{
		isDone = false;
		if (jointControlMode == JointControlMode.POSITION_CONTROL)
		{
			disableJointPositionControl();
		}
	}

	public void enableJointPositionControl()
	{
		for (OneDoFJoint joint : oneDoFJoints)
		{
			joint.setIntegrateDesiredAccelerations(false);
			joint.setUnderPositionControl(true);
		}
	}

	public void disableJointPositionControl()
	{
		for (int i = 0; i < oneDoFJoints.length; i++)
		{
			OneDoFJoint joint = oneDoFJoints[i];
			joint.setIntegrateDesiredAccelerations(doIntegrateDesiredAccelerations[i]);
			joint.setUnderPositionControl(false);
		}
	}

	public boolean isForceControlled(){
		return forcefeedback.getBooleanValue();
	}

	public double getReferenceError(){
		return referenceError.getDoubleValue();
	}

	@Override
	public void doAction()
	{
		currentTimeInState.set(getTimeInCurrentState());

		if (TRAJECTORY_FORCEFEEDBACK)
		{
			mpaMpcControl();
		}
		else
		{
			trajectoryPositionControl();
		}

		switch (jointControlMode)
		{
		case FORCE_CONTROL :
			for (int i = 0; i < oneDoFJoints.length; i++)
			{
				OneDoFJoint joint = oneDoFJoints[i];
				double q = joint.getQ();
				double qd = joint.getQd();
				double qDes = joint.getqDesired();
				double qdDes = joint.getQdDesired();

				RateLimitedYoVariable rateLimitedAcceleration = jointRateLimitedAccelerations.get(joint);

				PIDController pidController = jointPIDControllers.get(joint);
				double desiredAcceleration = pidController.computeForAngles(q, qDes, qd, qdDes, dtControl);

				desiredAcceleration = MathTools.clipToMinMax(desiredAcceleration, jointMaxAcceleration.getDoubleValue());
				rateLimitedAcceleration.update(desiredAcceleration);
				desiredAcceleration = rateLimitedAcceleration.getDoubleValue();

				momentumBasedController.setOneDoFJointAcceleration(joint, desiredAcceleration);
			}
			break;

		case POSITION_CONTROL :
			for (OneDoFJoint joint : oneDoFJoints)
			{
				// set desired qdd to zero.
				momentumBasedController.setOneDoFJointAcceleration(joint, 0.0);
			}
			break;

		default :
			PrintTools.error(this, "No joint control mode set.");
		}
		isDone = poseTrajectoryGenerator.isDone();
	}
}