package us.ihmc.simpleWholeBodyWalking.states;

<<<<<<< HEAD
<<<<<<< HEAD
=======
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
<<<<<<< HEAD
<<<<<<< HEAD
=======
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.TouchdownErrorCompensator;
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
import us.ihmc.simpleWholeBodyWalking.*;
=======
import us.ihmc.simpleWholeBodyWalking.SimpleCenterOfMassHeightManager;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
import us.ihmc.simpleWholeBodyWalking.SimpleFeetManager;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
import us.ihmc.simpleWholeBodyWalking.SimpleBalanceManager;
import us.ihmc.simpleWholeBodyWalking.SimpleCenterOfMassHeightManager;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
import us.ihmc.simpleWholeBodyWalking.SimpleFeetManager;
>>>>>>> 2fb58d4d161... did the simple balance manager
=======
import us.ihmc.simpleWholeBodyWalking.*;
>>>>>>> 5942e55c22c... got simple pelvis orietnation manager
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleSingleSupportState extends SimpleWalkingState
{
   protected final RobotSide swingSide;
   protected final RobotSide supportSide;

   private final YoBoolean hasMinimumTimePassed = new YoBoolean("hasMinimumTimePassed", registry);
   protected final YoDouble minimumSwingFraction = new YoDouble("minimumSwingFraction", registry);

   protected final WalkingMessageHandler walkingMessageHandler;
   protected final SideDependentList<FootSwitchInterface> footSwitches;
   protected final FullHumanoidRobotModel fullRobotModel;

<<<<<<< HEAD
<<<<<<< HEAD
   protected final SimpleBalanceManager balanceManager;
=======
   protected final BalanceManager balanceManager;
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
   protected final SimpleBalanceManager balanceManager;
>>>>>>> 2fb58d4d161... did the simple balance manager
   private final SimpleCenterOfMassHeightManager comHeightManager;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Footstep nextFootstep = new Footstep();
   private final FootstepTiming footstepTiming = new FootstepTiming();
   private final FootstepShiftFractions footstepShiftFraction = new FootstepShiftFractions();
   private double swingTime;

   private static final int additionalFootstepsToConsider = 2;
   private final Footstep[] footsteps = Footstep.createFootsteps(additionalFootstepsToConsider);
   private final FootstepTiming[] footstepTimings = FootstepTiming.createTimings(additionalFootstepsToConsider);
   private final FootstepShiftFractions[] footstepShiftFractions = FootstepShiftFractions.createShiftFractions(additionalFootstepsToConsider);

   private final FramePose3D actualFootPoseInWorld = new FramePose3D(worldFrame);
   private final FramePose3D desiredFootPoseInWorld = new FramePose3D(worldFrame);

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

<<<<<<< HEAD
<<<<<<< HEAD
   private final SimplePelvisOrientationManager pelvisOrientationManager;
=======
   private final PelvisOrientationManager pelvisOrientationManager;
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
   private final SimplePelvisOrientationManager pelvisOrientationManager;
>>>>>>> 5942e55c22c... got simple pelvis orietnation manager
   private final SimpleFeetManager feetManager;

   private final FramePoint3D desiredCoM = new FramePoint3D();

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
   private final YoDouble remainingSwingTimeAccordingToPlan = new YoDouble("remainingSwingTimeAccordingToPlan", registry);
   private final YoDouble estimatedRemainingSwingTimeUnderDisturbance = new YoDouble("estimatedRemainingSwingTimeUnderDisturbance", registry);
   private final YoDouble icpErrorThresholdToSpeedUpSwing = new YoDouble("icpErrorThresholdToSpeedUpSwing", registry);

<<<<<<< HEAD
<<<<<<< HEAD
=======
   private final BooleanProvider minimizeAngularMomentumRateZDuringSwing;

>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
   private final FrameQuaternion tempOrientation = new FrameQuaternion();

   public SimpleSingleSupportState(SimpleWalkingStateEnum stateEnum,
                                   WalkingMessageHandler walkingMessageHandler,
                                   HighLevelHumanoidControllerToolbox controllerToolbox,
                                   SimpleControlManagerFactory managerFactory,
                                   WalkingControllerParameters walkingControllerParameters,
                                   WalkingFailureDetectionControlModule failureDetectionControlModule,
                                   YoVariableRegistry parentRegistry)
   {
      super(stateEnum, parentRegistry);

      this.supportSide = stateEnum.getSupportSide();
      swingSide = supportSide.getOppositeSide();

      minimumSwingFraction.set(0.5);

      this.walkingMessageHandler = walkingMessageHandler;
      footSwitches = controllerToolbox.getFootSwitches();
      fullRobotModel = controllerToolbox.getFullRobotModel();

      balanceManager = managerFactory.getOrCreateBalanceManager();
      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();

      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;

      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();

      icpErrorThresholdToSpeedUpSwing.set(walkingControllerParameters.getICPErrorThresholdToSpeedUpSwing());
<<<<<<< HEAD
<<<<<<< HEAD
=======
      minimizeAngularMomentumRateZDuringSwing = new BooleanParameter("minimizeAngularMomentumRateZDuringSwing",
                                                                     registry,
                                                                     walkingControllerParameters.minimizeAngularMomentumRateZDuringSwing());
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager

      setYoVariablesToNaN();
   }

<<<<<<< HEAD
<<<<<<< HEAD
=======
   public RobotSide getSwingSide()
   {
      return swingSide;
   }

>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
   @Override
   public RobotSide getSupportSide()
   {
      return supportSide;
   }

   @Override
   public void doAction(double timeInState)
   {

      boolean requestSwingSpeedUp = balanceManager.getICPErrorMagnitude() > icpErrorThresholdToSpeedUpSwing.getDoubleValue();

      boolean footstepIsBeingAdjusted = balanceManager.checkAndUpdateFootstepFromICPOptimization(nextFootstep);

      if (footstepIsBeingAdjusted)
      {
         requestSwingSpeedUp = true;
         walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
         failureDetectionControlModule.setNextFootstep(nextFootstep);
         updateFootstepParameters();

         feetManager.adjustSwingTrajectory(swingSide, nextFootstep, swingTime);

         balanceManager.updateCurrentICPPlan();
         //legConfigurationManager.prepareForLegBracing(swingSide);

         updateHeightManager();
      }

      // if the footstep was adjusted, shift the CoM plan, if there is one.
      walkingMessageHandler.setPlanOffsetFromAdjustment(balanceManager.getEffectiveICPAdjustment());

      if (requestSwingSpeedUp)
      {
         double swingTimeRemaining = requestSwingSpeedUpIfNeeded();
         balanceManager.updateSwingTimeRemaining(swingTimeRemaining);
      }

      walkingMessageHandler.clearFootTrajectory();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      hasMinimumTimePassed.set(hasMinimumTimePassed(timeInState));

      if (hasMinimumTimePassed.getBooleanValue() && footSwitches.get(swingSide).hasFootHitGround())
         return true;

      return balanceManager.isICPPlanDone();
   }

   @Override
   public void onEntry()
   {
      balanceManager.clearICPPlan();
      footSwitches.get(swingSide).reset();

      comHeightManager.setSupportLeg(swingSide.getOppositeSide());

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();

      double finalTransferSplitFraction = walkingMessageHandler.getFinalTransferSplitFraction();
      double finalTransferWeightDistribution = walkingMessageHandler.getFinalTransferWeightDistribution();

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
      swingTime = walkingMessageHandler.getNextSwingTime();
      walkingMessageHandler.poll(nextFootstep, footstepTiming, footstepShiftFraction);

      /** 1/08/2018 RJG this has to be done before calling #updateFootstepParameters() to make sure the contact points are up to date */
      feetManager.setContactStateForSwing(swingSide);

      updateFootstepParameters();

<<<<<<< HEAD
<<<<<<< HEAD
=======
      balanceManager.minimizeAngularMomentumRateZ(minimizeAngularMomentumRateZDuringSwing.getValue());
      balanceManager.setNextFootstep(nextFootstep);
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.setFinalTransferSplitFraction(finalTransferSplitFraction);
      balanceManager.setFinalTransferWeightDistribution(finalTransferWeightDistribution);
      balanceManager.addFootstepToPlan(nextFootstep, footstepTiming, footstepShiftFraction);

      int stepsToAdd = Math.min(additionalFootstepsToConsider, walkingMessageHandler.getCurrentNumberOfFootsteps());
      boolean isLastStep = stepsToAdd == 0;
      for (int i = 0; i < stepsToAdd; i++)
      {
         walkingMessageHandler.peekFootstep(i, footsteps[i]);
         walkingMessageHandler.peekTiming(i, footstepTimings[i]);
         walkingMessageHandler.peekShiftFraction(i, footstepShiftFractions[i]);
         balanceManager.addFootstepToPlan(footsteps[i], footstepTimings[i], footstepShiftFractions[i]);
      }

      balanceManager.setICPPlanSupportSide(supportSide);
<<<<<<< HEAD
<<<<<<< HEAD
      balanceManager.initializeICPPlanForSingleSupport(finalTransferTime);
=======
      balanceManager.initializeICPPlanForSingleSupport(footstepTiming.getSwingTime(), footstepTiming.getTransferTime(), finalTransferTime);
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
      balanceManager.initializeICPPlanForSingleSupport(finalTransferTime);
>>>>>>> 2fb58d4d161... did the simple balance manager

      updateHeightManager();

      feetManager.requestSwing(swingSide, nextFootstep, swingTime);

      if (feetManager.adjustHeightIfNeeded(nextFootstep))
      {
         walkingMessageHandler.updateVisualizationAfterFootstepAdjustement(nextFootstep);
         feetManager.adjustSwingTrajectory(swingSide, nextFootstep, swingTime);
      }

      if (isLastStep)
      {
         pelvisOrientationManager.initializeSwing(supportSide, swingTime, finalTransferTime, 0.0);
      }
      else
      {
         FootstepTiming nextTiming = footstepTimings[0];
         pelvisOrientationManager.initializeSwing(supportSide, swingTime, nextTiming.getTransferTime(), nextTiming.getSwingTime());
      }

      nextFootstep.getPose(desiredFootPoseInWorld);
      desiredFootPoseInWorld.changeFrame(worldFrame);

      actualFootPoseInWorld.setToZero(fullRobotModel.getSoleFrame(swingSide));
      actualFootPoseInWorld.changeFrame(worldFrame);
      walkingMessageHandler.reportFootstepStarted(swingSide, desiredFootPoseInWorld, actualFootPoseInWorld, swingTime);
   }

   @Override
   public void onExit()
   {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      feetManager.setFlatFootContactState(swingSide);
=======
      balanceManager.resetPushRecovery();

      balanceManager.minimizeAngularMomentumRateZ(false);
>>>>>>> 13a03c33b98... set up the simple walking state controller

=======
>>>>>>> 2fb58d4d161... did the simple balance manager
=======
      feetManager.setFlatFootContactState(swingSide);

>>>>>>> 11a3a73b3c3... set the foot to be in support at the end of swing
      actualFootPoseInWorld.setToZero(fullRobotModel.getSoleFrame(swingSide));
      actualFootPoseInWorld.changeFrame(worldFrame);

      walkingMessageHandler.reportFootstepCompleted(swingSide, desiredFootPoseInWorld, actualFootPoseInWorld, swingTime);
      walkingMessageHandler.registerCompletedDesiredFootstep(nextFootstep);

      MovingReferenceFrame soleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(nextFootstep.getRobotSide());
      tempOrientation.setIncludingFrame(nextFootstep.getFootstepPose().getOrientation());
      tempOrientation.changeFrame(soleZUpFrame);

<<<<<<< HEAD
<<<<<<< HEAD
      setYoVariablesToNaN();
   }

=======

      setYoVariablesToNaN();
   }


>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
      setYoVariablesToNaN();
   }

>>>>>>> 2fb58d4d161... did the simple balance manager
   private void setYoVariablesToNaN()
   {
      estimatedRemainingSwingTimeUnderDisturbance.setToNaN();
      remainingSwingTimeAccordingToPlan.setToNaN();
   }

   /**
    * Request the swing trajectory to speed up using
    * {@link us.ihmc.commonWalkingControlModules.capturePoint.ICPPlannerInterface#estimateTimeRemainingForStateUnderDisturbance(FramePoint2D)}.
    * It is clamped w.r.t. to
    * {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    *
    * @return the current swing time remaining for the swing foot trajectory
    */
   private double requestSwingSpeedUpIfNeeded()
   {
      remainingSwingTimeAccordingToPlan.set(balanceManager.getTimeRemainingInCurrentState());

      double remainingTime = balanceManager.estimateTimeRemainingForSwingUnderDisturbance();
      estimatedRemainingSwingTimeUnderDisturbance.set(remainingTime);

      if (remainingTime > 1.0e-3)
      {
         double swingSpeedUpFactor = remainingSwingTimeAccordingToPlan.getDoubleValue() / remainingTime;
         return feetManager.requestSwingSpeedUp(swingSide, swingSpeedUpFactor);
      }
      else if (remainingSwingTimeAccordingToPlan.getDoubleValue() > 1.0e-3)
      {
         return feetManager.requestSwingSpeedUp(swingSide, Double.POSITIVE_INFINITY);
      }
      return remainingSwingTimeAccordingToPlan.getDoubleValue();
   }

   private void updateFootstepParameters()
   {
      // Update the contact states based on the footstep. If the footstep doesn't have any predicted contact points, then use the default ones in the ContactablePlaneBodies.
      controllerToolbox.updateContactPointsForUpcomingFootstep(nextFootstep);
      controllerToolbox.updateBipedSupportPolygons();

      pelvisOrientationManager.setTrajectoryTime(swingTime);
      pelvisOrientationManager.setUpcomingFootstep(nextFootstep);
      pelvisOrientationManager.updateTrajectoryFromFootstep(); // fixme this shouldn't be called when the footstep is updated
   }

   private void updateHeightManager()
   {
      balanceManager.getFinalDesiredCoMPosition(desiredCoM);

      NewTransferToAndNextFootstepsData transferToAndNextFootstepsData = walkingMessageHandler.createTransferToAndNextFootstepDataForSingleSupport(nextFootstep,
                                                                                                                                                   swingSide);
      transferToAndNextFootstepsData.setComAtEndOfState(desiredCoM);
      double extraToeOffHeight = 0.0;
      comHeightManager.initialize(transferToAndNextFootstepsData, extraToeOffHeight);
<<<<<<< HEAD
<<<<<<< HEAD
=======

      FixedFramePoint3DBasics stanceFootPosition = walkingMessageHandler.getFootstepAtCurrentLocation(swingSide.getOppositeSide())
                                                                        .getFootstepPose()
                                                                        .getPosition();
      FixedFramePoint3DBasics touchdownPosition = nextFootstep.getFootstepPose().getPosition();
      double swingTime = footstepTiming.getSwingTime(); // TODO: Should be swing time remaining for step adjustments.
      comHeightManager.step(stanceFootPosition, touchdownPosition, swingTime, swingSide, extraToeOffHeight);
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
   }

   protected boolean hasMinimumTimePassed(double timeInState)
   {
      double minimumSwingTime = swingTime * minimumSwingFraction.getDoubleValue();

      return timeInState > minimumSwingTime;
   }
}