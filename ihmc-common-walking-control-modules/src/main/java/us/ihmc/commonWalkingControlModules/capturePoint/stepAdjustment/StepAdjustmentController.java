package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ParameterizedICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.*;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.footstep.SimpleAdjustableFootstep;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector2d;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.List;

public class StepAdjustmentController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final boolean VISUALIZE = true;
   private static final boolean CONTINUOUSLY_UPDATE_DESIRED_POSITION = true;

   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final BooleanProvider allowStepAdjustment;
   private final YoBoolean useStepAdjustment = new YoBoolean(yoNamePrefix + "UseStepAdjustment", registry);
   private final YoBoolean footstepIsAdjustable = new YoBoolean(yoNamePrefix + "FootstepIsAdjustable", registry);

   private final YoDouble swingDuration = new YoDouble(yoNamePrefix + "SwingDuration", registry);
   private final YoDouble nextTransferDuration = new YoDouble(yoNamePrefix + "NextTransferDuration", registry);

   private final YoFramePose3D upcomingFootstep = new YoFramePose3D(yoNamePrefix + "UpcomingFootstepPose", worldFrame, registry);
   private final YoEnum<RobotSide> upcomingFootstepSide = new YoEnum<>(yoNamePrefix + "UpcomingFootstepSide", registry, RobotSide.class);
   private final RecyclingArrayList<Point2D> upcomingFootstepContactPoints = new RecyclingArrayList<>(Point2D.class);

   private final FramePoint3D referencePositionInControlPlane = new FramePoint3D();

   private final YoDouble footstepMultiplier = new YoDouble(yoNamePrefix + "TotalFootstepMultiplier", registry);
   private final DoubleProvider footstepDeadband;

   private final YoFrameVector2D footstepAdjustmentInControlPlane = new YoFrameVector2D(yoNamePrefix + "footstepAdjustmentInControlPlane", worldFrame, registry);
   private final YoFrameVector2D deadbandedAdjustment = new YoFrameVector2D(yoNamePrefix + "DeadbandedAdjustment", worldFrame, registry);

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final YoFramePose3D footstepSolution = new YoFramePose3D(yoNamePrefix + "FootstepSolutionLocation", worldFrame, registry);
   private final YoFramePoint2D adjustedSolutionInControlPlane = new YoFramePoint2D(yoNamePrefix + "adjustedSolutionInControlPlane", worldFrame, registry);

   private final DoubleProvider transferDurationSplitFraction;

   private final YoBoolean isInSwing = new YoBoolean(yoNamePrefix + "IsInSwing", registry);
   private final YoDouble initialTime = new YoDouble(yoNamePrefix + "InitialTime", registry);
   private final YoDouble timeInCurrentState = new YoDouble(yoNamePrefix + "TimeInCurrentState", registry);
   private final YoDouble timeRemainingInState = new YoDouble(yoNamePrefix + "TimeRemainingInState", registry);

   private final YoDouble recursionTime = new YoDouble(yoNamePrefix + "RecursionTime", registry);
   private final YoDouble recursionMultiplier = new YoDouble(yoNamePrefix + "RecursionMultiplier", registry);

   private final YoBoolean swingSpeedUpEnabled = new YoBoolean(yoNamePrefix + "SwingSpeedUpEnabled", registry);
   private final YoBoolean footstepWasAdjusted = new YoBoolean(yoNamePrefix + "FootstepWasAdjusted", registry);
   private final YoDouble speedUpTime = new YoDouble(yoNamePrefix + "SpeedUpTime", registry);

   private final YoFrameVector2D icpError = new YoFrameVector2D(yoNamePrefix + "ICPError", "", worldFrame, registry);

   private final StepAdjustmentReachabilityConstraint reachabilityConstraintHandler;

   private final DoubleProvider minimumFootstepMultiplier;
   private final DoubleProvider maximumTimeFromTransfer;

   private final ICPControlPlane icpControlPlane;

   public StepAdjustmentController(WalkingControllerParameters walkingControllerParameters, SideDependentList<ReferenceFrame> soleZUpFrames,
                                   BipedSupportPolygons bipedSupportPolygons, ICPControlPolygons icpControlPolygons,
                                   SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT, YoVariableRegistry parentRegistry,
                                   YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters, walkingControllerParameters.getICPOptimizationParameters(), soleZUpFrames, bipedSupportPolygons, icpControlPolygons,
           contactableFeet, controlDT, parentRegistry, yoGraphicsListRegistry);
   }

   public StepAdjustmentController(WalkingControllerParameters walkingControllerParameters, ICPOptimizationParameters icpOptimizationParameters,
                                   SideDependentList<ReferenceFrame> soleZUpFrames, BipedSupportPolygons bipedSupportPolygons,
                                   ICPControlPolygons icpControlPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet, double controlDT,
                                   YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpControlPlane = icpControlPolygons.getIcpControlPlane();

      allowStepAdjustment = new BooleanParameter(yoNamePrefix + "AllowStepAdjustment", registry, icpOptimizationParameters.allowStepAdjustment());

      minimumFootstepMultiplier = new DoubleParameter(yoNamePrefix + "MinimumFootstepMultiplier", registry,
                                                      icpOptimizationParameters.getMinimumFootstepMultiplier());
      maximumTimeFromTransfer = new DoubleParameter(yoNamePrefix + "MaximumTimeFromTransfer", registry,
                                                    icpOptimizationParameters.maximumTimeFromTransferInFootstepMultiplier());

      transferDurationSplitFraction = new DoubleParameter(yoNamePrefix + "TransferDurationSplitFraction", registry,
                                                          icpOptimizationParameters.getTransferSplitFraction());

      footstepDeadband = new DoubleParameter(yoNamePrefix + "FootstepDeadband", registry, icpOptimizationParameters.getAdjustmentDeadband());
      reachabilityConstraintHandler = new StepAdjustmentReachabilityConstraint(soleZUpFrames, icpOptimizationParameters, walkingControllerParameters.getSteppingParameters(),
                                                                               yoNamePrefix, VISUALIZE, registry, yoGraphicsListRegistry);


      if (walkingControllerParameters != null)
         swingSpeedUpEnabled.set(walkingControllerParameters.allowDisturbanceRecoveryBySpeedingUpSwing());

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition clippedFootstepSolution = new YoGraphicPosition(yoNamePrefix + "ClippedFootstepSolution", this.footstepSolution.getPosition(), 0.005,
                                                                        YoAppearance.DarkRed(), YoGraphicPosition.GraphicType.BALL);

      artifactList.add(clippedFootstepSolution.createArtifact());

      artifactList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void setFootstepToAdjust(SimpleAdjustableFootstep footstep, double swingDuration, double nextTransferDuration)
   {
      FramePose3DReadOnly footstepPose = footstep.getSoleFramePose();
      footstepPose.checkReferenceFrameMatch(worldFrame);
      if (!footstepPose.containsNaN())
      {
         upcomingFootstep.set(footstepPose);
         upcomingFootstepSide.set(footstep.getRobotSide());
         upcomingFootstepContactPoints.clear();
         ConvexPolygon2DReadOnly foothold = footstep.getFoothold();
         for (int i = 0; i < foothold.getNumberOfVertices(); i++)
         {
            upcomingFootstepContactPoints.add().set(foothold.getVertex(i));
         }

         footstepSolution.set(footstepPose);

         this.swingDuration.set(swingDuration);

         footstepIsAdjustable.set(footstep.getIsAdjustable());
         useStepAdjustment.set(allowStepAdjustment.getValue() && footstepIsAdjustable.getBooleanValue());
         this.nextTransferDuration.set(nextTransferDuration);
      }
      else
      {
         LogTools.warn("Received bad footstep: " + footstep);
      }
   }

   private void computeTimeInCurrentState(double currentTime)
   {
      timeInCurrentState.set(currentTime - initialTime.getDoubleValue() + speedUpTime.getDoubleValue());
   }

   private void computeTimeRemainingInState()
   {
      timeRemainingInState.set(swingDuration.getDoubleValue() - timeInCurrentState.getDoubleValue());
   }

   public void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing)
   {
      if (swingSpeedUpEnabled.getBooleanValue() && remainingTimeForSwing < timeRemainingInState.getDoubleValue())
      {
         double speedUpTime = timeRemainingInState.getDoubleValue() - remainingTimeForSwing;
         this.speedUpTime.add(speedUpTime);
      }
   }

   private double computeFootstepAdjustmentMultiplier(double omega0)
   {
      double timeInTransferForShifting = Math
            .min(maximumTimeFromTransfer.getValue(), transferDurationSplitFraction.getValue() * nextTransferDuration.getDoubleValue());
      recursionTime.set(Math.max(timeRemainingInState.getDoubleValue(), 0.0) + timeInTransferForShifting);
      recursionMultiplier.set(Math.exp(-omega0 * recursionTime.getDoubleValue()));

      // This is the maximum possible multiplier
      double finalRecursionMultiplier = Math.exp(-omega0 * timeInTransferForShifting);

      // The recursion multiplier is guaranteed to be between the max and min values. This forces it to interpolate between those two.
      double minimumFootstepMultiplier = Math.min(this.minimumFootstepMultiplier.getValue(), finalRecursionMultiplier);
      return minimumFootstepMultiplier + (1.0 - minimumFootstepMultiplier / finalRecursionMultiplier) * recursionMultiplier.getDoubleValue();
   }

   public void reset()
   {
      reachabilityConstraintHandler.reset();
      isInSwing.set(false);
      upcomingFootstep.setToNaN();
      footstepSolution.setToNaN();
      footstepWasAdjusted.set(false);
   }

   public void initialize(double initialTime, RobotSide supportSide)
   {
      isInSwing.set(true);
      this.initialTime.set(initialTime);
      reachabilityConstraintHandler.initializeReachabilityConstraint(supportSide, upcomingFootstep);
   }

   public void compute(double currentTime, FramePoint2DReadOnly desiredICP, FramePoint2DReadOnly currentICP, FrameVector2DReadOnly residualICPError, double omega0)
   {
      if (!isInSwing.getBooleanValue())
         return;

      computeTimeInCurrentState(currentTime);
      computeTimeRemainingInState();

      if (!useStepAdjustment.getBooleanValue())
         return;

      icpError.sub(desiredICP, currentICP);

      footstepMultiplier.set(computeFootstepAdjustmentMultiplier(omega0));
      footstepAdjustmentInControlPlane.set(residualICPError);
      footstepAdjustmentInControlPlane.scale(footstepMultiplier.getDoubleValue());

      if (footstepAdjustmentInControlPlane.length() < footstepDeadband.getValue())
      {
         footstepWasAdjusted.set(false);
         deadbandedAdjustment.setToZero();
      }
      else
      {
         footstepWasAdjusted.set(true);
         deadbandedAdjustment.set(footstepAdjustmentInControlPlane);
      }

      icpControlPlane.projectPointOntoControlPlane(worldFrame, upcomingFootstep.getPosition(), referencePositionInControlPlane);

      adjustedSolutionInControlPlane.set(referencePositionInControlPlane);
      adjustedSolutionInControlPlane.add(deadbandedAdjustment);

      icpControlPlane.projectPointFromPlaneOntoSurface(worldFrame, adjustedSolutionInControlPlane, tempPoint, upcomingFootstep.getPosition().getZ());

      footstepSolution.getPosition().set(tempPoint);

      if (wasFootstepAdjusted() && CONTINUOUSLY_UPDATE_DESIRED_POSITION)
         upcomingFootstep.set(footstepSolution);
   }

   public boolean wasFootstepAdjusted()
   {
      return footstepWasAdjusted.getBooleanValue();
   }
}
