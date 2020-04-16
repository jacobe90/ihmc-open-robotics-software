package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.ArrayList;
import java.util.List;

public class BetterLookAheadCoMHeightTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private boolean visualize = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean initializeToCurrent = new YoBoolean("initializeCoMHeightToCurrent", registry);
   private final BooleanProvider processGoHome = new BooleanParameter("ProcessGoHome", registry, false);

   private final HeightOffsetHandler heightOffsetHandler;

   private final YoDouble minimumHeightAboveGround = new YoDouble("minimumHeightAboveGround", registry);
   private final YoDouble nominalHeightAboveGround = new YoDouble("nominalHeightAboveGround", registry);
   private final YoDouble maximumHeightAboveGround = new YoDouble("maximumHeightAboveGround", registry);
   private final YoDouble extraToeOffHeight = new YoDouble("extraToeOffHeight", registry);

   private final YoDouble nominalDoubleSupportPercentageIn = new YoDouble("nominalDoubleSupportPercentageIn", registry);
   private final YoDouble doubleSupportPercentageIn = new YoDouble("doubleSupportPercentageIn", registry);
   private final YoDouble percentageThroughSegment = new YoDouble("percentageThroughSegment", registry);
   private final YoDouble splineQuery = new YoDouble("splineQuery", registry);

   private final YoDouble offsetFromNominalInPlan = new YoDouble("offsetFromNominalInPlan", registry);

   private final FramePoint3D transferFromPosition = new FramePoint3D();
   private final FramePoint3D transferToPosition = new FramePoint3D();
   private final YoFramePoint3D yoTransferFromPosition = new YoFramePoint3D("transferFromPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D yoTransferToPosition = new YoFramePoint3D("transferToPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D desiredCoMPositionAtStart = new YoFramePoint3D("desiredCoMPositionAtStart", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint3D desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble desiredCoMHeight = new YoDouble("desiredCoMHeight", registry);
   private final YoFramePoint3D queryPosition = new YoFramePoint3D("queryPosition", ReferenceFrame.getWorldFrame(), registry);

   private final RecyclingArrayList<CoMHeightTrajectoryWaypoint> heightWaypoints;

   private final DoubleProvider yoTime;

   private ReferenceFrame frameOfSupportLeg;
   private final ReferenceFrame centerOfMassFrame;
   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   private final FramePoint3D com = new FramePoint3D();

   private final FramePoint3D tempFramePoint = new FramePoint3D();

   private final FramePoint3D startCoMPosition = new FramePoint3D();
   private final FramePoint3D endCoMPosition = new FramePoint3D();

   private final SplinedHeightTrajectory splinedHeightTrajectory;

   public BetterLookAheadCoMHeightTrajectoryGenerator(double minimumHeightAboveGround,
                                                      double nominalHeightAboveGround,
                                                      double maximumHeightAboveGround,
                                                      double defaultOffsetHeightAboveGround,
                                                      double doubleSupportPercentageIn,
                                                      ReferenceFrame centerOfMassFrame,
                                                      SideDependentList<? extends ReferenceFrame> soleFrames,
                                                      DoubleProvider yoTime,
                                                      YoGraphicsListRegistry yoGraphicsListRegistry,
                                                      YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.soleFrames = soleFrames;
      this.yoTime = yoTime;

      heightOffsetHandler = new HeightOffsetHandler(yoTime, defaultOffsetHeightAboveGround, registry);

      setMinimumHeightAboveGround(minimumHeightAboveGround);
      setNominalHeightAboveGround(nominalHeightAboveGround);
      setMaximumHeightAboveGround(maximumHeightAboveGround);

      heightWaypoints = new RecyclingArrayList<>(5, SupplierBuilder.indexedSupplier(this::createHeightWaypoint));

      splinedHeightTrajectory = new SplinedHeightTrajectory(registry, yoGraphicsListRegistry);

      setSupportLeg(RobotSide.LEFT);

      this.nominalDoubleSupportPercentageIn.set(doubleSupportPercentageIn);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry == null)
         visualize = false;

      if (visualize)
      {
         double pointSize = 0.03;

         String prefix = "better_";

         List<AppearanceDefinition> colors = new ArrayList<>();
         colors.add(YoAppearance.CadetBlue());
         colors.add(YoAppearance.Chartreuse());
         colors.add(YoAppearance.Yellow());
         colors.add(YoAppearance.BlueViolet());
         colors.add(YoAppearance.Azure());

         String graphicListName = "BetterCoMHeightTrajectoryGenerator";

         heightWaypoints.clear();
         for (int i = 0; i < colors.size(); i++)
         {
            heightWaypoints.add().setupViz(graphicListName, prefix + "HeightWaypoint" + i, colors.get(i), yoGraphicsListRegistry);
         }

         YoGraphicPosition desiredCoMPositionViz = new YoGraphicPosition(prefix + "desiredCoMPosition",
                                                                         desiredCoMPosition,
                                                                         1.1 * pointSize,
                                                                         YoAppearance.Gold());

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, desiredCoMPositionViz);
      }
      heightWaypoints.clear();
   }

   private CoMHeightTrajectoryWaypoint createHeightWaypoint(int i)
   {
      return new CoMHeightTrajectoryWaypoint("heightWaypoint" + i, registry);
   }

   public void reset()
   {
      tempFramePoint.setToZero(centerOfMassFrame);
      tempFramePoint.changeFrame(frameOfSupportLeg);
      tempFramePoint.setZ(nominalHeightAboveGround.getDoubleValue());
      desiredCoMHeight.set(nominalHeightAboveGround.getDoubleValue());
      tempFramePoint.changeFrame(worldFrame);

      desiredCoMPosition.set(tempFramePoint);

      offsetFromNominalInPlan.set(0.0);
      extraToeOffHeight.set(0.0);

      heightOffsetHandler.reset();
   }

   public void setMinimumHeightAboveGround(double minimumHeightAboveGround)
   {
      this.minimumHeightAboveGround.set(minimumHeightAboveGround);
   }

   public void setNominalHeightAboveGround(double nominalHeightAboveGround)
   {
      this.nominalHeightAboveGround.set(nominalHeightAboveGround);
   }

   public void setMaximumHeightAboveGround(double maximumHeightAboveGround)
   {
      this.maximumHeightAboveGround.set(maximumHeightAboveGround);
   }

   public double getDoubleSupportPercentageIn()
   {
      return doubleSupportPercentageIn.getDoubleValue();
   }

   public void setSupportLeg(RobotSide supportLeg)
   {
      frameOfSupportLeg = soleFrames.get(supportLeg);
      splinedHeightTrajectory.setReferenceFrame(frameOfSupportLeg);
      heightOffsetHandler.setReferenceFrame(frameOfSupportLeg);
   }

   public void initialize(NewTransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight)
   {
      FramePoint3DReadOnly transferToFootstepPosition = transferToAndNextFootstepsData.getTransferToPosition();
      offsetFromNominalInPlan.set(heightOffsetHandler.getOffsetHeightAboveGround());
      this.extraToeOffHeight.set(extraToeOffHeight);

      transferToPosition.setIncludingFrame(transferToFootstepPosition);
      transferFromPosition.setToZero(soleFrames.get(transferToAndNextFootstepsData.getTransferToSide().getOppositeSide()));
      transferToPosition.changeFrame(frameOfSupportLeg);
      transferFromPosition.changeFrame(frameOfSupportLeg);
      double midstanceWidth = 0.5 * (transferToPosition.getY() + transferFromPosition.getY());
      double startAnkleZ = transferFromPosition.getZ();

      yoTransferFromPosition.setMatchingFrame(transferFromPosition);
      yoTransferToPosition.setMatchingFrame(transferToPosition);

      desiredCoMPositionAtStart.set(desiredCoMPosition);

      startCoMPosition.setIncludingFrame(desiredCoMPosition);
      startCoMPosition.changeFrame(frameOfSupportLeg);

      endCoMPosition.setIncludingFrame(transferToFootstepPosition);
      endCoMPosition.changeFrame(frameOfSupportLeg);
      double middleAnkleZ = endCoMPosition.getZ();
      endCoMPosition.addZ(nominalHeightAboveGround.getDoubleValue() + offsetFromNominalInPlan.getDoubleValue());

      startCoMPosition.setY(midstanceWidth);
      endCoMPosition.setY(midstanceWidth);

      if (!transferToAndNextFootstepsData.getCoMAtEndOfState().containsNaN())
      {
         tempFramePoint.setIncludingFrame(transferToAndNextFootstepsData.getCoMAtEndOfState(), 0.0);
         tempFramePoint.changeFrame(frameOfSupportLeg);
         tempFramePoint.setZ(nominalHeightAboveGround.getDoubleValue());
         double percentIn = EuclidGeometryTools.percentageAlongLineSegment3D(tempFramePoint, startCoMPosition, endCoMPosition);
         percentIn = MathTools.clamp(percentIn, nominalDoubleSupportPercentageIn.getDoubleValue(), 1.0 - nominalDoubleSupportPercentageIn.getDoubleValue());
         doubleSupportPercentageIn.set(percentIn);
      }
      else
      {
         doubleSupportPercentageIn.set(nominalDoubleSupportPercentageIn.getDoubleValue());
      }

      heightWaypoints.clear();

      computeWaypoints(startCoMPosition, endCoMPosition,
                       startAnkleZ,
                       middleAnkleZ,
                       minimumHeightAboveGround.getDoubleValue(),
                       maximumHeightAboveGround.getDoubleValue(),
                       this.extraToeOffHeight.getDoubleValue());

      for (int i = 0; i < heightWaypoints.size(); i++)
         heightWaypoints.get(i).update();

      splinedHeightTrajectory.clearWaypoints();
      splinedHeightTrajectory.addWaypoints(heightWaypoints);
      splinedHeightTrajectory.computeSpline();
   }

   private void computeWaypoints(FramePoint3DReadOnly startCoMPosition,
                                 FramePoint3DReadOnly endCoMPosition,
                                 double startGroundHeight,
                                 double endGroundHeight,
                                 double minimumHeight,
                                 double maximumHeight,
                                 double extraToeOffHeight)
   {
      double midstanceWidth = 0.5 * (startCoMPosition.getY() + endCoMPosition.getY());

      double startX = startCoMPosition.getX();
      double endX = endCoMPosition.getX();

      double firstAlpha = 0.5 * doubleSupportPercentageIn.getDoubleValue();
      double secondAlpha = doubleSupportPercentageIn.getDoubleValue();
      double thirdAlpha = 0.5 * (1.0 + doubleSupportPercentageIn.getDoubleValue());

      double firstMidpointX = InterpolationTools.linearInterpolate(startX, endX, firstAlpha);
      double secondMidpointX = InterpolationTools.linearInterpolate(startX, endX, secondAlpha);
      double thirdMidpointX = InterpolationTools.linearInterpolate(startX, endX, thirdAlpha);

      CoMHeightTrajectoryWaypoint startWaypoint = heightWaypoints.size() > 0 ? heightWaypoints.getLast() : getWaypointInFrame(frameOfSupportLeg);
      CoMHeightTrajectoryWaypoint firstMidpoint = getWaypointInFrame(frameOfSupportLeg);
      CoMHeightTrajectoryWaypoint secondMidpoint = getWaypointInFrame(frameOfSupportLeg);
      CoMHeightTrajectoryWaypoint thirdMidpoint = getWaypointInFrame(frameOfSupportLeg);
      CoMHeightTrajectoryWaypoint endWaypoint = getWaypointInFrame(frameOfSupportLeg);

      startWaypoint.setXY(startX, midstanceWidth);
      firstMidpoint.setXY(firstMidpointX, midstanceWidth);
      secondMidpoint.setXY(secondMidpointX, midstanceWidth);
      thirdMidpoint.setXY(thirdMidpointX, midstanceWidth);
      endWaypoint.setXY(endX, midstanceWidth);

      double startMinHeight = findWaypointHeight(minimumHeight, 0.0, startX, startGroundHeight);
      double startMaxHeight = findWaypointHeight(maximumHeight + extraToeOffHeight, 0.0, startX, startGroundHeight);
      double firstMinHeight = findWaypointHeight(minimumHeight, 0.0, firstMidpointX, startGroundHeight);
      double firstMaxHeight = findWaypointHeight(maximumHeight, 0.0, firstMidpointX, startGroundHeight);

      double exchangeFromMinHeight = findWaypointHeight(minimumHeight, 0.0, secondMidpointX, startGroundHeight);
      double exchangeToMinHeight = findWaypointHeight(minimumHeight, endX, secondMidpointX, endGroundHeight);
      double exchangeFromMaxHeight = findWaypointHeight(maximumHeight + extraToeOffHeight, 0.0, secondMidpointX, startGroundHeight);
      double exchangeToMaxHeight = findWaypointHeight(maximumHeight, endX, secondMidpointX, endGroundHeight);
      double secondMinHeight = Math.min(Math.max(exchangeFromMinHeight, exchangeToMinHeight), Math.min(exchangeFromMaxHeight, exchangeToMaxHeight));
      double secondMaxHeight = Math.min(exchangeFromMaxHeight, exchangeToMaxHeight);
      double thirdMinHeight = findWaypointHeight(minimumHeight, endX, thirdMidpointX, endGroundHeight);
      double thirdMaxHeight = findWaypointHeight(maximumHeight, endX, thirdMidpointX, endGroundHeight);
      double endMinHeight = minimumHeight + endGroundHeight;
      double endMaxHeight = maximumHeight + endGroundHeight;

      startWaypoint.setMinMax(startMinHeight, startMaxHeight);
      firstMidpoint.setMinMax(firstMinHeight, firstMaxHeight);
      secondMidpoint.setMinMax(secondMinHeight, secondMaxHeight);
      thirdMidpoint.setMinMax(thirdMinHeight, thirdMaxHeight);
      endWaypoint.setMinMax(endMinHeight, endMaxHeight);

      startWaypoint.setHeight(MathTools.clamp(startCoMPosition.getZ(), startMinHeight, startMaxHeight));
      endWaypoint.setHeight(MathTools.clamp(endCoMPosition.getZ(), endMinHeight, endMaxHeight));
   }

   private CoMHeightTrajectoryWaypoint getWaypointInFrame(ReferenceFrame referenceFrame)
   {
      CoMHeightTrajectoryWaypoint waypoint = heightWaypoints.add();
      waypoint.setToZero(referenceFrame);

      return waypoint;
   }

   private static double findWaypointHeight(double desiredDistanceFromFoot, double startAnkleX, double queryX, double extraHeight)
   {
      return Math.sqrt(MathTools.square(desiredDistanceFromFoot) - MathTools.square(queryX - startAnkleX)) + extraHeight;
   }

   public void solve(CoMHeightPartialDerivativesData comHeightPartialDerivativesDataToPack, boolean isInDoubleSupport)
   {
      com.setToZero(centerOfMassFrame);
      com.changeFrame(worldFrame);

      solve(comHeightPartialDerivativesDataToPack, com, isInDoubleSupport);

      desiredCoMPosition.set(com.getX(), com.getY(), comHeightPartialDerivativesDataToPack.getComHeight());
   }

   private final Point2D point = new Point2D();

   void solve(CoMHeightPartialDerivativesData comHeightPartialDerivativesDataToPack, FramePoint3DBasics queryPoint, boolean isInDoubleSupport)
   {
      this.queryPosition.set(queryPoint);

      percentageThroughSegment.set(splinedHeightTrajectory.solve(comHeightPartialDerivativesDataToPack, queryPoint, point));

      heightOffsetHandler.update(splinedHeightTrajectory.getHeightSplineSetpoint());

      handleInitializeToCurrent(point.getY() + heightOffsetHandler.getOffsetHeightAboveGround() - offsetFromNominalInPlan.getDoubleValue());

      point.addY(heightOffsetHandler.getOffsetHeightAboveGround() - offsetFromNominalInPlan.getDoubleValue());
      comHeightPartialDerivativesDataToPack.setCoMHeight(worldFrame,
                                                         comHeightPartialDerivativesDataToPack.getComHeight()
                                                         + heightOffsetHandler.getOffsetHeightAboveGround() - offsetFromNominalInPlan.getDoubleValue());

      this.splineQuery.set(point.getX());
      this.desiredCoMHeight.set(point.getY());

   }

   private void handleInitializeToCurrent(double normalDesiredHeight)
   {
      if (!initializeToCurrent.getBooleanValue())
         return;

      initializeToCurrent.set(false);

      tempFramePoint.setToZero(centerOfMassFrame);
      tempFramePoint.changeFrame(frameOfSupportLeg);

      double heightOffset = tempFramePoint.getZ() - normalDesiredHeight;

      heightOffsetHandler.initializeToCurrent(heightOffset);
   }

   private final PelvisHeightTrajectoryCommand tempPelvisHeightTrajectoryCommand = new PelvisHeightTrajectoryCommand();

   public boolean handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();

      if (!se3Trajectory.getSelectionMatrix().isLinearZSelected())
         return false; // The user does not want to control the height, do nothing.

      se3Trajectory.changeFrame(worldFrame);
      tempPelvisHeightTrajectoryCommand.set(command);
      handlePelvisHeightTrajectoryCommand(tempPelvisHeightTrajectoryCommand);
      return true;
   }

   public boolean handlePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command)
   {
      return heightOffsetHandler.handlePelvisHeightTrajectoryCommand(command, splinedHeightTrajectory.getHeightSplineSetpoint());
   }

   public void goHome(double trajectoryTime)
   {
      if (!processGoHome.getValue())
         return;

      heightOffsetHandler.goHome(trajectoryTime);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      heightOffsetHandler.handleStopAllTrajectoryCommand(command);
   }

   public void initializeDesiredHeightToCurrent()
   {
      initializeToCurrent.set(true);
   }

   public void getCurrentDesiredHeight(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(desiredCoMPosition);
   }

   public double getOffsetHeightTimeInTrajectory()
   {
      return yoTime.getValue() - heightOffsetHandler.getOffsetHeightAboveGroundChangedTime();
   }
}
