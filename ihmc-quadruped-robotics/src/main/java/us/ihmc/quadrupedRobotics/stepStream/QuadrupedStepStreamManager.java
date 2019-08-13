package us.ihmc.quadrupedRobotics.stepStream;

import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.QuadrupedTeleopCommand;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.util.YoQuadrupedTimedStep;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class QuadrupedStepStreamManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   static final int MAXIMUM_PLAN_CAPACITY = 200;

   /** Preplanned step stream generates steps from QuadrupedTimedStepListCommand */
   private final QuadrupedPreplannedStepStream preplannedStepStream;

   /** XGait step stream generates steps from QuadrupedTeleopCommand */
   private final QuadrupedXGaitStepStream xGaitStepStream;

   /** Active mode. If both commands are received, PREPLANNED is given precedence */
   private final YoEnum<QuadrupedStepMode> stepMode = new YoEnum<>("StepMode", registry, QuadrupedStepMode.class, false);

   /** Flags indicating touchdown status, set externally. True indicates a foot is in contact */
   private final QuadrantDependentList<YoBoolean> touchdownFlags = new QuadrantDependentList<>();

   /** Entire step sequence, including current steps */
   private final PreallocatedList<YoQuadrupedTimedStep> stepSequence;

   /** List of active steps. Steps get added to the list when based on start time and removed when touchdown is triggered */
   private final List<YoQuadrupedTimedStep> activeSteps = new ArrayList<>();

   /** Transfer duration before liftoff of first step after onEntry is called */
   private final DoubleProvider initialTransferDurationForShifting = new DoubleParameter("initialTransferDurationForShifting", registry, 1.0);

   /** Holds current or next step on each robot end. Helper object for calculating step delay */
   private final EndDependentList<YoQuadrupedTimedStep> currentSteps = new EndDependentList<>();

   private final YoBoolean pausedRequested = new YoBoolean("pauseRequested", registry);
   private final YoBoolean stopRequested = new YoBoolean("stopRequested", registry);

   private final YoDouble timestamp;

   public QuadrupedStepStreamManager(YoDouble timestamp, QuadrupedReferenceFrames referenceFrames, double controlDt, QuadrupedXGaitSettings xGaitSettings,
                                     YoVariableRegistry parentRegistry)
   {
      this.timestamp = timestamp;
      this.preplannedStepStream = new QuadrupedPreplannedStepStream(timestamp);
      this.xGaitStepStream = new QuadrupedXGaitStepStream(referenceFrames, timestamp, controlDt, xGaitSettings, registry);

      this.stepMode.set(QuadrupedStepMode.PREPLANNED);

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         touchdownFlags.put(quadrant, new YoBoolean("stepStreamTouchdown" + quadrant.getShortName().toLowerCase(), registry));
      }

      Supplier<YoQuadrupedTimedStep> stepSupplier = SupplierBuilder.indexedSupplier(i -> new YoQuadrupedTimedStep("step" + i, registry));
      stepSequence = new PreallocatedList<>(YoQuadrupedTimedStep.class, stepSupplier, MAXIMUM_PLAN_CAPACITY);

      parentRegistry.addChild(registry);
   }

   public boolean isStepPlanAvailable()
   {
      return preplannedStepStream.isPlanAvailable() || xGaitStepStream.isPlanAvailable();
   }

   public void onEntry()
   {
      // default to preplanned steps if available
      if(preplannedStepStream.isPlanAvailable())
      {
         stepMode.set(QuadrupedStepMode.PREPLANNED);
      }
      else if (xGaitStepStream.isPlanAvailable())
      {
         stepMode.set(QuadrupedStepMode.XGAIT);
      }
      else
      {
         return;
      }

      // Assume all feet are in contact when onEntry is called
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         touchdownFlags.get(quadrant).set(true);
      }

      stopRequested.set(false);

      stepSequence.clear();
      getStepStream().onEntry(stepSequence, initialTransferDurationForShifting);

      // Ensure step sequence is ordered according to start time
      stepSequence.sort(TimeIntervalTools.startTimeComparator);

      // Initialize current steps from step sequence
      for(RobotEnd end : RobotEnd.values)
      {
         currentSteps.put(end, getFirstStep(end));
      }
   }

   public void doAction()
   {
      // Delay current steps and shift subsequent steps by maximum delay amount
      double currentStepDelay = getCurrentStepDelay();
      for (int i = 0; i < stepSequence.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.get(i);
         if(currentSteps.get(step.getRobotQuadrant().getEnd()) != step)
         {
            step.getTimeInterval().shiftInterval(currentStepDelay);
         }
      }

      // Remove completed steps
      TimeIntervalTools.removeEndTimesLessThan(timestamp.getDoubleValue(), stepSequence);

      // Update current steps
      for (RobotEnd end : RobotEnd.values)
      {
         currentSteps.put(end, getFirstStep(end));
      }

      if (stopRequested.getValue())
      {
         TimeIntervalTools.removeStartTimesGreaterThan(timestamp.getDoubleValue(), stepSequence);
      }
      else
      {
         getStepStream().doAction(stepSequence);
      }

      // Ensure step sequence is ordered according to start time
      stepSequence.sort(TimeIntervalTools.startTimeComparator);

      updateActiveSteps();
   }

   /**
    * Shifts time interval of active steps to current include current time. Returns maximum shift value
    */
   private double getCurrentStepDelay()
   {
      double currentStepDelay = 0.0;
      double delayEpsilon = 1e-3;

      for (RobotEnd end : RobotEnd.values)
      {
         QuadrupedTimedStep currentStep = currentSteps.get(end);
         if (currentStep == null)
         {
            continue;
         }

         boolean touchdownHasNotTriggered = !touchdownFlags.get(currentStep.getRobotQuadrant()).getBooleanValue();
         boolean endTimeHasExpired = currentStep.getTimeInterval().getEndTime() < timestamp.getDoubleValue();
         if (touchdownHasNotTriggered && endTimeHasExpired)
         {
            double delay = timestamp.getDoubleValue() - currentStep.getTimeInterval().getEndTime() + delayEpsilon;
            currentStep.getTimeInterval().shiftInterval(delay);
            currentStepDelay = Math.max(delay, currentStepDelay);
         }
      }
      return currentStepDelay;
   }

   /**
    * Returns entire step sequence, including current steps
    */
   public PreallocatedList<? extends QuadrupedTimedStep> getSteps()
   {
      return stepSequence;
   }

   public List<? extends QuadrupedTimedStep> getActiveSteps()
   {
      return activeSteps;
   }

   /**
    * Returns first step in stepSequence on the given end
    */
   private YoQuadrupedTimedStep getFirstStep(RobotEnd end)
   {
      for (int i = 0; i < stepSequence.size(); i++)
      {
         if(stepSequence.get(i).getRobotQuadrant().getEnd() == end)
         {
            return stepSequence.get(i);
         }
      }

      return null;
   }

   private void updateActiveSteps()
   {
      activeSteps.clear();
      for (int i = 0; i < stepSequence.size(); i++)
      {
         if(stepSequence.get(i).getTimeInterval().getStartTime() <= timestamp.getDoubleValue())
         {
            activeSteps.add(stepSequence.get(i));
         }
      }
   }

   public void onLiftOff(RobotQuadrant quadrant)
   {
      touchdownFlags.get(quadrant).set(false);
   }

   public void onTouchDown(RobotQuadrant quadrant)
   {
      touchdownFlags.get(quadrant).set(true);
   }

   public void acceptTimedStepListCommand(QuadrupedTimedStepListCommand timedStepListCommand)
   {
      preplannedStepStream.accept(timedStepListCommand);
   }

   public void acceptTeleopCommand(QuadrupedTeleopCommand teleopCommand)
   {
      xGaitStepStream.accept(teleopCommand);
   }

   private QuadrupedStepStream getStepStream()
   {
      return stepMode.getEnumValue() == QuadrupedStepMode.PREPLANNED ? preplannedStepStream : xGaitStepStream;
   }

   public void requestStop()
   {
      stopRequested.set(true);
   }
}
