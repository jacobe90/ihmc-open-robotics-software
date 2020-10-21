package us.ihmc.humanoidBehaviors.stairs;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

public class TraverseStairsExecuteStepsState implements State
{
   private final BehaviorHelper helper;
   private final TraverseStairsBehaviorParameters parameters;
   private final Supplier<FootstepPlannerOutput> footstepPlannerOutput;
   private final RemoteHumanoidRobotInterface robotInterface;

   private final AtomicBoolean isWalking = new AtomicBoolean();
   private final AtomicBoolean walkingComplete = new AtomicBoolean();
   private final AtomicBoolean rebroadcastSteps = new AtomicBoolean();

   public TraverseStairsExecuteStepsState(BehaviorHelper helper, TraverseStairsBehaviorParameters parameters, Supplier<FootstepPlannerOutput> footstepPlannerOutput)
   {
      this.helper = helper;
      this.parameters = parameters;
      this.footstepPlannerOutput = footstepPlannerOutput;
      this.robotInterface = helper.getOrCreateRobotInterface();

      helper.createROS2ControllerCallback(WalkingStatusMessage.class, message ->
      {
         LogTools.info("Received WalkingStatusMessage: " + WalkingStatus.fromByte(message.getWalkingStatus()));
         isWalking.set(message.getWalkingStatus() == WalkingStatus.STARTED.toByte());

         if (message.getWalkingStatus() == WalkingStatus.COMPLETED.toByte())
         {
            walkingComplete.set(true);
         }
         else if (message.getWalkingStatus() == WalkingStatus.ABORT_REQUESTED.toByte())
         {
            FootstepPlan footstepPlan = footstepPlannerOutput.get().getFootstepPlan();
            FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan, -1.0, -1.0);
            robotInterface.requestWalk(footstepDataListMessage);
         }
      });

      new IHMCROS2Callback<>(helper.getManagedROS2Node(), TraverseStairsBehaviorAPI.EXECUTE_STEPS, r ->
      {
         LogTools.info("Execute steps requested");
         rebroadcastSteps.set(true);
      });
   }

   @Override
   public void onEntry()
   {
      clearStatusFlags();
      sendPlan();
   }

   private void sendPlan()
   {
      FootstepPlannerOutput footstepPlannerOutput = this.footstepPlannerOutput.get();
      if (footstepPlannerOutput == null)
      {
         throw new RuntimeException("Footstep planner output is null");
      }

      FootstepPlan footstepPlan = footstepPlannerOutput.getFootstepPlan();
      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan, -1.0, -1.0);
      LogTools.info("Requesting walk");
      robotInterface.requestWalk(footstepDataListMessage);
   }

   @Override
   public void doAction(double timeInState)
   {
      if (rebroadcastSteps.getAndSet(false) && !isWalking.get())
      {
         sendPlan();
      }
   }

   @Override
   public void onExit(double timeInState)
   {
      clearStatusFlags();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return walkingIsComplete() && !planEndsAtGoal();
   }

   public void clearStatusFlags()
   {
      isWalking.set(false);
      walkingComplete.set(false);
      rebroadcastSteps.set(false);
   }

   boolean planEndsAtGoal()
   {
      return footstepPlannerOutput.get() != null && footstepPlannerOutput.get().getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION;
   }

   boolean walkingIsComplete()
   {
      return walkingComplete.get();
   }
}
