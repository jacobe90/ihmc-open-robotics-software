package us.ihmc.humanoidBehaviors.stairs;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.ros2.ROS2Topic;

import java.util.List;

public class TraverseStairsBehaviorAPI
{
   private static final String ROS_TOPIC_QUALIFIER = "/stairs";

   /**
    * Goal input, should be sent prior to sending "start"
    */
   public static final ROS2Topic<Pose3D> GOAL_INPUT = ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Pose3D.class).withSuffix(ROS_TOPIC_QUALIFIER + "/goal");
   /**
    * Begins the behavior, should have received the goal input prior
    */
   public static final ROS2Topic<Empty> START = ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Empty.class).withSuffix(ROS_TOPIC_QUALIFIER + "/start");
   /**
    * Stops the behavior, if a step is being executed it will finish that step then stand in place. Can be restarted through the start topic, the goal is persistent
    */
   public static final ROS2Topic<Empty> STOP = ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Empty.class).withSuffix(ROS_TOPIC_QUALIFIER + "/stop");
   /**
    * Signals that the robot has reached the goal
    */
   public static final ROS2Topic<Empty> COMPLETED = ROS2Tools.BEHAVIOR_MODULE.withOutput().withType(Empty.class).withSuffix(ROS_TOPIC_QUALIFIER + "/completed");
   /**
    * Planned steps to visualize
    */
   public static final ROS2Topic<FootstepDataListMessage> PLANNED_STEPS = ROS2Tools.BEHAVIOR_MODULE.withOutput().withType(FootstepDataListMessage.class).withSuffix(ROS_TOPIC_QUALIFIER);
   /**
    * Signals the behavior to execute the steps, sent from the ui
    */
   public static final ROS2Topic<Empty> EXECUTE_STEPS = ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Empty.class).withSuffix(ROS_TOPIC_QUALIFIER + "/execute_steps");
   /**
    * Signals the behavior to replan steps, sent from the ui
    */
   public static final ROS2Topic<Empty> REPLAN = ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Empty.class).withSuffix(ROS_TOPIC_QUALIFIER + "/replan");

   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("TraverseStairsBehavior");
   private static final MessagerAPIFactory.CategoryTheme BehaviorTheme = apiFactory.createCategoryTheme("TraverseStairs");

   public static final MessagerAPIFactory.Topic<Boolean> Enabled = topic("Enabled");

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return RootCategory.child(BehaviorTheme).topic(apiFactory.createTypedTopicTheme(name));
   }

   public static MessagerAPIFactory.MessagerAPI create()
   {
      return apiFactory.getAPIAndCloseFactory();
   }
}
