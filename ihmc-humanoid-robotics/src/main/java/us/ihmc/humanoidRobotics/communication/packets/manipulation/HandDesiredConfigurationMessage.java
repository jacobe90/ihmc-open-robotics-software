package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation = "Packet for commanding the hands to perform various predefined grasps."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/hand_desired_configuration")
public class HandDesiredConfigurationMessage extends Packet<HandDesiredConfigurationMessage>
{
   @RosExportedField(documentation = "Specifies the side of the robot that will execute the trajectory")
   public byte robotSide;
   @RosExportedField(documentation = "Specifies the grasp to perform")
   public byte desiredHandConfiguration;

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public HandDesiredConfigurationMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(HandDesiredConfigurationMessage other)
   {
      robotSide = other.robotSide;
      desiredHandConfiguration = other.desiredHandConfiguration;
      setPacketInformation(other);
   }

   public byte getHandDesiredConfiguration()
   {
      return desiredHandConfiguration;
   }

   public byte getRobotSide()
   {
      return robotSide;
   }

   @Override
   public boolean equals(Object other)
   {
      return ((other instanceof HandDesiredConfigurationMessage) && this.epsilonEquals((HandDesiredConfigurationMessage) other, 0));
   }

   @Override
   public String toString()
   {
      return RobotSide.fromByte(robotSide).toString() + " State= " + HandConfiguration.fromByte(desiredHandConfiguration).toString();
   }

   @Override
   public boolean epsilonEquals(HandDesiredConfigurationMessage other, double epsilon)
   {
      boolean ret = (this.getRobotSide() == other.getRobotSide());
      ret &= (this.getHandDesiredConfiguration() == other.getHandDesiredConfiguration());

      return ret;
   }
}
