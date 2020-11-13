package us.ihmc.avatar.ros.networkTest;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.net.InetAddress;
import java.util.ArrayList;
import java.util.List;

public abstract class ROS2NetworkTestProfile
{
   protected String localHostname = ExceptionTools.handle(() -> InetAddress.getLocalHost().getHostName(), DefaultExceptionHandler.RUNTIME_EXCEPTION);

   public abstract List<ROS2NetworkTestMachine> getMachines();

   public abstract void runExperiment();

   public abstract YoRegistry getYoRegistry();

   public abstract void destroy();

   public List<ROS2NetworkTestMachine> getRemoteMachines()
   {
      ArrayList<ROS2NetworkTestMachine> remoteMachines = new ArrayList<>();
      for (ROS2NetworkTestMachine machines : getMachines())
      {
         if (!machines.getHostname().equals(localHostname))
         {
            remoteMachines.add(machines);
         }
      }
      return remoteMachines;
   }
}