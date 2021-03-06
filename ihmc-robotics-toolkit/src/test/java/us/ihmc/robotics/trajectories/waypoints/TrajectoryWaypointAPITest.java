package us.ihmc.robotics.trajectories.waypoints;

import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import us.ihmc.robotics.Assert;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.EuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoFrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoFrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoFrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameEuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSE3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSO3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameTrajectoryPointListBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SO3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.TrajectoryPointListBasics;
import us.ihmc.robotics.math.trajectories.waypoints.EuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPointTest;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPointTest;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPointTest;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGeneratorTest;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGeneratorTest;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGeneratorTest;
import us.ihmc.robotics.math.trajectories.waypoints.SE3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.SO3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleEuclideanTrajectoryPointTest;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleSE3TrajectoryPointTest;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleSO3TrajectoryPointTest;
import us.ihmc.robotics.math.trajectories.waypoints.TrajectoryPointOptimizerTest;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameEuclideanTrajectoryPointTest;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameEuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameSE3TrajectoryPointTest;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameSE3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameSO3TrajectoryPointTest;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameSO3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointBasics;

public class TrajectoryWaypointAPITest
{
   private final static Map<Class<?>, Class<?>> framelessTypesToFrameTypesTable;
   static
   {
      HashMap<Class<?>, Class<?>> modifiableMap = new HashMap<>();

      modifiableMap.put(EuclideanWaypointBasics.class, FrameEuclideanWaypointBasics.class);
      modifiableMap.put(SO3WaypointBasics.class, FrameSO3WaypointBasics.class);
      modifiableMap.put(SE3WaypointBasics.class, FrameSE3WaypointBasics.class);

      modifiableMap.put(EuclideanTrajectoryPointBasics.class, FrameEuclideanTrajectoryPointBasics.class);
      modifiableMap.put(SO3TrajectoryPointBasics.class, FrameSO3TrajectoryPointBasics.class);
      modifiableMap.put(SE3TrajectoryPointBasics.class, FrameSE3TrajectoryPointBasics.class);

      modifiableMap.put(TrajectoryPointListBasics.class, FrameTrajectoryPointListBasics.class);

      framelessTypesToFrameTypesTable = Collections.unmodifiableMap(modifiableMap);
   }

   private final static Map<Class<?>, Class<?>> waypointTypesToTrajectoryTypesTable;
   static
   {
      HashMap<Class<?>, Class<?>> modifiableMap = new HashMap<>();

      modifiableMap.put(EuclideanWaypointBasics.class, EuclideanTrajectoryPointBasics.class);
      modifiableMap.put(SO3WaypointBasics.class, SO3TrajectoryPointBasics.class);
      modifiableMap.put(SE3WaypointBasics.class, SE3TrajectoryPointBasics.class);

      modifiableMap.put(FrameEuclideanWaypointBasics.class, FrameEuclideanTrajectoryPointBasics.class);
      modifiableMap.put(FrameSO3WaypointBasics.class, FrameSO3TrajectoryPointBasics.class);
      modifiableMap.put(FrameSE3WaypointBasics.class, FrameSE3TrajectoryPointBasics.class);

      waypointTypesToTrajectoryTypesTable = Collections.unmodifiableMap(modifiableMap);
   }

   private final static Map<Class<?>, Class<?>> argumentTypeFramelessToFrameMap;
   static
   {
      HashMap<Class<?>, Class<?>> modifiableMap = new HashMap<>();

      modifiableMap.put(EuclideanWaypointBasics.class, FrameEuclideanWaypointBasics.class);
      modifiableMap.put(SO3WaypointBasics.class, FrameSO3WaypointBasics.class);
      modifiableMap.put(SE3WaypointBasics.class, FrameSE3WaypointBasics.class);

      modifiableMap.put(EuclideanTrajectoryPointBasics.class, FrameEuclideanTrajectoryPointBasics.class);
      modifiableMap.put(SO3TrajectoryPointBasics.class, FrameSO3TrajectoryPointBasics.class);
      modifiableMap.put(SE3TrajectoryPointBasics.class, FrameSE3TrajectoryPointBasics.class);

      modifiableMap.put(Point3DReadOnly.class, FramePoint3DReadOnly.class);
      modifiableMap.put(QuaternionReadOnly.class, FrameQuaternionReadOnly.class);
      modifiableMap.put(Vector3DReadOnly.class, FrameVector3DReadOnly.class);
      modifiableMap.put(Pose3DReadOnly.class, FramePose3DReadOnly.class);

      modifiableMap.put(Point3DBasics.class, FixedFramePoint3DBasics.class);
      modifiableMap.put(QuaternionBasics.class, FixedFrameQuaternionBasics.class);
      modifiableMap.put(Vector3DBasics.class, FixedFrameVector3DBasics.class);
      modifiableMap.put(Pose3DBasics.class, FixedFramePose3DBasics.class);

      modifiableMap.put(TrajectoryPointListBasics.class, FrameTrajectoryPointListBasics.class);

      // unchanged:
      modifiableMap.put(double.class, double.class);
      modifiableMap.put(int.class, int.class);
      modifiableMap.put(Transform.class, Transform.class);
      modifiableMap.put(TrajectoryPointBasics.class, TrajectoryPointBasics.class);

      argumentTypeFramelessToFrameMap = Collections.unmodifiableMap(modifiableMap);
   }

   private final static Map<Class<?>, Class<?>> returnTypeFramelessToFrameMap;
   static
   {
      HashMap<Class<?>, Class<?>> modifiableMap = new HashMap<>();

      modifiableMap.put(Point3DReadOnly.class, FramePoint3DReadOnly.class);
      modifiableMap.put(QuaternionReadOnly.class, FrameQuaternionReadOnly.class);
      modifiableMap.put(Vector3DReadOnly.class, FrameVector3DReadOnly.class);
      modifiableMap.put(Pose3DReadOnly.class, FramePose3DReadOnly.class);

      modifiableMap.put(Point3DBasics.class, FramePoint3DBasics.class);
      modifiableMap.put(QuaternionBasics.class, FrameQuaternionBasics.class);
      modifiableMap.put(Vector3DBasics.class, FrameVector3DBasics.class);
      modifiableMap.put(Pose3DBasics.class, FramePose3DBasics.class);

      // unchanged:
      modifiableMap.put(double.class, double.class);
      modifiableMap.put(int.class, int.class);
      modifiableMap.put(boolean.class, boolean.class);
      modifiableMap.put(TrajectoryPointBasics.class, TrajectoryPointBasics.class);

      returnTypeFramelessToFrameMap = Collections.unmodifiableMap(modifiableMap);
   }

   private final static Map<Class<?>, Class<?>> waypointToTrajectoryPointMap;
   static
   {
      HashMap<Class<?>, Class<?>> modifiableMap = new HashMap<>();

      modifiableMap.put(EuclideanWaypointBasics.class, EuclideanTrajectoryPointBasics.class);
      modifiableMap.put(SO3WaypointBasics.class, SO3TrajectoryPointBasics.class);
      modifiableMap.put(SE3WaypointBasics.class, SE3TrajectoryPointBasics.class);

      modifiableMap.put(FrameEuclideanWaypointBasics.class, FrameEuclideanTrajectoryPointBasics.class);
      modifiableMap.put(FrameSO3WaypointBasics.class, FrameSO3TrajectoryPointBasics.class);
      modifiableMap.put(FrameSE3WaypointBasics.class, FrameSE3TrajectoryPointBasics.class);

      // unchanged:
      modifiableMap.put(Point3DReadOnly.class, Point3DReadOnly.class);
      modifiableMap.put(QuaternionReadOnly.class, QuaternionReadOnly.class);
      modifiableMap.put(Vector3DReadOnly.class, Vector3DReadOnly.class);
      modifiableMap.put(Pose3DReadOnly.class, Pose3DReadOnly.class);

      modifiableMap.put(Point3DBasics.class, Point3DBasics.class);
      modifiableMap.put(QuaternionBasics.class, QuaternionBasics.class);
      modifiableMap.put(Vector3DBasics.class, Vector3DBasics.class);
      modifiableMap.put(Pose3DBasics.class, Pose3DBasics.class);

      modifiableMap.put(FramePoint3DBasics.class, FramePoint3DBasics.class);
      modifiableMap.put(FrameQuaternionBasics.class, FrameQuaternionBasics.class);
      modifiableMap.put(FrameVector3DBasics.class, FrameVector3DBasics.class);
      modifiableMap.put(FramePose3DBasics.class, FramePose3DBasics.class);

      modifiableMap.put(FixedFramePoint3DBasics.class, FixedFramePoint3DBasics.class);
      modifiableMap.put(FixedFrameQuaternionBasics.class, FixedFrameQuaternionBasics.class);
      modifiableMap.put(FixedFrameVector3DBasics.class, FixedFrameVector3DBasics.class);
      modifiableMap.put(FixedFramePose3DBasics.class, FixedFramePose3DBasics.class);

      modifiableMap.put(FramePoint3DReadOnly.class, FramePoint3DReadOnly.class);
      modifiableMap.put(FrameQuaternionReadOnly.class, FrameQuaternionReadOnly.class);
      modifiableMap.put(FrameVector3DReadOnly.class, FrameVector3DReadOnly.class);
      modifiableMap.put(FramePose3DReadOnly.class, FramePose3DReadOnly.class);

      modifiableMap.put(double.class, double.class);
      modifiableMap.put(boolean.class, boolean.class);
      modifiableMap.put(Transform.class, Transform.class);
      modifiableMap.put(ReferenceFrame.class, ReferenceFrame.class);
      modifiableMap.put(ReferenceFrameHolder.class, ReferenceFrameHolder.class);

      waypointToTrajectoryPointMap = Collections.unmodifiableMap(modifiableMap);
   }

   private final static Set<String> geometryMethodsToIgnore;
   static
   {
      Set<String> modifiableSet = new HashSet<>();

      modifiableSet.add("geometricallyEquals");
      modifiableSet.add("positionDistance");
      modifiableSet.add("orientationDistance");

      geometryMethodsToIgnore = Collections.unmodifiableSet(modifiableSet);
   }

   @Test
   public void testFrameOverwrites()
   {
      framelessTypesToFrameTypesTable.forEach((framelessClass, frameClass) -> {
         assertOverloadingWithFrameObjects(frameClass, framelessClass);
      });
   }

   @Test
   public void testTrajectoryOverwrites()
   {
      waypointTypesToTrajectoryTypesTable.forEach((framelessClass, frameClass) -> {
         assertOverloadingWithTrajectoryObjects(frameClass, framelessClass);
      });
   }

   private static void assertOverloadingWithFrameObjects(Class<?> frameClass, Class<?> framelessClass)
   {
      Arrays.asList(framelessClass.getMethods()).forEach(method -> {
         Class<?>[] expectedSignature = createExpectedSignature(method, argumentTypeFramelessToFrameMap);
         Class<?> expectedReturn = createExpectedReturn(method, returnTypeFramelessToFrameMap);
         asertMethodExists(frameClass, method.getName(), expectedSignature, expectedReturn);
      });
   }

   private static void assertOverloadingWithTrajectoryObjects(Class<?> frameClass, Class<?> framelessClass)
   {
      Arrays.asList(framelessClass.getMethods()).forEach(method -> {
         if (geometryMethodsToIgnore.contains(method.getName()))
         {
            return;
         }

         Class<?>[] expectedSignature = createExpectedSignature(method, waypointToTrajectoryPointMap);
         Class<?> expectedReturn = createExpectedReturn(method, waypointToTrajectoryPointMap);
         asertMethodExists(frameClass, method.getName(), expectedSignature, expectedReturn);
      });
   }

   private static void asertMethodExists(Class<?> clazz, String methodName, Class<?>[] signature, Class<?> returnType)
   {
      boolean foundMatch = Arrays.asList(clazz.getMethods()).stream().filter(method -> {
         if (!method.getName().equals(methodName))
         {
            return false;
         }
         Class<?>[] methodSignature = method.getParameterTypes();
         if (signature.length != methodSignature.length)
         {
            return false;
         }
         for (int i = 0; i < signature.length; i++)
         {
            if (signature[i] != methodSignature[i])
            {
               return false;
            }
         }
         if (method.getReturnType() != returnType)
         {
            return false;
         }
         return true;
      }).findFirst().isPresent();

      String methodString = createMethodString(methodName, signature, returnType);
      Assert.assertTrue("Did not find method \n" + methodString + "\nIn class " + clazz.getSimpleName(), foundMatch);
   }

   private static String createMethodString(String methodName, Class<?>[] signature, Class<?> returnType)
   {
      String ret = returnType.getSimpleName() + " " + methodName + "(";
      for (int i = 0; i < signature.length; i++)
      {
         ret += signature[i].getSimpleName();
         if (i < signature.length - 1)
         {
            ret += ", ";
         }
      }
      ret += ")";
      return ret;
   }

   private static Class<?>[] createExpectedSignature(Method method, Map<Class<?>, Class<?>> argumentTypeMap)
   {
      Class<?>[] parameterTypes = method.getParameterTypes();
      Class<?>[] ret = new Class<?>[parameterTypes.length];

      for (int i = 0; i < parameterTypes.length; i++)
      {
         Class<?> parameterType = parameterTypes[i];
         if (!argumentTypeMap.containsKey(parameterType))
         {
            throw new RuntimeException("Please add a matching type for " + parameterType.getSimpleName() + " to argument types.");
         }
         ret[i] = argumentTypeMap.get(parameterType);
      }

      return ret;
   }

   private static Class<?> createExpectedReturn(Method method, Map<Class<?>, Class<?>> returnTypeMap)
   {
      Class<?> returnType = method.getReturnType();
      if (returnType == void.class)
      {
         return void.class;
      }
      if (!returnTypeMap.containsKey(returnType))
      {
         throw new RuntimeException("Please add a matching type for " + returnType.getSimpleName() + " to return types.");
      }
      return returnTypeMap.get(returnType);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator mutationTestFacilitator = new MutationTestFacilitator();

      mutationTestFacilitator.addClassesToMutate(EuclideanWaypoint.class);
      mutationTestFacilitator.addClassesToMutate(EuclideanWaypoint.class);
      mutationTestFacilitator.addClassesToMutate(EuclideanWaypoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameEuclideanWaypoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameEuclideanWaypoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameEuclideanWaypoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameEuclideanWaypoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameEuclideanWaypoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameEuclideanWaypoint.class);
      mutationTestFacilitator.addClassesToMutate(EuclideanWaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(EuclideanWaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(EuclideanWaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameEuclideanWaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameEuclideanWaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameEuclideanWaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(SO3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(SO3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(SO3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameSO3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameSO3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameSO3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameSO3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameSO3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameSO3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(SO3WaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(SO3WaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(SO3WaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameSO3WaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameSO3WaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameSO3WaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(SE3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(SE3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(SE3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameSE3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameSE3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameSE3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameSE3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameSE3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameSE3Waypoint.class);
      mutationTestFacilitator.addClassesToMutate(SE3WaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(SE3WaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(SE3WaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameSE3WaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameSE3WaypointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameSE3WaypointBasics.class);

      mutationTestFacilitator.addClassesToMutate(EuclideanTrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(EuclideanTrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(EuclideanTrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameEuclideanTrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameEuclideanTrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameEuclideanTrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameEuclideanTrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameEuclideanTrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameEuclideanTrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(EuclideanTrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(EuclideanTrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(EuclideanTrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameEuclideanTrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameEuclideanTrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameEuclideanTrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(SO3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(SO3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(SO3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameSO3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameSO3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameSO3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameSO3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameSO3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameSO3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(SO3TrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(SO3TrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(SO3TrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameSO3TrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameSO3TrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameSO3TrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(SE3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(SE3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(SE3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameSE3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameSE3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(FrameSE3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameSE3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameSE3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(YoFrameSE3TrajectoryPoint.class);
      mutationTestFacilitator.addClassesToMutate(SE3TrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(SE3TrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(SE3TrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameSE3TrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameSE3TrajectoryPointBasics.class);
      mutationTestFacilitator.addClassesToMutate(FrameSE3TrajectoryPointBasics.class);

      mutationTestFacilitator.addTestClassesToRun(FrameEuclideanTrajectoryPointTest.class);
      mutationTestFacilitator.addTestClassesToRun(FrameSE3TrajectoryPointTest.class);
      mutationTestFacilitator.addTestClassesToRun(FrameSO3TrajectoryPointTest.class);
      mutationTestFacilitator.addTestClassesToRun(MultipleWaypointsOrientationTrajectoryGeneratorTest.class);
      mutationTestFacilitator.addTestClassesToRun(MultipleWaypointsPositionTrajectoryGeneratorTest.class);
      mutationTestFacilitator.addTestClassesToRun(MultipleWaypointsTrajectoryGeneratorTest.class);
      mutationTestFacilitator.addTestClassesToRun(SimpleEuclideanTrajectoryPointTest.class);
      mutationTestFacilitator.addTestClassesToRun(SimpleSE3TrajectoryPointTest.class);
      mutationTestFacilitator.addTestClassesToRun(SimpleSO3TrajectoryPointTest.class);
      mutationTestFacilitator.addTestClassesToRun(TrajectoryPointOptimizerTest.class);
      mutationTestFacilitator.addTestClassesToRun(YoFrameEuclideanTrajectoryPointTest.class);
      mutationTestFacilitator.addTestClassesToRun(YoFrameSE3TrajectoryPointTest.class);
      mutationTestFacilitator.addTestClassesToRun(YoFrameSO3TrajectoryPointTest.class);
      mutationTestFacilitator.addTestClassesToRun(TrajectoryWaypointAPITest.class);

      mutationTestFacilitator.doMutationTest();
      mutationTestFacilitator.openResultInBrowser();
   }
}
