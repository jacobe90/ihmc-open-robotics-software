package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.IhmcSLAMFrame;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.Scan;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;

public class IhmcSLAMTools
{
   public static Point3D[] extractPointsFromMessage(StereoVisionPointCloudMessage message)
   {
      int numberOfPoints = message.getColors().size();
      Point3D[] pointCloud = new Point3D[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         pointCloud[i] = new Point3D();
         MessageTools.unpackScanPoint(message, i, pointCloud[i]);
      }
      return pointCloud;
   }

   public static RigidBodyTransform extractSensorPoseFromMessage(StereoVisionPointCloudMessage message)
   {
      return new RigidBodyTransform(message.getSensorOrientation(), message.getSensorPosition());
   }

   public static Scan toScan(Point3DReadOnly[] points, Tuple3DReadOnly sensorPosition)
   {
      PointCloud pointCloud = new PointCloud();

      for (int i = 0; i < points.length; i++)
      {
         double x = points[i].getX();
         double y = points[i].getY();
         double z = points[i].getZ();
         pointCloud.add(x, y, z);
      }
      return new Scan(new Point3D(sensorPosition), pointCloud);
   }

   public static Point3D[] createConvertedPointsToSensorPose(RigidBodyTransformReadOnly sensorPose, Point3DReadOnly[] pointCloud)
   {
      Point3D[] convertedPoints = new Point3D[pointCloud.length];
      RigidBodyTransform inverseTransformer = new RigidBodyTransform(sensorPose);
      inverseTransformer.invert();
      Point3D convertedPoint = new Point3D();
      for (int i = 0; i < pointCloud.length; i++)
      {
         inverseTransformer.transform(pointCloud[i], convertedPoint);
         convertedPoints[i] = new Point3D(convertedPoint);
      }

      return convertedPoints;
   }

   public static Point3D[] createConvertedPointsToOtherSensorPose(RigidBodyTransformReadOnly sensorPose, RigidBodyTransformReadOnly otherSensorPose,
                                                                  Point3DReadOnly[] pointCloudToSensorPose)
   {
      Point3D[] pointCloudToWorld = new Point3D[pointCloudToSensorPose.length];
      for (int i = 0; i < pointCloudToWorld.length; i++)
      {
         pointCloudToWorld[i] = new Point3D(pointCloudToSensorPose[i]);
         sensorPose.transform(pointCloudToWorld[i]);
      }

      return createConvertedPointsToSensorPose(otherSensorPose, pointCloudToWorld);
   }

   public static Point3D[] createConvertedPointsToWorld(RigidBodyTransformReadOnly otherSensorPose, Point3DReadOnly[] pointCloudToSensorPose)
   {
      Point3D[] pointCloudToWorld = new Point3D[pointCloudToSensorPose.length];
      for (int i = 0; i < pointCloudToWorld.length; i++)
      {
         pointCloudToWorld[i] = new Point3D(pointCloudToSensorPose[i]);
         otherSensorPose.transform(pointCloudToWorld[i]);
      }

      return pointCloudToWorld;
   }

   public static NormalOcTree computeOctreeData(Point3DReadOnly[] pointCloud, Tuple3DReadOnly sensorPosition, double octreeResolution)
   {
      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = pointCloud.length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(toScan(pointCloud, sensorPosition));

      NormalOcTree referenceOctree = new NormalOcTree(octreeResolution);

      referenceOctree.insertScanCollection(scanCollection, false);

      referenceOctree.enableParallelComputationForNormals(true);
      referenceOctree.enableParallelInsertionOfMisses(true);
      referenceOctree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      referenceOctree.setNormalEstimationParameters(normalEstimationParameters);

      referenceOctree.updateNormals();
      return referenceOctree;
   }

   public static NormalOcTree computeOctreeData(List<Point3DReadOnly[]> pointCloudMap, List<RigidBodyTransformReadOnly> sensorPoses, double octreeResolution)
   {
      ScanCollection scanCollection = new ScanCollection();
      for (int i = 0; i < pointCloudMap.size(); i++)
      {
         int numberOfPoints = pointCloudMap.get(i).length;

         scanCollection.setSubSampleSize(numberOfPoints);
         scanCollection.addScan(toScan(pointCloudMap.get(i), sensorPoses.get(i).getTranslation()));
      }

      NormalOcTree referenceOctree = new NormalOcTree(octreeResolution);

      referenceOctree.insertScanCollection(scanCollection, false);

      referenceOctree.enableParallelComputationForNormals(true);
      referenceOctree.enableParallelInsertionOfMisses(true);
      referenceOctree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      referenceOctree.setNormalEstimationParameters(normalEstimationParameters);

      referenceOctree.updateNormals();
      return referenceOctree;
   }

   public static List<PlanarRegionSegmentationRawData> computePlanarRegionRawData(List<Point3DReadOnly[]> pointCloudMap,
                                                                                  List<RigidBodyTransformReadOnly> sensorPoses, double octreeResolution,
                                                                                  PlanarRegionSegmentationParameters planarRegionSegmentationParameters)
   {
      NormalOcTree referenceOctree = computeOctreeData(pointCloudMap, sensorPoses, octreeResolution);

      PlanarRegionSegmentationCalculator segmentationCalculator = new PlanarRegionSegmentationCalculator();

      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(false);

      segmentationCalculator.setParameters(planarRegionSegmentationParameters);
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);
      segmentationCalculator.setSensorPosition(new Point3D(0.0, 0.0, 20.0)); //TODO: work this for every poses.

      segmentationCalculator.compute(referenceOctree.getRoot());

      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();

      return rawData;
   }

   public static List<PlanarRegionSegmentationRawData> computePlanarRegionRawData(Point3DReadOnly[] pointCloud, Tuple3DReadOnly sensorPosition,
                                                                                  double octreeResolution,
                                                                                  PlanarRegionSegmentationParameters planarRegionSegmentationParameters)
   {
      return computePlanarRegionRawData(pointCloud, sensorPosition, octreeResolution, planarRegionSegmentationParameters, true);
   }

   public static List<PlanarRegionSegmentationRawData> computePlanarRegionRawData(Point3DReadOnly[] pointCloud, Tuple3DReadOnly sensorPosition,
                                                                                  double octreeResolution,
                                                                                  PlanarRegionSegmentationParameters planarRegionSegmentationParameters,
                                                                                  boolean useSurfaceNormalFilter)
   {
      NormalOcTree referenceOctree = computeOctreeData(pointCloud, sensorPosition, octreeResolution);

      PlanarRegionSegmentationCalculator segmentationCalculator = new PlanarRegionSegmentationCalculator();

      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(useSurfaceNormalFilter);

      segmentationCalculator.setParameters(planarRegionSegmentationParameters);
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);
      segmentationCalculator.setSensorPosition(sensorPosition);

      segmentationCalculator.compute(referenceOctree.getRoot());

      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();

      return rawData;
   }

   public static List<Point3D> closestOctreePoints = new ArrayList<>();

   public static double computeDistanceToNormalOctree(NormalOcTree octree, Point3DReadOnly point, int maximumSearchingSize)
   {
      OcTreeKey occupiedKey = octree.coordinateToKey(point);

      Point3D closestPoint = new Point3D();
      double minDistance = -1.0;

      if (octree.search(occupiedKey) != null)
      {
         return 0.0;
      }

      for (int searchingSize = 1; searchingSize < maximumSearchingSize + 1; searchingSize++)
      {
         OcTreeKey[] candidateKeys = createCandidateOctreeKey(occupiedKey, searchingSize);
         for (int i = 0; i < candidateKeys.length; i++)
         {
            OcTreeKey candidateKey = candidateKeys[i];

            NormalOcTreeNode searchNode = octree.search(candidateKey);
            if (searchNode != null)
            {
               if (minDistance < 0.0)
                  minDistance = Double.MAX_VALUE;

               Point3D candidatePoint = new Point3D(searchNode.getX(), searchNode.getY(), searchNode.getZ());

               double distance = candidatePoint.distance(point);
               if (distance < minDistance)
               {
                  minDistance = distance;
                  closestPoint.set(candidatePoint);
               }
            }
            else
            {
               continue;
            }
         }
         if (minDistance > 0)
         {
            break;
         }
      }

      closestOctreePoints.add(closestPoint);

      return minDistance;
   }

   private static OcTreeKey[] createCandidateOctreeKey(OcTreeKey key, int searchingSize)
   {
      int outerSize = 2 * searchingSize + 1;
      int innerSize = 2 * searchingSize - 1;
      int numberOfCandidates = outerSize * outerSize * outerSize - innerSize * innerSize * innerSize;
      OcTreeKey[] candidates = new OcTreeKey[numberOfCandidates];

      int index = 0;
      for (int i = 0; i < outerSize; i++)
      {
         for (int j = 0; j < outerSize; j++)
         {
            for (int k = 0; k < outerSize; k++)
            {
               if (i == 0 || i == outerSize - 1)
               {
                  OcTreeKey candidate = new OcTreeKey();
                  candidate.setKey(0, key.getKey(0) - searchingSize + i);
                  candidate.setKey(1, key.getKey(1) - searchingSize + j);
                  candidate.setKey(2, key.getKey(2) - searchingSize + k);
                  candidates[index] = candidate;
                  index++;
               }
               else
               {
                  if (j == 0 || j == outerSize - 1 || k == 0 || k == outerSize - 1)
                  {
                     OcTreeKey candidate = new OcTreeKey();
                     candidate.setKey(0, key.getKey(0) - searchingSize + i);
                     candidate.setKey(1, key.getKey(1) - searchingSize + j);
                     candidate.setKey(2, key.getKey(2) - searchingSize + k);
                     candidates[index] = candidate;
                     index++;
                  }
               }
            }
         }
      }

      return candidates;
   }

   private static double computeDistanceToNormalOctreeNode(NormalOcTreeNode searchNode, Point3DReadOnly point)
   {
      Plane3D plane = new Plane3D(searchNode.getHitLocationCopy(), searchNode.getNormalCopy());

      return plane.distance(point);
   }

   /**
    * if there is not enough source points, think this frame is key frame.
    * return null.
    */
   public static Point3D[] createSourcePointsToSensorPose(IhmcSLAMFrame frame, int numberOfSourcePoints, ConvexPolygon2D previousWindow, double windowDepth,
                                                          double minimumOverlappedRatio)
   {
      IhmcSLAMFrame previousFrame = frame.getPreviousFrame();

      //List<Point3DReadOnly> pointsOutOfPreviousWindow = new ArrayList<>();
      List<Point3DReadOnly> pointsInPreviousWindow = new ArrayList<>();

      RigidBodyTransformReadOnly previousSensorPoseToWorld = previousFrame.getSensorPose();
      Point3DReadOnly[] originalPointCloudToSensorPose = frame.getOriginalPointCloudToSensorPose();
      Point3DReadOnly[] convertedPointsToPreviousSensorPose = IhmcSLAMTools.createConvertedPointsToOtherSensorPose(frame.getInitialSensorPoseToWorld(),
                                                                                                                   previousSensorPoseToWorld,
                                                                                                                   originalPointCloudToSensorPose);

      int numberOfPointsInPreviousView = 0;
      for (int i = 0; i < convertedPointsToPreviousSensorPose.length; i++)
      {
         Point3DReadOnly pointInPreviousView = convertedPointsToPreviousSensorPose[i];
         if (previousWindow.isPointInside(pointInPreviousView.getX(), pointInPreviousView.getY()) && pointInPreviousView.getZ() > windowDepth)
         {
            pointsInPreviousWindow.add(originalPointCloudToSensorPose[i]);
            numberOfPointsInPreviousView++;
         }
         else
         {
            //pointsOutOfPreviousWindow.add(originalPointCloudToSensorPose[i]);
         }
      }
      double overlappedRatio = (double) numberOfPointsInPreviousView / convertedPointsToPreviousSensorPose.length;
      if (overlappedRatio < minimumOverlappedRatio)
      {
         return null;
      }
      if (pointsInPreviousWindow.size() < numberOfSourcePoints)
      {
         return null;
      }

      TIntArrayList indexOfSourcePoints = new TIntArrayList();
      int index = 0;
      Point3D[] sourcePoints = new Point3D[numberOfSourcePoints];
      Random randomSelector = new Random(0612L);
      while (indexOfSourcePoints.size() != numberOfSourcePoints)
      {
         int selectedIndex = randomSelector.nextInt(pointsInPreviousWindow.size());
         if (!indexOfSourcePoints.contains(selectedIndex))
         {
            Point3DReadOnly selectedPoint = pointsInPreviousWindow.get(selectedIndex);
            sourcePoints[index] = new Point3D(selectedPoint);
            index++;
            indexOfSourcePoints.add(selectedIndex);
         }
      }

      System.out.println(convertedPointsToPreviousSensorPose.length + " " + numberOfPointsInPreviousView + " " + overlappedRatio);

      return sourcePoints;
   }

   public static int countNumberOfInliers(NormalOcTree octree, RigidBodyTransformReadOnly sensorPoseToWorld, Point3DReadOnly[] sourcePointsToSensor,
                                          int maximumSearchingSize)
   {
      int numberOfInliers = 0;
      Point3D newSourcePointToWorld = new Point3D();
      for (Point3DReadOnly sourcePoint : sourcePointsToSensor)
      {
         newSourcePointToWorld.set(sourcePoint);
         sensorPoseToWorld.transform(newSourcePointToWorld);

         double distance = IhmcSLAMTools.computeDistanceToNormalOctree(octree, newSourcePointToWorld, maximumSearchingSize);

         if (distance >= 0)
         {
            if (distance < octree.getResolution())
            {
               numberOfInliers++;
            }
         }
      }
      return numberOfInliers;
   }
}
