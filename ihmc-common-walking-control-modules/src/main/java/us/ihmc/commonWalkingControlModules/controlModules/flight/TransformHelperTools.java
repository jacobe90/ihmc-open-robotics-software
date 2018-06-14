package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;

/**
 * Some functions to ease transformation between reference frames and poses 
 * <p> Uses a static {@code RigidBodyTransform} to facilitate transformations rather than have 
 * various objects littered across different objects </p>
 * @author Apoorv S
 *
 */
public class TransformHelperTools
{
   private static final RigidBodyTransform transform = new RigidBodyTransform();
   private static final RigidBodyTransform transform2 = new RigidBodyTransform();

   // Hidden constructor to ensure no instantiation 
   private TransformHelperTools()
   {
   }

   public static void transformFromPoseToPose(FramePose2DReadOnly poseToTransformFrom, FramePose2DReadOnly poseToTransformTo,
                                              ConvexPolygon2D polygonToTransform)
   {
      transformFromPoseToReferenceFrame(poseToTransformFrom, polygonToTransform);
      poseToTransformFrom.getReferenceFrame().getTransformToDesiredFrame(transform, poseToTransformTo.getReferenceFrame());
      polygonToTransform.applyTransform(transform);
      transformFromReferenceFrameToPose(poseToTransformTo, polygonToTransform);
   }

   public static void transformFromPoseToReferenceFrame(FramePose2DReadOnly poseToTransformFrom, ConvexPolygon2D polygonToTransform)
   {
      poseToTransformFrom.get(transform);
      polygonToTransform.applyTransform(transform);
   }

   public static void transformFromReferenceFrameToPose(FramePose2DReadOnly poseToTransformTo, ConvexPolygon2D polygonToTransform)
   {
      poseToTransformTo.get(transform);
      polygonToTransform.applyInverseTransform(transform);
   }

   public static void transformFromPoseToPose(FramePose2DReadOnly poseToTransformFrom, FramePose2DReadOnly poseToTransformTo, Point2D pointToTransform)
   {
      transformFromPoseToReferenceFrame(poseToTransformFrom, pointToTransform);
      poseToTransformFrom.getReferenceFrame().getTransformToDesiredFrame(transform, poseToTransformTo.getReferenceFrame());
      pointToTransform.applyTransform(transform);
      transformFromReferenceFrameToPose(poseToTransformTo, pointToTransform);
   }

   public static void transformFromPoseToReferenceFrame(FramePose2DReadOnly poseToTransformFrom, Point2D pointToTransform)
   {
      poseToTransformFrom.get(transform);
      pointToTransform.applyTransform(transform);
   }

   public static void transformFromReferenceFrameToPose(FramePose2DReadOnly poseToTransformTo, Point2D pointToTransform)
   {
      poseToTransformTo.get(transform);
      pointToTransform.applyInverseTransform(transform);
   }

   public static void transformFromPoseToReferenceFrameByProjection(FramePose3DReadOnly poseToTransformFrom, ConvexPolygon2D polygonToTransform)
   {
      poseToTransformFrom.get(transform);
      polygonToTransform.applyTransform(transform, false);
   }

   public static void transformFromPoseToReferenceFrameByProjection(FramePose3DReadOnly poseToTransformFrom, FrameConvexPolygon2D polygonToTransform)
   {
      polygonToTransform.checkReferenceFrameMatch(polygonToTransform);
      poseToTransformFrom.get(transform);
      polygonToTransform.applyTransform(transform, false);
   }

   public static void transformFromReferenceFrameToPoseByProjection(FramePose3DReadOnly poseToTransformTo, ConvexPolygon2D polygonToTransform)
   {
      poseToTransformTo.get(transform);
      transform.invert();
      polygonToTransform.applyTransform(transform, false);
   }

   public static void transformFromPoseToReferenceFrame(FramePose3DReadOnly poseToTranformFrom, FramePoint3D framePointToTranform)
   {
      poseToTranformFrom.checkReferenceFrameMatch(framePointToTranform);
      poseToTranformFrom.get(transform);
      framePointToTranform.applyTransform(transform);
   }

   public static void transformFromReferenceFrameToPose(FramePose3DReadOnly poseToTransformTo, FramePoint3D framePointToTransform)
   {
      poseToTransformTo.checkReferenceFrameMatch(framePointToTransform);
      poseToTransformTo.get(transform);
      framePointToTransform.applyInverseTransform(transform);
   }

   public static void transformFromReferenceFrameToReferenceFrame(ReferenceFrame referenceFrameToTransformFrom, ReferenceFrame referenceFrameToTransformTo,
                                                                  ConvexPolygon2D polygonToTransform)
   {
      referenceFrameToTransformFrom.getTransformToDesiredFrame(transform, referenceFrameToTransformTo);
      polygonToTransform.applyTransform(transform);
   }

   public static void transformFromPoseToReferenceFrameByProjection(FramePose3DReadOnly poseToTransforFrom, FramePoint2D framePointToTransform)
   {
      poseToTransforFrom.get(transform);
      framePointToTransform.applyTransform(transform, false);
   }

   public static void transformFromPoseToReferenceFrameByProjection(FramePose3DReadOnly poseToTransformFrom, Point2D pointToTransform)
   {
      poseToTransformFrom.get(transform);
      pointToTransform.applyTransform(transform, false);
   }

   public static void transformFromReferenceFrameToReferenceFrameByProjection(ReferenceFrame referenceFrameToTransformFrom,
                                                                              ReferenceFrame referenceFrameToTransformTo, Point2D pointToTransform)
   {
      referenceFrameToTransformFrom.getTransformToDesiredFrame(transform, referenceFrameToTransformTo);
      pointToTransform.applyTransform(transform, false);
   }

   public static void transformFromPoseToReferenceFrameByProjection(FramePose3DReadOnly poseToTransformFrom, ReferenceFrame referenceFrameToTransformTo,
                                                        Point2D pointToTransform)
   {
      poseToTransformFrom.get(transform);
      poseToTransformFrom.getReferenceFrame().getTransformToDesiredFrame(transform2, referenceFrameToTransformTo);
      transform.preMultiply(transform2);
      transform.transform(pointToTransform, false);
   }
}
