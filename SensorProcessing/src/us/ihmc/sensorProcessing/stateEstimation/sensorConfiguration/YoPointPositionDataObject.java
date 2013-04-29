package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import javax.vecmath.Point3d;
import java.lang.ref.Reference;

/**
 * @author twan
 *         Date: 4/27/13
 */
public class YoPointPositionDataObject extends PointPositionDataObject
{
   private final YoFramePoint yoMeasurementPointInBodyFrame;
   private final YoFramePoint yoMeasurementPointInWorldFrame;
   private final BooleanYoVariable isValid;

   public YoPointPositionDataObject(String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      yoMeasurementPointInBodyFrame = new YoFramePoint(namePrefix + "PointBody", frame, registry);
      yoMeasurementPointInWorldFrame = new YoFramePoint(namePrefix + "PointWorld", ReferenceFrame.getWorldFrame(), registry);
      isValid = new BooleanYoVariable(namePrefix + "IsValid", registry);
   }

   @Override
   public void set(FramePoint measurementPointInBodyFrame, FramePoint positionOfMeasurementPointInWorldFrame)
   {
      this.yoMeasurementPointInBodyFrame.set(measurementPointInBodyFrame);
      this.yoMeasurementPointInWorldFrame.set(positionOfMeasurementPointInWorldFrame);
   }

   @Override
   public FramePoint getMeasurementPointInWorldFrame()
   {
      yoMeasurementPointInWorldFrame.getFramePointAndChangeFrameOfPackedPoint(positionOfMeasurementPointInWorldFrame);

      return positionOfMeasurementPointInWorldFrame;
   }

   @Override
   public FramePoint getMeasurementPointInBodyFrame()
   {
      yoMeasurementPointInBodyFrame.getFramePointAndChangeFrameOfPackedPoint(measurementPointInBodyFrame);

      return measurementPointInBodyFrame;
   }

   @Override
   public void set(PointPositionDataObject other)
   {
      set(other.getMeasurementPointInBodyFrame(), other.getMeasurementPointInWorldFrame());
   }

   public boolean isValid()
   {
      return isValid.getBooleanValue();
   }

   public void setValid(boolean valid)
   {
      this.isValid.set(valid);
   }
}
