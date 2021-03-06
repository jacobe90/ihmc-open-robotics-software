package us.ihmc.sensorProcessing.encoder.processors;


import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class AnotherEncoderProcessor extends AbstractEncoderProcessor
{
   private double previousTime;
   private int previousPosition;
   private boolean previousTickUp;

   public AnotherEncoderProcessor(String name, YoInteger rawPosition, YoDouble time, double distancePerTick, YoRegistry registry)
   {
      super(name, rawPosition, time, distancePerTick, registry);
   }

   public void update()
   {
      double time = this.time.getDoubleValue();
      int position = this.rawTicks.getIntegerValue();

      boolean thisTickUp;

      if (position == previousPosition)
      {
         this.processedTickRate.set((position - previousPosition) / (time - previousTime));

         return;
      }

      thisTickUp = position > previousPosition;

      if (thisTickUp == previousTickUp)
      {
         this.processedTicks.set(rawTicks.getIntegerValue());
         this.processedTickRate.set((position - previousPosition) / (time - previousTime));
      }

      else
      {
         this.processedTickRate.set(0.0);
      }

      previousPosition = position;
      previousTime = time;
      previousTickUp = thisTickUp;
   }

   public void initialize()
   {
      // empty
   }
}
