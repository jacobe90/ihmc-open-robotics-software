package us.ihmc.simulationConstructionSetTools.util.inputdevices;

import javax.swing.JSlider;
import javax.swing.JTextField;

import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;

public class JSliderControl extends JSlider implements CloseableAndDisposable
{
   private static final long serialVersionUID = 8570638563928914747L;
   
   // TODO: Make all these fields private and add getters and setters appropriately.

   protected boolean changeLock = false;
   protected boolean updatedRemotly = false;

   protected JTextField value;
   protected MidiControl midiControl;
   protected int sliderMax;

   public JSliderControl(int orientation, int min, int max, int value, MidiControl midiControl, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      super(orientation, min, max, value);
      this.midiControl = midiControl;
      setSnapToTicks(true);

      if (midiControl.sliderType == MidiControl.SliderType.BOOLEAN)
      {
         sliderMax = 1;

      }
      else if (midiControl.sliderType == MidiControl.SliderType.ENUM)
      {
         sliderMax = ((YoEnum<?>) midiControl.var).getEnumValues().length - 1;
      }
      else
         sliderMax = 127;

      closeableAndDisposableRegistry.registerCloseableAndDisposable(this);
   }

   public MidiControl getControl()
   {
      return midiControl;
   }

   synchronized public void lock()
   {
      if (!changeLock)
      {
         Thread timer = new Thread(new Runnable()
         {
            @Override
            public void run()
            {
               changeLock = true;

               try
               {
                  Thread.sleep(1000);
               }
               catch (InterruptedException e)
               {
               }

               changeLock = false;
            }
         }, "JSliderControl");
         timer.start();
      }
   }

   @Override
   public void closeAndDispose()
   {
      midiControl = null;
   }
}
