package us.ihmc.robotEnvironmentAwareness.planarRegion;

import us.ihmc.jOctoMap.tools.ScannerTools;

import java.util.Scanner;

public class SurfaceNormalFilterParameters
{
   private boolean useSurfaceNormalFilter;
   private double surfaceNormalUpperBound;
   private double surfaceNormalLowerBound;
   private static final double DEFAULT_LOWER_BOUND_DEGREE = -45.0;
   private static final double DEFAULT_UPPER_BOUND_DEGREE = 45.0;
   
   public SurfaceNormalFilterParameters()
   {
      setDefaultParameters();
   }
   
   public SurfaceNormalFilterParameters(SurfaceNormalFilterParameters other)
   {
      set(other);
   }
   
   public void set(SurfaceNormalFilterParameters other)
   {
      useSurfaceNormalFilter = other.useSurfaceNormalFilter;
      surfaceNormalUpperBound = other.surfaceNormalUpperBound;
      surfaceNormalLowerBound = other.surfaceNormalLowerBound;
   }
   
   public void setDefaultParameters()
   {
      useSurfaceNormalFilter = true;
      surfaceNormalUpperBound = Math.toRadians(DEFAULT_UPPER_BOUND_DEGREE);
      surfaceNormalLowerBound = Math.toRadians(DEFAULT_LOWER_BOUND_DEGREE);
   }

   public boolean isUseSurfaceNormalFilter()
   {
      return useSurfaceNormalFilter;
   }

   public double getSurfaceNormalUpperBound()
   {
      return surfaceNormalUpperBound;
   }

   public double getSurfaceNormalLowerBound()
   {
      return surfaceNormalLowerBound;
   }

   public void setUseSurfaceNormalFilter(boolean useSurfaceNormalFilter)
   {
      this.useSurfaceNormalFilter = useSurfaceNormalFilter;
   }

   public void setSurfaceNormalUpperBound(double surfaceNormalUpperBound)
   {
      this.surfaceNormalUpperBound = surfaceNormalUpperBound;
   }

   public void setSurfaceNormalLowerBound(double surfaceNormalLowerBound)
   {
      this.surfaceNormalLowerBound = surfaceNormalLowerBound;
   }

   @Override
   public String toString()
   {
      return "surfaceNormalUpperBound: " + surfaceNormalUpperBound + ", surfaceNormalLowerBound: " + surfaceNormalLowerBound + ", useSurfaceNormalFilter: " + useSurfaceNormalFilter;
   }

   public static SurfaceNormalFilterParameters parse(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);

      SurfaceNormalFilterParameters parameters = new SurfaceNormalFilterParameters();
      parameters.setSurfaceNormalUpperBound(ScannerTools.readNextDouble(scanner, parameters.getSurfaceNormalUpperBound()));
      parameters.setSurfaceNormalLowerBound(ScannerTools.readNextDouble(scanner, parameters.getSurfaceNormalLowerBound()));
      parameters.setUseSurfaceNormalFilter(ScannerTools.readNextBoolean(scanner, parameters.useSurfaceNormalFilter));
      scanner.close();
      return parameters;
   }
}
