package us.ihmc.robotics.math;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoSolvePseudoInverseSVDWithDampedLeastSquaresNearSingularities implements LinearSolverDense<DMatrixRMaj>
{
   private final SingularValueDecomposition_F64<DMatrixRMaj> svd;
   private final DMatrixRMaj pseudoInverse = new DMatrixRMaj(1, 1);

   private final YoRegistry registry;
   private final YoDouble mu;
   private final YoDouble firstSingularValueThreshold;
   private final YoDouble secondSingularValueThreshold;
   private final YoDouble singularValueAlpha;
   private final YoDouble yoMinSingularValue;
   private final YoDouble[] yoSingularValues;
   private final YoDouble[] yoSingularValuesInverse;

   private final DMatrixRMaj tempV;
   
   public YoSolvePseudoInverseSVDWithDampedLeastSquaresNearSingularities(String namePrefix, int maxRows, int maxCols, YoRegistry parentRegistry)
   {
      svd = DecompositionFactory_DDRM.svd(maxRows, maxCols, true, true, true);
      tempV = new DMatrixRMaj(maxCols, maxCols);

      registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      mu = new YoDouble(namePrefix + "Mu", registry);
      firstSingularValueThreshold = new YoDouble(namePrefix + "FirstSingularValueThreshold", registry);
      secondSingularValueThreshold = new YoDouble(namePrefix + "SecondSingularValueThreshold", registry);
      singularValueAlpha = new YoDouble(namePrefix + "SingularValueAlpha", registry);
      yoMinSingularValue = new YoDouble(namePrefix + "MinSingularValue", registry);

      mu.set(0.003);
      firstSingularValueThreshold.set(5.0e-3);
      secondSingularValueThreshold.set(1.0e-5);

      yoSingularValues = new YoDouble[Math.max(maxRows, maxCols)];
      yoSingularValuesInverse = new YoDouble[Math.max(maxRows, maxCols)];

      for (int i = 0; i < yoSingularValues.length; i++)
      {
         yoSingularValues[i] = new YoDouble(namePrefix + "SingularValue_" + i, registry);
         yoSingularValues[i].set(Double.NaN);
      }

      for (int i = 0; i < yoSingularValuesInverse.length; i++)
      {
         yoSingularValuesInverse[i] = new YoDouble(namePrefix + "SingularValueInverse_" + i, registry);
         yoSingularValuesInverse[i].set(Double.NaN);
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void setThresholds(double firstThreshold, double secondThreshold)
   {
      firstSingularValueThreshold.set(firstThreshold);
      secondSingularValueThreshold.set(secondThreshold);
   }

   public void setDampedLeastSquaresMu(double mu)
   {
      this.mu.set(mu);
   }

   private double alpha = 1.0;

   @Override
   public boolean setA(DMatrixRMaj A)
   {
      pseudoInverse.reshape(A.numCols, A.numRows, false);
      tempV.reshape(A.numCols, A.numRows, false);

      if (!svd.decompose(A))
         return false;

      DMatrixRMaj U_t = svd.getU(null, true);
      DMatrixRMaj V = svd.getV(tempV, false);
      double[] S = svd.getSingularValues();
      int N = Math.min(A.numRows, A.numCols);

      double minSingular = Double.POSITIVE_INFINITY;
      for (int i = 0; i < N; i++)
      {
         yoSingularValues[i].set(S[i]);

         if (S[i] < minSingular)
            minSingular = S[i];
      }

      yoMinSingularValue.set(minSingular);
      double deltaThresholds = firstSingularValueThreshold.getDoubleValue() - secondSingularValueThreshold.getDoubleValue();
      double muSquare = mu.getDoubleValue() * mu.getDoubleValue();

      alpha = 1.0;
      if (minSingular < secondSingularValueThreshold.getDoubleValue())
      {
         alpha = 0.0;
      }
      else if (minSingular < firstSingularValueThreshold.getDoubleValue())
      {
         alpha = (minSingular - secondSingularValueThreshold.getDoubleValue()) / deltaThresholds;
      }
      alpha = MathTools.clamp(alpha, 0.0, 1.0);
      alpha *= alpha;
      
      singularValueAlpha.set(alpha);

      // computer the pseudo inverse of A
      for (int i = 0; i < N; i++)
      {
         double s = S[i];

         if (minSingular <= secondSingularValueThreshold.getDoubleValue())
         {
            S[i] = s / (s * s + muSquare);
         }
         else
         {
            S[i] = alpha / s + (1.0 - alpha) * s / (s * s + muSquare);
         }

         yoSingularValuesInverse[i].set(S[i]);
      }

      // V*W
      for (int i = 0; i < V.numRows; i++)
      {
         int index = i * V.numCols;
         for (int j = 0; j < V.numCols; j++)
         {
            V.data[index++] *= S[j];
         }
      }

      // V*W*U^T
      CommonOps_DDRM.mult(V, U_t, pseudoInverse);

      return true;
   }

   @Override
   public double quality()
   {
      return alpha;
   }

   @Override
   public void solve(DMatrixRMaj b, DMatrixRMaj x)
   {
      CommonOps_DDRM.mult(pseudoInverse, b, x);
   }

   @Override
   public void invert(DMatrixRMaj A_inv)
   {
      A_inv.set(pseudoInverse);
   }

   @Override
   public boolean modifiesA()
   {
      return svd.inputModified();
   }

   @Override
   public boolean modifiesB()
   {
      return false;
   }

   @Override
   public SingularValueDecomposition_F64<DMatrixRMaj> getDecomposition()
   {
      return svd;
   }
}
