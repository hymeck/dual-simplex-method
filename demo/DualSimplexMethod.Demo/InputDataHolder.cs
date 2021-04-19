using System.Collections.Generic;

namespace DualSimplexMethod.Demo
{
    public record InputDataHolder(
        double[,] Conditions,
        double[] ObjectionFunctionComponents,
        double[] Constraints,
        ISet<int> BasisIndices);
}
