using System.Collections.Generic;

namespace DualSimplexMethod.Demo
{
    public class Data
    {
        public static Dictionary<int, InputDataHolder> GetInputDataHolders()
        {
            var conditions = new double[,]
            {
                {2, -3, 1, 0},
                {-1, -1, 1, 1}
            };
            var objectiveFunction = new double[] {1, -4, 1, 0};
            var constraints = new double[] {-3, -6};
            var basisIndices = new HashSet<int>(new[] {2, 3});

            var holder12 = new InputDataHolder(conditions, objectiveFunction, constraints, basisIndices);

            var variants = new Dictionary<int, InputDataHolder>
            {
                {12, holder12},
            };
            
            return variants;
        }
    }
}
