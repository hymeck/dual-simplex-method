using System.Collections.Generic;
using DualSimplexMethod.Library;
// ReSharper disable InconsistentNaming

namespace DualSimplexMethod.Demo
{
    public class Executor
    {
        public readonly Dictionary<int, InputDataHolder> Variants;

        public Executor(Dictionary<int, InputDataHolder> variants) => 
            Variants = variants;

        public DualResult Perform(int variant)
        {
            var (A, c, b, Jb) = Variants[variant];
            return DualSimplexMethodService.Solve(A, c, b, Jb);
        }

        public static DualResult Perform(Dictionary<int, InputDataHolder> variants, int variant)
        {
            var executor = new Executor(variants);
            return executor.Perform(variant);
        }
    }
}
