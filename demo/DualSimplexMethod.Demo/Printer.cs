using System;
using System.Linq;
using DualSimplexMethod.Library;

namespace DualSimplexMethod.Demo
{
    public class Printer
    {
        public static void PrintResult(DualResult dualResult)
        {
            var output = dualResult.IsEmpty
                ? "Incompatible"
                : $"[{string.Join("; ", dualResult.FeasibleSolution.AsEnumerable())}]";
            
            Console.WriteLine(output);
        }
    }
}
