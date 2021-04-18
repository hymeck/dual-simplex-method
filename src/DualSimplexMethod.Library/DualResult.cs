using System.Collections.Generic;
using System.Collections.Immutable;

namespace DualSimplexMethod.Library
{
    public sealed class DualResult
    {
        public static readonly DualResult Empty = new (ImmutableArray<double>.Empty);
        public readonly ImmutableArray<double> FeasibleSolution;

        private DualResult(ImmutableArray<double> feasibleSolution) => 
            FeasibleSolution = feasibleSolution;

        private DualResult(IEnumerable<double> feasibleSolution) : 
            this(feasibleSolution.ToImmutableArray())
        {
        }

        public bool IsEmpty => FeasibleSolution.IsEmpty;

        public static DualResult Create(IEnumerable<double> feasibleSolution) => 
            feasibleSolution == null ? Empty : new DualResult(feasibleSolution);
    }
}
