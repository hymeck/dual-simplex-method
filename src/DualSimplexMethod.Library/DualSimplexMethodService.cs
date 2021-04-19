using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using static DualSimplexMethod.Library.LinearAlgebraUtils; // BuildMatrixFrom2DArray, BuildVectorFromArray, BuildVectorFromArray

namespace DualSimplexMethod.Library
{
    public sealed class DualSimplexMethodService
    {
        private readonly double[,] _conditions;
        private readonly double[] _objectionFunctionComponents;
        private readonly double[] _constraints;
        private readonly ISet<int> _basisIndices;

        public DualSimplexMethodService(double[,] conditions, double[] objectionFunctionComponents, double[] constraints, ISet<int> basisIndices)
        {
            // todo: checks
            
            _conditions = conditions;
            _objectionFunctionComponents = objectionFunctionComponents;
            _constraints = constraints;
            _basisIndices = basisIndices;
        }

        public DualResult Solve()
        {
            var (conditions, objectionFunctionComponents, constraints, basisIndices) = GetConvenientData();
            return Core.Solve(conditions, basisIndices, objectionFunctionComponents, constraints);
        }

        private (
            Matrix<double> conditions, 
            Vector<double> objectionFunctionComponents, 
            Vector<double> constraints, 
            List<int> basisIndices)
            GetConvenientData()
        {
            var conditions = BuildMatrixFrom2DArray(_conditions);
            var objectionFunctionComponents = BuildVectorFromArray(_objectionFunctionComponents);
            var constraints = BuildVectorFromArray(_constraints);
            var basisIndices = new List<int>(_basisIndices);
            return (conditions, objectionFunctionComponents, constraints, basisIndices);
        }

        public static DualResult Solve(
            double[,] conditions, 
            double[] objectionFunctionComponents, 
            double[] constraints,
            ISet<int> basisIndices)
        {
            var service =
                new DualSimplexMethodService(conditions, objectionFunctionComponents, constraints, basisIndices);
            return service.Solve();
        }
    }
}
