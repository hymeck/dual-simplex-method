using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;

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
            // form 
            var conditions = BuildMatrixFrom2DArray(_conditions);
            var objectionFunctionComponents = BuildVectorFromArray(_objectionFunctionComponents);
            var constraints = BuildVectorFromArray(_constraints);
            var basisIndices = new List<int>(_basisIndices);

            var iteration = 1;
            Vector<double> y;
            Vector<double> plan;
            while (true)
            {
                Matrix<double> basisMatrix = GetBasisMatrix(conditions, basisIndices);
                
                var inversedBasisMatrix = basisMatrix.Inverse();

                Vector<double> basisObjectiveComponents =
                    GetBasisObjectiveFunctionComponents(objectionFunctionComponents, basisIndices);

                if (iteration == 1)
                {
                    y = inversedBasisMatrix.Multiply(basisObjectiveComponents);
                }

                var basisPlan = inversedBasisMatrix.Multiply(constraints);
                plan = Vector<double>.Build.Dense(conditions.RowCount, value: 0d);
                foreach (var (index, basisItem) in basisIndices.Select((basisItem, index) => (index, basisItem)))
                {
                    plan[basisItem] = basisPlan[index];
                }
                
                if (plan.Minimum() >= 0)
                    return DualResult.Create(plan);

                var negativeComponentIndex = -1;
                for (var i = 0; i < plan.Count; i++)
                {
                    if (plan[i] < 0)
                    {
                        negativeComponentIndex = i;
                        break; // optional. just choose first occurence
                    }
                }
                
                

                iteration++;
            }
            
            return DualResult.Create(plan);
        }

        public static Vector<double> GetBasisObjectiveFunctionComponents(Vector<double> objectionFunctionComponents, IEnumerable<int> basisIndices)
        {
            var basisComponents = basisIndices.Select(index => objectionFunctionComponents[index]);
            return Vector<double>.Build.DenseOfEnumerable(basisComponents);
        }

        public static Matrix<double> GetBasisMatrix(Matrix<double> conditions, IEnumerable<int> basisIndices)
        {
            var basisColumns = basisIndices.Select(conditions.Column);
            return Matrix<double>.Build.DenseOfColumns(basisColumns);
        }

        public static Vector<double> BuildVectorFromArray(double[] vector) => 
            Vector<double>.Build.DenseOfArray(vector);

        public static Matrix<double> BuildMatrixFrom2DArray(double[,] matrix) => 
            Matrix<double>.Build.DenseOfArray(matrix);

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
