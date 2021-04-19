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
            var y = Vector<double>.Build.Dense(conditions.ColumnCount, 0d);
            Vector<double> plan;
            while (true)
            {
                var basisMatrix = GetBasisMatrix(conditions, basisIndices);
                
                var inversedBasisMatrix = basisMatrix.Inverse();

                var basisObjectiveComponents =
                    GetBasisObjectiveFunctionComponents(objectionFunctionComponents, basisIndices);

                if (iteration == 1)
                {
                    y = inversedBasisMatrix.Multiply(basisObjectiveComponents);
                }

                var basisPlan = inversedBasisMatrix.Multiply(constraints);
                plan = Vector<double>.Build.Dense(conditions.ColumnCount, value: 0d);
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

                var indexInBasisIndices = 0;

                foreach (var (index, basisItem) in basisIndices.Select((basisItem, index) => (index, basisItem)))
                {
                    if (basisItem == negativeComponentIndex)
                    {
                        indexInBasisIndices = index;
                        break; // optional. just choose first occurence
                    }
                }

                var deltaY = inversedBasisMatrix.Row(indexInBasisIndices);

                int[] nonBasisIndices = Enumerable.Range(0, conditions.ColumnCount).Except(basisIndices).ToArray();

                var u = Vector<double>.Build.DenseOfEnumerable(nonBasisIndices.Select(index => (double) index));

                for (var i = 0; i < u.Count; i++)
                {
                    u[i] = deltaY.DotProduct(conditions.Column(nonBasisIndices[i]));
                }
                
                if (u.Minimum() >= 0)
                    return DualResult.Empty;

                var sigma = Vector<double>.Build.DenseOfVector(u);

                for (var i = 0; i < sigma.Count; i++)
                {
                    if (u[i] >= 0)
                        sigma[i] = double.MaxValue;
                    else
                    {
                        var nonBasisIndex = nonBasisIndices[i];
                        var product = conditions.Column(nonBasisIndex).DotProduct(y);
                        sigma[i] =
                            (objectionFunctionComponents[nonBasisIndex] - product) / u[i];
                    }
                }

                var minSigma = sigma.Minimum();
                var minSigmaIndex = nonBasisIndices[sigma.MinimumIndex()];
                basisIndices[minSigmaIndex] = minSigmaIndex;

                y += deltaY.Multiply(minSigma);

                iteration++;
            }
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
