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
            var (conditions, objectionFunctionComponents, constraints, basisIndices) = GetConvenientData();

            var iteration = 1;
            var y = Vector<double>.Build.Dense(conditions.ColumnCount, 0d);
            Vector<double> plan;

            var possibleIndices = Enumerable.Range(0, conditions.ColumnCount).ToArray();
            while (true)
            {
                var basisMatrix = GetBasisMatrix(conditions, basisIndices); // build basis matrix
                
                var inversedBasisMatrix = basisMatrix.Inverse(); // inverse basis matrix

                // build vector from basis objective function components
                var basisObjectiveComponents =
                    GetBasisObjectiveFunctionComponents(objectionFunctionComponents, basisIndices);

                if (iteration == 1)
                {
                    // set initial value to y which equals to
                    // product of inversed basis matrix and basis components of objective function 
                    y = inversedBasisMatrix.Multiply(basisObjectiveComponents);
                }
                
                var basisPlan = inversedBasisMatrix.Multiply(constraints);
                
                plan = GetInitialFeasibleSolution(conditions, basisIndices, basisPlan);

                // todo: add comment what it means 
                if (plan.Minimum() >= 0)
                    return DualResult.Create(plan);

                // find index of the first negative component occurence of feasibile solution vector
                var negativeComponentIndex = FindNegativeComponentIndex(plan);

                // find index of basis item which equals to founded negative component index
                var indexInBasisIndices = FindIndexInBasisIndices(basisIndices, negativeComponentIndex);
                
                var deltaY = inversedBasisMatrix.Row(indexInBasisIndices);

                var nonBasisIndices = possibleIndices.Except(basisIndices).ToArray();

                var mu = GetMu(nonBasisIndices, deltaY, conditions);

                // todo: add comment what it means (2)
                if (mu.Minimum() >= 0)
                    return DualResult.Empty;

                var sigma = GetSigma(mu, nonBasisIndices, conditions, y, objectionFunctionComponents);

                var (minSigma, minSigmaIndex) = GetSigmaData(sigma, nonBasisIndices);
                
                basisIndices[minSigmaIndex] = minSigmaIndex;
                y += deltaY.Multiply(minSigma);

                iteration++;
            }
        }

        private static (double minSigma, int minSigmaIndex) GetSigmaData(Vector<double> sigma, int[] nonBasisIndices)
        {
            var minSigma = sigma.Minimum();
            var minSigmaIndex = nonBasisIndices[sigma.MinimumIndex()];
            return (minSigma, minSigmaIndex);
        }

        private static Vector<double> GetSigma(Vector<double> u, int[] nonBasisIndices, Matrix<double> conditions, Vector<double> y,
            IList<double> objectionFunctionComponents)
        {
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

            return sigma;
        }

        private static Vector<double> GetMu(int[] nonBasisIndices, Vector<double> deltaY, Matrix<double> conditions)
        {
            var u = Vector<double>.Build.DenseOfEnumerable(nonBasisIndices.Select(index => (double) index));

            for (var i = 0; i < u.Count; i++)
            {
                u[i] = deltaY.DotProduct(conditions.Column(nonBasisIndices[i]));
            }

            return u;
        }

        private static int FindIndexInBasisIndices(IEnumerable<int> basisIndices, int negativeComponentIndex)
        {
            var indexInBasisIndices = 0;
            foreach (var (index, basisItem) in basisIndices.Select((basisItem, index) => (index, basisItem)))
            {
                if (basisItem == negativeComponentIndex)
                {
                    indexInBasisIndices = index;
                    break; // optional. just choose first occurence
                }
            }

            return indexInBasisIndices;
        }

        private static int FindNegativeComponentIndex(IList<double> plan)
        {
            var negativeComponentIndex = -1;
            for (var i = 0; i < plan.Count; i++)
            {
                if (plan[i] < 0)
                {
                    negativeComponentIndex = i;
                    break; // optional. just choose first occurence
                }
            }

            return negativeComponentIndex;
        }

        private static Vector<double> GetInitialFeasibleSolution(Matrix<double> conditions, IEnumerable<int> basisIndices, IList<double> basisPlan)
        {
            Vector<double> plan;
            plan = Vector<double>.Build.Dense(conditions.ColumnCount, value: 0d);
            foreach (var (index, basisItem) in basisIndices.Select((basisItem, index) => (index, basisItem)))
            {
                plan[basisItem] = basisPlan[index];
            }

            return plan;
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
