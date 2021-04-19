using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using static DualSimplexMethod.Library.BasisUtils; // GetBasisMatrix, GetBasisObjectiveFunctionComponents, GetNonBasisIndices
// ReSharper disable ArgumentsStyleLiteral
// ReSharper disable SuggestBaseTypeForParameter
// ReSharper disable ArgumentsStyleNamedExpression

namespace DualSimplexMethod.Library
{
    internal static class Core
    {
        private static Vector<double> GetInitialFeasibleSolution(Matrix<double> conditions, IEnumerable<int> basisIndices, IList<double> basisPlan)
        {
            var plan = Vector<double>.Build.Dense(conditions.ColumnCount, value: 0d);
            foreach (var (index, basisItem) in basisIndices.Select((basisItem, index) => (index, basisItem)))
            {
                plan[basisItem] = basisPlan[index];
            }

            return plan;
        }
        
        private static int FindNegativeComponentIndex(IList<double> plan)
        {
            var negativeComponentIndex = 0;
            for (var i = 0; i < plan.Count; i++)
            {
                if (plan[i] < 0)
                {
                    negativeComponentIndex = i;
                    break; // optional break. so just choose first occurence
                }
            }
            return negativeComponentIndex;

            // todo: shit, it does not work properly
            // var (negativeComponentIndex, _) = plan
            //     .Select((item, index) => (index, item))
            //     .Where((_, basisItem) => basisItem < 0)
            //     .FirstOrDefault();
            //
            // return negativeComponentIndex;
        }
        
        private static int FindIndexInBasisIndices(IEnumerable<int> basisIndices, int negativeComponentIndex)
        {
            var indexInBasisIndices = 0;
            foreach (var (index, basisItem) in basisIndices.Select((basisItem, index) => (index, basisItem)))
            {
                if (basisItem == negativeComponentIndex)
                {
                    indexInBasisIndices = index;
                    break; // it is not necessary to choose the first occurence
                }
            }
            return indexInBasisIndices;
            
            // todo: may be this crap also does not work correctly
            // var (indexInBasisIndices, _) = basisIndices
            //     .Select((basisItem, index) => (index, basisItem)) // project item to item with index because we need to find out index 
            //     .Where((_, basisItem) => basisItem == negativeComponentIndex) // match occurences 
            //     .FirstOrDefault(); // it is not necessary to choose the first occurence
            //
            // return indexInBasisIndices;
        }
        
        private static Vector<double> GetMu(IEnumerable<int> nonBasisIndices, Vector<double> deltaY, Matrix<double> conditions)
        {
            var dotProducts = nonBasisIndices
                .Select(conditions.Column)
                .Select(deltaY.DotProduct);
            
            return Vector<double>.Build.DenseOfEnumerable(dotProducts);
        }
        
        private static (double minSigma, int minSigmaIndex) GetMinSigmaData(Vector<double> sigma, int[] nonBasisIndices)
        {
            var minSigma = sigma.Minimum();
            var minSigmaIndex = nonBasisIndices[sigma.MinimumIndex()];
            return (minSigma, minSigmaIndex);
        }

        private static Vector<double> GetSigmaVector(
            Vector<double> u, 
            int[] nonBasisIndices, 
            Matrix<double> conditions, 
            Vector<double> y,
            IList<double> objectionFunctionComponents)
        {
            var sigma = Vector<double>.Build.Dense(u.Count, double.MaxValue);
            
            for (var i = 0; i < sigma.Count; i++)
            {
                if (u[i] < 0)
                {
                    var nonBasisIndex = nonBasisIndices[i];
                    var product = conditions
                        .Column(nonBasisIndex)
                        .DotProduct(y);
                    sigma[i] = (objectionFunctionComponents[nonBasisIndex] - product) / u[i];
                }
            }

            return sigma;
        }

        public static DualResult Solve(
            Matrix<double> conditions, 
            IList<int> basisIndices, 
            Vector<double> objectionFunctionComponents,
            Vector<double> constraints)
        {
            var iteration = 1;
            var y = Vector<double>.Build.Dense(conditions.ColumnCount, 0d);
            Vector<double> plan;

            var possibleIndices = Enumerable.Range(0, conditions.ColumnCount).ToArray();
            while (true)
            {
                // build inversed basis matrix
                var inversedBasisMatrix = GetBasisMatrix(conditions, basisIndices).Inverse();

                // build vector that consists of basis objective function components
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

                var nonBasisIndices = GetNonBasisIndices(possibleIndices, basisIndices).ToArray();

                var mu = GetMu(nonBasisIndices, deltaY, conditions);

                // incompatible case
                if (mu.Minimum() >= 0)
                    return DualResult.Empty;

                var sigma = GetSigmaVector(mu, nonBasisIndices, conditions, y, objectionFunctionComponents);

                var (minSigma, minSigmaIndex) = GetMinSigmaData(sigma, nonBasisIndices);

                basisIndices[indexInBasisIndices] = minSigmaIndex;
                y += deltaY.Multiply(minSigma);

                iteration++;
            }
        }
    }
}
