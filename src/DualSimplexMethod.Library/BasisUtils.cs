using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;

namespace DualSimplexMethod.Library
{
    public static class BasisUtils
    {
        public static Matrix<double> GetBasisMatrix(Matrix<double> conditions, IEnumerable<int> basisIndices)
        {
            var columns = basisIndices.Select(conditions.Column);
            return LinearAlgebraUtils.CreateMatrixFromColumns(columns);
        }
        
        public static Vector<double> GetBasisObjectiveFunctionComponents(Vector<double> objectionFunctionComponents, IEnumerable<int> basisIndices)
        {
            // var basisComponents = basisIndices.Select(index => objectionFunctionComponents[index]);
            var basisComponents = basisIndices.Select(objectionFunctionComponents.At);
            return Vector<double>.Build.DenseOfEnumerable(basisComponents);
        }
        
        public static IEnumerable<int> GetNonBasisIndices(IEnumerable<int> possibleIndices, IEnumerable<int> basisIndices) => 
            possibleIndices.Except(basisIndices);
    }
}
