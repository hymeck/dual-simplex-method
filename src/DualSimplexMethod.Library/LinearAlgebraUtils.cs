using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;

namespace DualSimplexMethod.Library
{
    internal static class LinearAlgebraUtils
    {
        public static Matrix<double> CreateMatrixFromColumns(IEnumerable<Vector<double>> columns) => 
            Matrix<double>.Build.DenseOfColumns(columns);
        
        public static Vector<double> BuildVectorFromArray(double[] vector) => 
            Vector<double>.Build.DenseOfArray(vector);

        public static Matrix<double> BuildMatrixFrom2DArray(double[,] matrix) => 
            Matrix<double>.Build.DenseOfArray(matrix);
    }
}
