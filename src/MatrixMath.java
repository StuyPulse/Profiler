//Some midly complicated linear algebra with matrices, used in solving equations
public class MatrixMath {
  public static double determinitive(int matrixSize, double[][] matrix) {
    if(matrixSize < 2) {
      System.out.println("Cannot find determinitive");
      return 0;
    }else if(matrixSize == 2) {
      return (matrix[0][0] * matrix[1][1]) - (matrix[1][0] * matrix[0][1]);
    }else {
      double total = 0;
      //i Iterates through the columns of row zero
      //j is the multiplier factor
      for(int i = 0, j = 1; i < matrixSize; i++, j *= -1) {
        total += j * matrix[0][i] * determinitive(matrixSize - 1, subMatrix(matrixSize, matrix, 0, i));
      }
      return total;
    }
  }

  public static double[][] subMatrix(int matrixSize, double[][] matrix, int row, int column) {
    double[][] sub = new double[matrixSize - 1][matrixSize - 1];
    row:
    //i is the row of the matrix, and k is the row of the sub
    for(int i = 0, k = 0; i < matrixSize; i++) {
      if(i == row) {
        continue row;
      }
      column:
      //j is the column of the matrix, and l is the column of the sub
      for(int j = 0, l = 0;  j < matrixSize; j++) {
        if(j == column) {
          continue column;
        }else {
          sub[k][l] = matrix[i][j];
          l++;
        }
      }
      k++;
    }
    return sub;
  }
  
  //Takes in equations of the form {x, y, result}
  //Returns the answers as {x, y}
  public static double[] solveLinearSystem(double[] equation1, double[] equation2) {
    double[][] coefficientMatrix = { {equation1[0], equation1[1]},
                                    {equation2[0], equation2[1]} };
    double coefficientDeterminitive = determinitive(2, coefficientMatrix);
    double[][] xMatrix = { {equation1[2], equation1[1]}, 
                            {equation2[2], equation2[1]} }; 
    double xDeterminitive = determinitive(2, xMatrix);
    double x = xDeterminitive / coefficientDeterminitive;
    double yMatrix[][] = { {equation1[0], equation1[2]}, 
                            {equation2[0], equation2[2]} }; 
    double yDeterminitive = determinitive(2, yMatrix); 
    double y = yDeterminitive / coefficientDeterminitive;
    double[] solutionSet = {x, y}; 
    return solutionSet; 
  }
}
