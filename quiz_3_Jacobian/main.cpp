#include <iostream>
#include <vector>
#include <math.h>
#include "../Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

MatrixXd CalculateJacobian(const VectorXd &x_state);

int main()
{
    /**
   * Compute the Jacobian Matrix
   */

    // predicted state example
    // px = 1, py = 2, vx = 0.2, vy = 0.4
    VectorXd x_predicted(4);
    x_predicted << 1, 2, 0.2, 0.4;

    MatrixXd Hj = CalculateJacobian(x_predicted);

    cout << "Calculate Jacobian Matrix" << endl
         << endl;

    cout << "Hj:" << endl
         << Hj << endl;

    return 0;
}

MatrixXd CalculateJacobian(const VectorXd &x_state)
{

    MatrixXd Hj(3, 4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // TODO: YOUR CODE HERE

    // check division by zero
    if (px <= 0 && py <= 0)
    {
        cout << "Both px and py are zero!  Cannot compute Jacobian matrix." << endl;

        // cout << "Hj = " << endl
        //      << Hj << endl
        //      << endl;

        return Hj;
    }

    // compute the Jacobian matrix
    Hj << (px / sqrt(pow(px, 2) + pow(py, 2))), (py / sqrt(pow(px, 2) + pow(py, 2))), 0, 0,
          (-1 * py / (pow(px, 2) + pow(py, 2))), (px / (pow(px, 2) + pow(py, 2))), 0, 0,
          (py * ((vx * py) - (vy * px)) / pow(pow(px, 2) + pow(py, 2), 3 / 2)), 
          (px * ((vy * px) - (vx * py)) / pow(pow(px, 2) + pow(py, 2), 3 / 2)), 
          (px / sqrt(pow(px, 2) + pow(py, 2))), 
          (py / sqrt(pow(px, 2) + pow(py, 2)));

    return Hj;
}