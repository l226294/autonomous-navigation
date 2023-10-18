#ifndef _POLYNOMIALS_HPP_
#define _POLYNOMIALS_HPP_

#include <vector>
#include "Eigen/Dense"

class QuinticPolynomial
{
public:
  explicit QuinticPolynomial(const std::vector<double> &start, const std::vector<double> &end, double T)
  {
    Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
    A << T * T * T, T * T * T * T, T * T * T * T * T,
        3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
        6 * T, 12 * T * T, 20 * T * T * T;

    Eigen::MatrixXd B = Eigen::MatrixXd(3, 1);
    B << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T * T),
        end[1] - (start[1] + start[2] * T),
        end[2] - start[2];

    Eigen::MatrixXd Ai = A.inverse();
    Eigen::MatrixXd C = Ai * B;

    coefficients = {start[0], start[1], 0.5 * start[2]};

    for (int i = 0; i < C.size(); i++)
    {
        coefficients.push_back(C.data()[i]);
    }
  }
  double calculatePoint(double t)
  {
    return coefficients[0] + coefficients[1]*t + coefficients[2]*t*t + coefficients[3]*t*t*t + coefficients[4]*t*t*t*t + coefficients[5]*t*t*t*t*t;
  }

  double calculateFirstDerivative(double t)
  {
    return coefficients[1] + 2*coefficients[2]*t + 3*coefficients[3]*t*t + 4*coefficients[4]*t*t*t + 5*coefficients[5]*t*t*t*t;
  }

  double calculateSecondDerivative(double t)
  {
    return 2*coefficients[2]*t + 6*coefficients[3]*t + 12*coefficients[4]*t*t + 20*coefficients[5]*t*t*t;
  }

  double calculateThirdDerivative(double t)
  {
    return 6*coefficients[3] + 24*coefficients[4]*t + 60*coefficients[5]*t*t;
  }

private:
  std::vector<double> coefficients;

};

class QuarticPolynomial
{
 public:
	explicit QuarticPolynomial(const std::vector<double> &start, const std::vector<double> &end, double T)
    {
      Eigen::MatrixXd A = Eigen::MatrixXd(2, 2);
        A << 3 * T * T, 4 * T * T * T,
            6 * T, 12 * T * T;

        Eigen::MatrixXd B = Eigen::MatrixXd(2, 1);
        B << end[0] - (start[1] + start[2] * T),
            end[1] - start[2];

        Eigen::MatrixXd Ai = A.inverse();
        Eigen::MatrixXd C = Ai * B;

        coefficients = {start[0], start[1], 0.5 * start[2]};

        for (int i = 0; i < C.size(); i++)
        {
            coefficients.push_back(C.data()[i]);
        }
    }

	double calculatePoint(double t)
    {
      return coefficients[0] + coefficients[1]*t + coefficients[2]*t*t + coefficients[3]*t*t*t + coefficients[4]*t*t*t*t;
    }

	double calculateFirstDerivative(double t)
    {
      return coefficients[1] + 2*coefficients[2]*t + 3*coefficients[3]*t*t + 4*coefficients[4]*t*t*t;
    }

	double calculateSecondDerivative(double t)
    {
      return 2*coefficients[2]*t + 6*coefficients[3]*t + 12*coefficients[4]*t*t;
    }

	double calculateThirdDerivative(double t)
    {
      return 6*coefficients[3] + 24*coefficients[4]*t;
    }
	
 private:
	std::vector<double> coefficients;
};


#endif // _POLYNOMIALS_HPP_