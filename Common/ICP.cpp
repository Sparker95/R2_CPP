#include "ICP.h"
#include "Common.h"
#include "Scan.h"

#include <vector>
#include <cmath>

#include <Eigen/Dense>

#include "Sensor.h"

using Scan::TScanPolar;
using Scan::TScanCart;

namespace ICP
{
	Eigen::Matrix2f MRot(float angle)
	{
		float _cos = std::cos(angle);
		float _sin = std::sin(angle);
		Eigen::Matrix2f m{
			{_cos, -_sin},
			{_sin, _cos}
		};
		return m;
	}

	Eigen::Matrix2f MRotDerivative(float angle)
	{
		float _cos = std::cos(angle);
		float _sin = std::sin(angle);
		Eigen::Matrix2f m{
			{-_sin, -_cos},
			{_cos, -_sin}
		};
		return m;
	}

	Eigen::Matrix<float, 2, 3> MJacobian(Eigen::Vector2f& point, const Eigen::Matrix2f& mRotDer)
	{
		Eigen::Matrix<float, 2, 3> J;
		J.setZero();
		J(0, 0) = 1.0f;
		J(1, 1) = 1.0f;
		J.col(2) = mRotDer * point;
		return J;
	}

	float WeightFunc(const Eigen::Vector2f& diff, float threshold)
	{
		float dx = diff(0);
		float dy = diff(1);
		float d = std::sqrt(dx * dx + dy * dy);
		if (d < threshold)
			return 1.0f;
		return threshold / d;
	}

	// Returns array of ints. Id - index of pt in scan0. value[id] - index of pt in scan1.
	void GetPointCorrespondence(const TScanCart& scan0, const TScanCart& scan1, std::vector<uint>& outCorrespondence)
	{
		uint nPoints0 = static_cast<uint>(scan0.cols());
		uint nPoints1 = static_cast<uint>(scan1.cols());

		outCorrespondence.resize(nPoints0);

		for (uint i = 0; i < nPoints0; i++)
		{
			float x0 = scan0(0, i);
			float y0 = scan0(1, i);

			float minDistSq = std::numeric_limits<float>::max();
			uint nearestj = -1;
			for (uint j = 0; j < nPoints1; j++)
			{
				float dx = x0 - scan1(0, j);
				float dy = y0 - scan1(1, j);
				float distSq = dx * dx + dy * dy;
				if (distSq < minDistSq)
				{
					minDistSq = distSq;
					nearestj = j;
				}
			}

			outCorrespondence[i] = nearestj;
		}
	}

	Pose MatchScans(const TScanPolar& scan0Polar, const TScanPolar& scan1Polar, uint nIterMax, Pose initialT)
	{
		TScanCart scan0, scan1;
		Scan::ScanPolarToCart(scan0Polar, scan0);
		Scan::ScanPolarToCart(scan1Polar, scan1);

		const float distThresholdCoeff = 1.0f;
		Eigen::RowVectorXf maxDist0 = (distThresholdCoeff * PI / static_cast<float>(scan0.cols())) * scan0Polar.row(1);
		Eigen::RowVectorXf maxDist1 = (distThresholdCoeff * PI / static_cast<float>(scan0.cols())) * scan1Polar.row(1);

		uint nPoints = static_cast<uint>(scan0.cols());
		std::vector<float> weights(nPoints);

		// Current estimate
		Eigen::Vector3f T { initialT.x, initialT.y, initialT.dir };

        TScanCart scan0Transformed;
		TScanCart scanErrors(2, nPoints); // Matches scan0 array
		std::vector<uint> corresp;
        for (uint iteration = 0; iteration < nIterMax; iteration++)
        {
			Eigen::Matrix2f mRot = MRot(T(2));
            scan0Transformed = mRot * scan0;
            for (uint i = 0; i < scan0Transformed.cols(); i++)
            {
                scan0Transformed(0, i) = scan0Transformed(0, i) + T(0);
                scan0Transformed(1, i) = scan0Transformed(1, i) + T(1);
            }

			GetPointCorrespondence(scan0Transformed, scan1, corresp);

			// Calculate errors
			for (uint i = 0; i < nPoints; i++)
			{
				if (corresp[i] == -1) // It means outlier doesn't contribute to JxError
					continue;

				Eigen::Vector2f e = scan0Transformed.col(i) - scan1.col(i);

				// Correct error by weight
				float weightThreshold = std::max(maxDist0(i), maxDist1(corresp[i]));
				float weight = WeightFunc(e, weightThreshold);
				weights[i] = weight;
				Eigen::Vector2f eWeighted = e * weight;
				scanErrors.col(i) = eWeighted;
			}

	        // Calculate Jacobian, Hessian
			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, 0, 3, 3> H; // Must be resizable for SVD to work (below)
			H.resize(3, 3);
			H.setZero();
			Eigen::Matrix2f mRotDer = MRotDerivative(T(2));
			Eigen::Vector3f JxError;
			JxError.setZero();
			for (uint i = 0; i < nPoints; i++)
			{
				Eigen::Vector2f pointi = scan0Transformed.col(i);
				Eigen::Matrix<float, 2, 3> Ji = MJacobian(pointi, mRotDer);
				Eigen::Matrix<float, 3, 2> Ji_T = Ji.transpose();
				JxError = JxError + (Ji_T * scanErrors.col(i)); // Error already multiplied by weight
				H = H + (weights[i] * (Ji_T * Ji));
			}

			Eigen::Vector3f deltaT = H.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-JxError);
			T = T + deltaT;

			Eigen::Vector2f scanError{
				scanErrors.row(0).sum(),
				scanErrors.row(1).sum()
			};
        }

		return Pose(T(0), T(1), T(2));
	}
}