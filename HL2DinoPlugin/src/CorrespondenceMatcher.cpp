#include "pch.h"
#include "CorrespondenceMatcher.h"
/**
 * @file        CorrespondenceMatcher.cpp
 * @brief       Implementations for \p CorrespondenceMatcher
 * @author      Hisham Iqbal
 * @author      Ferdinando Rodriguez y Baena
 * @copyright   &copy; Hisham Iqbal 2023
 *
 */

namespace 
{
    // Threshold static definitions, can be tuned to convenience
    static constexpr float  THRESH_FOR_DISTANCE = 0.0025; // metres
    static constexpr double THRESH_FOR_DUPLICATES = 0.001; // metres
    
    //! @brief Util function for updating a working copy of an index list
    //! @param inputList            List of potential index configurations
    //! @param unregPointsCount     Length of points yet to be registered 
    //! @return 
    static bool CreateIndexList(std::vector<std::vector<int>>& inputList, int unregPointsCount) 
    {
        typedef std::vector<int> IntegerList;
        std::vector<IntegerList>::iterator pConfigsList; // list of different potential configurations
        std::vector<IntegerList> oldConfigList; // previous list of potential configs
        IntegerList::iterator pConfigListElement;
        IntegerList tempConfig;

        // If input list is empty create a 1 column list of possible points (possibilities = number
        // of points in dataset)
        if (inputList.size() == 0)
        {
            for (int i = 0; i < unregPointsCount; i++)
            {
                tempConfig.clear();
                tempConfig.push_back(i);
                inputList.push_back(tempConfig);
            }

            return true;
        }

        oldConfigList = inputList;
        inputList.clear();

        for (pConfigsList = oldConfigList.begin(); pConfigsList != oldConfigList.end(); pConfigsList++)
        {
            for (int i = 0; i < unregPointsCount; i++)
            {
                bool found = false;

                for (pConfigListElement = (*pConfigsList).begin(); pConfigListElement != (*pConfigsList).end(); pConfigListElement++)
                {
                    if ((*pConfigListElement) == i) { found = true; }
                }

                if (!found)
                {
                    tempConfig = (*pConfigsList);
                    tempConfig.push_back((i));
                    inputList.push_back(tempConfig);
                }
            }
        }


        // In this case, no more combinations can be produced from the input points
        // available, which means registration cannot be completed successfully
        // (e.g. for a perfectly symmetrical set of points).
        if (inputList.size() == 0) { return false; }

        return true;
    }

    //! @brief Walk through \p unregisteredPoints to see if we can find a matching pair of points, which are \p inDistance apart in 3D space
    //! @param inDistance           Euclidean distance to compare against
    //! @param inputList            Current set of possible configurations 
    //! @param unregisteredPoints   Array of 3D points 
    //! @return                     True if we could find at least one match 
    static bool FilterByDistance(float inDistance, std::vector<std::vector<int>>& inputList, Eigen::Vector3d* unregisteredPoints)
    {
        using namespace Eigen;
        typedef std::vector<int> IntegerList;
        std::vector<IntegerList>::iterator pOptionList;
        std::vector<IntegerList>::iterator temp;

        Vector3d differenceVector;

        for (pOptionList = inputList.begin(); pOptionList != inputList.end();)
        {
            // Need at least two points per possible list for distance pruning
            if ((*pOptionList).size() < 2)
                return false;

            differenceVector = unregisteredPoints[*((*pOptionList).end() - 1)] - unregisteredPoints[*((*pOptionList).end() - 2)];

            float diffMag = fabs(differenceVector.norm() - inDistance);
            if (diffMag > THRESH_FOR_DISTANCE)	// get rid of this option
            {
                pOptionList = inputList.erase(pOptionList);
            }
            else
            {
                ++pOptionList;
            }
        }

        return true;
    }

    //! @brief Remove duplicate points in \p ioPointList based on Euclidean distance
    //! @param ioPointList 
    //! @return 
    static bool RemoveDuplicates(std::vector<Eigen::Vector3d>& ioPointList)
    {
        using namespace CorrespondenceMatcher;
        using namespace Eigen;
        std::vector<Vector3d>::iterator pPoint1, pPoint2;
        Vector3d differenceVector;

        // Clear up any duplicates
        for (pPoint1 = ioPointList.begin(); pPoint1 != ioPointList.end(); pPoint1++)
        {
            for (pPoint2 = (pPoint1 + 1); pPoint2 != ioPointList.end(); pPoint2++)
            {
                differenceVector = (*pPoint1) - (*pPoint2);

                if (differenceVector.norm() < THRESH_FOR_DUPLICATES)
                {
                    ioPointList.erase(pPoint2);
                    pPoint2--;
                }
            }
        }

        return true;
    }
}

namespace CorrespondenceMatcher 
{ 

    //! An index-matching list.
    //!
    //! This type, represented as `IntegerList`, is designed to associate indices with points.
    //! It is typically expected to have the same length as your reference point list.
    //! To use it, create an `IntegerList` and populate it with indices that correspond to
    //! points in your reference point list. You can then access the indices to match points
    //! between sets.
    //!
    //! Example usage:
    //!
    //! @code{.cpp}
    //! IntegerList listPoints;
    //! for (int i = 0; i < listPoints.size(); ++i)
    //! {
    //!     std::cout << "Point " << i << " in the original set is Point " << listPoints[i] <<
    //!     " in the floating set.\n";
    //! }
    //! @endcode
    typedef std::vector<int> IntegerList;

    Eigen::Matrix4d ComputeRigidTransform(const std::vector<Eigen::Vector3d>& src,
        const std::vector<Eigen::Vector3d>& dst)
    {
        using namespace Eigen;
        Matrix4d returnMat = Matrix4d::Identity();

        // not possible to do the registration
        if (src.size() != dst.size()) { return returnMat; }

        int pairSize = src.size();

        // calculate the centroid of each set
        Vector3d center_src(0, 0, 0), center_dst(0, 0, 0);
        for (int i = 0; i < pairSize; ++i)
        {
            center_src += src[i];
            center_dst += dst[i];
        }
        center_src /= static_cast<double>(pairSize);
        center_dst /= static_cast<double>(pairSize);

        MatrixXd S(pairSize, 3), D(pairSize, 3);
        for (int i = 0; i < pairSize; ++i)
        {
            for (int j = 0; j < 3; ++j)
                S(i, j) = src[i][j] - center_src[j];
            for (int j = 0; j < 3; ++j)
                D(i, j) = dst[i][j] - center_dst[j];
        }
        MatrixXd Dt = D.transpose();
        Matrix3d H = Dt * S;
        Matrix3d W, U, V;

        JacobiSVD<MatrixXd> svd;
        MatrixXd H_(3, 3);
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) H_(i, j) = H(i, j);
        svd.compute(H_, Eigen::ComputeThinU | Eigen::ComputeThinV);
        if (!svd.computeU() || !svd.computeV()) {
            return returnMat;
        }

        // calculate rotation mat
        Matrix3d Vt = svd.matrixV().transpose();
        Matrix3d R = svd.matrixU() * Vt;

        if (R.determinant() < 0.) {
            Vt(2, 0) *= -1.;
            Vt(2, 1) *= -1.;
            Vt(2, 2) *= -1.;
            R = svd.matrixU() * Vt;
        }

        // calculate translation vector
        Vector3d t = center_dst - R * center_src;

        returnMat.block(0, 0, 3, 3) = R;
        returnMat.block(0, 3, 3, 1) = t;

        return returnMat;
    }

    bool GetPointCorrespondence(std::vector<Eigen::Vector3d>& ReferencePoints,
        std::vector<Eigen::Vector3d>& CollectedPoints,
        std::vector<std::vector<int>>& CorrespondenceList)
    {
        using namespace Eigen;
        
        std::vector<IntegerList> indicesList;
        std::vector<Vector3d>::iterator pInputPoint;
        IntegerList::iterator pCandidate;

        Vector3d differenceVector;
        float eucDistMagnitude;

        // Handling duplicates in both datasets, shouldn't really be any
        // dupes in reference points though...
        RemoveDuplicates(ReferencePoints);
        RemoveDuplicates(CollectedPoints);

        // Need at least three points to match
        if (ReferencePoints.size() < 3 || CollectedPoints.size() < 3)
            return false;

        // Add first point to list
        CreateIndexList(indicesList, CollectedPoints.size());

        for (pInputPoint = (ReferencePoints.begin() + 1); pInputPoint != ReferencePoints.end(); pInputPoint++)
        {
            // Add another point to the list
            CreateIndexList(indicesList, CollectedPoints.size());

            // difference between two consecutive points
            differenceVector = (*(pInputPoint)) - (*(pInputPoint - 1));

            eucDistMagnitude = differenceVector.norm();

            // Prune options depending on distance between first two points
            FilterByDistance(eucDistMagnitude, indicesList, &CollectedPoints[0]);
        }

        CorrespondenceList = indicesList;

        return (CorrespondenceList.size() > 0); // List non-zero if a correspondence was found        
    }    
}
