/** @file           CorrespondenceMatcher.h
 *  @brief          Utility class for computing point-correspondences between 
 *                  two sets of points
 * 
 *  @note           Thanks Ferdi for all of the iterator-based magic in
 *                  this file!
 *
 *  @author         Hisham Iqbal
 *  @author         Ferdinando Rodriguez y Baena
 *  @copyright      &copy; 2023 Hisham Iqbal
 */

#ifndef CORRESPONDENCE_MATCHER_H
#define CORRESPONDENCE_MATCHER_H

#include <Eigen/Dense>
#include <vector>

 /**
  * @namespace   CorrespondenceMatcher
  * @brief       Utility namespace for doing finding correspondence and registration of two point-sets
 */

namespace CorrespondenceMatcher
{
    //-----------------------------------------------------------------------------------------------------------------------------------------------
    //! Computes a transform which maps \p src onto \p dst
    //!
    //! \param src                  Points in a floating frame
    //! \param dst                  Points in the target frame
    //! \note                       This function is expecting \p src and \p dst to be in corresponding order (src[0] should correspond to dst[0] etc.)
    //! \return Eigen::Matrix4d     4x4 transform matrix
    Eigen::Matrix4d ComputeRigidTransform(
        const std::vector<Eigen::Vector3d>& src,
        const std::vector<Eigen::Vector3d>& dst);
    //-----------------------------------------------------------------------------------------------------------------------------------------------
    
    //-----------------------------------------------------------------------------------------------------------------------------------------------
    //! Function which tries to find a correspondence/graph-matching map between two sets of coordinates based on Euclidean distances
    //!
    //! \param inReferencePoints        Original/known point-set in a fixed frame
    //! \param inCollectedPoints        Floating points we're trying to match to our known points
    //! \param outCorrespondenceList    If corresponding, a list of indexes which map \p inCollectedPoints onto \p inReferencePoints
    //!
    //! \return                         True if a correspondence could be found
    bool GetPointCorrespondence(
        std::vector<Eigen::Vector3d>& inReferencePoints,
        std::vector<Eigen::Vector3d>& inCollectedPoints,
        std::vector<std::vector<int>>& outCorrespondenceList);
    //-----------------------------------------------------------------------------------------------------------------------------------------------
};

#endif // CORRESPONDENCE_MATCHER_H