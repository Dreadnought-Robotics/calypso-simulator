#ifndef GAZEBO_7_SHIMS_HPP
#define GAZEBO_7_SHIMS_HPP

#include <gazebo/gazebo_config.h>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#if GAZEBO_MAJOR_VERSION >= 8
namespace gazebo_underwater
{
    inline ignition::math::Matrix3d const& IgnMatrix3(ignition::math::Matrix3d const& matrix
    ) {
        return matrix;
    }
    inline ignition::math::Matrix3d const& GzMatrix3(
        ignition::math::Matrix3d const& matrix
    ) {
        return matrix;
    }

}
#define GzGet(object, getter, args) \
	object.getter args
#define GzGetIgn(object, getter, args) \
	object.getter args
#define ToIgn(value) \
	value
#else

#include <gazebo/math/Pose.hh>
#include <gazebo/math/Matrix3.hh>

namespace gazebo_underwater
{
    inline ignition::math::Matrix3d IgnMatrix3(gazebo::math::Matrix3 const& matrix)
    {
        return ignition::math::Matrix3d(
            matrix[0][0], matrix[0][1], matrix[0][2],
            matrix[1][0], matrix[1][1], matrix[1][2],
            matrix[2][0], matrix[2][1], matrix[2][2]
        );
    }
    inline gazebo::math::Matrix3 GzMatrix3(ignition::math::Matrix3d const& matrix)
    {
        return gazebo::math::Matrix3(
            matrix(0, 0), matrix(0, 1), matrix(0, 2),
            matrix(1, 0), matrix(1, 1), matrix(1, 2),
            matrix(2, 0), matrix(2, 1), matrix(2, 2)
        );
    }

}
#define GzGet(object, getter, args) \
	object.Get ## getter args
#define GzGetIgn(object, getter, args) \
	object.Get ## getter args.Ign()
#define ToIgn(value) \
	value.Ign()
#endif

#endif
