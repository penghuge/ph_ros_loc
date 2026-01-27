#ifndef FORMAT_TRANSFORM_H_
#define FORMAT_TRANSFORM_H_

#include "slam_math.h"
#include "rigid_transformer.h"
#include "cartographer/logging/log_data.h"

namespace SlamCommon {

inline void AddDirectoryFlag(std::string &str)
{
    std::string s("");
    s += str.back();
    if (s != DIR_FLAG)
    {
        str += DIR_FLAG;
    }
}

inline void ReplaceStr(std::string &str, const std::string &target_str, const std::string &new_str)
{
    for(std::string::size_type pos(0); pos != std::string::npos; pos += new_str.length())
    {
        pos = str.find(target_str, pos);
        if(pos == std::string::npos)
        {
            return;
        }
        str.replace(pos, target_str.length(), new_str);
    }
}

template <typename T>
std::string NumToString(const T num, int decplaces = 3)
{
    std::ostringstream oss;
    //std::fixed means keep decplaces behind decimal point
    oss << std::fixed << std::setprecision(decplaces) << num;
    return oss.str();
}

inline void SplitStringIntoUSet(const std::string &str, std::unordered_set<std::string> &uset,
                                const char split_char)
{
    std::string remain_str(str);
    int index = 0;
    while ((index = remain_str.find_first_of(split_char)) >= 0)
    {
        if (index > 0)
        {
            uset.insert(remain_str.substr(0, index));
        }
        remain_str = remain_str.substr(index + 1, remain_str.size() - 1);
    }
    if (!remain_str.empty())
    {
        uset.insert(remain_str.substr(0, index - 1));
    }
}

inline void SplitString(const std::string &str_in, std::vector<std::string> &str_out,
                        const char split_char)
{
    std::string remain_str(str_in);
    int index = 0;
    while ((index = remain_str.find_first_of(split_char)) >= 0)
    {
        if (index > 0)
        {
            str_out.push_back(remain_str.substr(0, index));
        }
        remain_str = remain_str.substr(index + 1, remain_str.size() - 1);
    }
    if (!remain_str.empty())
    {
        str_out.push_back(remain_str.substr(0, index - 1));
    }
}

inline bool SplitStringToIntList(const std::string &str, std::vector<int> &nums)
{
    std::string str_in(str);
    nums.clear();
    if(str_in.empty())
    {
        return false;
    }

    std::string str_t("");
    str_t += str_in.back();
    if("/" != str_t)
    {
        str_in += "/";
    }

    int substr_index = 0, colon_index = 0;
    int start_num = 0, end_num = 0;
    while(!str_in.empty())
    {
        substr_index = str_in.find_first_of("/");
        str_t = str_in.substr(0, substr_index);
        colon_index = str_t.find_first_of(":");
        if(colon_index >= 0)
        {
            start_num = StringToNum<int>(str_t.substr(0, colon_index));
            end_num = StringToNum<int>(str_t.substr(colon_index + 1, str_t.length()));
            for(int i = start_num; i <= end_num; ++i)
            {
                nums.push_back(i);
            }
        }
        else
        {
            start_num = SlamCommon::StringToNum<int>(str_t);
            nums.push_back(start_num);
        }
        str_in = str_in.substr(substr_index + 1, str_in.length());
    }
    return true;
}

template <typename T>
bool SplitStringToNumList(const std::string &str, std::vector<T> &nums, const char *split_flag)
{
    std::string str_in(str);
    nums.clear();
    if(str_in.empty())
    {
        return false;
    }

    std::string str_t("");
    str_t += str_in.back();
    if(split_flag != str_t)
    {
        str_in += split_flag;
    }

    int substr_index = 0;
    T num = 0;
    while(!str_in.empty())
    {
        substr_index = str_in.find_first_of(split_flag);
        str_t = str_in.substr(0, substr_index);
        num = SlamCommon::StringToNum<T>(str_t);
        nums.push_back(num);
        str_in = str_in.substr(substr_index + 1, str_in.length());
    }
    return true;
}

inline int StrHexToDec(const std::string &hex)
{
    std::stringstream ss;
    int i = 0;
    ss <<  std::hex << hex;
    ss >> i;
    return i;
}

inline std::string DecToStrHex(const int i)
{
    std::stringstream ss;
    ss << std::hex << i;
    std::string str("");
    ss >> str;
    return str;
}

inline double DegToRad(const double deg)
{
	return M_PI * deg / 180.;
}
inline double RadToDeg(const double rad)
{
	return 180 * rad / M_PI;
}

inline void Uint32ToUnsignedCharForSmall(const uint32_t in, unsigned char out[], const int len)
{
	for (int i = 0; i < len; ++i)
	{
		out[i] = ((unsigned char *)&in)[i];
	}
}
inline void Uint32ToUnsignedCharForBig(const uint32_t in, unsigned char out[], const int len)
{
	for (int i = 0; i < len; ++i)
	{
		out[i] = ((unsigned char *)&in)[len - 1 - i];
	}
}
inline void FloatToUnsignedChar(const float in, unsigned char *out)
{
	for (size_t i = 0; i < 4; i++)
	{
		out[i] = ((unsigned char *)&in)[i];
	}
}
inline float UnsignedCharToFloat(const unsigned char *in)
{
	return *(float *)in;
}
inline uint32_t UsignedCharToUint32(const unsigned char *in)
{
	return *(uint32_t *)in;
}
inline int16_t UnsignedCharToInt16(const unsigned char *in)
{
	return *(int16_t *)in;
}
inline void PolarCoordsToCartesianCoords(const double len, const double ang, double &x, double &y)
{
	x = len * cos(ang);
	y = len * sin(ang);
}
inline void CartesianCoordsToPolarCoords(const double x, const double y, double &len, double &ang)
{
    ang = atan2(y, x);
    len = std::hypot(x, y);
}
inline Eigen::Quaterniond EulerToQuaternion(const Eigen::Vector3d &euler_angle)
{
	double roll_half = euler_angle.x() * 0.5;
	double pitch_half = euler_angle.y() * 0.5;
	double yaw_half = euler_angle.z() * 0.5;
	double roll_sin = sin(roll_half);
	double roll_cos = cos(roll_half);
	double pitch_sin = sin(pitch_half);
	double pitch_cos = cos(pitch_half);
	double yaw_sin = sin(yaw_half);
	double yaw_cos = cos(yaw_half);
	Eigen::Quaterniond q;
	q.w() = roll_cos * pitch_cos * yaw_cos + roll_sin * pitch_sin * yaw_sin;
	q.x() = roll_sin * pitch_cos * yaw_cos - roll_cos * pitch_sin * yaw_sin;
	q.y() = roll_cos * pitch_sin * yaw_cos + roll_sin * pitch_cos * yaw_sin;
	q.z() = roll_cos * pitch_cos * yaw_sin - roll_sin * pitch_sin * yaw_cos;
	return q.normalized();
}
inline Eigen::Vector3d QuaterniondToEuler(const Eigen::Quaterniond &q)
{
	Eigen::Vector3d euler;
	double x = q.x(), y = q.y(), z = q.z(), w = q.w();
	euler.x() = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
	euler.y() = asin(2 * (w * y - z * x));
	euler.z() = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
	return euler;
}
inline Eigen::Vector3d MatrixToEuler(const Eigen::Matrix3d &m)
{
    //该方法角度范围为[0~180]
//    Eigen::Vector3d v(m.eulerAngles(2, 1, 0));  // ypr
//    return Eigen::Vector3d(v.z(), v.y(), v.x());// x-y-z, that is rpy
    //该方法角度范围为[-180~180]
    return m.eulerAngles(0, 1, 2);
}

// Returns an angle-axis vector (a vector with the length of the rotation angle
// pointing to the direction of the rotation axis) representing the same
// rotation as the given 'quaternion'.
template <typename T>
Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(
	const Eigen::Quaternion<T>& quaternion)
{
	Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
	// We choose the quaternion with positive 'w', i.e., the one with a smaller
	// angle that represents this orientation.
	if (normalized_quaternion.w() < 0.)
	{
		// Multiply by -1. http://eigen.tuxfamily.org/bz/show_bug.cgi?id=560
		normalized_quaternion.w() *= T(-1.);
		normalized_quaternion.x() *= T(-1.);
		normalized_quaternion.y() *= T(-1.);
		normalized_quaternion.z() *= T(-1.);
	}
	// We convert the normalized_quaternion into a vector along the rotation axis
	// with length of the rotation angle.
	const T angle = T(2.) * atan2(normalized_quaternion.vec().norm(),
		normalized_quaternion.w());
	const double kCutoffAngle = 1e-7;  // We linearize below this angle.
	const T scale = angle < kCutoffAngle ? T(2.) : angle / sin(angle / T(2.));
	return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
		scale * normalized_quaternion.y(),
		scale * normalized_quaternion.z());
}

// Returns a quaternion representing the same rotation as the given 'angle_axis' vector.
template <typename T>
Eigen::Quaternion<T> AngleAxisVectorToRotationQuaternion(
	const Eigen::Matrix<T, 3, 1>& angle_axis)
{
	T scale = T(0.5);
	T w = T(1.);
	const double kCutoffAngle = 1e-8;  // We linearize below this angle.

	if (angle_axis.squaredNorm() > kCutoffAngle)
	{
		const T norm = angle_axis.norm();
		scale = sin(norm / 2.) / norm;
		w = cos(norm / 2.);
	}
	const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
	return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(),
		quaternion_xyz.z()).normalized();
}

// Returns the non-negative rotation angle in radians of the 3D transformation
// 'transform'.
template <typename FloatType>
FloatType GetAngle(const Rigid3<FloatType>& transform) {
  return FloatType(2) * std::atan2(transform.rotation().vec().norm(),
                                   std::abs(transform.rotation().w()));
}

// Returns the yaw component in radians of the given 3D 'rotation'. Assuming
// 'rotation' is composed of three rotations around X, then Y, then Z, returns
// the angle of the Z rotation.
template <typename T>
T GetYaw(const Eigen::Quaternion<T>& rotation)
{
	const Eigen::Matrix<T, 3, 1> direction =
		rotation * Eigen::Matrix<T, 3, 1>::UnitX();
	return atan2(direction.y(), direction.x());
}

// Returns the yaw component in radians of the given 3D transformation
// 'transform'.
template <typename T>
T GetYaw(const Rigid3<T>& transform)
{
    return GetYaw(transform.rotation());
}

// Projects 'transform' onto the XY plane.
template <typename T>
Rigid2<T> Project2D(const Rigid3<T>& transform)
{
  return Rigid2<T>(transform.translation().template head<2>(),
                   GetYaw(transform));
}

// Embeds 'transform' into 3D space in the XY plane.
template <typename T>
Rigid3<T> Embed3D(const Rigid2<T>& transform) {
  return Rigid3<T>(
      {transform.translation().x(), transform.translation().y(), T(0)},
      Eigen::AngleAxis<T>(transform.rotation().angle(),
                          Eigen::Matrix<T, 3, 1>::UnitZ()));
}

//transform translation(x, y, z) and orientation(roll, pitch, yaw)
inline Eigen::Isometry3d TransformFromLocalToGlobal(const Eigen::Vector3d &trans,
	const Eigen::Quaterniond &rot, const Eigen::Isometry3d &local_in_global)
{
	Eigen::Isometry3d pt_in_local = Eigen::Isometry3d::Identity();
	pt_in_local.rotate(rot.toRotationMatrix());
	pt_in_local.pretranslate(trans);

	Eigen::Isometry3d pt_in_global = Eigen::Isometry3d::Identity();
	pt_in_global = local_in_global * pt_in_local;
	return pt_in_global;
}
//transform translation(x, y, z) and orientation(roll, pitch, yaw)
inline Eigen::Isometry3d TransformFromLocalToGlobal(const Eigen::Vector3d &trans,
	const Eigen::Vector3d &rot, const Eigen::Isometry3d &local_in_global)
{
	Eigen::Isometry3d pt_in_local = Eigen::Isometry3d::Identity();
	pt_in_local.rotate(SlamCommon::EulerToQuaternion(rot).toRotationMatrix());
	pt_in_local.pretranslate(trans);

	Eigen::Isometry3d pt_in_global = Eigen::Isometry3d::Identity();
	pt_in_global = local_in_global * pt_in_local;
	return pt_in_global;
}
//transform translation(x, y) and orientation(yaw)
inline Eigen::Isometry3d TransformFromLocalToGlobal(const Pose3D &pose,
	const Eigen::Isometry3d &local_in_global)
{
	Eigen::Isometry3d pt_in_local = Eigen::Isometry3d::Identity();
	pt_in_local.rotate(Eigen::AngleAxisd(pose.theta, Eigen::Vector3d::UnitZ()));
	pt_in_local.pretranslate(Eigen::Vector3d(pose.x, pose.y, 0.0));

	Eigen::Isometry3d pt_in_global = Eigen::Isometry3d::Identity();
	pt_in_global = local_in_global * pt_in_local;
	return pt_in_global;
}
//transform translation(x, y) and orientation(yaw)
inline Pose3D TransformFromLocalToGlobal(const Pose3D &pt_in_local,
                                         const Pose3D &local_in_global)
{
	Pose3D pt_in_global;
	pt_in_global.x = local_in_global.x + pt_in_local.x * cos(local_in_global.theta)
		- pt_in_local.y * sin(local_in_global.theta);
	pt_in_global.y = local_in_global.y + pt_in_local.x * sin(local_in_global.theta)
		+ pt_in_local.y * cos(local_in_global.theta);
	pt_in_global.theta = local_in_global.theta + pt_in_local.theta;
	pt_in_global.theta = NormalizeAngle(pt_in_global.theta);

	return pt_in_global;
}
//only can transform translation(x, y, z), can't transform orientation(roll, pitch, yaw)
inline Eigen::Vector3d TransformFromLocalToGlobal(const Eigen::Vector3d &pt_in_local,
                                                  const Eigen::Isometry3d &local_in_global)
{
	Eigen::Vector3d pt_in_global = Eigen::Vector3d::Zero();
	pt_in_global = local_in_global * pt_in_local;
	return pt_in_global;
}
//transform translation(x, y, z) and orientation(roll, pitch, yaw)
inline Eigen::Isometry3d TransformFromLocalToGlobal(const Eigen::Isometry3d &pt_in_local,
                                                    const Eigen::Isometry3d &local_in_global)
{
	Eigen::Isometry3d pt_in_global = Eigen::Isometry3d::Identity();
	pt_in_global = local_in_global * pt_in_local;
	return pt_in_global;
}
//transform translation(x, y, z) and orientation(yaw)
inline Eigen::Isometry3d TransformFromLocalToGlobal(const Eigen::Isometry3d &pt_in_local,
    const Eigen::Vector3d &local_in_global_trans, const Eigen::Vector3d &local_in_global_rot)
{
    Eigen::Isometry3d local_in_global = Eigen::Isometry3d::Identity();
    local_in_global.rotate(Eigen::AngleAxisd(local_in_global_rot.z(), Eigen::Vector3d::UnitZ()));
    local_in_global.pretranslate(local_in_global_trans);

    Eigen::Isometry3d pt_in_global = Eigen::Isometry3d::Identity();
    pt_in_global = local_in_global * pt_in_local;
    return pt_in_global;
}
//transform translation(x, y, z) and orientation(yaw)
inline Eigen::Isometry3d TransformFromLocalToGlobal(const Eigen::Isometry3d &pt_in_local,
    const Eigen::Vector3d &local_in_global_trans, const Eigen::Quaterniond &local_in_global_rot)
{
    Eigen::Isometry3d local_in_global = Eigen::Isometry3d::Identity();
    local_in_global.rotate(Eigen::AngleAxisd(SlamCommon::GetYaw(local_in_global_rot), Eigen::Vector3d::UnitZ()));
    local_in_global.pretranslate(local_in_global_trans);

    Eigen::Isometry3d pt_in_global = Eigen::Isometry3d::Identity();
    pt_in_global = local_in_global * pt_in_local;
    return pt_in_global;
}
//transform translation(x, y, z) and orientation(roll, pitch, yaw)
inline Eigen::Isometry3d TransformFromGlobalToLocal(const Eigen::Vector3d &trans,
	const Eigen::Vector3d &rot, const Eigen::Isometry3d &local_in_global)
{
	return TransformFromLocalToGlobal(trans, rot, local_in_global.inverse());
}
//transform translation(x, y, z) and orientation(roll, pitch, yaw)
inline Eigen::Isometry3d TransformFromGlobalToLocal(const Eigen::Vector3d &trans,
	const Eigen::Quaterniond &rot, const Eigen::Isometry3d &local_in_global)
{
	return TransformFromLocalToGlobal(trans, rot, local_in_global.inverse());
}
//transform translation(x, y, z) and orientation(roll, pitch, yaw)
inline Eigen::Isometry3d TransformFromGlobalToLocal(const Eigen::Isometry3d &pt_in_global,
                                                    const Eigen::Isometry3d &local_in_global)
{
	return TransformFromLocalToGlobal(pt_in_global, local_in_global.inverse());
}
//transform translation(x, y) and orientation(yaw)
inline Eigen::Isometry3d TransformFromGlobalToLocal(const Pose3D &pose,
                                                    const Eigen::Isometry3d &local_in_global)
{
	return TransformFromLocalToGlobal(pose, local_in_global.inverse());
}
//transform translation(x, y) and orientation(yaw)
inline Pose3D TransformFromGlobalToLocal(const Pose3D &pt_in_global,
                                         const Pose3D &local_in_global)
{
	Pose3D pt_in_local;
    pt_in_local.x = (pt_in_global.x - local_in_global.x) * cos(local_in_global.theta)
		+ (pt_in_global.y - local_in_global.y) * sin(local_in_global.theta);
	pt_in_local.y = -(pt_in_global.x - local_in_global.x) * sin(local_in_global.theta)
        + (pt_in_global.y - local_in_global.y) * cos(local_in_global.theta);
    pt_in_local.theta = NormalizeAngle(pt_in_global.theta - local_in_global.theta);

	return pt_in_local;
}
//inverse coordinate1-->coordinate2 to coordinate2-->coordinate1
inline Pose3D InverseTransform(const Pose3D &coord1_to_coord2)
{
	Pose3D tf_t, coord2_to_coord1;
	tf_t.x = -1. * coord1_to_coord2.x;
	tf_t.y = -1. * coord1_to_coord2.y;
	tf_t.theta = -1. * coord1_to_coord2.theta;

	coord2_to_coord1.x = tf_t.x * cos(tf_t.theta) - tf_t.y * sin(tf_t.theta);
	coord2_to_coord1.y = tf_t.x * sin(tf_t.theta) + tf_t.y * cos(tf_t.theta);
	coord2_to_coord1.theta = tf_t.theta;
	return coord2_to_coord1;
}

inline Eigen::Vector3d RotationTransform3D(const Eigen::Vector3d &rotation, const Eigen::Vector3d &src)
{
	//rotation matrix
	Eigen::Matrix3d x_mat;
	x_mat << 1, 0, 0,
		0, cos(rotation.x()), -sin(rotation.x()),
		0, sin(rotation.x()), cos(rotation.x());
	Eigen::Matrix3d y_mat;
	y_mat << cos(rotation.y()), 0, sin(rotation.y()),
		0, 1, 0,
		-sin(rotation.y()), 0, cos(rotation.y());
	Eigen::Matrix3d z_mat;
	z_mat << cos(rotation.z()), -sin(rotation.z()), 0,
		sin(rotation.z()), cos(rotation.z()), 0,
		0, 0, 1;
	//3D transform
	return x_mat * y_mat * z_mat * src;
}

inline Rigid3d Pose3DToRigid3d(const Pose3D &pose)
{
    return Rigid3d({pose.x, pose.y, 0.},
                   AngleAxisVectorToRotationQuaternion(
                       Eigen::Vector3d(0., 0., pose.theta)));
}

inline Pose3D Rigid3dToPose3D(const Rigid3d &pose)
{
    return Pose3D(pose.translation().x(), pose.translation().y(),
                  SlamCommon::GetYaw(pose.rotation()));
}

inline Rigid3d Pose6DVecQuaToRigid3d(const Pose6DVecQua &pose)
{
    return Rigid3d(pose.translation, pose.rotation);
}

inline Pose6DVecQua Rigid3dToPose6DVecQua(const Rigid3d &pose)
{
    return Pose6DVecQua(pose.translation(), pose.rotation());
}

inline Pose6DVecQua Pose3DToPose6DVecQua(const Pose3D &pose)
{
    return Pose6DVecQua(Eigen::Vector3d(pose.x, pose.y, 0.),
                        Eigen::Quaterniond(Eigen::AngleAxisd(
                                               pose.theta,
                                               Eigen::Vector3d::UnitZ())));
}

inline Pose3D Pose6DVecQuaToPose3D(const Pose6DVecQua &pose)
{
    return Pose3D(pose.translation.x(), pose.translation.y(),
                  SlamCommon::GetYaw(pose.rotation));
}

}
#endif // !FORMAT_TRANSFORM_H_
