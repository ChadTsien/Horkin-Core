#include "kine/robot_model.hpp"

#include "robot_model.hpp"
#include "types/joint_layout.hpp"
#include "types/pose.hpp"
#include "types/joint.hpp"
#include "types/jacobian.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <cstddef>
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/frame.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/joint/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <Eigen/Core>

#include <pinocchio/spatial/fwd.hpp>
#include <stdexcept>
#include <string>

namespace horkin
{
    struct RobotModel::Impl
    {
        pinocchio::Model model;
        mutable pinocchio::Data data;
        pinocchio::FrameIndex ee{0};  // 末端坐标系
        JointVec q_lower{};
        JointVec q_upper{};
    };

    namespace
    {
        bool is_prismatic(const pinocchio::JointModel& j)
        {
            const std::string s = j.shortname();
            return s.find("Prismatic") != std::string::npos ||
           s == "JointModelPX" || s == "JointModelPY" || s == "JointModelPZ";

        }

        // 类型转换，外部用的是 std::array, 而涉及到 pinocchio的需要 Eigen::VectorXd
        Eigen::VectorXd to_qpin(const pinocchio::Model& model, const JointVec& q)
        {
            Eigen::VectorXd qpin(model.nq);
            for (int i = 0; i < model.nq; ++i)
            {
                qpin[i] = q[static_cast<std::size_t>(i)];
            }
            return qpin;
        }
    }

    RobotModel::RobotModel(const std::string& urdf_path, const std::string& ee_frame)
        : impl_(std::make_unique<Impl>())
    {   
        // 读取 URDF，并将模型写入impl_->model
        pinocchio::urdf::buildModel(urdf_path, impl_->model, /*verbose=*/false);

        // 比较 URDF 读取到的关节数是否和定义数 kNJoints 相等
        if (impl_->model.nq != static_cast<int>(kNJoints))
        {
            throw std::runtime_error(
                "URDF nq=" + std::to_string(impl_->model.nq) +
                " != kNJoints=" + std::to_string(kNJoints));
        }

        std::size_t logical = 0;
        for (pinocchio::JointIndex i = 1; i < static_cast<pinocchio::JointIndex>(impl_->model.njoints); ++i)
        {
            const pinocchio::JointModel& j = impl_->model.joints[i];

            // 检查可活动关节数是否和定义的对齐
            if (j.nq() == 0)   // Pinocchio 中 nq()==0 表示固定关节，nq()==1 表示转动 / 移动关节
            {
                continue;
            }
            if (logical >= kNJoints)
            {
                throw std::runtime_error("URDF has more actuated joints than kNJoints");
            }

            // 检查可关节类型是否和定义的对齐
            const bool urdf_pris = is_prismatic(j);
            const bool layout_pris = (kJointKind[logical] == JointKind::Prismatic);
            if (urdf_pris != layout_pris)
            {
                throw std::runtime_error("URDF joint " + std::to_string(logical) + 
                                         " kind does not match kJointKind");
            }
            ++logical;
        }
        if (logical != kNJoints)
        {
            throw std::runtime_error("URDF actuated joint count != kNJoints");
        }

        if (!impl_->model.existFrame(ee_frame))
        {
            throw std::runtime_error("URDF missing frame: " + ee_frame);
        }

        impl_->ee = impl_->model.getFrameId(ee_frame, pinocchio::FIXED_JOINT);
        impl_->data = pinocchio::Data(impl_->model);

        // 读取关节限位
        for (int i = 0; i < impl_->model.nq; ++i)
        {
            const auto k = static_cast<std::size_t>(i);
            impl_->q_lower[k] = impl_->model.lowerPositionLimit[i];
            impl_->q_upper[k] = impl_->model.upperPositionLimit[i];
        }
    }

    RobotModel::~RobotModel() = default;

    JointVec RobotModel::q_lower() const { return impl_->q_lower; }
    JointVec RobotModel::q_upper() const { return impl_->q_upper; }

    /*  前向运动学  */
    Pose RobotModel::fk(const JointVec& q) const
    {
        const Eigen::VectorXd qpin = to_qpin(impl_->model, q);

        pinocchio::forwardKinematics(impl_->model, impl_->data, qpin);
        pinocchio::updateFramePlacements(impl_->model, impl_->data);

        const pinocchio::SE3& M = impl_->data.oMf[impl_->ee];
        const Eigen::Vector3d& t = M.translation();
        const Eigen::Matrix3d& R = M.rotation();

        Pose pose;
        pose.position = {t.x(), t.y(), t.z()};
        pose.rotation = {
            R(0, 0), R(0, 1), R(0, 2),
            R(1, 0), R(1, 1), R(1, 2),
            R(2, 0), R(2, 1), R(2, 2),
        };
        return pose;
    }

    /*  雅可比矩阵  */
    Jacobian RobotModel::jacobian(const JointVec& q) const
    {
        const Eigen::VectorXd qpin = to_qpin(impl_->model, q);

        // model.nv 表示速度的维度， 转/平动：1， 球关节：3， 浮动基：6
        Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, impl_->model.nv);
        J.setZero();  // Pinocchio 要先清零

        pinocchio::computeFrameJacobian(impl_->model, impl_->data, qpin, 
                                      impl_->ee, pinocchio::LOCAL_WORLD_ALIGNED, J);
        
        
        Jacobian out{};
        for (std::size_t col = 0; col < kNJoints; ++col)
            for (std::size_t row = 0; row < 6; ++row)
                out[row][col] = J(static_cast<int>(row), static_cast<int>(col));

        return out;
    }

}