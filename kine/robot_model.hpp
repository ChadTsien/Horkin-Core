// 机器人模型定义文件，对接P
#pragma once

#include "types/joint.hpp"
#include "types/pose.hpp"
#include "types/jacobian.hpp"

#include <memory>
#include <string>

namespace horkin 
{
    class RobotModel
    {
        public:
            // Load once at process start. Not for the RT loop.
            explicit RobotModel(const std::string& urdf_path,
                                const std::string& ee_frame = "tool");
            
            RobotModel(const RobotModel&) = delete;
            RobotModel& operator=(const RobotModel&) = delete;
            ~RobotModel();

            Pose fk(const JointVec& q) const;
            Jacobian jacobian(const JointVec& q) const;

            JointVec q_lower() const;
            JointVec q_upper() const;

        private:
            struct Impl;
            std::unique_ptr<Impl> impl_;  // 智能指针
    };
}