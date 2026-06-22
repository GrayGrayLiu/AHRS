/******************************************************************************
 * @file    Aided_INS.hpp
 * @brief
 *
 * @details
 *
 * @author  Gray
 * @email   grayme12345@gmail.com
 * @date    2026/4/7
 *
 * @copyright
 * Copyright (c) 2026 Gray
 *
 * This software is provided "as is" without any warranties.
 *
 ******************************************************************************/

#ifndef AHRS_AIDED_INS_H
#define AHRS_AIDED_INS_H

#include <cstdint>
#include <cstdio>
#include <Eigen/Dense>
#include "Aided_INS_Types.hpp"

// [PROFILE] 临时：分段耗时诊断结构体
struct InsProfile
{
    uint32_t process_new_data_us{0};
    uint32_t ins_propagation_us{0};
    uint32_t ins_mech_us{0};
    uint32_t phi_f_q_g_us{0};
    uint32_t ekf_predict_us{0};
    uint32_t acc_feedback_us{0};
    // [PROFILE] Phase-2: fmx 子段
    uint32_t fmx_alc_us{0};
    uint32_t fmx_fill_us{0};
    uint32_t fmx_q1_us{0};
    uint32_t fmx_q2_us{0};
    // [ACC_DBG] 临时 AccUpdate 诊断计数
    uint32_t acc_try{0};
    uint32_t acc_accept{0};
    uint32_t acc_fail_small{0};
    uint32_t acc_fail_norm{0};
    uint32_t acc_fail_cos{0};
    uint32_t acc_feedback{0};
    float    last_f_norm{0.0f};
    float    last_gravity{0.0f};
    float    last_norm_diff{0.0f};
    float    last_cos_gn_gb{0.0f};
    float    last_f_b[3]{0.0f, 0.0f, 0.0f};
    float    last_g_b_ByImu[3]{0.0f, 0.0f, 0.0f};
    // [EKF_DBG] 临时 EKFPredict 分段计时
    uint32_t ekf_dx_us{0};
    uint32_t ekf_phi_p_us{0};
    uint32_t ekf_m_phi_t_q_us{0};
    // [ATT_DBG] 临时姿态/重力/比力一致性诊断
    float    euler_r_deg{0.0f};
    float    euler_p_deg{0.0f};
    float    euler_y_deg{0.0f};
    float    g_l_n[3]{0.0f, 0.0f, 0.0f};
    float    g_b_ByImu[3]{0.0f, 0.0f, 0.0f};
    float    cbn_f_b[3]{0.0f, 0.0f, 0.0f};
    float    cbn_f_plus_g[3]{0.0f, 0.0f, 0.0f};
    float    cbn_f_plus_g_norm{0.0f};
    float    cos_f_gb{0.0f};
};

class Aided_INS
{
public:
    explicit Aided_INS(uint8_t id);

    /**
     * @brief 组合惯性导航系统初始化
     * @param
     * @retval
     */
    int Init();

    /**
     * @brief  注入外部 IMU 增量数据，供后续 GetImuData() 和 Run() 消费。
     * @param  imu  已聚合到 200 Hz 的 IMU delta 数据，坐标系为应用机体系 b 系/FRD，单位匹配 INS 输入要求。
     */
    void SetImuData(const Aided_INS_Space::IMU &imu);

    /**
      * @brief 运行导航算法
      * @param
      * @retval
      */
    int Run();

    // [PROFILE] 临时：获取最近一次 Run() 的分段耗时
    const InsProfile &GetLastProfile() const { return profile_; }

private:
    using NavState = Aided_INS_Space::NavState;
    using IMU      = Aided_INS_Space::IMU;
    using GNSS     = Aided_INS_Space::GNSS;
    using PVA      = Aided_INS_Space::PVA;
    using ImuError = Aided_INS_Space::ImuError;
    using Mag      = Aided_INS_Space::Mag;
    using Config   = Aided_INS_Space::Config;
    using Matrix3d = Eigen::Matrix3d;
    using Vector3d = Eigen::Vector3d;

    // 固定尺寸矩阵实验：用编译期常量替代动态 MatrixXd
    static constexpr int kStateRank = 21;
    static constexpr int kNoiseRank = 18;

    using StateVector      = Eigen::Matrix<double, kStateRank, 1>;
    using StateMatrix      = Eigen::Matrix<double, kStateRank, kStateRank>;
    using NoiseMatrix      = Eigen::Matrix<double, kNoiseRank, kNoiseRank>;
    using NoiseDriveMatrix = Eigen::Matrix<double, kStateRank, kNoiseRank>;

    template<int MeasDim>
    using MeasurementVector = Eigen::Matrix<double, MeasDim, 1>;

    template<int MeasDim>
    using MeasurementMatrix = Eigen::Matrix<double, MeasDim, kStateRank>;

    template<int MeasDim>
    using MeasurementNoise = Eigen::Matrix<double, MeasDim, MeasDim>;

    enum class InsStatus : uint8_t
    {
        Unaligned = 0,
        Aligning,
        Running
    };
    InsStatus status_{InsStatus::Unaligned};
    uint32_t alignStartTime_{0};
    Vector3d alignGyroSum_ = Vector3d::Zero();
    Vector3d alignAccSum_  = Vector3d::Zero();
    Vector3d alignMagSum_  = Vector3d::Zero();
    uint32_t alignCount_{0};
    uint32_t alignMagCount_{0};

    /**
     * @brief 初始对准
     * @param
     * @retval
     */
    int InitialAlignment();

    /**
     * @brief 获取IMU数据
     */
    bool GetImuData();

    /**
     * @brief 获取磁力计数据
     */
    bool GetMagData();

    /**
     * @brief
     * @return
     */
    static Config LoadConfig();

    /**
     * @brief 初始化系统状态和协方差
     *        initialize state and state covariance
     */
    void Initialize();

    /**
    * @brief 内插增量形式的IMU数据到指定时刻
    *        interpolate incremental imudata to given timestamp
    * @param [in]     imuPre      前一时刻IMU数据
    *                           the previous imudata
    * @param [in,out] imuCur      当前时刻IMU数据
    *                           the current imudata
    * @param [in]     timestamp 给定内插到的时刻
    *                           given interpolate timestamp
    * @param [in,out] middle    输出内插时刻的IMU数据
    *                           output imudata at given timestamp
    */
    static void imuInterpolate(const IMU &imuPre, IMU &imuCur, const double timestamp, IMU &middle)
    {

        if (imuPre.time > timestamp || imuCur.time < timestamp)
            return;

        const double lamda = (timestamp - imuPre.time) / (imuCur.time - imuPre.time);

        middle.time       = timestamp;
        middle.deltaTheta = imuCur.deltaTheta * lamda;
        middle.deltaVel   = imuCur.deltaVel * lamda;
        middle.dt         = timestamp - imuPre.time;

        imuCur.deltaTheta = imuCur.deltaTheta - middle.deltaTheta;
        imuCur.deltaVel   = imuCur.deltaVel - middle.deltaVel;
        imuCur.dt         = imuCur.dt - middle.dt;
    }

    /**
    * @brief 当前IMU误差补偿到IMU数据中
    *        componsate imu error to the imudata
    * @param [in,out] imu 需要补偿的IMU数据
    *                     imudata to be compensated
    */
    void ImuCompensate(IMU &imu) const;

    /**
     * @brief 进行INS状态更新(IMU机械编排算法), 并计算IMU状态转移矩阵和噪声阵
     *        do INS state update(INS mechanization), and compute state transition matrix and noise matrix
     * @param [in,out] imuPre 前一时刻IMU数据
     *                        imudata at the previous epoch
     * @param [in,out] imuCur 当前时刻IMU数据
     *                        imudata at the current epoch
     */
    void InsPropagation(const IMU &imuPre, IMU &imuCur);

    /**
     * @brief Kalman 预测,
     *        Kalman Filter Predict process
     * @param [in,out] Phi 状态转移矩阵
     *                     state transition matrix
     * @param [in,out] Q  传播噪声矩阵
     *                     propagation noise matrix
     */
    void EKFPredict(const StateMatrix &Phi, const StateMatrix &Q);

    /**
     * @brief 加速度计观测方程
     */
    bool AccUpdate(const IMU& imuData, const PVA& pvaCur, const Config& config);

    /**
     * @brief 磁力计观测方程
     */
    void MagUpdate(const Mag &magData, const PVA &pvaCur, const Config &config);

    /**
     * @brief 使用GNSS位置观测更新系统状态
     */
    void GnssUpdate(GNSS &gnssData);

    /**
     * @brief Kalman 更新（固定测量维度模板）
     */
    template<int MeasDim>
    void EkfUpdate(const MeasurementVector<MeasDim> &dz, const MeasurementMatrix<MeasDim> &H,
                   const MeasurementNoise<MeasDim> &R);

    /**
     * @brief 反馈误差状态到当前状态
     *        feedback error state to the current state
     */
    void StateFeedback();

    enum class KfUpdateType: uint8_t
    {
        None = 0,  //不需要KF更新
        Prev,  //需要KF更新上一IMU状态
        Curr,  //需要KF更新当前IMU状态
        Middle //需要将IMU进行内插到观测量更新时间
    };

    /**
     * @brief 判断是否需要KF更新
     * @param imuTime1 上一IMU状态时间
     * @param imuTime2 当前IMU状态时间
     * @param updateTime 观测量更新的时间
     * @return 需要KF的更新类型
     */
    [[nodiscard]] KfUpdateType IsToUpdate(double imuTime1, double imuTime2, double updateTime) const;

    /**
     * @brief 处理新数据
     */
    void ProcessNewData();

    // ========================================================================
    // 结构化传播辅助函数
    // ========================================================================
    // 下列 helper 是对 Kalman 传播公式的等价块结构实现，不改变数学模型：
    //   Qbase = G * q * Gᵀ
    //   Q     = Phi * Qbase * Phiᵀ
    //   M     = Phi * P
    // 优化利用 G、Phi、Qbase 的固定 3×3 块结构，P 仍按满矩阵处理。
    // ========================================================================

    // --- BuildGQGt / BuildGQGtFromNoise ---
    /**
     * @brief 结构化 Qbase = G * q_ * Gᵀ（接受显式 q 参数，供验证和正式路径共用）
     */
    static void BuildGQGtFromNoise(const Matrix3d &Cbn, const NoiseMatrix &q, StateMatrix &Qbase);

    /** @brief 薄封装：BuildGQGtFromNoise(Cbn, q_, Qbase) */
    void BuildGQGt(const Matrix3d &Cbn, StateMatrix &Qbase) const;

    // --- AccumulatePhiQbaseColumn / BuildPhiQbasePhiT ---
    /** @brief 行对累加：Qout(i,j) += Phi(i,k)*Qkk*Phi(j,k)ᵀ，对单个 k 列 */
    static void AccumulatePhiQbaseColumn(const StateMatrix &Phi, const StateMatrix &Qbase,
                                         int k_id, const int *rows, int row_count,
                                         StateMatrix &Qout);

    /** @brief 结构化 Q = Phi * Qbase * Phiᵀ（利用 Qbase 仅 6 个对角块非零和 Phi 的块稀疏结构） */
    static void BuildPhiQbasePhiT(const StateMatrix &Phi, const StateMatrix &Qbase, StateMatrix &Qout);

    // --- AccumulatePhiPRow / BuildPhiTimesP ---
    /** @brief 行累加：Mout.row(i) += Σ_k Phi(i,k)*P.row(k)，k 遍历 cols[] */
    static void AccumulatePhiPRow(const StateMatrix &Phi, const StateMatrix &P,
                                  int row_id, const int *cols, int col_count,
                                  StateMatrix &Mout);

    /** @brief 结构化 M = Phi * P（利用 Phi 的 3×3 块稀疏结构，P 视为满矩阵） */
    static void BuildPhiTimesP(const StateMatrix &Phi, const StateMatrix &P, StateMatrix &Mout);

    /** @brief 结构化 Pout = M * Phiᵀ + Q（利用 Phi 块稀疏，M/Q 按满矩阵处理，不做对称优化） */
    static void BuildMTimesPhiTAndAddQ(const StateMatrix &M, const StateMatrix &Phi,
                                       const StateMatrix &Q, StateMatrix &Pout);

#if 1  // 验证用：确认后改为 #if 0 即可关闭
    /** @brief 数值验证：确认结构化实现与稠密参考等价 */
    static void VerifyStructuredQ();
#endif

    Config config_;
    uint8_t id_{0};

    double timestamp_{0.0};

    // 更新时间对齐误差，IMU状态和观测信息误差小于它则认为两者对齐
    const double TIME_ALIGN_ERR_ = 0.001;

    // IMU、磁力计和GNSS原始数据
    IMU imuPre_;
    IMU imuCur_;
    bool imuReady_{false};
    Mag magData_;
    GNSS gnssData_;

    // IMU状态（位置、速度、姿态和IMU误差）
    PVA pvaCur_;
    PVA pvaPre_;
    ImuError imuError_;

    // Kalman滤波相关
    StateMatrix P_;  //协方差矩阵
    NoiseMatrix q_;  //系统噪声矩阵
    StateVector dx_; //误差状态向量

    // 状态ID和噪声ID
    enum StateID { P_ID = 0, V_ID = 3, PHI_ID = 6, GB_ID = 9, AB_ID = 12, GS_ID = 15, AS_ID = 18 };
    enum NoiseID { VRW_ID = 0, ARW_ID = 3, GBSTD_ID = 6, ABSTD_ID = 9, GSSTD_ID = 12, ASSTD_ID = 15 };

    // [PROFILE] 临时：分段耗时快照
    InsProfile profile_;

    // 固定尺寸 scratch buffer：避免每次调用 InsPropagation/EkfUpdate 都 malloc
    StateMatrix      Phi_scratch_;   // 状态转移矩阵（复用）
    StateMatrix      F_scratch_;     // F 矩阵（复用）
    StateMatrix      Q_scratch_;     // Q 传播噪声矩阵（复用）
    StateMatrix      I_scratch_;     // EkfUpdate 临时 I 矩阵（复用）
    StateMatrix      Qbase_pre_scratch_; // BuildGQGt(pre-Cbn) 输出（复用）
    StateMatrix      Qbase_cur_scratch_; // BuildGQGt(cur-Cbn) 输出（复用）
    StateMatrix      M_scratch_;         // EKFPredict: M = Phi * P_ 临时矩阵

    //传感器驱动类
    // ImuDriver& imu_;
    // MagDriver& mag_;
    // GNSSDriver& gnss_;
};

#endif //AHRS_AIDED_INS_H
