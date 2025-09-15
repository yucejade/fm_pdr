#include <algorithm>
#include "direction_predictor.h"
#include "DSP-Cpp-filters/so_butterworth_lpf.h"

using namespace Dsp;

CFmDirectionPredictor::CFmDirectionPredictor(double butter_Wn)
{
    // f.setup(sampling_rate, cutoff_freq);
    m_f.setupN(butter_Wn);
}

CFmDirectionPredictor::~CFmDirectionPredictor()
{
}

// 实现零相位滤波 (Eigen版本)
Eigen::VectorXd CFmDirectionPredictor::filtfilt(Iir::Butterworth::LowPass<2, Iir::DirectFormII> &filter, const Eigen::VectorXd &input)
{
    const int N = input.size();
    if (N < 3)
        return input;

    // 1. 镜像填充 (使用Eigen块操作)
    const int pad_len = std::min(100, N / 2);
    Eigen::VectorXd padded(2 * pad_len + N);

    // 前端镜像填充 (前pad_len个元素的反向)
    padded.head(pad_len) = input.head(pad_len).reverse();

    // 原始数据
    padded.segment(pad_len, N) = input;

    // 后端镜像填充 (后pad_len个元素的反向)
    padded.tail(pad_len) = input.tail(pad_len).reverse();

    // 2. 预热滤波器建立稳态
    filter.reset();
    double init_val = padded[0];
    for (int i = 0; i < 1000; i++)
        filter.filter(init_val);

    // 3. 正向滤波
    Eigen::VectorXd forward(padded.size());
    for (int i = 0; i < padded.size(); i++)
        forward[i] = filter.filter(padded[i]);

    // 4. 反向滤波
    filter.reset();
    Eigen::VectorXd reversed = forward.reverse();
    Eigen::VectorXd backward(reversed.size());
    for (int i = 0; i < reversed.size(); i++)
        backward[i] = filter.filter(reversed[i]);

    // 5. 反转并裁剪结果
    Eigen::VectorXd full_result = backward.reverse();
    return full_result.segment(pad_len, N);
}

void CFmDirectionPredictor::butterworth_filter(const CFmDataLoader &data, MatrixXd &mag, MatrixXd &grv)
{
    m_f.reset();

    // 低通滤波
    VectorXd x = filtfilt(m_f, data.m_m_x);
    VectorXd y = filtfilt(m_f, data.m_m_y);
    VectorXd z = filtfilt(m_f, data.m_m_z);

    mag.col(0) = x;
    mag.col(1) = y;
    mag.col(2) = z;

    // 对 a 低通滤波得到重力加速度
    VectorXd g_x = filtfilt(m_f, data.m_g_x);
    VectorXd g_y = filtfilt(m_f, data.m_g_y);
    VectorXd g_z = filtfilt(m_f, data.m_g_z);

    grv.col(0) = g_x;
    grv.col(1) = g_y;
    grv.col(2) = g_z;
}

Eigen::MatrixXd CFmDirectionPredictor::calc_east_vector(const MatrixXd &mag, const MatrixXd &grv, const int &rows)
{
    const int k_cols = 3;
    Eigen::MatrixXd e(rows, k_cols);

    for (int i = 0; i < rows; ++i)
    {
        Eigen::Vector3d m_vec = mag.row(i);        // 提取磁场向量
        Eigen::Vector3d g_vec = grv.row(i);        // 提取重力向量
        e.row(i) = g_vec.cross(m_vec).transpose(); // 叉乘得到东向量
    }

    return e;
}

StartInfo CFmDirectionPredictor::start(const CFmDataLoader &start_data, const int least_point)
{
    // 必须有两个及以上点才能计算方向
    const size_t mag_rows = start_data.m_m.rows();
    if ((int)mag_rows <= least_point)
        throw std::invalid_argument("Input data length must be greater than " + std::to_string(least_point));

    const int k_rows = 50;
    const int k_cols = 3;
    Eigen::Index data_rows = mag_rows;
    MatrixXd mag(data_rows, k_cols);
    MatrixXd grv(data_rows, k_cols);
    butterworth_filter(start_data, mag, grv);

    // 计算前50行东向量
    int number_of_point = std::min(k_rows, (int)data_rows);
    Eigen::MatrixXd e = calc_east_vector(mag, grv, number_of_point);

    // 东向量平均值作为初始东向量
    Vector3d no_opt_e0 = e.colwise().mean();

    // 计算前least_point个点平均方向作为计算初始direction
    // 计算与北方向的角度（0°=北，90°=东），角度规范化到 [0, 360) 范围
    const int sample_count = std::min(least_point, (int)(data_rows - 1)); // 取前least_point段位移
    Eigen::Vector2d avg_delta(0, 0);

    for (int i = 0; i < sample_count; ++i)
    {
        avg_delta.x() += start_data.m_m_x[i + 1] - start_data.m_m_x[i];
        avg_delta.y() += start_data.m_m_y[i + 1] - start_data.m_m_y[i];
    }
    avg_delta /= sample_count; // 平均位移向量

    double heading_rad = std::atan2(avg_delta.x(), avg_delta.y());
    double heading_deg = heading_rad * 180.0 / M_PI;
    if (heading_deg < 0)
        heading_deg += 360.0;
    double no_opt_direction0 = heading_deg;

    return {no_opt_e0.x(), no_opt_e0.y(), no_opt_e0.z(), no_opt_direction0};
}

Eigen::VectorXd CFmDirectionPredictor::predict_direction(const StartInfo &start_info, const CFmDataLoader &process_data)
{
    // 必须有两个及以上点才能计算方向
    const size_t mag_rows = process_data.m_m.rows();
    if (mag_rows <= 0)
        throw std::invalid_argument("Input data length must be greater than 0");

    const int k_cols = 3;
    Eigen::Index rows = mag_rows;
    MatrixXd mag(rows, k_cols);
    MatrixXd grv(rows, k_cols);
    butterworth_filter(process_data, mag, grv); //TODO: 第一次送过来的数据进行了2次巴特沃斯滤波

    // 计算所有行东向量
    Eigen::MatrixXd e = calc_east_vector(mag, grv, rows);

    // 求出所有东向量和初始东向量的角度
    Vector3d no_opt_e0(start_info.e0_x, start_info.e0_y, start_info.e0_z);
    double norm_e0 = no_opt_e0.norm();
    VectorXd no_opt_angles(rows);
    VectorXd no_opt_signs(rows);

    for (int i = 0; i < rows; ++i)
    {
        // 显式创建固定大小向量
        Vector3d current_e(e.row(i)[0], e.row(i)[1], e.row(i)[2]);
        Vector3d current_g(grv.row(i)[0], grv.row(i)[1], grv.row(i)[2]);

        // 角度计算
        double dot_val = current_e.dot(no_opt_e0);
        double norm_ei = current_e.norm();
        double cos_angle = dot_val / (norm_ei * norm_e0);
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
        no_opt_angles[i] = std::acos(cos_angle) * 180.0 / M_PI;

        // 叉积和点积计算
        Vector3d cross_vec = current_e.cross(no_opt_e0);
        double dot_cg = cross_vec.dot(current_g);

        no_opt_signs[i] = ((dot_cg > 0) ? -1.0 : (dot_cg < 0) ? 1.0
                                                              : 0.0);
    }

    // 计算预测方向并取模
    VectorXd no_opt_direction_pred = no_opt_signs.cwiseProduct(no_opt_angles).array() + start_info.direction0;

    // 取模360并处理负值
    no_opt_direction_pred = no_opt_direction_pred.unaryExpr([](double x)
                                                            {x = std::fmod(x, 360.0);
                                                             return (x < 0) ? x + 360.0 : x; });

    return no_opt_direction_pred;
}

// 计算两个方向之间的最小夹角
double CFmDirectionPredictor::direction_diff(double a, double b)
{
    // 确保角度在 [0, 360) 范围内
    a = std::fmod(a, 360.0);
    b = std::fmod(b, 360.0);
    if (a < 0)
        a += 360.0;
    if (b < 0)
        b += 360.0;

    // 计算两种可能的夹角
    double diff1 = std::abs(a - b);
    double diff2 = 360.0 - diff1;

    // 返回较小的夹角
    return std::min(diff1, diff2);
}

// 向量化版本：计算两个方向序列之间的最小夹角
VectorXd CFmDirectionPredictor::direction_diff(const VectorXd &a, const VectorXd &b)
{
    assert(a.size() == b.size());
    VectorXd result(a.size());

    for (int i = 0; i < a.size(); ++i)
    {
        result[i] = direction_diff(a[i], b[i]);
    }

    return result;
}

// 按比例混合两个方向
VectorXd CFmDirectionPredictor::direction_mix(const VectorXd &a, const VectorXd &b, double ratio)
{
    // 检查输入向量长度是否一致
    if (a.size() != b.size())
    {
        throw std::invalid_argument("Input vectors must have the same size");
    }

    // 创建结果向量
    VectorXd result(a.size());

    // 遍历每个元素进行计算
    for (int i = 0; i < a.size(); ++i)
    {
        // 归一化角度到 [0, 360)
        double a_norm = std::fmod(a[i], 360.0);
        double b_norm = std::fmod(b[i], 360.0);
        if (a_norm < 0)
            a_norm += 360.0;
        if (b_norm < 0)
            b_norm += 360.0;

        // 计算绝对角度差（考虑圆上最短路径）
        double diff = std::abs(a_norm - b_norm);
        if (diff > 180.0)
        {
            diff = 360.0 - diff;
        }

        // 根据角度差选择混合方式
        if (diff < 180.0)
        {
            // 直接线性混合
            result[i] = a_norm * ratio + b_norm * (1 - ratio);
        }
        else
        {
            // 跨0°调整后混合
            double adjusted = (a_norm - 360.0) * ratio + b_norm * (1 - ratio);
            result[i] = std::fmod(adjusted, 360.0);
            // 确保结果在 [0, 360)
            if (result[i] < 0)
                result[i] += 360.0;
        }
    }
    return result;
}

Eigen::VectorXd CFmDirectionPredictor::predict_direction(const CFmDataLoader &data_loader,
                                                         double optimized_mode_ratio,
                                                         const int butter_N,
                                                         double butter_Wn)
{
    m_f.reset();

    // 低通滤波
    VectorXd m_x = filtfilt(m_f, data_loader.m_m_x);
    VectorXd m_y = filtfilt(m_f, data_loader.m_m_y);
    VectorXd m_z = filtfilt(m_f, data_loader.m_m_z);

    MatrixXd m_m;
    m_m.resize(m_x.size(), 3);
    m_m.col(0) = m_x;
    m_m.col(1) = m_y;
    m_m.col(2) = m_z;

    // 对 a 低通滤波得到重力加速度
    VectorXd m_g_x = filtfilt(m_f, data_loader.m_g_x);
    VectorXd m_g_y = filtfilt(m_f, data_loader.m_g_y);
    VectorXd m_g_z = filtfilt(m_f, data_loader.m_g_z);

    MatrixXd m_g;
    m_g.resize(m_g_x.size(), 3);
    m_g.col(0) = m_g_x;
    m_g.col(1) = m_g_y;
    m_g.col(2) = m_g_z;

    // 创建结果矩阵
    Eigen::MatrixXd e(m_g.rows(), 3);

    // 逐行计算叉乘
    for (int i = 0; i < m_g.rows(); ++i)
    {
        Eigen::Vector3d g_vec = m_g.row(i);        // 提取重力向量
        Eigen::Vector3d m_vec = m_m.row(i);        // 提取磁场向量
        e.row(i) = g_vec.cross(m_vec).transpose(); // 叉乘得到东向量
    }

    // 第 10% 附近的 50 个东向量平均值作为初始东向量
    int no_opt_size = data_loader.m_direction.size();
    int total_samples = e.rows();
    Vector3d no_opt_e0 = e.block(50 * no_opt_size - 50, 0, 50, 3).colwise().mean().transpose();

    // 第 10% 的 direction 作为初始 direction
    double no_opt_direction0 = data_loader.m_direction[no_opt_size - 1];

    // 求出所有东向量和初始东向量的角度
    double norm_e0 = no_opt_e0.norm();
    VectorXd no_opt_angles(total_samples);
    VectorXd no_opt_signs(total_samples);

    for (int i = 0; i < total_samples; ++i)
    {
        // 显式创建固定大小向量
        Vector3d current_e(e.row(i)[0], e.row(i)[1], e.row(i)[2]);
        Vector3d current_g(m_g.row(i)[0], m_g.row(i)[1], m_g.row(i)[2]);

        // 角度计算
        double dot_val = current_e.dot(no_opt_e0);
        double norm_ei = current_e.norm();
        double cos_angle = dot_val / (norm_ei * norm_e0);
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
        no_opt_angles[i] = std::acos(cos_angle) * 180.0 / M_PI;

        // 叉积和点积计算
        Vector3d cross_vec = current_e.cross(no_opt_e0);
        double dot_cg = cross_vec.dot(current_g);

        no_opt_signs[i] = (dot_cg > 0) ? -1.0 : (dot_cg < 0) ? 1.0
                                                             : 0.0;
    }

    // 计算预测方向并取模
    VectorXd no_opt_direction_pred = no_opt_signs.cwiseProduct(no_opt_angles).array() + no_opt_direction0;

    // 取模360并处理负值
    no_opt_direction_pred = no_opt_direction_pred.unaryExpr([](double x)
                                                            {
        x = std::fmod(x, 360.0);
        return (x < 0) ? x + 360.0 : x; });

    ///////////////
    // 取前 10% 个东向量平均值作为初始东向量
    ///////////////
    VectorXd opt_direction_pred(total_samples);

    // 1. 计算初始东向量（前10%的平均）
    int opt_size = data_loader.m_len_input * 50;
    Vector3d opt_e0 = e.topRows(opt_size).colwise().mean();

    // 2. 计算所有向量与初始向量的夹角
    double norm_opt_e0 = opt_e0.norm();
    VectorXd opt_angles(total_samples);
    VectorXd opt_signs(total_samples);

    for (int i = 0; i < total_samples; ++i)
    {
        // 1. 显式创建固定大小向量（关键修改）
        Vector3d current_e(e.row(i)[0], e.row(i)[1], e.row(i)[2]);
        Vector3d current_g(m_g.row(i)[0], m_g.row(i)[1], m_g.row(i)[2]);

        // 2. 角度计算（保持不变）
        double dot_val = current_e.dot(opt_e0);
        double norm_ei = current_e.norm();
        double cos_angle = dot_val / (norm_ei * norm_opt_e0);
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
        opt_angles[i] = std::acos(cos_angle) * 180.0 / M_PI;

        // 3. 叉积和点积计算（使用固定大小向量）
        Vector3d cross_vec = current_e.cross(opt_e0);
        double dot_cg = cross_vec.dot(current_g);

        opt_signs[i] = (dot_cg > 0) ? -1.0 : (dot_cg < 0) ? 1.0
                                                          : 0.0;
    }

    // 4. 计算方向偏移（每50点取平均）
    VectorXd direction_offset(data_loader.m_len_input);
    for (int i = 0, idx = 0; i < opt_size; i += 50, ++idx)
    {
        // 提取50个样本的符号和角度
        VectorXd segment_angles = opt_angles.segment(i, 50);
        VectorXd segment_signs = opt_signs.segment(i, 50);

        // 计算符号*角度
        VectorXd segment = segment_signs.cwiseProduct(segment_angles);

        // 计算平均值
        direction_offset[idx] = segment.mean();
    }

    // 5. 定义误差函数并最小化
    double best_offset = 0.0;
    double min_error = std::numeric_limits<double>::max();

    // 网格搜索寻找最佳偏移（替代scipy.optimize.minimize）
    for (double offset = 0.0; offset < 360.0; offset += 1.0)
    {
        double total_error = 0.0;

        // 计算当前偏移下的误差
        for (int i = 0; i < (int)data_loader.m_len_input; ++i)
        {
            double predicted = direction_offset[i] + offset;
            total_error += direction_diff(predicted, data_loader.m_direction[i]);
        }

        double avg_error = total_error / data_loader.m_len_input;

        // 更新最小误差和最佳偏移
        if (avg_error < min_error)
        {
            min_error = avg_error;
            best_offset = offset;
        }
    }

    // 6. 计算最终方向预测
    VectorXd signed_angles = opt_signs.cwiseProduct(opt_angles);
    opt_direction_pred = signed_angles.array() + best_offset;

    // 7. 取模360并处理负值
    for (int i = 0; i < total_samples; ++i)
    {
        opt_direction_pred[i] = std::fmod(opt_direction_pred[i], 360.0);
        if (opt_direction_pred[i] < 0)
            opt_direction_pred[i] += 360.0;
    }

    // std::ofstream outfile("filtered_output.txt");
    // outfile << "=============================method1=============================\n";
    // outfile << "e0=(" << no_opt_e0[0] << "," << no_opt_e0[1] << "," << no_opt_e0[2] << "), direction=(" << no_opt_direction0 << ")\n";

    // for (int i = 0; i < e.rows(); ++i)
    //     outfile << i << ", e=(" << e.row(i).col(0) << "," << e.row(i).col(1) << "," << e.row(i).col(2) << "), direction=("
    //             << no_opt_angles[i] << "," << no_opt_signs[i] << "," << no_opt_direction_pred[i] << ")\n";
    // outfile << "=============================method2=============================\n";
    // outfile << "e0=(" << opt_e0[0] << "," << opt_e0[1] << "," << opt_e0[2] << "), direction=(" << best_offset << ")\n";

    // for (int i = 0; i < e.rows(); ++i)
    //     outfile << i << ", e=(" << e.row(i).col(0) << "," << e.row(i).col(1) << "," << e.row(i).col(2) << "), direction=("
    //             << opt_angles[i] << "," << opt_signs[i] << "," << opt_direction_pred[i] << ")\n";
    // outfile.close();

    return direction_mix(opt_direction_pred, no_opt_direction_pred, optimized_mode_ratio);
}