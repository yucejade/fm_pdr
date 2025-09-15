#include "pdr.h"

bool compare_time(double t_val, const PDRPosition &pos);

CFmPDR::CFmPDR(const PDRConfig &config, const CFmDataLoader &train_data, std::vector<PDRPosition> &train_position)
    : m_merge_direction_step(config, train_data, train_position)
{
}

CFmPDR::CFmPDR(const PDRConfig &config) : m_merge_direction_step(config)
{
}

CFmPDR::~CFmPDR()
{
}

StartInfo CFmPDR::start(const CFmDataLoader &start_data)
{
    return m_merge_direction_step.start(start_data);
}

// 普通函数实现：比较函数
bool compare_time(double t_val, const PDRPosition &pos)
{
    return t_val < pos.time;
}

// 普通成员函数实现：查找插值区间
size_t CFmPDR::find_interval(double t, const std::vector<PDRPosition> &trajectory) const
{
    // 使用普通函数作为比较器
    auto it = std::upper_bound(
        trajectory.begin(), trajectory.end(), t, compare_time);

    const size_t traj_size = trajectory.size();

    // 边界处理
    if (it == trajectory.begin())
        return 0;
    if (it == trajectory.end())
        return traj_size - 2;
    return std::distance(trajectory.begin(), it) - 1;
}

const std::vector<PDRPosition> CFmPDR::linear_interpolation(const Eigen::VectorXd &target_times,
                                                            const std::vector<PDRPosition> &trajectory)
{
    // 0. 处理边界情况
    const size_t traj_size = trajectory.size();
    if (traj_size == 0 || target_times.size() == 0)
        return {};

    // 1. 直接准备结果容器
    std::vector<PDRPosition> result;
    result.reserve(target_times.size());

    // 2. 处理只有一个轨迹点的情况
    if (traj_size == 1)
    {
        for (int i = 0; i < target_times.size(); ++i)
        {
            PDRPosition pos = trajectory[0];
            pos.time = target_times(i);
            result.push_back(pos);
        }
        return result;
    }

    // 3. 遍历所有目标时间点
    for (int i = 0; i < target_times.size(); ++i)
    {
        const double t = target_times(i);
        PDRPosition interp_pos;
        interp_pos.time = t;

        // 4. 边界处理
        if (t <= trajectory.front().time)
        {
            interp_pos = trajectory.front();
        }
        else if (t >= trajectory.back().time)
        {
            interp_pos = trajectory.back();
        }
        else
        {
            // 5. 使用普通成员函数查找合适的区间
            size_t idx = find_interval(t, trajectory);
            const PDRPosition &p0 = trajectory[idx];
            const PDRPosition &p1 = trajectory[idx + 1];

            // 6. 计算插值参数
            const double time_diff = p1.time - p0.time;
            const double ratio = (t - p0.time) / time_diff;

            // 7. 插值X和Y坐标
            interp_pos.x = p0.x + (p1.x - p0.x) * ratio;
            interp_pos.y = p0.y + (p1.y - p0.y) * ratio;

            // 8. 角度感知插值
            double v0 = p0.direction;
            double v1 = p1.direction;
            double diff = v1 - v0;

            // 处理角度环绕
            if (diff > 180.0)
                diff -= 360.0;
            else if (diff < -180.0)
                diff += 360.0;

            double interpolated_dir = v0 + diff * ratio;

            // 标准化到0-360范围
            if (interpolated_dir >= 360.0)
                interpolated_dir -= 360.0;
            else if (interpolated_dir < 0.0)
                interpolated_dir += 360.0;

            interp_pos.direction = interpolated_dir;
        }

        result.push_back(interp_pos);
    }

    return result;
}

std::vector<PDRPosition> CFmPDR::pdr(StartInfo &start_info, const CFmDataLoader &process_data)
{
    std::vector<PDRPosition> trajectory = m_merge_direction_step.merge_dir_step(start_info, process_data);

    Eigen::VectorXd time_location = Eigen::Map<const Eigen::VectorXd>(process_data.m_time_location.data(),
                                                                      process_data.m_time_location.size());
        
    // 插值
    return linear_interpolation(time_location, trajectory);
}

void CFmPDR::eval_model(const CFmDataLoader &data)
{
    data.eval_model();
}

// 线性插值函数
Eigen::VectorXd CFmPDR::linear_interpolation(const Eigen::VectorXd &time,
                                             const Eigen::VectorXd &time_data,
                                             const Eigen::VectorXd &data)
{
    Eigen::VectorXd data_interp(time.size());
    int i = 0;

    for (int idx = 0; idx < time.size(); ++idx)
    {
        double t = time[idx];

        // 找到合适的区间 [time_data[i], time_data[i+1]]
        while (i < time_data.size() - 2 && t >= time_data[i + 1])
            i++;

        // 确保索引在有效范围内
        if (i < time_data.size() - 1)
        {
            double t0 = time_data[i];
            double t1 = time_data[i + 1];
            double v0 = data[i];
            double v1 = data[i + 1];

            // 线性插值公式
            data_interp[idx] = v0 + (v1 - v0) / (t1 - t0) * (t - t0);
        }
        else
        {
            // 超出范围时使用最后一个值
            data_interp[idx] = data[data.size() - 1];
        }
    }

    return data_interp;
}

// PDR 主函数
void CFmPDR::pdr(CFmDataLoader &test_case,
                 const std::string &model_name,
                 double distance_frac_step,
                 int clean_data,
                 double optimized_mode_ratio,
                 double butter_Wn)
{
    // 获取合并的步伐
    MergeResult result = m_merge_direction_step.merge_dir_step(test_case, model_name, distance_frac_step,
                                                               clean_data, optimized_mode_ratio, butter_Wn);

    // 提取步数据
    Eigen::MatrixXd steps = result.trajectory;
    Eigen::VectorXd time_step = steps.col(0);
    Eigen::VectorXd x_step = steps.col(1);
    Eigen::VectorXd y_step = steps.col(2);
    Eigen::VectorXd part_direction_pred = result.direction_pred;

    // 创建时间位置向量
    Eigen::VectorXd time_location = Eigen::Map<Eigen::VectorXd>(
        test_case.m_time_location.data(), test_case.m_time_location.size());

    // 插值
    Eigen::VectorXd x_interp = linear_interpolation(time_location, time_step, x_step);
    Eigen::VectorXd y_interp = linear_interpolation(time_location, time_step, y_step);

    // 处理方向数据
    std::vector<double> direction_interp;

    // 添加初始 clean_data 个原始方向值
    for (int i = 0; i < clean_data && i < test_case.m_direction.size(); ++i)
        direction_interp.push_back(test_case.m_direction[i]);

    // 每50个点取平均
    for (int i = 0; i < part_direction_pred.size(); i += 50)
    {
        int end = std::min(i + 50, static_cast<int>(part_direction_pred.size()));
        Eigen::VectorXd segment = part_direction_pred.segment(i, end - i);
        direction_interp.push_back(segment.mean());
    }

    // 转换为 Eigen 向量
    Eigen::VectorXd dir_interp_eigen = Eigen::Map<Eigen::VectorXd>(
        direction_interp.data(), direction_interp.size());
    
    // 将结果存储到 test_case 中
    test_case.set_location_output(x_interp, y_interp, dir_interp_eigen);
}