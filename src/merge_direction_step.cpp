#include "merge_direction_step.h"

CFmMergeDirectionStep::CFmMergeDirectionStep(const PDRConfig &config,
                                             const CFmDataLoader &train_data,
                                             std::vector<PDRPosition> &train_position)
    : m_config(config),
      m_valid_peak_value(0.0f),
      m_mean_step(0.0f),
      m_step_predictor(train_data, config.clean_data, 0),
      m_direction_predictor(config.butter_wn)
{
    // 添加切片后的原始轨迹点
    for (Eigen::Index i = 0; i < (Eigen::Index)train_data.m_len_input; ++i)
    {
        PDRPosition position = {train_data.m_time_location[i],
                                train_data.m_x[(Eigen::Index)i],
                                train_data.m_y[(Eigen::Index)i],
                                train_data.m_direction[(Eigen::Index)i]};
        train_position.push_back(position);
    }

    // 获取训练数据方向预测信息
    // StartInfo si = m_direction_predictor.start(train_data, m_config.least_start_point);
    // Eigen::VectorXd direction_pred = m_direction_predictor.predict_direction(si, train_data);

    // 步长模型选择
    if (string(config.model_name) != "Mean")
        m_model = m_step_predictor.step_process_regression(config.model_name,
                                                           config.move_average,
                                                           config.min_distance,
                                                           config.distance_frac_step,
                                                           config.model_file_name,
                                                           m_valid_peak_value,
                                                           false);
    else
        m_mean_step = m_step_predictor.step_process_mean(config.move_average,
                                                         config.min_distance,
                                                         config.model_file_name,
                                                         m_valid_peak_value);
}

CFmMergeDirectionStep::CFmMergeDirectionStep(const PDRConfig &config)
    : m_config(config),
      m_valid_peak_value(0.0f),
      m_mean_step(0.0f),
      m_step_predictor(),
      m_direction_predictor(config.butter_wn)
{
    // 步长模型选择
    if (string(config.model_name) != "Mean")
        m_step_predictor.load_model(config.model_file_name, m_model, m_valid_peak_value);
    else
        m_step_predictor.load_model(config.model_file_name, m_mean_step, m_valid_peak_value);
}

CFmMergeDirectionStep::~CFmMergeDirectionStep()
{
}

StartInfo CFmMergeDirectionStep::start(const CFmDataLoader &start_data)
{
    StartInfo si = m_direction_predictor.start(start_data, m_config.least_start_point);
    si.last_x = 0;
    si.last_y = 0;
    return si;
}

// TODO: 暂时限定除最后一个送进来的数据，其它必须是查找峰值间隔数（20）的整数倍
std::vector<PDRPosition> CFmMergeDirectionStep::merge_dir_step(StartInfo &start_info, const CFmDataLoader &process_data)
{
    // 存储轨迹的列表
    std::vector<PDRPosition> trajectory_list;

    // 预测方向
    Eigen::VectorXd direction_pred = m_direction_predictor.predict_direction(start_info, process_data);
    
    Eigen::VectorXd filtered_accel_data;
    Eigen::VectorXi real_peak_indices = m_step_predictor.find_real_peak_indices(process_data.m_a_mag,
                                                                                m_config.move_average,
                                                                                m_config.min_distance,
                                                                                filtered_accel_data,
                                                                                m_valid_peak_value);

    Eigen::Index peak_size = real_peak_indices.size();
    for (Eigen::Index i = 1; i < peak_size; ++i)
    {
        // 预测步长
        double step_pred;
        if (std::string(m_config.model_name) != "Mean")
        {
            FeatureMatrix features = m_step_predictor.calculate_features(process_data, real_peak_indices, filtered_accel_data, i - 1, i);
            step_pred = m_model(features);
        }
        else
        {
            step_pred = m_mean_step;
        }

        // 计算平均方向
        double mean_direction;
        int start_idx = real_peak_indices[i];
        int end_idx = (i == real_peak_indices.size() - 1 ? direction_pred.size() - real_peak_indices[i]
                                                         : real_peak_indices[i + 1] - real_peak_indices[i] + 1);
        Eigen::VectorXd dir_segment = direction_pred.segment(start_idx, end_idx);
        mean_direction = dir_segment.mean();

        // 计算位移
        double rad = mean_direction * M_PI / 180.0;
        double dx = step_pred * std::cos(rad);
        double dy = step_pred * std::sin(rad);

        start_info.last_x = trajectory_list.empty() ? start_info.last_x : trajectory_list.back().x;
        start_info.last_y = trajectory_list.empty() ? start_info.last_y : trajectory_list.back().y;

        // 更新位置，这里修改为存储每一步的方向
        PDRPosition pos;
        pos.time = process_data.m_time[real_peak_indices[i]];
        pos.x = start_info.last_x + dx;
        pos.y = start_info.last_y + dy;
        pos.direction = mean_direction;
        trajectory_list.push_back(pos);
    }

    return trajectory_list;
}

MergeResult CFmMergeDirectionStep::merge_dir_step(const CFmDataLoader &test_case,
                                                  const std::string &model_name,
                                                  double distance_frac_step,
                                                  int clean_data,
                                                  double optimized_mode_ratio,
                                                  double butter_Wn)
{
    // 存储轨迹的列表
    std::vector<double> time_list;
    std::vector<double> x_list;
    std::vector<double> y_list;
    double mean_step = 0.0;

    // 添加初始clean_data个点
    for (size_t i = 0; i < (size_t)clean_data && i < test_case.m_time_location.size(); ++i)
    {
        time_list.push_back(test_case.m_time_location[i]);
        x_list.push_back(test_case.m_x[(Eigen::Index)i]);
        y_list.push_back(test_case.m_y[(Eigen::Index)i]);
    }

    // 切片处理
    CFmDataLoader sliced_case = slice(test_case, clean_data, 0);

    // 方向预测
    Eigen::VectorXd direction_pred = m_direction_predictor.predict_direction(sliced_case, optimized_mode_ratio,
                                                                             2, butter_Wn);

    // 步长模型选择
    AnyModel model;
    if (model_name != "Mean")
        model = m_step_predictor.step_process_regression(model_name,
                                                         m_config.move_average,
                                                         m_config.min_distance,
                                                         distance_frac_step,
                                                         "model.dat",
                                                         m_valid_peak_value,
                                                         true);
    else
        mean_step = m_step_predictor.step_process_mean(m_config.move_average,
                                                       m_config.min_distance,
                                                       m_config.model_file_name,
                                                       m_valid_peak_value);

    Eigen::VectorXd filtered_accel_data;
    const Eigen::VectorXi &real_peak_indices = m_step_predictor.find_real_peak_indices(sliced_case.m_a_mag,
                                                                                       m_config.move_average,
                                                                                       m_config.min_distance,
                                                                                       filtered_accel_data,
                                                                                       m_valid_peak_value);

    // 确定步处理的起始点
    Eigen::Index test_begin = sliced_case.m_x.size() - 1; // sliced_case.m_x.size()作用与sliced_case.m_len_input一样
    Eigen::Index step_test_begin = 0;
    while (sliced_case.m_time[real_peak_indices[step_test_begin]] < sliced_case.m_time_location[test_begin])
        step_test_begin++;

    // 添加切片后的原始轨迹点
    for (Eigen::Index i = 0; i < sliced_case.m_x.size(); ++i)
    {
        time_list.push_back(sliced_case.m_time_location[i]);
        x_list.push_back(sliced_case.m_x[i]);
        y_list.push_back(sliced_case.m_y[i]);
    }

    Eigen::Index peak_size = real_peak_indices.size();
    for (Eigen::Index i = (step_test_begin > 0 ? step_test_begin : 1); i < peak_size; ++i)
    {
        // 预测步长
        double step_pred;
        if (model_name != "Mean")
        {
            FeatureMatrix features = m_step_predictor.calculate_features(sliced_case, real_peak_indices,
                                                                         filtered_accel_data,
                                                                         i - 1, i);
            step_pred = model(features);
        }
        else
        {
            step_pred = mean_step;
        }

        // 计算平均方向
        double mean_direction;
        int start_idx = real_peak_indices[i];
        int end_idx = (i == real_peak_indices.size() - 1 ? direction_pred.size() - real_peak_indices[i]
                                                         : real_peak_indices[i + 1] - real_peak_indices[i] + 1);
        Eigen::VectorXd dir_segment = direction_pred.segment(start_idx, end_idx);
        mean_direction = dir_segment.mean();

        // 计算位移
        double rad = mean_direction * M_PI / 180.0;
        double dx = step_pred * std::cos(rad);
        double dy = step_pred * std::sin(rad);

        // 更新位置
        time_list.push_back(sliced_case.m_time[real_peak_indices[i]]);
        x_list.push_back(x_list.back() + dx);
        y_list.push_back(y_list.back() + dy);
    }

    // 构造轨迹矩阵
    Eigen::MatrixXd trajectory(time_list.size(), 3);
    for (size_t i = 0; i < time_list.size(); ++i)
    {
        trajectory(i, 0) = time_list[i];
        trajectory(i, 1) = x_list[i];
        trajectory(i, 2) = y_list[i];
    }

    return {trajectory, direction_pred};
}