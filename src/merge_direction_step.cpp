#include "merge_direction_step.h"
#include "fm_pdr.h"

CFmMergeDirectionStep::CFmMergeDirectionStep(const PDRConfig &config,
                                             const CFmDataManager &train_data,
                                             Eigen::MatrixXd &train_position)
    : m_config(config),
      m_valid_peak_value(0.0f),
      m_mean_step(0.0f),
      m_step_predictor(config, train_data),
      m_direction_predictor(config)
{
    // 添加切片后的原始轨迹点
    train_position.resize(train_data.get_train_data_size(), 4);
    train_position.col(0) = train_data.get_true_data(TRUE_DATA_FIELD_TIME);
    train_position.col(1) = train_data.get_true_data(TRUE_DATA_FIELD_LATITUDE);
    train_position.col(2) = train_data.get_true_data(TRUE_DATA_FIELD_LONGITUDE);
    train_position.col(3) = train_data.get_true_data(TRUE_DATA_FIELD_DIRECTION);
    
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
      m_step_predictor(config),
      m_direction_predictor(config)
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

StartInfo CFmMergeDirectionStep::start(const CFmDataManager &start_data)
{
    StartInfo si = m_direction_predictor.start(start_data, m_config.least_start_point);
    si.last_x = 0;
    si.last_y = 0;
    return si;
}

// TODO: 暂时限定除最后一个送进来的数据，其它必须是查找峰值间隔数（20）的整数倍
Eigen::MatrixXd CFmMergeDirectionStep::merge_dir_step(StartInfo &start_info, const CFmDataManager &process_data)
{
    // 预测方向
    Eigen::VectorXd direction_pred = m_direction_predictor.predict_direction(start_info, process_data);
    // for (auto dp : direction_pred)
    //     cout << dp << ",";
    // cout << endl;

    Eigen::VectorXd filtered_accel_data;
    const VectorXd& accelerometer_data_mag = process_data.get_pdr_data(PDR_DATA_FIELD_ACC_MAG);
    Eigen::VectorXi real_peak_indices = m_step_predictor.find_real_peak_indices(accelerometer_data_mag,
                                                                                m_config.move_average,
                                                                                m_config.min_distance,
                                                                                filtered_accel_data,
                                                                                m_valid_peak_value);
    // for (auto idx : real_peak_indices)
    //     cout << idx << ",";
    // cout << "size: " << real_peak_indices.size() << endl;
    
    Eigen::Index peak_size = real_peak_indices.size();
    Eigen::MatrixXd trajectory(peak_size - 1, 4);

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

        start_info.last_x = (i == 1) ? start_info.last_x : trajectory(i - 2, 1);
        start_info.last_y = (i == 1) ? start_info.last_y : trajectory(i - 2, 2);

        // 更新位置，这里修改为存储每一步的方向
        const VectorXd& process_data_time = process_data.get_pdr_data(PDR_DATA_FIELD_TIME);
        trajectory(i - 1, 0) = process_data_time[real_peak_indices[i]];
        trajectory(i - 1, 1) = start_info.last_x + dx;
        trajectory(i - 1, 2) = start_info.last_y + dy;
        trajectory(i - 1, 3) = mean_direction;

        // cout << "time: " << process_data_time[real_peak_indices[i]] << ", x: " << start_info.last_x + dx << ", y: " << start_info.last_y + dy << ", direction: " << mean_direction << endl;
    }

    return trajectory;
}