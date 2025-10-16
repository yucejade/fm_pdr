#include "step_predictor.h"
#include "data_buffer_loader.h"
#include "data_file_loader.h"
#include "fm_pdr.h"
#include <Eigen/Dense>
#include <cmath>
#include <dlib/matrix.h>
#include <dlib/serialize.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

CFmStepPredictor::CFmStepPredictor( const PDRConfig& config, const CFmDataManager& train_data ) : m_config( config ), m_train_data( nullptr )
{
    int start = ( config.clean_start <= 0 ? 0 : config.clean_start ) * config.sample_rate;
    int end   = ( config.clean_end <= 0 ? ( int )train_data.get_train_data_size() : config.clean_end ) * config.sample_rate;

    if ( train_data.get_data_type() == DATA_TYPE_FILE )
    {
        const CFmDataFileLoader& file_loader = dynamic_cast< const CFmDataFileLoader& >( train_data );
        m_train_data                         = slice( file_loader, start, end );
    }
    else
    {
        const CFmDataBufferLoader& buffer_loader = dynamic_cast< const CFmDataBufferLoader& >( train_data );
        m_train_data                             = slice( buffer_loader, start, end );
    }
}

CFmStepPredictor::CFmStepPredictor( const PDRConfig& config ) : m_config( config ) {}

CFmStepPredictor::~CFmStepPredictor() {}

Eigen::VectorXd CFmStepPredictor::filter( int range, const Eigen::VectorXd& data )
{
    const int n = data.size();
    if ( n == 0 )
        throw std::invalid_argument( "Input data cannot be empty" );

    if ( range <= 0 || range > n )
        throw std::invalid_argument( "Filter range=" + std::to_string( range ) + " needs to be greater than or equal to 0 and less than " + std::to_string( n ) + "." );

    // 创建平均滤波器核
    Eigen::VectorXd kernel = Eigen::VectorXd::Ones( range ) / static_cast< double >( range );

    // 计算填充大小
    const int pad = ( range - 1 ) / 2;

    // 创建结果向量
    Eigen::VectorXd filter_data( n );

    // 实现与NumPy相同的卷积行为
    for ( int i = 0; i < n; ++i )
    {
        double sum = 0.0;

        // 对于每个输出位置，计算卷积和
        for ( int k = 0; k < range; ++k )
        {
            const int data_index = i - pad + k;

            // 处理边界情况（零填充）
            if ( data_index >= 0 && data_index < n )
                sum += data( data_index ) * kernel( k );
            // 对于超出边界的情况，NumPy使用零填充，所以不需要做任何操作
        }

        filter_data( i ) = sum;
    }

    return filter_data;
}

Eigen::VectorXi CFmStepPredictor::find_peaks( const VectorXd& data, int min_distance )
{
    vector< int > peak_indices;

    for ( int i = 1; i < data.size() - 1; ++i )
    {
        // 检测局部极大值
        if ( data( i ) > data( i - 1 ) && data( i ) > data( i + 1 ) )
        {
            bool valid = true;

            // 检查最小距离约束
            if ( ! peak_indices.empty() )
            {
                int last_idx = peak_indices.back();
                if ( i - last_idx < min_distance )  // 在0.4秒间隔范围内查找到新的波峰
                {
                    if ( data( i ) > data( last_idx ) )  // 大于原来波峰，则替换旧峰
                        peak_indices.pop_back();
                    else
                        valid = false;  // 小于等于原来波峰，丢弃当前峰，保留旧峰
                }
            }

            if ( valid )
                peak_indices.push_back( i );
        }
    }

    // 转换为Eigen向量
    return Map< VectorXi >( peak_indices.data(), peak_indices.size() );
}

Eigen::VectorXi CFmStepPredictor::find_real_peak_indices( const Eigen::VectorXd& data, int range, int min_distance, Eigen::VectorXd& filtered_accel_data, double& valid_peak_value, bool is_train )
{
    // 滤波处理
    filtered_accel_data = filter( range, data );

    // 峰值检测
    Eigen::VectorXi peak_indices = find_peaks( filtered_accel_data, min_distance );

    // 计算峰值均值
    Eigen::VectorXd peak_values = filtered_accel_data( peak_indices );
    if ( is_train )
    {
        double mean_peak = peak_values.mean();
        valid_peak_value = 0.8 * mean_peak;
    }

    // 筛选有效峰值>80%均值
    Eigen::Array< bool, Eigen::Dynamic, 1 > valid_mask = peak_values.array() > valid_peak_value;
    Eigen::VectorXi                         real_peak_indices( valid_mask.count() );
    Eigen::Index                            mask_size = valid_mask.size();
    int                                     count     = 0;
    for ( int i = 0; i < mask_size; ++i )
        if ( valid_mask[ i ] )
            real_peak_indices[ count++ ] = peak_indices[ i ];

    return real_peak_indices;
}

double CFmStepPredictor::compute_variance( const Eigen::VectorXd& data, int start_idx, int end_idx )
{
    // 边界检查
    const int n = data.size();
    if ( start_idx < 0 || end_idx > n || start_idx >= end_idx )
        return 0.0;  // 返回安全值

    // 计算数据段长度
    const int segment_size = end_idx - start_idx + 1;

    // 使用Eigen的block获取数据子集
    const auto segment = data.segment( start_idx, segment_size );

    // 计算均值
    const double mean = segment.mean();

    // 计算方差
    const double variance = ( segment.array() - mean ).square().sum() / segment_size;

    return variance;
}

FeatureMatrix CFmStepPredictor::calculate_features( const CFmDataManager& data, const Eigen::VectorXi& real_peak_indices, const Eigen::VectorXd& filtered_accel_data, int start_step_index, int end_step_index )
{
    // 计算频率f
    const VectorXd& data_time     = data.get_pdr_data( PDR_DATA_FIELD_TIME );
    double          time_interval = data_time[ real_peak_indices[ end_step_index ] ] - data_time[ real_peak_indices[ start_step_index ] ];
    double          f             = ( end_step_index - start_step_index ) / time_interval;

    // 计算方差sigma
    double sigma = compute_variance( filtered_accel_data, real_peak_indices[ start_step_index ], real_peak_indices[ end_step_index ] );

    // 存储特征
    FeatureMatrix features;
    features( 0 ) = f;
    features( 1 ) = sigma;

    return features;
}

FeatureMatrix CFmStepPredictor::calculate_features( const Eigen::VectorXi& real_peak_indices, const Eigen::VectorXd& filtered_accel_data, int start_step_index, int end_step_index )
{
    // 计算频率f
    const VectorXd& train_data_time = m_train_data->get_pdr_data( PDR_DATA_FIELD_TIME );
    double          time_interval   = train_data_time[ real_peak_indices[ end_step_index ] ] - train_data_time[ real_peak_indices[ start_step_index ] ];
    double          f               = ( end_step_index - start_step_index ) / time_interval;

    // 计算方差sigma
    double sigma = compute_variance( filtered_accel_data, real_peak_indices[ start_step_index ], real_peak_indices[ end_step_index ] );

    // 存储特征
    FeatureMatrix features;
    features( 0 ) = f;
    features( 1 ) = sigma;

    return features;
}

LinearModel CFmStepPredictor::select_model( const string& model_str, std::vector< FeatureMatrix >& x, std::vector< double >& y )
{
    LinearModel model;

    if ( model_str == "Linear" )
    {
        // 线性回归
        dlib::rr_trainer< dlib::linear_kernel< FeatureMatrix > > trainer;
        trainer.set_lambda( 0 );
        model = trainer.train( x, y );
    }
#if 0
    else if (model_str == "DecisionTree")
    {
        // Dlib没有直接的决策树回归器，使用回归树代替
        dlib::regression_tree_trainer trainer;
        model = trainer.train(x, y);
    }
    else if (model_str == "SVR")
    {
        // SVR回归 - 需要指定核类型
        typedef dlib::linear_kernel<FeatureMatrix> kernel_type;
        dlib::svr_trainer<kernel_type> trainer;
        model = trainer.train(x, y);
    }
    else if (model_str == "KNeighbors")
    {
        knn_regressor knn(5);
        knn.train(x, y);
        model = knn;
    }
    else if (model_str == "RandomForest")
    {
        // 随机森林回归
        dlib::random_forest_regression_trainer trainer;
        trainer.set_num_trees(20);
        model = trainer.train(x, y);
    }
    else if (model_str == "AdaBoost")
    {
        // AdaBoost回归 - Dlib可能不支持，使用梯度提升代替
        dlib::gradient_tree_regression_trainer trainer;
        trainer.set_num_trees(50);
        model = trainer.train(x, y);
    }
    else if (model_str == "GradientBoosting")
    {
        // 梯度提升回归树
        dlib::gradient_tree_regression_trainer trainer;
        trainer.set_num_trees(100);
        model = trainer.train(x, y);
    }
    else if (model_str == "Bagging")
    {
        // Bagging回归 - Dlib可能不支持，使用其他回归器代替
        dlib::linear_least_squares_regression_trainer trainer;
        model = trainer.train(x, y);
    }
    else if (model_str == "ExtraTree")
    {
        throw std::runtime_error("ExtraTree not supported in Dlib");
    }
#endif
    else
    {
        throw std::runtime_error( "Unknown model: " + model_str );
    }

    return model;
}

void CFmStepPredictor::save_model( const LinearModel& model, const double& valid_peak_value, const std::string& filename )
{
    std::ofstream fout( filename, std::ios::binary );
    if ( ! fout )
        throw std::runtime_error( "Unable to open file: " + filename );

    // 使用dlib的序列化机制
    dlib::serialize( model, fout );
    dlib::serialize( valid_peak_value, fout );
}

void CFmStepPredictor::save_model( const double& mean_model, const double& valid_peak_value, const std::string& filename )
{
    std::ofstream fout( filename, std::ios::binary );
    if ( ! fout )
        throw std::runtime_error( "Unable to open file: " + filename );

    // 使用dlib的序列化机制
    dlib::serialize( mean_model, fout );
    dlib::serialize( valid_peak_value, fout );
}

void CFmStepPredictor::load_model( const std::string& filename, LinearModel& model, double& valid_peak_value )
{
    std::ifstream fin( filename, std::ios::binary );
    if ( ! fin )
        throw std::runtime_error( "Unable to open file: " + filename );

    // 使用dlib的反序列化机制
    dlib::deserialize( model, fin );
    dlib::deserialize( valid_peak_value, fin );
}

void CFmStepPredictor::load_model( const std::string& filename, double& mean_model, double& valid_peak_value )
{
    std::ifstream fin( filename, std::ios::binary );
    if ( ! fin )
        throw std::runtime_error( "Unable to open file: " + filename );

    // 使用dlib的反序列化机制
    dlib::deserialize( mean_model, fin );
    dlib::deserialize( valid_peak_value, fin );
}

// 步长回归处理函数
LinearModel CFmStepPredictor::step_process_regression( const std::string& model_str, int move_average, int min_distance, size_t distance_frac_step, const std::string& save_model_name, double& valid_peak_value, bool write_log )
{
    Eigen::VectorXd filtered_accel_data;
    const VectorXd& accelerometer_data_mag = m_train_data->get_pdr_data( PDR_DATA_FIELD_ACC_MAG );
    Eigen::VectorXi real_peak_indices      = find_real_peak_indices( accelerometer_data_mag, move_average, min_distance, filtered_accel_data, valid_peak_value, true );

    // 特征提取
    const VectorXd&              train_data_time      = m_train_data->get_pdr_data( PDR_DATA_FIELD_TIME );
    const VectorXd&              train_true_data_time = m_train_data->get_true_data( TRUE_DATA_FIELD_TIME );
    Eigen::Index                 step_index           = 0;
    Eigen::Index                 n_segments           = m_train_data->get_train_data_size() / distance_frac_step;  // 按每个坐标点分段计算一次步长sigma、f
    std::vector< FeatureMatrix > x;
    std::vector< double >        y;

    for ( Eigen::Index i = 1; i < n_segments; ++i )
    {
        Eigen::Index last_step_index = step_index;

        // 寻找当前段内的步数
        while ( train_data_time[ real_peak_indices[ step_index ] ] <= train_true_data_time[ i * distance_frac_step ] )
            step_index++;
        if ( step_index == last_step_index )
            continue;  // 没有检测到步伐，跳过

        // 计算行走距离
        const VectorXd& true_data_x = m_train_data->get_true_data( TRUE_DATA_FIELD_X );
        const VectorXd& true_data_y = m_train_data->get_true_data( TRUE_DATA_FIELD_Y );
        double          distance    = 0.0;
        Eigen::Index    start_idx   = ( i - 1 ) * distance_frac_step;
        Eigen::Index    end_idx     = i * distance_frac_step;
        for ( Eigen::Index k = start_idx; k < end_idx; ++k )
        {
            double dx = true_data_x[ k + 1 ] - true_data_x[ k ];
            double dy = true_data_y[ k + 1 ] - true_data_y[ k ];
            distance += std::sqrt( dx * dx + dy * dy );
        }

        // 计算步长（目标值）
        double step_length = distance / ( step_index - last_step_index );
        y.push_back( step_length );

        // 存储特征
        FeatureMatrix features = calculate_features( real_peak_indices, filtered_accel_data, last_step_index, step_index );
        x.push_back( features );
    }

    LinearModel model = select_model( model_str, x, y );

    // 保存模型（可选）
    if ( ! save_model_name.empty() )
        save_model( dlib::any_cast< LinearModel >( model ), valid_peak_value, save_model_name );

    // LinearModel m;
    // double vpv = 0.0;
    // load_model(save_model_name, m, vpv);

    // 输出预测结果（可选）
    if ( write_log )
    {
        // std::cout << "Actual: " << valid_peak_value << ", valid_peak_value: " << vpv << std::endl;

        for ( Eigen::Index i = 0; i < ( Eigen::Index )y.size(); ++i )
        {
            double prediction = model( x[ i ] );
            std::cout << "Actual: " << y[ i ] << ", Predicted: " << prediction << std::endl;
        }
    }

    return model;
}

double CFmStepPredictor::step_process_mean( int move_average, int min_distance, const std::string& save_model_name, double& valid_peak_value )
{
    Eigen::VectorXd filtered_accel_data;
    const VectorXd& accelerometer_data_mag = m_train_data->get_pdr_data( PDR_DATA_FIELD_ACC_MAG );
    Eigen::VectorXi real_peak_indices      = find_real_peak_indices( accelerometer_data_mag, move_average, min_distance, filtered_accel_data, valid_peak_value, true );

    // 计算总移动距离
    const VectorXd& train_data_time      = m_train_data->get_pdr_data( PDR_DATA_FIELD_TIME );
    const VectorXd& train_true_data_time = m_train_data->get_true_data( TRUE_DATA_FIELD_TIME );
    int             n                    = m_train_data->get_train_data_size();
    Eigen::VectorXd dx                   = m_train_data->get_true_data( TRUE_DATA_FIELD_X ).tail( n - 1 ) - m_train_data->get_true_data( TRUE_DATA_FIELD_X ).head( n - 1 );
    Eigen::VectorXd dy                   = m_train_data->get_true_data( TRUE_DATA_FIELD_Y ).tail( n - 1 ) - m_train_data->get_true_data( TRUE_DATA_FIELD_Y ).head( n - 1 );
    double          total_distance       = ( dx.array().square() + dy.array().square() ).sqrt().sum();

    // 统计有效步数
    Eigen::VectorXd peak_times = train_data_time( real_peak_indices );
    int             step_count = ( peak_times.array() < train_true_data_time[ n - 1 ] ).count();

    // 计算平均步长
    double step_length = ( step_count > 0 ) ? total_distance / step_count : 0.0;

    // 保存模型（可选）
    if ( ! save_model_name.empty() )
        save_model( step_length, valid_peak_value, save_model_name );

    return step_length;
}