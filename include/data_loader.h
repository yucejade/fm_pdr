#pragma once
#include "fm_pdr.h"
#include <eigen3/Eigen/Dense>
#include <rapidcsv.h>

using namespace std;
using namespace rapidcsv;
using namespace Eigen;

class CFmDataLoader
{
public:
    CFmDataLoader( const string& test_case_path = "" );
    ~CFmDataLoader();

    friend CFmDataLoader slice( const CFmDataLoader& loader, size_t start, size_t end );
    void          set_location_output( const std::vector< PDRPosition >& positions, const int from );
    void          set_location_output( VectorXd x, VectorXd y, VectorXd direction );
    void          eval_model() const;
private:
    Document load_csv( const string& filename );
    void     load_data_from_csv();

    MatrixXd nearest_neighbor_interpolation( const VectorXd& time_query, const VectorXd& time_data, const MatrixXd& data ) const;
    MatrixXd extract_eigen_matrix( Document& data, int start_col, int end_col, long num_rows );
    bool     save_to_csv( const Eigen::MatrixXd& matrix, const std::string& filename, const vector< string >& col_names );
    void     preprocess_data( bool is_save = false );

    VectorXd magnitude( const MatrixXd& matrix );
    void     generate_data();

    double        get_dir_error() const;
    double        get_dir_ratio( double diff = 15.0 ) const;
    double        get_dist_error() const;
public:
    vector< double > m_time_location;  // public
    size_t           m_len_input = 0;  // public
    VectorXd         m_time;           // public
    MatrixXd         m_m;              // public 计算数据尺寸用，可替代
    VectorXd         m_m_x;            // public 获取水平方向
    VectorXd         m_m_y;            // public 获取水平方向
    VectorXd         m_m_z;            // public 获取东向
    VectorXd         m_a_mag;          // public 计算步长
    VectorXd         m_g_x;            // public 获取东向
    VectorXd         m_g_y;            // public 获取东向
    VectorXd         m_g_z;            // public 获取东向
    VectorXd         m_direction;      // public 原始代码需要使用，做方向判断
    VectorXd         m_x;              // public 训练结果用
    VectorXd         m_y;              // public 训练结果用
private:
    static constexpr double kK = 1e5;
    string                  m_test_case_path;                // private
    Document                m_doc_accelerometer;             // private
    Document                m_doc_linear_accelererometer;    // private
    Document                m_doc_gyroscope;                 // private
    Document                m_doc_magnetometer;              // private
    Document                m_doc_location_input;            // private
    Document                m_doc_location_output;           // private
    Document                m_doc_location;                  // private
    bool                    m_have_location_output = false;  // private
    bool                    m_have_location_valid  = false;  // private

    double                 m_slice_start = 0.0;  // private
    double                 m_slice_end   = 0.0;  // private
    MatrixXd               m_location;           // private
    pair< double, double > m_origin;             // private
    MatrixXd               m_location_valid;     // private
    MatrixXd               m_location_output;    // private

    MatrixXd m_a;   // private and delete
    MatrixXd m_la;  // private
    MatrixXd m_gs;  // private

    VectorXd m_a_x;                         // private
    VectorXd m_a_y;                         // private
    VectorXd m_a_z;                         // private
    VectorXd m_la_x;                        // private
    VectorXd m_la_y;                        // private
    VectorXd m_la_z;                        // private
    VectorXd m_gs_x;                        // private
    VectorXd m_gs_y;                        // private
    VectorXd m_gs_z;                        // private
    VectorXd m_la_mag;                      // private and delete
    VectorXd m_gs_mag;                      // private
    VectorXd m_m_mag;                       // private
    MatrixXd m_g;                           // private
    VectorXd m_g_mag;                       // private
    VectorXd m_latitude;                    // private
    VectorXd m_longitude;                   // private
    VectorXd m_height;                      // private
    VectorXd m_velocity;                    // private
    VectorXd m_horizontal_accuracy;         // private
    VectorXd m_vertical_accuracy;           // private
    VectorXd m_latitude_valid;              // private
    VectorXd m_longitude_valid;             // private
    VectorXd m_height_valid;                // private
    VectorXd m_velocity_valid;              // private
    VectorXd m_direction_valid;             // private
    VectorXd m_horizontal_accuracy_valid;   // private
    VectorXd m_vertical_accuracy_valid;     // private
    VectorXd m_x_valid;                     // private
    VectorXd m_y_valid;                     // private
    VectorXd m_latitude_output;             // private
    VectorXd m_longitude_output;            // private
    VectorXd m_height_output;               // private
    VectorXd m_velocity_output;             // private
    VectorXd m_direction_output;            // private
    VectorXd m_horizontal_accuracy_output;  // private
    VectorXd m_vertical_accuracy_output;    // private
    VectorXd m_x_output;                    // private
    VectorXd m_y_output;                    // private
// public:
    //     static constexpr double kK = 1e5;
    //     string                  m_test_case_path;   //private
    //     Document                m_doc_accelerometer;   //private
    //     Document                m_doc_linear_accelererometer;   //private
    //     Document                m_doc_gyroscope;   //private
    //     Document                m_doc_magnetometer;   //private
    //     Document                m_doc_location_input;   //private
    //     Document                m_doc_location_output;   //private
    //     Document                m_doc_location;   //private
    //     bool                    m_have_location_output = false;   //private
    //     bool                    m_have_location_valid  = false;   //private

    //     vector< double >       m_time_location;    //public
    //     double                 m_slice_start = 0.0;   //private
    //     double                 m_slice_end   = 0.0;   //private
    //     size_t                 m_len_input   = 0;    //public
    //     MatrixXd               m_location;   //private
    //     pair< double, double > m_origin;   //private
    //     MatrixXd               m_location_valid;   //private
    //     MatrixXd               m_location_output;   //private

    //     VectorXd m_time;    //public
    //     MatrixXd m_a;   //private and delete
    //     MatrixXd m_la;   //private
    //     MatrixXd m_gs;   //private
    //     MatrixXd m_m;    //public 计算数据尺寸用，可替代

    //     VectorXd m_a_x;    //private
    //     VectorXd m_a_y;    //private
    //     VectorXd m_a_z;    //private
    //     VectorXd m_la_x;    //private
    //     VectorXd m_la_y;    //private
    //     VectorXd m_la_z;    //private
    //     VectorXd m_gs_x;    //private
    //     VectorXd m_gs_y;    //private
    //     VectorXd m_gs_z;    //private
    //     VectorXd m_m_x;    //public 获取水平方向
    //     VectorXd m_m_y;    //public 获取水平方向
    //     VectorXd m_m_z;    //public 获取东向
    //     VectorXd m_a_mag;    //public 计算步长
    //     VectorXd m_la_mag;    //private and delete
    //     VectorXd m_gs_mag;    //private
    //     VectorXd m_m_mag;    //private
    //     MatrixXd m_g;    //private
    //     VectorXd m_g_x;    //public 获取东向
    //     VectorXd m_g_y;    //public 获取东向
    //     VectorXd m_g_z;    //public 获取东向
    //     VectorXd m_g_mag;    //private
    //     VectorXd m_latitude;    //private
    //     VectorXd m_longitude;    //private
    //     VectorXd m_height;    //private
    //     VectorXd m_velocity;    //private
    //     VectorXd m_direction;    //public 原始代码需要使用，做方向判断
    //     VectorXd m_horizontal_accuracy;    //private
    //     VectorXd m_vertical_accuracy;    //private
    //     VectorXd m_x;    //public 训练结果用
    //     VectorXd m_y;    //public 训练结果用
    //     VectorXd m_latitude_valid;    //private
    //     VectorXd m_longitude_valid;    //private
    //     VectorXd m_height_valid;    //private
    //     VectorXd m_velocity_valid;    //private
    //     VectorXd m_direction_valid;    //private
    //     VectorXd m_horizontal_accuracy_valid;    //private
    //     VectorXd m_vertical_accuracy_valid;    //private
    //     VectorXd m_x_valid;    //private
    //     VectorXd m_y_valid;    //private
    //     VectorXd m_latitude_output;    //private
    //     VectorXd m_longitude_output;    //private
    //     VectorXd m_height_output;    //private
    //     VectorXd m_velocity_output;    //private
    //     VectorXd m_direction_output;    //private
    //     VectorXd m_horizontal_accuracy_output;    //private
    //     VectorXd m_vertical_accuracy_output;    //private
    //     VectorXd m_x_output;    //private
    //     VectorXd m_y_output;    //private
};