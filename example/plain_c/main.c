#define _POSIX_C_SOURCE 200809L
#define _XOPEN_SOURCE 700

#include "fm_pdr.h"
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdatomic.h>

// 全局参数存储
int   evaluation_value   = 0;
char* config_path_value  = ( char* )"../conf/config.json";
char* train_dir_value    = NULL;
char* dataset_dir_value  = NULL;
int   x_value            = 0;
int   y_value            = 0;
char* output_path_value  = NULL;
char* raw_data_dir_value = NULL;

// 长选项定义
static struct option long_options[] = {
    { "evaluation", no_argument, NULL, 'e' },
    { "config-path", required_argument, NULL, 'c' },
    { "train-dir", required_argument, NULL, 't' },
    { "dataset-dir", required_argument, NULL, 'd' },
    { "start-lon", required_argument, NULL, 'x' },
    { "start-lat", required_argument, NULL, 'y' },
    { "output-path", required_argument, NULL, 'o' },
    { "save-pdr-data", required_argument, NULL, 'r' },
    { "help", no_argument, NULL, 'h' },
    { 0, 0, 0, 0 }  // 结束标记
};

static volatile sig_atomic_t g_is_running = 1;

void show_help( char* prog_name )
{
    printf( "使用方法: %s [选项]\n\n", prog_name );
    printf( "选项:\n" );
    printf( "  -e, --evaluation\t\t\t是否打印指标评估\n" );
    printf( "  -c, --config-path <配置文件路径>\t\t指定PDR配置文件路径，默认使用../conf/config.json\n" );
    printf( "  -t, --train-dir <样本数据路径>\t\t表示需要加载的<样本数据路径>，训练模型输出到model_file_name配置项设置的路径下\n" );
    printf( "  -d, --dataset-dir <PDR数据路径>\t\t表示需要加载的<PDR测试数据路径>，不设置该标志使用传感器数据进行测试，使用model_file_name配置项设置路径下的模型文件进行推算\n" );
    printf( "  -x, --start-lon <经度>\t\t设置起始点经度值（WGS84坐标系，基于实时数据测试时生效）\n" );
    printf( "  -y, --start-lat <纬度>\t\t设置起始点纬度值（WGS84坐标系，基于实时数据测试时生效）\n" );
    printf( "  -o, --output-path <行人航迹数据文件路径>\t表示需要保存的<行人航迹数据文件路径>，使用model_file_name配置项设置路径下的模型文件进行推算\n" );
    printf( "  -r, --raw-data-dir <传感器数据保存路径>\t基于传感器数据进行PDR测试时，表示需要保存的原始传感器测量数据路径，不设置改选项不保存数据文件\n" );
    printf( "  -h, --help\t\t\t\t帮助信息\n" );
    printf( "示例:\n" );
    printf( "训练模型:\n" );
    printf( "    %s --train-dir \"./test_data/train_data\"\n", prog_name );
    printf( "基于实时数据做PDR测试:\n" );
    printf( "    %s -x 32.11199920 -y 118.9528682 --output-path \"./Trajectory.csv\" --raw-data-dir \"./output_sensor_data\"\n", prog_name );
    printf( "基于数据文件做PDR测试:\n" );
    printf( "    %s --dataset-dir \"./test_data/sensor_data\" --output-path \"./Trajectory.csv\"\n", prog_name );
    printf( "\n\n" );
}

void sigterm_handler( int signum )
{
    g_is_running = 0;
}

int main( int argc, char** argv )
{
    struct sigaction sa;
    sa.sa_handler = sigterm_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);

    int opt;
    int option_index = 0;

    // 禁用自动错误提示
    opterr = 0;

    while ( ( opt = getopt_long( argc, argv, "c:t:d:x:y:o:r:eh", long_options, &option_index ) ) != -1 )
    {
        switch ( opt )
        {
            case 'e':
                evaluation_value = 1;
                break;
            case 'c':
                config_path_value = optarg;
                break;
            case 't':
                train_dir_value = optarg;
                break;
            case 'd':
                dataset_dir_value = optarg;
                break;
            case 'x':
                x_value = atof( optarg );
                break;
            case 'y':
                y_value = atof( optarg );
                break;
            case 'o':
                output_path_value = optarg;
                break;
            case 'r':
                raw_data_dir_value = optarg;
                break;
            case 'h':
                show_help( argv[ 0 ] );
                return 0;
            case '?':
                // 处理未知选项或缺少参数
                if ( optopt == 'c' || optopt == 't' || optopt == 'd' || optopt == 'x' || optopt == 'y' || optopt == 'o' || optopt == 'r' )
                {
                    fprintf( stderr, "Option '-%c' requires an argument\n", optopt );
                }
                else if ( optopt != 0 )
                {
                    fprintf( stderr, "Unknown option '-%c'\n", optopt );
                }
                else
                {
                    fprintf( stderr, "Bad option format\n" );
                }
                return -1;
            default:
                abort();
        }
    }

    // 打印结果
    // printf( "\nParsed parameters:\n" );
    // printf( "Evaluation (-e/--evaluation): %s\n", evaluation_value ? "true" : "false" );
    // printf( "Config (-s/--config): %s\n", config_path_value );
    // printf( "Train (-t/--train): %s\n", train_dir_value );
    // printf( "Dataset (-d/--dataset): %s\n", dataset_dir_value );
    // printf( "Start-Lon (-x/--start-lon): %f\n", x_value );
    // printf( "Start-Lot (-y/--start-lat): %f\n", y_value );
    // printf( "Output (-o/--output): %s\n", output_path_value );

    PDRHandler         pdr_handler;
    PDRTrajectoryArray trajectories_array;
    int                ret;

    // 这三个参数只能同时传递一个
    if ( ( train_dir_value != NULL ) + ( dataset_dir_value != NULL ) + ( raw_data_dir_value != NULL ) != 1 )
    {
        show_help( argv[ 0 ] );
        return -1;
    }

    if ( train_dir_value )
    {
        // 通过训练数据初始化PDR，此时执行目录下会生成模型文件
        ret = fm_pdr_init_with_file( config_path_value, train_dir_value, &pdr_handler, &trajectories_array );
        if ( ret < PDR_RESULT_SUCCESS )
        {
            fprintf( stderr, "PDR初始化错误\n" );
            return -1;
        }

        // 保存训练数据的航迹
        if ( output_path_value )
        {
            ret = fm_pdr_save_trajectory_data( output_path_value, &trajectories_array );
            if ( ret != PDR_RESULT_SUCCESS )
            {
                fm_pdr_free_trajectory( &trajectories_array );
                fm_pdr_uninit( &pdr_handler );
                fprintf( stderr, "行人航迹数据保存失败\n" );
                return -1;
            }
        }

        // 释放行人轨迹数据(这里的行人轨迹数据不是计算出来的，而是直接传出的真实数据)与PDR句柄
        fm_pdr_free_trajectory( &trajectories_array );
        fm_pdr_uninit( &pdr_handler );
    }
    else
    {
        if ( dataset_dir_value )
        {
            // 初始化PDR
            ret = fm_pdr_init_with_file( config_path_value, NULL, &pdr_handler, NULL );
            if ( ret < PDR_RESULT_SUCCESS )
            {
                fprintf( stderr, "PDR初始化错误\n" );
                return -1;
            }

            // 启动行人航迹推算算法
            ret = fm_pdr_start_with_file( pdr_handler, dataset_dir_value );
            if ( ret != PDR_RESULT_SUCCESS )
            {
                fm_pdr_uninit( &pdr_handler );
                fprintf( stderr, "启动行人航迹推算程序时发生错误\n" );
                return -1;
            }

            // 预测行人航迹
            ret = fm_pdr_predict( pdr_handler, &trajectories_array );
            if ( ret < PDR_RESULT_SUCCESS )
            {
                fm_pdr_uninit( &pdr_handler );
                fprintf( stderr, "启动行人航迹推算程序时发生错误\n" );
                return -1;
            }

            // 保存推算出的航迹
            if ( output_path_value )
            {
                ret = fm_pdr_save_trajectory_data( ( char* )"Trajectory.csv", &trajectories_array );
                if ( ret != PDR_RESULT_SUCCESS )
                {
                    fm_pdr_free_trajectory( &trajectories_array );
                    fm_pdr_uninit( &pdr_handler );
                    fprintf( stderr, "行人航迹数据保存失败\n" );
                    return -1;
                }
            }

            // 释放计算出的行人轨迹与PDR句柄
            fm_pdr_free_trajectory( &trajectories_array );
            fm_pdr_uninit( &pdr_handler );
        }
        else
        {
            // 初始化PDR句柄
            ret = fm_pdr_init_with_file( config_path_value, NULL, &pdr_handler, NULL );
            if ( ret < PDR_RESULT_SUCCESS )
            {
                fprintf( stderr, "PDR初始化错误\n" );
                return -1;
            }

            // 启动行人航迹推算算法
            PDRPoint start_point;
            start_point.x = x_value;
            start_point.y = y_value;

            ret = fm_pdr_start( pdr_handler, &start_point, raw_data_dir_value );
            if ( ret != PDR_RESULT_SUCCESS )
            {
                fm_pdr_uninit( &pdr_handler );
                fprintf( stderr, "启动行人航迹推算程序时发生错误\n" );
                return -1;
            }

            int quit = 0;
            while ( 1 )
            {
                if ( g_is_running )
                {
                    // 预测行人航迹
                    ret = fm_pdr_predict( pdr_handler, &trajectories_array );
                    if ( ret < PDR_RESULT_SUCCESS )
                    {
                        fprintf( stderr, "取得行人航迹推算数据错误\n" );
                        continue;
                    }
                }
                else
                {
                    // 终止行人航迹推算算法
                    ret = fm_pdr_stop( pdr_handler, &trajectories_array );
                    if ( ret < PDR_RESULT_SUCCESS )
                    {
                        fprintf( stderr, "取得行人航迹推算数据错误\n" );
                        continue;
                    }

                    quit = 1;
                }

                // 保存推算出的航迹
                if ( output_path_value )
                {
                    ret = fm_pdr_save_trajectory_data( ( char* )"Trajectory.csv", &trajectories_array );
                    if ( ret != PDR_RESULT_SUCCESS )
                    {
                        fm_pdr_free_trajectory( &trajectories_array );
                        fprintf( stderr, "行人航迹数据保存失败\n" );
                        continue;
                    }
                }

                // 释放计算出的行人轨迹与
                fm_pdr_free_trajectory( &trajectories_array );

                if ( quit )
                    break;

                sleep( 4 );
            }

            // 释放PDR句柄
            fm_pdr_uninit( &pdr_handler );
        }
    }

    return 0;
}