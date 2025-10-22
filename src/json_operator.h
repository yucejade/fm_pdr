#pragma once
#include "fm_pdr.h"
#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>

class CFmJSONOperator
{
public:
    static PDRConfig readPDRConfigFromJson( const char* filename )
    {
        PDRConfig config{};  // 初始化结构体（避免未初始化的成员）

        // 1. 打开JSON文件
        FILE* fp = fopen( filename, "r" );
        if ( ! fp )
            throw std::runtime_error( "Failed to open JSON file: " + std::string( filename ) );

        // 2. 读取文件内容到缓冲区（rapidjson推荐的方式）
        char                      buffer[ 4096 ];
        rapidjson::FileReadStream is( fp, buffer, sizeof( buffer ) );

        // 3. 解析JSON
        rapidjson::Document doc;
        doc.ParseStream( is );
        fclose( fp );  // 解析完成后关闭文件

        // 4. 检查解析错误
        if ( doc.HasParseError() )
            throw std::runtime_error( "Failed to parse JSON file: " + std::string( filename ) + ", error: " + std::to_string( doc.GetParseError() ) + " at offset " + std::to_string( doc.GetErrorOffset() ) );

        // 5. 检查JSON根是否为对象
        if ( ! doc.IsObject() )
            throw std::runtime_error( "JSON root is not an object: " + std::string( filename ) );

        // 6. 读取各个字段（需检查字段存在性和类型）
        auto getIntMember = [ & ]( const char* key ) -> int
        {
            auto it = doc.FindMember( key );
            if ( it == doc.MemberEnd() || ! it->value.IsInt() )
                throw std::runtime_error( "Missing or invalid int member: " + std::string( key ) );

            return it->value.GetInt();
        };

        auto getStringMember = [ & ]( const char* key ) -> char*
        {
            auto it = doc.FindMember( key );
            if ( it == doc.MemberEnd() || ! it->value.IsString() )
                throw std::runtime_error( "Missing or invalid string member: " + std::string( key ) );

            const char* str = it->value.GetString();
            return strdup( str );  // 复制字符串（需手动释放）
        };

        auto getDoubleMember = [ & ]( const char* key ) -> double
        {
            auto it = doc.FindMember( key );
            if ( it == doc.MemberEnd() || ! it->value.IsDouble() )
                throw std::runtime_error( "Missing or invalid double member: " + std::string( key ) );

            return it->value.GetDouble();
        };

        // 映射字段到结构体
        config.sample_rate          = getIntMember( "sample_rate" );
        config.model_name           = getStringMember( "model_name" );
        config.model_file_name      = getStringMember( "model_file_name" );
        config.clean_start          = getIntMember( "clean_start" );
        config.clean_end            = getIntMember( "clean_end" );
        config.default_east_point   = getIntMember( "default_east_point" );
        config.move_average         = getIntMember( "move_average" );
        config.min_distance         = getIntMember( "min_distance" );
        config.distance_frac_step   = getDoubleMember( "distance_frac_step" );
        config.optimized_mode_ratio = getDoubleMember( "optimized_mode_ratio" );
        config.butter_wn            = getDoubleMember( "butter_wn" );
        config.least_start_point    = getIntMember( "least_start_point" );

        return config;
    }
};