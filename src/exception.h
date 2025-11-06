#pragma once
#include "fm_pdr.h"
#include <stdexcept>
#include <string>

/// @class PDRException
/// @brief 所有PDR异常的基类（支持错误码+详细信息）
class PDRException : public std::runtime_error
{
public:
    using Code = int;

    PDRException( Code code, const std::string& msg ) : std::runtime_error( msg ), m_code( code ) {}

    Code code() const noexcept
    {
        return m_code;
    }
private:
    Code m_code;
};

/// @defgroup 具体异常类型
/// @brief 通过继承实现错误分类
class DataException : public PDRException
{
public:
    enum SubCode
    {
        EMPTY_ERROR         = PDRResult::PDR_RESULT_EMPTY_ERROR,
        COLUMN_INCONSISTENT = PDRResult::PDR_RESULT_COLUMN_INCONSISTENT
    };
    DataException( SubCode sc, const std::string& description ) : PDRException( sc, "Failed to data check: " + description ) {}
};

/// @defgroup 具体异常类型
/// @brief 通过继承实现错误分类
class FileException : public PDRException
{
public:
    enum SubCode
    {
        OPEN_FAILED   = PDRResult::PDR_RESULT_OPEN_FAILED,
        CREATE_FAILED = PDRResult::PDR_RESULT_CREATE_FAILED,
        WRITE_FAILED  = PDRResult::PDR_RESULT_WRITE_FAILED,
        READ_FAILED   = PDRResult::PDR_RESULT_READ_FAILED,
        DIR_NOT_EXIST = PDRResult::PDR_RESULT_DIR_NOT_EXIST,
        NOT_DIRECTORY = PDRResult::PDR_RESULT_NOT_DIRECTORY
    };
    FileException( SubCode sc, const std::string& path ) : PDRException( sc, "Failed to open JSON file: " + path ) {}
};

class JsonException : public PDRException
{
public:
    enum SubCode
    {
        PARSE_ERROR   = PDRResult::PDR_RESULT_PARSE_ERROR,
        INVALID_ROOT  = PDRResult::PDR_RESULT_INVALID_ROOT,
        MISSING_FIELD = PDRResult::PDR_RESULT_MISSING_FIELD,
        TYPE_MISMATCH = PDRResult::PDR_RESULT_TYPE_MISMATCH
    };

    // 携带JSON解析偏移量信息
    JsonException( SubCode sc, const std::string& msg, size_t offset = 0 ) : PDRException( sc, msg + ( offset ? " [offset:" + std::to_string( offset ) + "]" : "" ) ) {}
};

class MemoryException : public PDRException
{
public:
    enum SubCode
    {
        ALLOC_FAILED = PDRResult::PDR_RESULT_ALLOC_FAILED
    };
    MemoryException( SubCode sc, const std::string& obj ) : PDRException( sc, "Memory allocation failed: " + obj ) {}
};