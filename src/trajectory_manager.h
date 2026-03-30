#include <Eigen/Dense>
#include <memory>
#include <stdexcept>

class TrajectoryManager
{
public:
    // cols: 矩阵列数
    // initialRows: 初始开辟的行数（预留空间）
    TrajectoryManager( int cols, int initialRows = 500 ) : m_cols( cols ), m_usedRows( 0 ), m_capacityRows( initialRows )
    {
        m_data = std::make_unique< Eigen::MatrixXd >( m_capacityRows, m_cols );
    }

    // 追加新数据（支持任意行数）
    void append( const Eigen::MatrixXd& newData )
    {
        if ( newData.cols() != m_cols )
        {
            throw std::runtime_error( "Column count mismatch!" );
        }

        int incomingRows = newData.rows();

        // 检查剩余空间是否足够
        if ( m_usedRows + incomingRows > m_capacityRows )
        {
            // 模仿 vector 扩容策略：至少翻倍，且确保能放下新数据
            int minRequired = m_usedRows + incomingRows;
            m_capacityRows  = std::max( m_capacityRows * 2, minRequired );

            m_data->conservativeResize( m_capacityRows, Eigen::NoChange );
        }

        // 将新数据拷贝到当前可用位置的起始处
        m_data->block( m_usedRows, 0, incomingRows, m_cols ) = newData;
        m_usedRows += incomingRows;
    }

    // 获取当前有效数据的视图（零拷贝）
    auto view() const
    {
        return m_data->topRows( m_usedRows );
    }

    // 重置轨迹（不释放内存，仅重置计数器，方便复用空间）
    void clear()
    {
        m_usedRows = 0;
    }

    int rows() const
    {
        return m_usedRows;
    }
    int cols() const
    {
        return m_cols;
    }
private:
    std::unique_ptr< Eigen::MatrixXd > m_data;
    int                                m_cols;
    int                                m_usedRows;      // 当前已填入的总行数
    int                                m_capacityRows;  // 当前内存块的总行数
};
