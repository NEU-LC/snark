// Copyright (c) 2020 Kent Hu

/// @author kent hu

#pragma once

#include <cuda_runtime_api.h>
#include <comma/base/exception.h>

#include <type_traits>
#include <vector>

#define CUDA_CHECK_ERRORS( err_code ) { cudaError_t CudaCheckErrorCode = err_code; if( CudaCheckErrorCode != cudaSuccess ) { COMMA_THROW( comma::exception, "cuda error " << CudaCheckErrorCode << ": " << cudaGetErrorString( CudaCheckErrorCode ) ) } }

namespace snark { namespace cuda { namespace device {

template < typename Numeric >
class array
{
    static_assert( std::is_arithmetic< Numeric >::value, "only arithmetics types are supported" );
public:
    array() = delete;

    explicit array( size_t size ) : size_( size )
    {
        CUDA_CHECK_ERRORS( cudaMalloc( reinterpret_cast< void** >( &data_ ), bytes() ) )
    }

    array( size_t size, const Numeric* data ) : size_( size )
    {
        CUDA_CHECK_ERRORS( cudaMalloc( reinterpret_cast< void** >( &data_ ), bytes() ) )
        cudaError_t err_code = to_device( data );
        if( err_code != cudaSuccess )
        {
            CUDA_CHECK_ERRORS( cudaFree( data_ ) )
            COMMA_THROW( comma::exception, "cuda error " << err_code << ": " << cudaGetErrorString( err_code ) )
        }
    }

    array( size_t size, const std::vector< Numeric >& h_array ) : size_( size )
    {
        CUDA_CHECK_ERRORS( cudaMalloc( reinterpret_cast< void** >( &data_ ), bytes() ) )
        cudaError_t err_code = to_device( h_array );
        if( err_code != cudaSuccess )
        {
            CUDA_CHECK_ERRORS( cudaFree( data_ ) )
            COMMA_THROW( comma::exception, "cuda error " << err_code << ": " << cudaGetErrorString( err_code ) )
        }
    }

    array( const array& other ) = delete;

    array& operator=( const array& other ) = delete;

    array( array&& other ) = delete;

    array& operator=( array&& other ) = delete;

    ~array() { cudaFree( data_ ); }

    size_t bytes() const noexcept { return sizeof( Numeric ) * size_; }

    Numeric* data() const noexcept { return data_; }

    cudaError_t fill( Numeric value ) noexcept { return cudaMemset( data_, value, bytes() ); }

    size_t size() const noexcept { return size_; }

    cudaError_t to_device( const Numeric* data ) noexcept
    {
        return cudaMemcpy( data_, data, bytes(), cudaMemcpyHostToDevice );
    }

    cudaError_t to_device( const std::vector< Numeric >& h_array ) noexcept
    {
        return t_device( h_array.data() );
    }

    cudaError_t to_host( Numeric* data ) const noexcept
    {
        return cudaMemcpy( data, data_, bytes(), cudaMemcpyDeviceToHost );
    }

    cudaError_t to_host( std::vector< Numeric >& h_array ) const noexcept
    {
        return to_host( h_array.data() );
    }

private:
    Numeric* data_ = nullptr;
    size_t size_;
};

template < typename Numeric >
class pitched_matrix
{
    static_assert( std::is_arithmetic< Numeric >::value, "only arithmetics types are supported" );
public:
    pitched_matrix() = delete;

    pitched_matrix( size_t nrows, size_t ncols ) : nrows_( nrows ), ncols_( ncols )
    {
        CUDA_CHECK_ERRORS( cudaMallocPitch( reinterpret_cast< void** >( &data_ ), &pitch_, ncols_ * sizeof( Numeric ), nrows_ ) )
    }

    pitched_matrix( size_t nrows, size_t ncols, const Numeric* data ) : nrows_( nrows ), ncols_( ncols )
    {
        CUDA_CHECK_ERRORS( cudaMallocPitch( reinterpret_cast< void** >( &data_ ), &pitch_, ncols_ * sizeof( Numeric ), nrows_ ) )
        cudaError_t err_code = to_device( data );
        if( err_code != cudaSuccess )
        {
            CUDA_CHECK_ERRORS( cudaFree( data_ ) )
            COMMA_THROW( comma::exception, "cuda error " << err_code << ": " << cudaGetErrorString( err_code ) )
        }
    }

    pitched_matrix( size_t nrows, size_t ncols, const std::vector< Numeric >& h_matrix ) : nrows_( nrows ), ncols_( ncols )
    {
        CUDA_CHECK_ERRORS( cudaMallocPitch( reinterpret_cast< void** >( &data_ ), &pitch_, ncols_ * sizeof( Numeric ), nrows_ ) )
        cudaError_t err_code = to_device( h_matrix );
        if( err_code != cudaSuccess )
        {
            CUDA_CHECK_ERRORS( cudaFree( data_ ) )
            COMMA_THROW( comma::exception, "cuda error " << err_code << ": " << cudaGetErrorString( err_code ) )
        }
    }

    pitched_matrix( const pitched_matrix& other ) = delete;

    pitched_matrix& operator=( const pitched_matrix& other ) = delete;

    pitched_matrix( pitched_matrix&& other ) = delete;

    pitched_matrix& operator=( pitched_matrix&& other ) = delete;

    ~pitched_matrix() { cudaFree( data_ ); }

    size_t bytes() const noexcept { return sizeof( Numeric ) * pitch_ * nrows_; }

    Numeric* data() const noexcept { return data_; }

    cudaError_t fill( Numeric value ) noexcept { return cudaMemset2D( data_, pitch_, value, ncols_ * sizeof( Numeric ), nrows_ ); }

    size_t pitch() const noexcept { return pitch_; }

    size_t rows() const noexcept { return nrows_; }

    size_t cols() const noexcept { return ncols_; }

    cudaError_t to_device( const Numeric* data ) noexcept
    {
        return cudaMemcpy2D( data_, pitch_, data, ncols_ * sizeof( Numeric ), ncols_ * sizeof( Numeric ), nrows_, cudaMemcpyHostToDevice );
    }

    cudaError_t to_device( const std::vector< Numeric >& h_matrix ) noexcept
    {
        return to_device( h_matrix.data() );
    }

    cudaError_t to_host( Numeric* data ) const noexcept
    {
        return cudaMemcpy2D( data, ncols_ * sizeof( Numeric ), data_, pitch_, ncols_ * sizeof( Numeric ), nrows_, cudaMemcpyDeviceToHost );
    }

    cudaError_t to_host( std::vector< Numeric >& h_matrix ) const noexcept
    {
        return to_host( h_matrix.data() );
    }

private:
    Numeric* data_ = nullptr;
    size_t nrows_;
    size_t ncols_;
    size_t pitch_ = 0;
};

template < typename Numeric >
class matrix
{
    static_assert( std::is_arithmetic< Numeric >::value, "only arithmetics types are supported" );
public:
    matrix() = delete;

    matrix( size_t nrows, size_t ncols ) : nrows_( nrows ), ncols_( ncols )
    {
        CUDA_CHECK_ERRORS( cudaMalloc( reinterpret_cast< void** >( &data_ ), bytes() ) )
    }

    matrix( size_t nrows, size_t ncols, const Numeric* data ) : nrows_( nrows ), ncols_( ncols )
    {
        CUDA_CHECK_ERRORS( cudaMalloc( reinterpret_cast< void** >( &data_ ), bytes() ) )
        cudaError_t err_code = to_device( data );
        if( err_code != cudaSuccess )
        {
            CUDA_CHECK_ERRORS( cudaFree( data_ ) )
            COMMA_THROW( comma::exception, "cuda error " << err_code << ": " << cudaGetErrorString( err_code ) )
        }
    }

    matrix( size_t nrows, size_t ncols, const std::vector< Numeric >& h_matrix ) : nrows_( nrows ), ncols_( ncols )
    {
        CUDA_CHECK_ERRORS( cudaMalloc( reinterpret_cast< void** >( &data_ ), bytes() ) )
        cudaError_t err_code = to_device( h_matrix );
        if( err_code != cudaSuccess )
        {
            CUDA_CHECK_ERRORS( cudaFree( data_ ) )
            COMMA_THROW( comma::exception, "cuda error " << err_code << ": " << cudaGetErrorString( err_code ) )
        }
    }

    matrix( const matrix& other ) = delete;

    matrix& operator=( const matrix& other ) = delete;

    matrix( matrix&& other ) = delete;

    matrix& operator=( matrix&& other ) = delete;

    ~matrix() { cudaFree( data_ ); }

    size_t bytes() const noexcept { return sizeof( Numeric ) * ncols_ * nrows_; }

    Numeric* data() const noexcept { return data_; }

    cudaError_t fill( Numeric value ) noexcept { return cudaMemset( data_, value, bytes() ); }

    size_t pitch() const noexcept { return ncols_ * sizeof( Numeric ); }

    size_t rows() const noexcept { return nrows_; }

    size_t cols() const noexcept { return ncols_; }

    cudaError_t to_device( const Numeric* data ) noexcept
    {
        return cudaMemcpy( data_, data, bytes(), cudaMemcpyHostToDevice );
    }

    cudaError_t to_device( const std::vector< Numeric >& h_array ) noexcept
    {
        return to_device( h_array.data() );
    }

    cudaError_t to_host( Numeric* data ) const noexcept
    {
        return cudaMemcpy( data, data_, bytes(), cudaMemcpyDeviceToHost );
    }

    cudaError_t to_host( std::vector< Numeric >& h_array ) const noexcept
    {
        return to_host( h_array.data() );
    }

private:
    Numeric* data_ = nullptr;
    size_t nrows_;
    size_t ncols_;
};

} } } // namespace snark { namespace cuda { namespace device {
