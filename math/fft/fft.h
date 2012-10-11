#ifndef SNARK_MATH_FFT_H_
#define SNARK_MATH_FFT_H_

#include <cmath>
#include <boost/array.hpp>

namespace snark{ 

/// Cooley-Tukey in-place fft based on Danielson-Lanczos algorithm
///
/// see: Cooley-Tukey algorithm from Numerical Recipes in C++
///
/// see also: Vlodymyr Myrnyy, Simple and Efficient Fast Fourier Transform, Dr. Dobbs' Journal
/// http://www.drdobbs.com/cpp/a-simple-and-efficient-fft-implementatio/199500857
template < typename T, std::size_t N >
void fft( boost::array< T, N >& data );

/// Cooley-Tukey in-place fft based on Danielson-Lanczos algorithm
template < typename T >
void fft( T* data, std::size_t size );

/// Cooley-Tukey fft based on Danielson-Lanczos algorithm (convenience function)
template < typename T, std::size_t N >
boost::array< T, N > fft( const boost::array< T, N >& data );

template < typename T, std::size_t N >
void fft( boost::array< T, N >& data )
{
    fft( &data[0], N );
}

template < typename T, std::size_t N >
inline boost::array< T, N > fft( const boost::array< T, N >& data )
{
    boost::array< T, N > a = data;
    fft( &a[0], N );
    return a;
}

template < typename T >
inline void fft( T* data, std::size_t size )
{
    unsigned long n, mmax, m, j, istep, i;
    double wtemp, wr, wpr, wpi, wi, theta;
    double tempr, tempi;
 
    // reverse-binary reindexing
    n = size << 1;
    j = 1;
    for( i = 1; i < n; i += 2 )
    {
        if( j > i )
        {
            std::swap( data[ j - 1 ], data[ i - 1 ] );
            std::swap( data[j], data[i]);
        }
        m = size;
        while( m >= 2 && j > m )
        {
            j -= m;
            m >>= 1;
        }
        j += m;
    };
 
    // Danielson-Lanczos method
    mmax = 2;
    while( n > mmax )
    {
        istep = mmax << 1;
        theta = -( 2 * M_PI / mmax );
        wtemp = std::sin( 0.5 * theta );
        wpr = -2.0 * wtemp * wtemp;
        wpi = std::sin( theta );
        wr = 1.0;
        wi = 0.0;
        for( m = 1; m < mmax; m += 2 )
        {
            for( i = m; i <= n; i += istep )
            {
                j = i + mmax;
                tempr = wr * data[j-1] - wi * data[j];
                tempi = wr * data[j] + wi * data[j-1];
                data[j-1] = data[i-1] - tempr;
                data[j] = data[i] - tempi;
                data[i-1] += tempr;
                data[i] += tempi;
            }
            wtemp = wr;
            wr += wr * wpr - wi * wpi;
            wi += wi * wpr + wtemp * wpi;
        }
        mmax = istep;
    }
}

}  // namespace snark{ 

#endif // SNARK_MATH_FFT_H_
