#ifndef rmax_matrix_h
#define rmax_matrix_h

#if defined(__cplusplus)
extern "C"
{
#endif

#define MAXSIZE 25
#define MAXFLEXSIZE 350
typedef void *POINTER;

#ifndef MIN
#define MIN(x1,x2) ((x1)<(x2)?(x1):(x2))
#endif
#ifndef MAX
#define MAX(x1,x2) ((x1)>(x2)?(x1):(x2))
#endif
#define LIMIT(x,xl,xu) ((x)>=(xu)?(xu):((x)<(xl)?(xl):(x)))
#define ABS(x) ((x)<0.0?(-(x)):(x))
#define SQ(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))
#define SIGN(x) ((x)<(0)?(-1):((x)>(0)?(1):(0)))
#define INRANGE(x,x1,x2) ((x)<=(x2)&&(x)>=(x1)?1:0)



#if defined(__cplusplus)
}
#endif

void mat_cross( const float *v1, const float *v2, float *cross );
void mat_mult( float *A, int na, int ma,
    float *B, int nb, int mb,
    float *C );
void mat_init( float *in, int rows, int cols, float init_val );

void mat_mult_T( float *A, int na, int ma,
    float *B, int nb, int mb,
    float *C );
void mat_transpose( float *A, int na, int ma,
        float *C );

void mat_qr_sub_T( float *A, int m, int n );

void mat_invert( float *A, int n, float *Ainv );


float mat_norm2(float *A, int n);
static void lu_decomp( double *A, int n, int *indx, double *d );
static void lu_back_sub( double *A, int n, int *indx,  double *b );

void mat_normalize( const float *A, int n, float *B);

void mat_display(const float* matrix, int rows, int cols);
void mat_qr_T( float *A, int m );
#endif

