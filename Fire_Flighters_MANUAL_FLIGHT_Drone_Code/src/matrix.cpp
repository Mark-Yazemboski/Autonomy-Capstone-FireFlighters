/*** BeginCopyright
 * Copyright 2002, Georgia Institute of Technology, All Rights Reserved.
 * Unauthorized use and/or redistribution is disallowed.
 * This library is distributed without any warranty; without even
 * the implied warranty of fitness for a particular purpose.
 *
 * UAV Laboratory
 * School of Aerospace Engineering
 * Georgia Institute of Technology
 * Atlanta, GA 30332
 * http://controls.ae.gatech.edu
 *
 * Contact Information:
 * Prof. Eric N. Johnson
 * http://www.ae.gatech.edu/~ejohnson
 * Tel : 404 385 2519
 * EndCopyright
 ***/
/***
 * $Id: matrix.cpp,v 1.5 2007-05-22 22:16:31 ejohnson Exp $
 * generic matrix manipulation routines, matrices are stored as arrays of doubles
 ***/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "matrix.h"
#include <iostream>
#include <iomanip>









/*  subroutine to initialize a two dimensional array to an arbitrary value
inputs are:  2D array, number of rows, number of columns, and
initial vaule.
*/

#define QUITIF(x) if(x) printf("matrix: dim problem\n")


void mat_init( float *in, int rows, int cols, float init_val ) {

	int i;
	for( i=0; i<rows*cols; i++ )
		in[i] = init_val;

}







/*  subroutine to multiply two matrices
C = A*B
inputs are:  A, na (rows in A), ma (cols in A)
B, nb (rows in B), mb (cols in B)
and C
*/

void mat_mult( float *A, int na, int ma,
	float *B, int nb, int mb,
	float *C ) {

	int i, j, k;

	QUITIF( ma != nb );

	mat_init( C, na, mb, 0.0 );

	for( i = 0; i < na; i++ )
		for( j = 0; j < mb; j++ )
			for( k = 0; k < ma; k++ )
				C[i*mb+j] += A[i*ma+k]*B[k*mb+j];

}




/*  subroutine to multiply a matrix another matrix transposed
C = A*B'
inputs are:  A, na (rows in A), ma (cols in A)
B, nb (rows in B), mb (cols in B)
and C
*/

void mat_mult_T( float *A, int na, int ma,
	float *B, int nb, int mb, float *C ) {

	int i, j, k;

	QUITIF( ma != mb );

	mat_init( C, na, nb, 0.0 );

	for( i = 0; i < na; i++ )
		for( j = 0; j < nb; j++ )
			for( k = 0; k < ma; k++ )
				C[i*nb+j] += A[i*ma+k]*B[j*ma+k];

}



/*  subroutine to multiply a matrix transposed by another matrix
C = (A')*B
inputs are:  A, na (rows in A), ma (cols in A)
B, nb (rows in B), mb (cols in B)
and C
*/

void mat_T_mult( float *A, int na, int ma,
	float *B, int nb, int mb, float *C ) {

	int i, j, k;

	QUITIF( na != nb );

	mat_init( C, ma, mb, 0.0 );

	for( i = 0; i < ma; i++ )
		for( j = 0; j < mb; j++ )
			for( k = 0; k < na; k++ )
				C[i*mb+j] += A[k*ma+i]*B[k*mb+j];

}


static void lu_decomp( double *A, int n, int *indx, double *d ) {

	double vv[MAXSIZE];  /* vv[n] stores implicit scaling of each row */
	register int       i, imax=0, j, k;
	register double big, dum, sum, temp;

	/*if( n > MAXSIZE ) {
	fprintf( stderr, "lu_decomp: n too large\n" );
	return ( -1 );
    }*/

	*(d) = 1.0;

	for( i = 0; i < n; i++ ) {
		big = 0.0;
		for( j = 0; j < n; j++ )
			if( (temp = ABS(A[i*n + j])) > big )
				big = temp;
				/*if ( big == 0.0 ) {
				fprintf( stderr, "lu_decomp: matrix is singular\n" );
				return ( -1 );
	            }*/
			vv[i] = 1.0/big;
	}

	for( j = 0; j < n; j++ ) {
		for( i = 0; i < j; i++ ) {
			sum = A[i*n + j];
			for( k = 0; k < i; k++ )
				sum -= A[i*n + k]*A[k*n + j];
			A[i*n + j] = sum;
		}
		big = 0.0;
		for( i = j; i < n; i++ ) {
			sum = A[i*n + j];
			for( k = 0; k < j; k++ )
				sum -= A[i*n + k]*A[k*n + j];
			A[i*n + j] = sum;
			if( (dum = vv[i]*ABS(sum)) >= big ) {
				big  = dum;
				imax = i;
			}
		}
		if( j != imax ) {
			for( k = 0; k < n; k++ ) {
				dum           = A[imax*n + k];
				A[imax*n + k] = A[j*n + k];
				A[j*n + k]    = dum;
			}
			*(d) = -(*(d));
			vv[imax] = vv[j];
		}
		indx[j] = imax;
		if( A[j*n + j] == 0.0 )
			A[j*n + j] = 1.0e-12;
		if( j != n-1 ) {
			dum = 1.0/A[j*n + j];
			for ( i = j+1; i < n; i++ ) A[i*n + j] *= dum;
		}
	}

}

void mat_transpose( float *A, int na, int ma, float *C ) {

	int i, j;

	mat_init( C, ma, na, 0.0 );

	for( i = 0; i < na; i++ )
		for( j = 0; j < ma; j++ )
			C[j*na+i] = A[i*ma+j];

}


static void lu_back_sub( double *A, int n, int *indx,  double *b ) {

	register int    i, ii, ip, j;
	register double sum;

	ii = -1;

	for( i = 0; i < n; i++ ) {
		ip  = indx[i];
		sum = b[ip];
		b[ip] = b[i];
		if( ii != -1 )
			for( j = ii; j < i; j++ )
				sum -= A[i*n + j]*b[j];
			else if ( sum )
				ii = i;
			b[i] = sum;
	}

	for( i = n-1; i > -1; i-- ) {
		sum = b[i];
		for ( j = i+1; j < n; j++ )
			sum -= A[i*n + j]*b[j];
		b[i] = sum/A[i*n + i];
	}

}


void mat_invert( float *A, int n, float *Ainv /*, double *det*/ ) {

	int indx[MAXSIZE];   /* index is dimension n */
	double b[MAXSIZE],   /* b is dimension n     */
		Acopy[MAXSIZE*MAXSIZE];   /* Acopy is n by n      */
	register int i, j;

	/* just to remove *det from arguments */
	double determinant, *det;
	det = &determinant;

	/*QUITIF( n > MAXSIZE );*/

	for( i = 0; i < n; i++ ) {
		for( j = 0; j < n; j++ )
			Acopy[i*n + j] = A[i*n + j];
	}

	lu_decomp( Acopy, n, indx, det );

	for( j = 0; j < n; j++ ) {
		*(det) *= Acopy[j*n + j];
		for( i = 0; i < n; i++ )
			b[i] = 0.0;
		b[j] = 1.0;
		lu_back_sub( Acopy, n, indx, b );
		for( i = 0; i < n; i++ )
			Ainv[i*n + j] = b[i];
	}

}

void mat_cross( const float *v1, const float *v2, float *cross ) {

  cross[0] = (v1[1]*v2[2]) - (v1[2]*v2[1]);
  cross[1] = (v1[2]*v2[0]) - (v1[0]*v2[2]);
  cross[2] = (v1[0]*v2[1]) - (v1[1]*v2[0]);

}

/* normalize the vector using 2 norm, A and B can be the same vector */
void mat_normalize( const float *A, int n, float *B)
{
	float sum=0;
	float mag=0;
	float eps = 1e-24;
	int i;

	for(i=0; i<n; i++)
		sum += SQ(A[i]);

	if(sum < eps) {printf("\nmat_normalize : very small < %f",eps); sum = eps;}

	mag = sqrt(sum);

	for(i=0; i<n; i++)
		B[i] = A[i]/mag;
}

void mat_display(const float* matrix, int rows, int cols) {
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			std::cout << std::setw(8) << matrix[i * cols + j]; // Adjust width as needed
		}
		std::cout << std::endl;
	}
}

#define MAT_MAX_QR (64*3)
/* subroutine to calculate the qr decomposion
 * of the transpose of a square matrix. Inputs
 * are the matrix A, the size of the subsection m,
 * and the full size of A, la .
 * output is A = R', upper triangular
 * Uses Householder transformations
 * See Tefethen and Bau Pg. 73 */
void mat_qr_sub_T( float *A, int m, int la ){
    
    
    if( m > MAT_MAX_QR || m > la ){
        printf("mat_qr_T: matrix too big");
        return;
    }
    
    /* triangulate Ut */
    float vtA[MAT_MAX_QR] = {0};
    float vec[MAT_MAX_QR] = {0};
    float d[MAT_MAX_QR]   = {0};
    float sx1;
    
    int j,k,q;
    
    for( j=m-1; j>=0; j-- ) {
        for( k=0; k<=j; k++ ) { 
            vec[k] = A[j*la+k];
        }
        
        d[j] = mat_norm2(vec, j+1);
        sx1 = SIGN(vec[j]);
        vec[j] += sx1*d[j];

        //memset(vtA, 0, j*sizeof( double ) );
        mat_init(vtA, j+1, 1, 0.0 );
        mat_normalize(vec, j+1, vec);
        for( k=0; k<=j; k++ ) {
            for( q=0; q<=j; q++ ) {
                vtA[q] += vec[k]*A[q*la+k];
            }
        }
        
        for( k=0; k<=j; k++ ) {
            for( q=0; q<=j; q++ ) {
                A[q*la+k] -= 2*vec[k]*vtA[q];
            }
        }
    }
    
    return;
    
}


void mat_qr_T( float *A, int m ){
    
    mat_qr_sub_T( A, m, m );
    
    return;
    
}


