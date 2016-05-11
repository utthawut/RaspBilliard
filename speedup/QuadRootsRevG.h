//---------------------------------------------------------------------------

#ifndef QuadRootsRevGH
#define QuadRootsRevGH

//---------------------------------------------------------------------------

 #define LDBL_EPSILON 1.084202172485504434E-19L
// #define M_SQRT3 1.7320508075688772935274463L   // sqrt(3)
 #define M_SQRT3_2 0.8660254037844386467637231L   // sqrt(3)/2
// #define DBL_EPSILON  2.2204460492503131E-16    // 2^-52  typically defined in the compiler's float.h
 #define ZERO_PLUS   8.88178419700125232E-16      // 2^-50 = 4*DBL_EPSILON
 #define ZERO_MINUS -8.88178419700125232E-16
 #define TINY_VALUE  1.0E-30                      // This is what we use to test for zero. Usually to avoid divide by zero.

 long double solve_quartic(const long double coef_a, const long double coef_b, const long double coef_c, 
							const long double coef_d, const long double coef_e);
 long double solve_cubic(const long double coef_a, const long double coef_b, const long double coef_c, 
							const long double coef_d);
 long double solve_quadratic(const long double coef_a, const long double coef_b, const long double coef_c);
 long double find_smallest(int n, long double *real_roots, long double *imag_roots);
 int QuadCubicRoots(long double *Coeff, int N, long double *RealRoot, long double *ImagRoot);
 void QuadRoots(long double *P, long double *RealPart, long double *ImagPart);
 void CubicRoots(long double *P, long double *RealPart, long double *ImagPart);
 void BiQuadRoots(long double *P, long double *RealPart, long double *ImagPart);
 void ReversePoly(long double *P, int N);
 void InvertRoots(int N, long double *RealRoot, long double *ImagRoot);

#endif
