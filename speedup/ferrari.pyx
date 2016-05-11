from libc.math cimport sqrt
import cmath

cdef extern from "complex.h":
    double complex cexp(double complex)

cdef double complex  J=cexp(2j*cmath.pi/3)
cdef double complex  Jc=1/J

cdef cardano(double a, double b, double c, double d):
    cdef double z0
    cdef double a2, b2
    cdef double p ,q, D
    cdef double complex r
    cdef double complex u, v, w
    cdef double w0, w1, w2
    cdef double complex r1, r2, r3


    z0=b/3/a
    a2,b2 = a*a,b*b
    p=-b2/3/a2 +c/a
    q=(b/27*(2*b2/a2-9*c/a)+d)/a
    D=-4*p*p*p-27*q*q
    r=cmath.sqrt(-D/27+0j)
    u=((-q-r)/2)**0.33333333333333333333333
    v=((-q+r)/2)**0.33333333333333333333333
    w=u*v
    w0=abs(w+p/3)
    w1=abs(w*J+p/3)
    w2=abs(w*Jc+p/3)
    if w0<w1:
      if w2<w0 : v = v*Jc
    elif w2<w1 : v = v*Jc
    else: v = v*J
    r1 = u+v-z0
    r2 = u*J+v*Jc-z0
    r3 = u*Jc+v*J-z0
    return r1, r2, r3

cdef roots_2(double a, double complex b, double complex c):
    cdef double complex bp
    cdef double complex delta
    cdef double complex r1, r2


    bp=b/2
    delta=bp*bp-a*c
    r1=(-bp-delta**.5)/a
    r2=-r1-b/a
    return r1, r2

def solve_ferrari(double a, double b, double c, double d, double e):
    "Ferrarai's Method"
    "resolution of P=ax^4+bx^3+cx^2+dx+e=0, coeffs reals"
    "First shift : x= z-b/4/a  =>  P=z^4+pz^2+qz+r"
    cdef double z0
    cdef double a2, b2, c2, d2
    cdef double p, q, r
    cdef double A, B, C, D
    cdef double complex y0, y1, y2
    cdef double complex a0, b0
    cdef double complex r0, r1, r2, r3


    z0=b/4.0/a
    a2,b2,c2,d2 = a*a,b*b,c*c,d*d
    p = -3.0*b2/(8*a2)+c/a
    q = b*b2/8.0/a/a2 - 1.0/2*b*c/a2 + d/a
    r = -3.0/256*b2*b2/a2/a2 + c*b2/a2/a/16 - b*d/a2/4+e/a
    "Second find y so P2=Ay^3+By^2+Cy+D=0"
    A=8.0
    B=-4*p
    C=-8*r
    D=4*r*p-q*q
    y0,y1,y2=cardano(A,B,C,D)
    # if abs(y1.imag)<abs(y0.imag): y0=y1
    # if abs(y2.imag)<abs(y0.imag): y0=y2
    a0=(-p+2*y0)**.5
    if a0==0 : b0=y0**2-r
    else : b0=-q/2/a0
    r0,r1=roots_2(1,a0,y0+b0)
    r2,r3=roots_2(1,-a0,y0-b0)
    return (r0-z0,r1-z0,r2-z0,r3-z0)

def solve_cardano(double a, double b, double c, double d):
    cdef double z0
    cdef double a2, b2
    cdef double p ,q, D
    cdef double complex r
    cdef double complex u, v, w
    cdef double w0, w1, w2
    cdef double complex r1, r2, r3


    z0=b/3/a
    a2,b2 = a*a,b*b
    p=-b2/3/a2 +c/a
    q=(b/27*(2*b2/a2-9*c/a)+d)/a
    D=-4*p*p*p-27*q*q
    r=cmath.sqrt(-D/27+0j)
    u=((-q-r)/2)**0.33333333333333333333333
    v=((-q+r)/2)**0.33333333333333333333333
    w=u*v
    w0=abs(w+p/3)
    w1=abs(w*J+p/3)
    w2=abs(w*Jc+p/3)
    if w0<w1:
      if w2<w0 : v = v*Jc
    elif w2<w1 : v = v*Jc
    else: v = v*J
    r1 = u+v-z0
    r2 = u*J+v*Jc-z0
    r3 = u*Jc+v*J-z0
    return (r1, r2, r3)

def solve_quadratic(double a, double b, double c):
    cdef double d, square_root_d
    cdef double t1, t2


    if a == 0:
        return []
    d = (b**2) - (4*a*c)
    if d < 0:
        return []

    if d > 0:
        square_root_d = sqrt(d)
        t1 = (-b + square_root_d) / (2 * a)
        t2 = (-b - square_root_d) / (2 * a)
        if t1 > 0:
            if t2 > 0:
                if t1 < t2:
                    return [t1, t2]
                return [t2, t1]
            return [t1]
        elif t2 > 0:
            return [t2]
        else:
            return []
    else:
        t1 = -b / (2*a)
        if t1 > 0:
            return [t1]
        return []
