# The MIT License (MIT)
#
# Copyright (c) 2016 Utthawut Bootdee
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cdef extern from "QuadRootsRevG.h":
    long double solve_quartic(const long double coef_a, const long double coef_b, const long double coef_c, 
                                const long double coef_d, const long double coef_e)
    long double solve_cubic(const long double coef_a, const long double coef_b, const long double coef_c, 
                                const long double coef_d)
    long double solve_quadratic(const long double coef_a, const long double coef_b, const long double coef_c)

def cython_solve_quartic(long double coef_a, long double coef_b, long double coef_c, 
                            long double coef_d, long double coef_e):
    return solve_quartic(coef_a, coef_b, coef_c, coef_d, coef_e)

def cython_solve_cubic(long double coef_a, long double coef_b, long double coef_c, 
                        long double coef_d):
    return solve_cubic(coef_a, coef_b, coef_c, coef_d)
    
def cython_solve_quadratic(long double coef_a, long double coef_b, long double coef_c):
    return solve_quadratic(coef_a, coef_b, coef_c)
