#pragma once

//Given a function f, and given a bracketing triplet of abscissas ax, bx, cx (such that bx is
//between ax and cx, and f(bx) is less than both f(ax) and f(cx)), this routine isolates
//the minimum to a fractional precision of about tol using Brent’s method. The abscissa of
//the minimum is returned as xmin, and the minimum function value is returned as brent, the
//returned function value.
double brentmin(double ax, double bx, double cx, double (*f)(void*, double), double tol, double* xmin, void* obj);

//Using Brent’s method, find the root of a function func known to lie between x1 and x2. The
//root, returned as zbrent, will be refined until its accuracy is tol.
double zbrent(double (*func)(void*, double), double x1, double x2, double tol, void* obj);