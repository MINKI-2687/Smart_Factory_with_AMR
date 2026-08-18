// linear_solve.h
// General linear system solver via Gaussian elimination with partial pivoting.
// C equivalent of numpy.linalg.solve(A, b). A is n x n (row-major), b is length n.
// Result written into x.

#ifndef LINEAR_SOLVE_H
#define LINEAR_SOLVE_H

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

// Solves without mutating A_in/b_in. Returns false on singular matrix.
static inline bool linear_solve(const double *A_in, const double *b_in, double *x_out, int n) {
    double *A = (double *)malloc(sizeof(double) * (size_t)n * n);
    double *b = (double *)malloc(sizeof(double) * (size_t)n);
    if (!A || !b) { free(A); free(b); return false; }
    memcpy(A, A_in, sizeof(double) * (size_t)n * n);
    memcpy(b, b_in, sizeof(double) * (size_t)n);

    for (int col = 0; col < n; col++) {
        int pivot_row = col;
        double best = fabs(A[col * n + col]);
        for (int r = col + 1; r < n; r++) {
            double v = fabs(A[r * n + col]);
            if (v > best) { best = v; pivot_row = r; }
        }
        if (best < 1e-12) { free(A); free(b); return false; }

        if (pivot_row != col) {
            for (int c = 0; c < n; c++) {
                double tmp = A[col * n + c];
                A[col * n + c] = A[pivot_row * n + c];
                A[pivot_row * n + c] = tmp;
            }
            double tmp = b[col]; b[col] = b[pivot_row]; b[pivot_row] = tmp;
        }

        double diag = A[col * n + col];
        for (int r = col + 1; r < n; r++) {
            double factor = A[r * n + col] / diag;
            if (factor == 0.0) continue;
            for (int c = col; c < n; c++) A[r * n + c] -= factor * A[col * n + c];
            b[r] -= factor * b[col];
        }
    }

    for (int r = n - 1; r >= 0; r--) {
        double sum = b[r];
        for (int c = r + 1; c < n; c++) sum -= A[r * n + c] * x_out[c];
        x_out[r] = sum / A[r * n + r];
    }

    free(A); free(b);
    return true;
}

#endif /* LINEAR_SOLVE_H */
