from . import BaseFunction as bf
import numpy as np
import time

try:
    from scipy.linalg import cholesky_banded, cho_solve_banded
except Exception:  # pragma: no cover - scipy is optional at runtime
    cholesky_banded = None
    cho_solve_banded = None


def _find_span(n, p, u, knot):
    """Find the knot span for parameter u with degree p and n + 1 control points."""
    if u >= knot[n + 1] - 1e-12:
        return n
    if u <= knot[p] + 1e-12:
        return p

    low = p
    high = n + 1
    mid = (low + high) // 2
    while u < knot[mid] or u >= knot[mid + 1]:
        if u < knot[mid]:
            high = mid
        else:
            low = mid
        mid = (low + high) // 2
    return mid


def _basis_funs(span, u, p, knot):
    """Compute the p + 1 non-zero basis functions for a given span and parameter."""
    values = np.zeros(p + 1, dtype=float)
    values[0] = 1.0
    left = np.zeros(p + 1, dtype=float)
    right = np.zeros(p + 1, dtype=float)

    for j in range(1, p + 1):
        left[j] = u - knot[span + 1 - j]
        right[j] = knot[span + j] - u
        saved = 0.0
        for r in range(j):
            denom = right[r + 1] + left[j - r]
            temp = 0.0 if abs(denom) < 1e-12 else values[r] / denom
            values[r] = saved + right[r + 1] * temp
            saved = left[j - r] * temp
        values[j] = saved
    return values


def _banded_to_dense(lower_band):
    """Expand lower-band storage to a dense symmetric matrix for fallback solving."""
    band_width, n_cols = lower_band.shape[0] - 1, lower_band.shape[1]
    dense = np.zeros((n_cols, n_cols), dtype=lower_band.dtype)
    for offset in range(band_width + 1):
        cols = np.arange(0, n_cols - offset)
        rows = cols + offset
        dense[rows, cols] = lower_band[offset, cols]
        if offset > 0:
            dense[cols, rows] = lower_band[offset, cols]
    return dense


def curve_interpolation(D, N, k, param, knot):
    '''
    Given a set of N data points, D0, D1, ..., Dn and a degree k,
    find a B-spline curve of degree k defined by N control points
    that passes all data points in the given order.
    :param D: data points (N x 2)
    :param N: the number of data points
    :param k: degree
    :param param: parameters
    :param knot: knot vector
    :return: control points (N x 2)
    '''
    Nik = np.zeros((N, N))

    for i in range(N):
        for j in range(N):
            Nik[i][j] = bf.BaseFunction(j, k+1, param[i], knot)
    Nik[N-1][N-1] = 1
    print(Nik)
    Nik_inv = np.linalg.inv(Nik)
    print(Nik_inv)
    P = []
    for i in range(len(D)):
        P.append(np.dot(Nik_inv, D[i]).tolist())
    print(P)
    return P


def curve_approximation(D, N, H, k, param, knot, profile=None):
    '''
    Given a set of N data points, D0, D1, ..., Dn, a degree k,
    and a number H, where N > H > k >= 1, find a B-spline curve
    of degree k defined by H control points that satisfies the
    following conditions:
        1. this curve contains the first and last data points;
        2. this curve approximates the data polygon in the sense
        of least square;
    :param D: data points (N x 2)
    :param H: the number of control points
    :param k: degree
    :param param: parameters
    :param knot: knot vector
    :return: control points (H x 2)
    '''
    t_total0 = time.perf_counter()
    P = []
    if H >= N or H <= k:
        print('参数 H 超出范围')
        return P

    if profile is not None:
        profile.setdefault("lsq_basis_build_s", 0.0)
        profile.setdefault("lsq_qk_build_s", 0.0)
        profile.setdefault("lsq_normal_mat_s", 0.0)
        profile.setdefault("lsq_solve_s", 0.0)
        profile.setdefault("lsq_total_s", 0.0)

    D_arr = np.asarray(D, dtype=float)
    n_dim = D_arr.shape[0]
    P_all = np.zeros((n_dim, H))
    P_all[:, 0] = D_arr[:, 0]
    P_all[:, H - 1] = D_arr[:, N - 1]

    n_unknown = H - 2
    band_width = min(k, max(0, n_unknown - 1))
    M_band = np.zeros((band_width + 1, n_unknown), dtype=float)
    Q_all = np.zeros((n_unknown, n_dim), dtype=float)

    t_basis0 = time.perf_counter()
    t_qk_acc = 0.0
    t_normal_acc = 0.0
    for row_idx in range(1, N - 1):
        span = _find_span(H - 1, k, param[row_idx], knot)
        basis_vals = _basis_funs(span, param[row_idx], k, knot)
        start = span - k

        active_cols = []
        active_vals = []

        t_qk0 = time.perf_counter()
        q_row = D_arr[:, row_idx].copy()
        for local_idx, coeff in enumerate(basis_vals):
            ctrl_idx = start + local_idx
            if ctrl_idx == 0:
                q_row -= coeff * P_all[:, 0]
            elif ctrl_idx == H - 1:
                q_row -= coeff * P_all[:, H - 1]
            elif 0 < ctrl_idx < H - 1:
                active_cols.append(ctrl_idx - 1)
                active_vals.append(coeff)
        t_qk_acc += time.perf_counter() - t_qk0

        if not active_cols:
            continue

        cols = np.asarray(active_cols, dtype=int)
        vals = np.asarray(active_vals, dtype=float)

        t_normal0 = time.perf_counter()
        for upper_idx, col_upper in enumerate(cols):
            for lower_idx in range(upper_idx + 1):
                col_lower = cols[lower_idx]
                M_band[col_upper - col_lower, col_lower] += vals[upper_idx] * vals[lower_idx]
        Q_all[cols, :] += vals[:, None] * q_row[None, :]
        t_normal_acc += time.perf_counter() - t_normal0

    if profile is not None:
        profile["lsq_basis_build_s"] += time.perf_counter() - t_basis0 - t_qk_acc - t_normal_acc
        profile["lsq_qk_build_s"] += t_qk_acc
        profile["lsq_normal_mat_s"] += t_normal_acc

    t_solve0 = time.perf_counter()
    if cholesky_banded is not None and cho_solve_banded is not None:
        chol_band = cholesky_banded(M_band, lower=True, check_finite=False)
        P_all[:, 1:H - 1] = cho_solve_banded((chol_band, True), Q_all, check_finite=False).transpose()
    else:
        M_dense = _banded_to_dense(M_band)
        P_all[:, 1:H - 1] = np.linalg.solve(M_dense, Q_all).transpose()
    if profile is not None:
        profile["lsq_solve_s"] += time.perf_counter() - t_solve0

    P = P_all.tolist()

    if profile is not None:
        profile["lsq_total_s"] += time.perf_counter() - t_total0
    return P


def curve(P, N, k, param, knot):
    '''
    Calculate B-spline curve.
    :param P: Control points
    :param N: the number of control points
    :param k: degree
    :param param: parameters
    :param knot: knot vector
    :return: data point on the b-spline curve
    '''
    Nik = np.zeros((len(param), N))

    for i in range(len(param)):
        for j in range(N):
            Nik[i][j] = bf.BaseFunction(j, k+1, param[i], knot)
    Nik[len(param)-1][N - 1] = 1
    print(Nik)
    # D = np.dot(Nik, P)
    D = []
    for i in range(len(P)):
        D.append(np.dot(Nik, P[i]).tolist())
    # print(D)
    return D
