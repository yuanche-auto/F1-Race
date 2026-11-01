"""Learning MPC, lap to lap (control expansion phase E, docs/CX_E_protocol.md section 3; literature E.1, E.2, E.4).

``LearningMPC`` is the NMPC's multiple-shooting problem (``FrenetMPC``'s Frenet dynamics with RK4 of 50 ms, the friction
ellipses at eta, slip, power and steering limits, soft limits, track limits with the margin) without a reference, with

* a **local safe set**: the terminal state's (s, n, xi, v_x, v_y, r) equals a convex combination D lambda of P K stored
  states (lambda >= 0, sum 1 - E.2's convex-hull relaxation), softly (an exactly penalised slack);
* the **cost-to-go** w_T c^T lambda: the stored states' times to their lap's finish (negative past the line) combined
  with the same lambda (E.2's terminal cost), plus the NMPC's input-rate costs;
* an **affine model correction** (E.4): the velocity states' step gets Gamma_k z_k added at stage k,
  z = (v_x, v_y, r, delta, delta_rate, F_drive / 1e4, F_brake / 1e4, 1); Gamma is a parameter of each solve.

D, c and Gamma are parameters set before every solve by ``LMPCController`` from a ``LapStore`` (the stored laps of the
car): the K states nearest in s to the candidate terminal arclength from each of the last P laps; Gamma_k from a local,
kernel-weighted ridge regression of the 50 ms residuals (``LapStore.regress``). The steering angle is not in the terminal
condition (protocol section 3.2, clarification m3).

**Phase H (docs/CX_H_protocol.md): ``ATVLearningMPC`` / ``ATVLMPCController``** – the same optimal control problem, row
for row, solved as one QP per solve: every constraint row linearised at a candidate z_bar (g(z_bar) + dg(z_bar) dz), the
cost (quadratic and linear in the variables) kept exactly, plus 1e-6 ||z - z_bar||^2 – one Gauss-Newton (RTI-type) step of
the NLP, i.e. the affine-time-varying QP of the racing LMPC. Solver OSQP (CasADi's conic interface). The candidate: the
previous solution shifted by one step, its terminal state z_t = sum_i lambda_i succ(D_i) – the stored states one 50 ms step
after the selected points, weighted with the previous lambda – and its last input the successors' inputs weighted the
same way; at a lap's first solve the fastest stored lap's first N + 1 states and N inputs. The local safe set is selected
around z_t's s; each stored lap's last point (no successor) is excluded (review m2). A failed QP applies the candidate's
first input – the shifted previous plan, or at a lap's first solve the stored lap's first input – and is counted.

Attribution (protocol clarification m3): the formulation follows Rosolia & Borrelli, *Learning how to autonomously race a
car: a predictive control approach* (E.2; IEEE TCST) and *Learning model predictive control for iterative tasks* (E.1;
IEEE TAC 2018), with the error model of Xue, Zhu, Dolan & Borrelli, *Learning model predictive control with error dynamics
regression for autonomous racing* (E.4; ICRA 2024). The open-source codes urosolia/RacingLMPC (UC Berkeley; H.1) and
MPC-Berkeley/Racing-LMPC-ROS2 (H.2) were read and no code was copied; the successor-weighted terminal candidate is the
construction of E.2 / H.1.

**Phase I (docs/CX_I_protocol.md, candidate I-1): the speed bound per lap** – ``LearningMPC(speed_bound=True)`` adds one
soft row per stage i = 1..N, v_x,i <= vbar_i + sigma_i (sigma >= 0, the track slacks' exact penalty); vbar (N) is a
parameter of each solve, appended after phase E's parameters. ``LMPCController(eps=...)`` sets vbar_i = (1 + eps)
v_ref(s_hat_i): v_ref the largest v_x at s among the last P stored laps (``LapStore.v_ref``, linear in the unwrapped s,
the run-out included), s_hat the shifted previous solution's arclengths (at a lap's first solve s_0 + i dt v_x,0). Off by
default: phase E's problem, parameters and variables are unchanged. The bound is this project's design (literature §I;
motivated by the lap-gated envelope of Zhao et al., arXiv:2608.28723, I.1).
"""
from __future__ import annotations

import time

import casadi as ca
import numpy as np

from ..dynamics import DynVehicle, body_accel, tyre_forces
from ..track import Track
from .dyn_tracking import traction_limit
from .nmpc import FS, BRAKE_REQUEST, MPCController, _periodic_interpolant  # noqa: F401

TERM = (0, 1, 2, 3, 4, 5)                 # s, n, xi, vx, vy, r in the terminal condition
TERM7 = (0, 1, 2, 3, 4, 5, 6)              # development option: with the steering angle
TERM_SCALE7 = np.array([1.0, 0.1, 0.01, 0.1, 0.1, 0.01, 0.005])   # the terminal slack's units (its penalty is per unit)
TERM_SCALE = TERM_SCALE7[:6]
NZ = 8                                    # regressor z = (vx, vy, r, delta, delta_rate, Fd/FS, Fb/FS, 1)


def regressor(X: np.ndarray, U: np.ndarray) -> np.ndarray:
    """z for states (n, 7) and inputs (n, 3) (forces in N)."""
    X, U = np.atleast_2d(X), np.atleast_2d(U)
    return np.column_stack([X[:, 3], X[:, 4], X[:, 5], X[:, 6], U[:, 0], U[:, 1] / FS, U[:, 2] / FS, np.ones(len(X))])


class LearningMPC:
    def __init__(self, track: Track, veh: DynVehicle, n_pts: int, N: int = 30, dt: float = 0.05, eta: float = 0.95,
                 margin: float = 0.6, w_T: float = 100.0, weights: dict | None = None, max_iter: int = 500,
                 term: tuple = TERM, speed_bound: bool = False):
        """``term``: the state components in the terminal condition (default TERM, without delta; development option
        TERM7 with it). ``speed_bound`` (phase I): the soft speed bound per stage, its bounds a parameter of each solve."""
        w = {"drate": 1.0, "dF": 0.5, "slack": 1e4, "limit_slack": 1e6, "term_slack": 1e4}
        w.update(weights or {})
        self.track, self.veh, self.N, self.dt, self.n_pts, self.w = track, veh, N, dt, n_pts, w
        self.term = tuple(term)
        self.speed_bound = bool(speed_bound)
        nt = len(self.term)
        L = track.length
        kap = _periodic_interpolant("kap", track.s, track.kappa, L)
        wl = _periodic_interpolant("wl", track.s, track.w_left, L)
        wr = _periodic_interpolant("wr", track.s, track.w_right, L)
        x, u = ca.SX.sym("x", 7), ca.SX.sym("u", 3)
        s, n, xi, vx, vy, r, d = (x[i] for i in range(7))
        k = kap(s)
        F = tyre_forces(veh, vx, vy, r, d, u[1] * FS, u[2] * FS, lib=ca)
        dvx, dvy, dr = body_accel(veh, vx, vy, r, d, F, lib=ca)
        sdot = (vx * ca.cos(xi) - vy * ca.sin(xi)) / (1 - n * k)
        f = ca.Function("f", [x, u], [ca.vertcat(sdot, vx * ca.sin(xi) + vy * ca.cos(xi), r - k * sdot, dvx, dvy, dr,
                                                 u[0])])
        h = dt
        k1 = f(x, u); k2 = f(x + h / 2 * k1, u); k3 = f(x + h / 2 * k2, u); k4 = f(x + h * k3, u)
        step = ca.Function("lmpc_step", [x, u], [x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)])
        self._step = step

        X, U = ca.SX.sym("X", 7, N + 1), ca.SX.sym("U", 3, N)
        S = ca.SX.sym("S", 2, N + 1)
        T = ca.SX.sym("T", 4, N)
        LAM = ca.SX.sym("LAM", n_pts)
        EP, EM = ca.SX.sym("EP", nt), ca.SX.sym("EM", nt)
        nP = 7 + 3 + nt * n_pts + n_pts + N * 3 * NZ + (N if self.speed_bound else 0)
        P = ca.SX.sym("P", nP)
        i0 = 10
        Dm = ca.reshape(P[i0:i0 + nt * n_pts], nt, n_pts)
        i0 += nt * n_pts
        cv = P[i0:i0 + n_pts]
        i0 += n_pts
        G = P[i0:i0 + N * 3 * NZ]
        g, lbg, ubg = [], [], []
        J = 0
        g.append(X[:, 0] - P[:7]); lbg += [0] * 7; ubg += [0] * 7
        body = veh.half_width + margin
        ap = veh.alpha_peak
        for i in range(N):
            z = ca.vertcat(X[3, i], X[4, i], X[5, i], X[6, i], U[0, i], U[1, i], U[2, i], 1.0)
            Gi = ca.reshape(G[i * 3 * NZ:(i + 1) * 3 * NZ], 3, NZ)
            corr = ca.vertcat(0, 0, 0, ca.mtimes(Gi, z), 0)
            g.append(X[:, i + 1] - (step(X[:, i], U[:, i]) + corr)); lbg += [0] * 7; ubg += [0] * 7
            Fi = tyre_forces(veh, X[3, i], X[4, i], X[5, i], X[6, i], U[1, i] * FS, U[2, i] * FS, lib=ca)
            uf2 = (Fi["fxf"] / (veh.mu_x * Fi["fzf"])) ** 2 + (Fi["fyf"] / (veh.mu_y * Fi["fzf"])) ** 2
            ur2 = (Fi["fxr"] / (veh.mu_x * Fi["fzr"])) ** 2 + (Fi["fyr"] / (veh.mu_y * Fi["fzr"])) ** 2
            # the NMPC's soft rows (FrenetMPC, soft_limits=True, no param_tightening)
            g += [uf2 - T[0, i], ur2 - T[1, i], Fi["af"] - ap * T[2, i], Fi["af"] + ap * T[2, i],
                  Fi["ar"] - ap * T[3, i], Fi["ar"] + ap * T[3, i], U[1, i] * FS * X[3, i],
                  Fi["fxf"] / (veh.mu_x * Fi["fzf"]), Fi["fxr"] / (veh.mu_x * Fi["fzr"])]
            lbg += [-ca.inf, -ca.inf, -ca.inf, -ap, -ca.inf, -ap, -ca.inf, -eta, -eta]
            ubg += [eta**2, eta**2, ap, ca.inf, ap, ca.inf, veh.P_max, eta, eta]
            J += w["limit_slack"] * h * (ca.sumsqr(T[:, i]) + 10.0 * ca.sum1(T[:, i]))
            du = U[:, i] - (P[7:10] if i == 0 else U[:, i - 1])
            J += h * (w["drate"] * U[0, i] ** 2 + w["dF"] * (du[1] ** 2 + du[2] ** 2) / h)
        for i in range(1, N + 1):
            si, ni, xii = X[0, i], X[1, i], X[2, i]
            ki = kap(si)
            for j, arm in enumerate((veh.lf, -veh.lr)):
                chord = 0.5 * arm**2 * ca.fabs(ki)
                na = ni + arm * ca.sin(xii)
                g += [na + (wr(si) - body - chord) + S[j, i], (wl(si) - body - chord) - na + S[j, i]]
                lbg += [0.0, 0.0]; ubg += [ca.inf, ca.inf]
            J += w["slack"] * h * ca.sumsqr(S[:, i]) + 10 * w["slack"] * h * ca.sum1(S[:, i])
        # the local safe set and the cost-to-go
        xN = ca.vertcat(*[X[c, N] for c in self.term])
        g.append(xN - ca.mtimes(Dm, LAM) - (EP - EM) * TERM_SCALE7[list(self.term)]); lbg += [0] * nt; ubg += [0] * nt
        g.append(ca.sum1(LAM)); lbg.append(1.0); ubg.append(1.0)
        J += w_T * ca.dot(cv, LAM) + w["term_slack"] * (ca.sumsqr(EP) + ca.sumsqr(EM) + 10.0 * (ca.sum1(EP) + ca.sum1(EM)))
        extra = []
        if self.speed_bound:                            # phase I: v_x,i <= vbar_i + sigma_i, i = 1..N (soft)
            SV = ca.SX.sym("SV", N)
            VB = P[nP - N:]
            for i in range(1, N + 1):
                g.append(X[3, i] - VB[i - 1] - SV[i - 1]); lbg.append(-ca.inf); ubg.append(0.0)
            J += w["slack"] * h * ca.sumsqr(SV) + 10 * w["slack"] * h * ca.sum1(SV)
            extra = [SV]

        zv = ca.vertcat(ca.vec(X), ca.vec(U), ca.vec(S), ca.vec(T), LAM, EP, EM, *extra)
        nx, nu = 7 * (N + 1), 3 * N
        lbz, ubz = np.full(zv.shape[0], -np.inf), np.full(zv.shape[0], np.inf)
        Xi = np.arange(nx).reshape(N + 1, 7)
        Ui = nx + np.arange(nu).reshape(N, 3)
        Si = nx + nu + np.arange(2 * (N + 1)).reshape(N + 1, 2)
        Ti = nx + nu + 2 * (N + 1) + np.arange(4 * N).reshape(N, 4)
        Li = nx + nu + 2 * (N + 1) + 4 * N + np.arange(n_pts)
        Ei = Li[-1] + 1 + np.arange(2 * nt)
        self.Vi = Ei[-1] + 1 + np.arange(N) if self.speed_bound else np.zeros(0, int)
        lbz[Xi[:, 3]] = 2.0
        lbz[Xi[:, 6]], ubz[Xi[:, 6]] = -veh.delta_max, veh.delta_max
        lbz[Ui[:, 0]], ubz[Ui[:, 0]] = -veh.delta_rate_max, veh.delta_rate_max
        lbz[Ui[:, 1]], ubz[Ui[:, 1]] = 0.0, np.inf
        lbz[Ui[:, 2]], ubz[Ui[:, 2]] = -np.inf, 0.0
        lbz[Si] = 0.0
        ubz[Si[0]] = 0.0
        lbz[Ti] = 0.0
        lbz[Li], ubz[Li] = 0.0, 1.0
        lbz[Ei] = 0.0
        lbz[self.Vi] = 0.0
        self.Xi, self.Ui, self.Si, self.Ti, self.Li, self.Ei = Xi, Ui, Si, Ti, Li, Ei
        self.lbz, self.ubz = lbz, ubz
        self.lbg, self.ubg = np.array(lbg, float), np.array(ubg, float)
        nlp = {"x": zv, "f": J, "g": ca.vertcat(*g), "p": P}
        self.solver = ca.nlpsol("lmpc", "ipopt", nlp, {
            "print_time": False,
            "ipopt": {"print_level": 0, "max_iter": max_iter, "tol": 1e-4, "acceptable_tol": 1e-3,
                      "acceptable_iter": 5, "warm_start_init_point": "yes", "mu_init": 1e-3, "linear_solver": "mumps"}})
        self._nlp = nlp
        self.reset()

    def reset(self) -> None:
        self.z = self.lam_g = self.lam_x = None
        self.stats: list[dict] = []

    def initial_guess(self, x0: np.ndarray, u0: np.ndarray) -> np.ndarray:
        z = np.zeros(self.lbz.shape)
        xs = np.array(x0, float)
        for i in range(self.N + 1):
            z[self.Xi[i]] = xs
            xs = xs.copy()
            xs[0] += self.dt * xs[3]
        for i in range(self.N):
            z[self.Ui[i]] = [0.0, u0[1] / FS, u0[2] / FS]
        z[self.Li] = 1.0 / self.n_pts
        return z

    def solve(self, x0: np.ndarray, u_prev: np.ndarray, D: np.ndarray, c: np.ndarray,
              G: np.ndarray | None = None, vbar: np.ndarray | None = None) -> tuple[np.ndarray, dict]:
        """``D`` (len(term), n_pts) the stored terminal states, ``c`` (n_pts) their times-to-go, ``G`` (N, 3, NZ) the model
        correction (None: zero), ``vbar`` (N) the speed bounds of stages 1..N (with ``speed_bound`` only). A failed solve
        applies the shifted previous plan (as ``FrenetMPC``)."""
        G = np.zeros((self.N, 3, NZ)) if G is None else np.asarray(G, float)
        p = np.concatenate([x0, [u_prev[0], u_prev[1] / FS, u_prev[2] / FS], np.asarray(D, float).ravel(order="F"),
                            np.asarray(c, float), G.ravel()])
        if self.speed_bound:
            vbar = np.asarray(vbar, float)
            if vbar.shape != (self.N,):
                raise ValueError("speed_bound: vbar (N,) required")
            p = np.concatenate([p, vbar])
        elif vbar is not None:
            raise ValueError("vbar given without speed_bound")
        if self.z is None:
            z0, args = self.initial_guess(x0, u_prev), {}
        else:
            z0 = self.z.copy()
            X, U = self.z[self.Xi], self.z[self.Ui]
            z0[self.Xi[:-1]], z0[self.Ui[:-1]] = X[1:], U[1:]
            z0[self.Xi[0]] = x0
            z0[self.Li] = 1.0 / self.n_pts          # the stored points change from solve to solve
            z0[self.Ei] = 0.0
            args = {}
        if self.speed_bound:                            # the speed slacks consistent with the guess
            z0[self.Vi] = np.maximum(z0[self.Xi[1:, 3]] - vbar, 0.0)
        t0 = time.time()
        sol = self.solver(x0=z0, p=p, lbx=self.lbz, ubx=self.ubz, lbg=self.lbg, ubg=self.ubg, **args)
        st = self.solver.stats()
        ok = st["return_status"] in ("Solve_Succeeded", "Solved_To_Acceptable_Level")
        fallback = False
        if ok or self.z is None:
            z = np.asarray(sol["x"]).ravel()
            self.z = z
        else:
            z, fallback = z0, True
            self.z = z0
        u = z[self.Ui[0]] * np.array([1.0, FS, FS])
        info = {"status": st["return_status"], "iter": st["iter_count"], "time": time.time() - t0,
                "slack": float(np.max(z[self.Si])), "fallback": fallback,
                "limit_slack": float(np.max(z[self.Ti])), "term_slack": float(np.max(z[self.Ei])),
                "lam_max": float(np.max(z[self.Li]))}
        if self.speed_bound:
            vx = z[self.Xi[1:, 3]]
            info.update(speed_slack=float(np.max(z[self.Vi])), speed_near_share=float(np.mean(vx >= 0.99 * vbar)))
        self.stats.append(info)
        return u, info

    def prediction(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """States (N + 1, 7), inputs (N, 3, forces in N) and lambda of the last solve."""
        return self.z[self.Xi].copy(), self.z[self.Ui] * np.array([1.0, FS, FS]), self.z[self.Li].copy()


class LapStore:
    """The stored laps of one car: per lap the 50 ms records (Frenet state with s from about 0 at the start, the commanded
    input) and the lap's finish time; the safe-set selection and the error regression of protocol section 3.2."""

    def __init__(self, nominal_step, M: int = 80, bandwidth: float = 1.0, ridge: float = 1e-4):
        self.laps: list[dict] = []
        self.step, self.M, self.h, self.eps = nominal_step, M, bandwidth, ridge
        self._tree = None

    def add(self, t: np.ndarray, X: np.ndarray, U: np.ndarray, t_finish: float) -> None:
        """One lap's 50 ms records: times t, states X (n, 7), commanded inputs U (n, 3, forces in N); t_finish the time
        at s = L (the time-to-go T - t is negative past the line)."""
        o = np.argsort(X[:, 0], kind="stable")
        self.laps.append({"t": t, "X": X, "U": U, "T": float(t_finish), "s_sorted": X[o, 0], "order": o})
        self._tree = None

    def __len__(self) -> int:
        return len(self.laps)

    def select(self, s_tilde: float, K: int, P: int = 2, term: tuple = TERM) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """The K stored states nearest in s to s_tilde from each of the last P laps: (D (len(term), P K), c (P K), the
        stored lap's index)."""
        cols, cs, lap_ids = [], [], []
        ids = list(range(len(self.laps)))[-P:]
        while len(ids) < P:                             # fewer stored laps than P: repeat the last (same points twice)
            ids = [ids[0]] + ids
        for li in ids:
            lap = self.laps[li]
            ss = lap["s_sorted"]
            j = int(np.searchsorted(ss, s_tilde))
            lo, hi = max(0, j - K), min(len(ss), j + K)
            cand = np.arange(lo, hi)
            near = cand[np.argsort(np.abs(ss[cand] - s_tilde), kind="stable")[:K]]
            if len(near) < K:                           # at the ends of the stored range: pad with the last point
                near = np.concatenate([near, np.full(K - len(near), near[-1] if len(near) else len(ss) - 1)])
            idx = lap["order"][near]
            cols.append(lap["X"][idx][:, list(term)].T)
            cs.append(lap["T"] - lap["t"][idx])
            lap_ids.append(np.full(K, li))
        return np.hstack(cols), np.concatenate(cs), np.concatenate(lap_ids)

    def v_ref(self, s: np.ndarray, P: int = 2) -> np.ndarray:
        """Phase I: the largest v_x at each arclength s among the last P stored laps – each lap's 50 ms records
        interpolated linearly in the unwrapped s (the run-out past the line included; constant beyond a lap's ends)."""
        s = np.atleast_1d(np.asarray(s, float))
        out = np.full(s.shape, -np.inf)
        for lap in self.laps[-P:]:
            out = np.maximum(out, np.interp(s, lap["s_sorted"], lap["X"][lap["order"], 3]))
        return out

    def fastest(self) -> dict:
        """The stored lap with the shortest finish time (phase H: a lap's first candidate)."""
        return min(self.laps, key=lambda lap: lap["T"])

    def select_succ(self, s_query: float, K: int, P: int = 2, term: tuple = TERM):
        """Phase H: as ``select`` around s_query, without each lap's last point (it has no successor, review m2), and
        with the successors of the selected points: (D (len(term), P K), c (P K), Dsucc (7, P K), Usucc (3, P K, forces in
        N))."""
        cols, cs, sx, su = [], [], [], []
        ids = list(range(len(self.laps)))[-P:]
        while len(ids) < P:
            ids = [ids[0]] + ids
        for li in ids:
            lap = self.laps[li]
            n = len(lap["X"])
            valid = lap["order"][lap["order"] != n - 1]
            ss = lap["X"][valid, 0]
            j = int(np.searchsorted(ss, s_query))
            lo, hi = max(0, j - K), min(len(ss), j + K)
            cand = np.arange(lo, hi)
            near = cand[np.argsort(np.abs(ss[cand] - s_query), kind="stable")[:K]]
            if len(near) < K:
                near = np.concatenate([near, np.full(K - len(near), near[-1] if len(near) else len(ss) - 1)])
            idx = valid[near]
            cols.append(lap["X"][idx][:, list(term)].T)
            cs.append(lap["T"] - lap["t"][idx])
            sx.append(lap["X"][idx + 1].T)
            su.append(lap["U"][idx + 1].T)
        return np.hstack(cols), np.concatenate(cs), np.hstack(sx), np.hstack(su)

    def _samples(self):
        """All stored 50 ms transitions: z (n, 8), the residual of the velocity states (n, 3) against the nominal step."""
        if self._tree is None:
            from scipy.spatial import cKDTree

            Z, E = [], []
            for lap in self.laps:
                X, U = lap["X"], lap["U"]
                if len(X) < 2:
                    continue
                nxt = np.array([np.asarray(self.step(X[k], U[k] / np.array([1.0, FS, FS]))).ravel()
                                for k in range(len(X) - 1)])
                Z.append(regressor(X[:-1], U[:-1]))
                E.append(X[1:, 3:6] - nxt[:, 3:6])
            Z, E = (np.vstack(Z), np.vstack(E)) if Z else (np.zeros((0, NZ)), np.zeros((0, 3)))
            scale = np.maximum(Z[:, :NZ - 1].std(axis=0), 1e-6) if len(Z) else np.ones(NZ - 1)
            self._Z, self._E, self._scale = Z, E, scale
            self._tree = cKDTree(Z[:, :NZ - 1] / scale) if len(Z) else None
        return self._Z, self._E

    def regress(self, Zc: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Gamma (n, 3, 8) for the candidate regressors Zc (n, 8): Epanechnikov-weighted ridge regression on the M nearest
        samples (scaled distance, bandwidth h); Gamma = 0 where no sample lies within h (E.4's fallback). Returns
        (Gamma, fallback flags)."""
        Z, E = self._samples()
        n = len(Zc)
        G = np.zeros((n, 3, NZ))
        fb = np.ones(n, bool)
        if self._tree is None or len(Z) < NZ:
            return G, fb
        m = min(self.M, len(Z))
        dist, idx = self._tree.query(Zc[:, :NZ - 1] / self._scale, k=m)
        dist, idx = np.atleast_2d(dist), np.atleast_2d(idx)
        for i in range(n):
            w = np.clip(1.0 - (dist[i] / self.h) ** 2, 0.0, None)
            if not np.any(w > 0):
                continue
            A, b = Z[idx[i]], E[idx[i]]
            Aw = A * w[:, None]
            G[i] = np.linalg.solve(A.T @ Aw + self.eps * np.eye(NZ), Aw.T @ b).T
            fb[i] = False
        return G, fb


class LMPCController(MPCController):
    """Drives a ``LearningMPC`` from a ``LapStore``; records the 50 ms states and commands of the lap (for the store) and
    the per-solve diagnostics. The lap's arclength starts near 0 (the start state projects to s = L - 1e-11)."""

    def __init__(self, lmpc: LearningMPC, store: LapStore, K: int, P: int = 2, regression: bool = True,
                 sim_dt: float = 0.01, tc_eta: float | None = 0.99, eps: float | None = None):
        """``eps`` (phase I): the speed bound's step, vbar_i = (1 + eps) v_ref(s_hat_i); needs ``lmpc.speed_bound``."""
        if (eps is not None) != lmpc.speed_bound:
            raise ValueError("eps and the problem's speed_bound go together")
        self.eps = eps
        self.mpc = lmpc
        self.sim_dt, self.tc_eta = sim_dt, tc_eta
        self.track = lmpc.track
        self.u = np.zeros(3)
        self.t_next = 0.0
        self.s_unwrapped = None
        self.log: list[tuple] = []
        self.store, self.K, self.P, self.regression = store, K, P, regression
        self.rec_t, self.rec_x, self.rec_u = [], [], []
        self.diag: list[dict] = []

    def frenet(self, x: np.ndarray) -> np.ndarray:
        first = self.s_unwrapped is None
        xf = super().frenet(x)
        if first and xf[0] > 0.5 * self.track.length:   # just before the line: the lap starts at s ~ 0
            xf[0] -= self.track.length
            self.s_unwrapped = xf[0]
        return xf

    def __call__(self, t: float, x: np.ndarray):
        xf = self.frenet(x)
        if t + 1e-9 >= self.t_next:
            m = self.mpc
            if m.z is None:
                s_tilde = xf[0] + m.N * m.dt * xf[3]
                Xc = np.tile(xf, (m.N, 1))
                Uc = np.tile(self.u, (m.N, 1))
                s_hat = xf[0] + m.dt * xf[3] * np.arange(1, m.N + 1)
            else:
                Xp, Up, _ = m.prediction()
                s_tilde = Xp[-1, 0] + m.dt * Xp[-1, 3]
                Xc, Uc = np.vstack([Xp[1:m.N], Xp[m.N:]]), np.vstack([Up[1:], Up[-1:]])
                s_hat = np.append(Xp[2:, 0], Xp[-1, 0] + m.dt * Xp[-1, 3])
            D, c, lap_ids = self.store.select(s_tilde, self.K, self.P, m.term)
            if self.regression:
                G, fb = self.store.regress(regressor(Xc, Uc))
            else:
                G, fb = None, np.ones(m.N, bool)
            if self.eps is not None:
                vbar = (1.0 + self.eps) * self.store.v_ref(s_hat, self.P)
                u, info = m.solve(xf, self.u, D, c, G, vbar=vbar)
            else:
                u, info = m.solve(xf, self.u, D, c, G)
            self.u = u
            self.t_next = t + m.dt
            self.log.append((t, xf[0], xf[1], info["time"], info["status"], info["slack"]))
            self.diag.append({"t": t, "s": xf[0], "term_slack": info["term_slack"], "fallback_share": float(np.mean(fb)),
                              "s_tilde": s_tilde, "c_min": float(np.min(c)), "past_line_pts": int(np.sum(D[0] > m.track.length))})
            if self.eps is not None:
                self.diag[-1].update(vx=float(xf[3]), v_ref=float(self.store.v_ref(xf[0], self.P)[0]),
                                     vbar1=float(vbar[0]), speed_slack=info["speed_slack"],
                                     speed_near_share=info["speed_near_share"])
            self.rec_t.append(t)
            self.rec_x.append(xf.copy())
            self.rec_u.append(u.copy())
        drate, fd, fb_ = float(self.u[0]), float(self.u[1]), float(self.u[2])
        if self.tc_eta is not None:
            veh = self.mpc.veh
            d_next = x[6] + np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max) * self.sim_dt
            fd, fb_ = traction_limit(veh, x, d_next, fd, fb_, self.tc_eta)
        return drate, fd, fb_


class Recorder:
    """Wraps any controller with ``.u`` (the command of its last solve) and ``.mpc.dt``: records the 50 ms Frenet states
    (s from about 0 at the start) and commands, for the store (the initialisation laps)."""

    def __init__(self, ctl, track: Track, dt: float = 0.05):
        self.ctl, self.track, self.dt = ctl, track, dt
        self.t_next = 0.0
        self.rec_t, self.rec_x, self.rec_u = [], [], []
        self._fr = MPCController.__new__(MPCController)
        self._fr.track, self._fr.s_unwrapped = track, None

    def __getattr__(self, name):
        return getattr(self.ctl, name)

    def __call__(self, t: float, x: np.ndarray):
        out = self.ctl(t, x)
        if t + 1e-9 >= self.t_next:
            first = self._fr.s_unwrapped is None
            xf = MPCController.frenet(self._fr, x)
            if first and xf[0] > 0.5 * self.track.length:
                xf[0] -= self.track.length
                self._fr.s_unwrapped = xf[0]
            self.rec_t.append(t)
            self.rec_x.append(xf.copy())
            self.rec_u.append(np.asarray(self.ctl.u, float).copy())
            self.t_next = t + self.dt
        return out


class ATVLearningMPC(LearningMPC):
    """``LearningMPC``'s problem solved as one QP per solve (phase H; module docstring). ``rho``: the regularisation
    weight on ||z - z_bar||^2; ``osqp``: OSQP's options. Development diagnostics only (protocol clarification m1; the
    registered design uses neither): ``qp_solver`` another CasADi conic plugin (e.g. "qpoases") with ``qp_opts``;
    ``trust`` = (steering rate [rad/s], force [N]): the QP's input step from the candidate bounded per stage."""

    def __init__(self, *args, rho: float = 1e-6, osqp: dict | None = None, qp_solver: str = "osqp",
                 qp_opts: dict | None = None, trust: tuple | None = None, **kw):
        super().__init__(*args, **kw)
        if self.speed_bound:
            raise NotImplementedError("phase I's speed bound is defined on the NLP only")
        z, J, g, p = (self._nlp[k] for k in ("x", "f", "g", "p"))
        self._gf = ca.Function("atv_g", [z, p], [g])
        self._jg = ca.Function("atv_jg", [z, p], [ca.jacobian(g, z)])
        self._dJ = ca.Function("atv_dJ", [z, p], [ca.gradient(J, z)])
        H = ca.Function("atv_H", [z, p], [ca.hessian(J, z)[0]])
        n = z.shape[0]
        # the cost is quadratic and linear in z: its Hessian is constant (checked at two points)
        H0 = H(np.zeros(n), np.zeros(p.shape[0]))
        H1 = H(np.random.default_rng(0).standard_normal(n), np.random.default_rng(1).standard_normal(p.shape[0]))
        if float(ca.norm_inf(H0 - H1)) > 1e-9:
            raise ValueError("the cost's Hessian is not constant")
        self._H = H0 + 2.0 * rho * ca.DM.eye(n)
        self.rho = rho
        if qp_solver == "osqp":
            opts = {"verbose": False, "eps_abs": 1e-5, "eps_rel": 1e-5, "polish": True, "max_iter": 20000}
            opts.update(osqp or {})
            copts = {"osqp": opts, "error_on_fail": False}
        else:                                           # development diagnostics: another CasADi conic plugin
            copts = {"error_on_fail": False, **(qp_opts or {})}
        self.qp_solver, self.trust = qp_solver, trust
        self.qp = ca.conic("atv_qp", qp_solver, {"h": self._H.sparsity(), "a": self._jg.sparsity_out(0)}, copts)

    def params(self, x0, u_prev, D, c, G=None) -> np.ndarray:
        G = np.zeros((self.N, 3, NZ)) if G is None else np.asarray(G, float)
        return np.concatenate([x0, [u_prev[0], u_prev[1] / FS, u_prev[2] / FS], np.asarray(D, float).ravel(order="F"),
                               np.asarray(c, float), G.ravel()])

    def candidate_vector(self, Xc: np.ndarray, Uc: np.ndarray) -> np.ndarray:
        """z_bar from candidate states (N + 1, 7) and inputs (N, 3, forces in N): slacks 0, lambda uniform (lambda enters
        only linear rows, so its candidate value does not change the QP)."""
        z = np.zeros(self.lbz.shape)
        z[self.Xi] = Xc
        z[self.Ui] = np.asarray(Uc, float) / np.array([1.0, FS, FS])
        z[self.Li] = 1.0 / self.n_pts
        return z

    def solve_qp(self, x0: np.ndarray, u_prev: np.ndarray, D: np.ndarray, c: np.ndarray, G, zbar: np.ndarray):
        """One QP at the candidate ``zbar``; a failed QP keeps the candidate (its first input is applied)."""
        p = self.params(x0, u_prev, D, c, G)
        t0 = time.time()
        gb = np.asarray(self._gf(zbar, p)).ravel()
        lbx, ubx = self.lbz - zbar, self.ubz - zbar
        if self.trust is not None:                      # development diagnostic: a box on the input step
            r = np.array([self.trust[0], self.trust[1] / FS, self.trust[1] / FS])
            lbx[self.Ui] = np.maximum(lbx[self.Ui], -r)
            ubx[self.Ui] = np.minimum(ubx[self.Ui], r)
        sol = self.qp(h=self._H, g=self._dJ(zbar, p), a=self._jg(zbar, p), lba=self.lbg - gb, uba=self.ubg - gb,
                      lbx=lbx, ubx=ubx)
        st = self.qp.stats()
        ok = bool(st.get("success", False))
        z = zbar + np.asarray(sol["x"]).ravel() if ok else zbar.copy()
        self.z = z
        u = z[self.Ui[0]] * np.array([1.0, FS, FS])
        info = {"status": st.get("return_status", "?"), "iter": int(st.get("iter_count", -1) or -1),
                "time": time.time() - t0, "slack": float(np.max(z[self.Si])), "fallback": not ok,
                "limit_slack": float(np.max(z[self.Ti])), "term_slack": float(np.max(z[self.Ei])),
                "lam_max": float(np.max(z[self.Li]))}
        self.stats.append(info)
        return u, info


class ATVLMPCController(LMPCController):
    """``LMPCController`` with E.2's candidate and the QP (phase H; module docstring). Records per solve the
    linearisation error: the model's rollout of the QP's inputs (the nominal step plus the correction Gamma) against the
    QP's predicted state at N – |ds|, |dn| [m], |dv_x| [m/s] (protocol section 5)."""

    def __init__(self, *args, **kw):
        super().__init__(*args, **kw)
        self.prev = None
        self.lin_err: list[tuple[float, float, float]] = []

    def _candidate(self):
        m = self.mpc
        if self.prev is None:                            # a lap's first solve: the fastest stored lap's beginning
            lap = self.store.fastest()
            return lap["X"][:m.N + 1].copy(), lap["U"][:m.N].copy()
        Xp, Up, lam, Dsucc, Usucc = self.prev
        return np.vstack([Xp[1:], Dsucc @ lam]), np.vstack([Up[1:], Usucc @ lam])

    def _rollout_error(self, xf, U, X, G):
        m = self.mpc
        x = np.array(xf, float)
        for k in range(m.N):
            xn = np.asarray(m._step(x, U[k] / np.array([1.0, FS, FS]))).ravel()
            if G is not None:
                xn[3:6] += G[k] @ regressor(x, U[k])[0]
            x = xn
        return abs(x[0] - X[m.N, 0]), abs(x[1] - X[m.N, 1]), abs(x[3] - X[m.N, 3])

    def __call__(self, t: float, x: np.ndarray):
        xf = self.frenet(x)
        if t + 1e-9 >= self.t_next:
            m = self.mpc
            Xc, Uc = self._candidate()
            s_q = float(Xc[-1, 0])
            D, c, Dsucc, Usucc = self.store.select_succ(s_q, self.K, self.P, m.term)
            if self.regression:
                G, fb = self.store.regress(regressor(Xc[:m.N], Uc))
            else:
                G, fb = None, np.ones(m.N, bool)
            u, info = m.solve_qp(xf, self.u, D, c, G, m.candidate_vector(Xc, Uc))
            Xs, Us, lam = m.prediction()
            self.prev = (Xs, Us, lam, Dsucc, Usucc)
            self.lin_err.append(self._rollout_error(xf, Us, Xs, G))
            self.u = u
            self.t_next = t + m.dt
            self.log.append((t, xf[0], xf[1], info["time"], info["status"], info["slack"]))
            self.diag.append({"t": t, "s": xf[0], "term_slack": info["term_slack"], "fallback_share": float(np.mean(fb)),
                              "s_tilde": s_q, "c_min": float(np.min(c)),
                              "past_line_pts": int(np.sum(D[0] > m.track.length))})
            self.rec_t.append(t)
            self.rec_x.append(xf.copy())
            self.rec_u.append(u.copy())
        drate, fd, fb_ = float(self.u[0]), float(self.u[1]), float(self.u[2])
        if self.tc_eta is not None:
            veh = self.mpc.veh
            d_next = x[6] + np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max) * self.sim_dt
            fd, fb_ = traction_limit(veh, x, d_next, fd, fb_, self.tc_eta)
        return drate, fd, fb_


__all__ = ["LearningMPC", "LapStore", "LMPCController", "Recorder", "regressor", "TERM", "TERM7", "NZ",
           "ATVLearningMPC", "ATVLMPCController"]
