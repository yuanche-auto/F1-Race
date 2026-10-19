"""The L2 NMPC (``nmpc.FrenetMPC``, ``mode="track"``) as an acados optimal control problem (L9).

The same optimal control problem, written for acados (docs/L9_protocol.md section 4):

* **dynamics** – the same discrete step as ``FrenetMPC`` (one RK4 step per 50 ms, with the exact
  first-order steering lag when ``steer_lag`` is given), passed to acados as discrete dynamics. The
  state is augmented by the previous input (3 entries), which carries the input-rate cost;
* **cost** – the same terms as nonlinear least squares (acados' ``0.5 ||y||^2`` with unit weights,
  so every residual is ``sqrt(2 w)`` times FrenetMPC's term): tracking of n, xi, v_x against the
  reference at the predicted s (nodes 1..N; at node 0 the term is a constant), the steering rate
  and the change of the two forces; ``cost_scaling`` is 1 at every node;
* **constraints** – the same hard bounds (v_x >= 2, steering angle and rate, force signs, power,
  the longitudinal force of each axle within eta) and the same soft constraints with the same exact
  penalties: friction ellipses (slack in units of the squared usage), slip angles (slack in units
  of the peak angle) and the track limits of the front and rear axle points (one slack per side;
  FrenetMPC uses one per point – the same optimum, since only one side can be active). The
  per-stage friction factor eta_k and the track back-offs b_k are stage bounds;
* **solver** – SQP-RTI (``nlp="SQP_RTI"``: one Gauss-Newton QP per call, HPIPM with partial
  condensing) or SQP to convergence (``nlp="SQP"``, for the equivalence check). A failed QP applies
  the shifted previous plan (the L6 failure handling).

``solve(x0, u_prev)`` has FrenetMPC's signature (so that ``MPCController`` and
``LagAwareMPCController`` drive it unchanged); in RTI mode it runs feedback then preparation and
reports both times (``info["time_feedback"]``, ``info["time_preparation"]``, wall clock).
"""
from __future__ import annotations

import os
import time
from pathlib import Path

import casadi as ca
import numpy as np

from ..dynamics import DynVehicle, tyre_forces
from ..track import Track
from .nmpc import FS, _periodic_interpolant

BIG = 1e4          # "no bound" for one-sided constraints and bounds (all scaled quantities are O(1-100))


def _model_functions(track: Track, veh: DynVehicle, ref: dict, dt: float, steer_lag: float | None):
    """The same symbolic pieces as ``FrenetMPC.__init__`` (interpolants, continuous model, discrete
    step)."""
    from ..dynamics import body_accel

    L = track.length
    fn = {k: _periodic_interpolant(k, s, f, L) for k, s, f in (
        ("kap", track.s, track.kappa), ("wl", track.s, track.w_left), ("wr", track.s, track.w_right),
        ("nref", ref["s"], ref["n"]), ("xiref", ref["s"], ref["xi"]), ("vref", ref["s"], ref["vx"]))}
    kap = fn["kap"]
    x = ca.SX.sym("x", 7)
    u = ca.SX.sym("u", 3)
    s, n, xi, vx, vy, r, d = (x[i] for i in range(7))
    k = kap(s)
    F = tyre_forces(veh, vx, vy, r, d, u[1] * FS, u[2] * FS, lib=ca)
    dvx, dvy, dr = body_accel(veh, vx, vy, r, d, F, lib=ca)
    sdot = (vx * ca.cos(xi) - vy * ca.sin(xi)) / (1 - n * k)
    f = ca.Function("f", [x, u], [ca.vertcat(sdot, vx * ca.sin(xi) + vy * ca.cos(xi), r - k * sdot, dvx, dvy, dr,
                                             u[0])])
    h = dt
    if steer_lag is None:
        k1 = f(x, u); k2 = f(x + h / 2 * k1, u); k3 = f(x + h / 2 * k2, u); k4 = f(x + h * k3, u)
        step = ca.Function("step", [x, u], [x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)])
    else:
        xl = ca.SX.sym("xl", 8)
        tau, a0, c0, rho = float(steer_lag), xl[6], xl[7], u[0]

        def d_at(t):
            return c0 + rho * (t - tau) + (a0 - c0 + rho * tau) * ca.exp(-t / tau)

        def f6(z, dd):
            return f(ca.vertcat(z, dd), u)[:6]

        z0 = xl[:6]
        k1 = f6(z0, a0); k2 = f6(z0 + h / 2 * k1, d_at(h / 2))
        k3 = f6(z0 + h / 2 * k2, d_at(h / 2)); k4 = f6(z0 + h * k3, d_at(h))
        step = ca.Function("step", [xl, u], [ca.vertcat(z0 + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4), d_at(h),
                                                        c0 + rho * h)])
    return fn, step


class AcadosFrenetMPC:
    def __init__(self, track: Track, veh: DynVehicle, ref: dict, N: int = 30, dt: float = 0.05, eta=0.97,
                 margin: float = 0.3, weights: dict | None = None, backoff=None, steer_lag: float | None = None,
                 nlp: str = "SQP_RTI", name: str = "l9mpc", build_dir: str | None = None, qp_cond_N: int | None = None,
                 sqp_max_iter: int = 200, tol: float = 1e-6, qp_iter_max: int = 100, levenberg_marquardt: float = 0.0):
        from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

        self.track, self.veh, self.N, self.dt, self.steer_lag, self.nlp = track, veh, N, dt, steer_lag, nlp
        self.mode = "track"
        self.fallback_mode = "shift"
        nx = 7 if steer_lag is None else 8
        self.nxs = nx
        eta_k = np.broadcast_to(np.asarray(eta, float), (N,)).copy()
        b_k = np.zeros(N + 1) if backoff is None else np.asarray(backoff, float)
        self.eta_k, self.backoff = eta_k, b_k
        w = {"n": 1.0, "xi": 10.0, "v": 0.2, "drate": 1.0, "dF": 0.5, "slack": 1e4, "limit_slack": 1e6}
        w.update(weights or {})
        self.w = w
        h = dt
        fn, step = _model_functions(track, veh, ref, dt, steer_lag)
        kap, wl, wr, nref, xiref, vref = (fn[k] for k in ("kap", "wl", "wr", "nref", "xiref", "vref"))

        xa = ca.SX.sym("xa", nx + 3)                    # state and previous input
        u = ca.SX.sym("u", 3)
        xs, up = xa[:nx], xa[nx:]
        model = AcadosModel()
        model.name = name
        model.x, model.u = xa, u
        model.disc_dyn_expr = ca.vertcat(step(xs, u), u)

        s, n, xi, vx = xs[0], xs[1], xs[2], xs[3]
        track_res = ca.vertcat(np.sqrt(2 * h * w["n"]) * (n - nref(s)), np.sqrt(2 * h * w["xi"]) * (xi - xiref(s)),
                               np.sqrt(2 * h * w["v"]) * (vx - vref(s)))
        input_res = ca.vertcat(np.sqrt(2 * h * w["drate"]) * u[0], np.sqrt(2 * w["dF"]) * (u[1] - up[1]),
                               np.sqrt(2 * w["dF"]) * (u[2] - up[2]))
        model.cost_y_expr_0 = ca.vertcat(track_res, input_res)
        model.cost_y_expr = ca.vertcat(track_res, input_res)
        model.cost_y_expr_e = track_res

        Fi = tyre_forces(veh, xs[3], xs[4], xs[5], xs[6], u[1] * FS, u[2] * FS, lib=ca)
        uf2 = (Fi["fxf"] / (veh.mu_x * Fi["fzf"])) ** 2 + (Fi["fyf"] / (veh.mu_y * Fi["fzf"])) ** 2
        ur2 = (Fi["fxr"] / (veh.mu_x * Fi["fzr"])) ** 2 + (Fi["fyr"] / (veh.mu_y * Fi["fzr"])) ** 2
        h_lim = ca.vertcat(uf2, ur2, Fi["af"], Fi["ar"], u[1] * FS * xs[3], Fi["fxf"] / (veh.mu_x * Fi["fzf"]),
                           Fi["fxr"] / (veh.mu_x * Fi["fzr"]))
        body = veh.half_width + margin
        ki = kap(s)
        h_trk = []
        for arm in (veh.lf, -veh.lr):
            chord = 0.5 * arm**2 * ca.fabs(ki)
            na = n + arm * ca.sin(xi)
            h_trk += [na + (wr(s) - body - chord), (wl(s) - body - chord) - na]
        h_trk = ca.vertcat(*h_trk)
        model.con_h_expr_0 = h_lim
        model.con_h_expr = ca.vertcat(h_lim, h_trk)
        model.con_h_expr_e = h_trk

        ocp = AcadosOcp()
        ocp.model = model
        ocp.solver_options.N_horizon = N
        ocp.solver_options.tf = N * dt
        for c in ("cost_type_0", "cost_type", "cost_type_e"):
            setattr(ocp.cost, c, "NONLINEAR_LS")
        ocp.cost.W_0, ocp.cost.W, ocp.cost.W_e = np.eye(6), np.eye(6), np.eye(3)
        ocp.cost.yref_0, ocp.cost.yref, ocp.cost.yref_e = np.zeros(6), np.zeros(6), np.zeros(3)
        ocp.solver_options.cost_scaling = np.ones(N + 1)

        # bounds: inputs; states at nodes 1..N (v_x >= 2, steering angle / commanded angle)
        ocp.constraints.idxbu = np.arange(3)
        ocp.constraints.lbu = np.array([-veh.delta_rate_max, 0.0, -BIG])
        ocp.constraints.ubu = np.array([veh.delta_rate_max, BIG, 0.0])
        ib = [3, 6] + ([7] if nx == 8 else [])
        ocp.constraints.idxbx = np.array(ib)
        ocp.constraints.lbx = np.array([2.0] + [-veh.delta_max] * (len(ib) - 1))
        ocp.constraints.ubx = np.array([BIG] + [veh.delta_max] * (len(ib) - 1))
        ocp.constraints.idxbx_e = np.array(ib)
        ocp.constraints.lbx_e, ocp.constraints.ubx_e = ocp.constraints.lbx.copy(), ocp.constraints.ubx.copy()
        ocp.constraints.x0 = np.zeros(nx + 3)

        # nonlinear constraints (bounds per stage are set after the solver is built)
        ap = veh.alpha_peak
        lh_lim = np.array([-1.0, -1.0, -ap, -ap, -BIG, -eta_k[0], -eta_k[0]])
        uh_lim = np.array([eta_k[0] ** 2, eta_k[0] ** 2, ap, ap, veh.P_max, eta_k[0], eta_k[0]])
        ocp.constraints.lh_0, ocp.constraints.uh_0 = lh_lim, uh_lim
        ocp.constraints.lh = np.concatenate([lh_lim, np.zeros(4)])
        ocp.constraints.uh = np.concatenate([uh_lim, np.full(4, BIG)])
        ocp.constraints.lh_e, ocp.constraints.uh_e = np.zeros(4), np.full(4, BIG)

        # soft constraints: ellipses (0, 1), slip angles (2, 3), track limits; exact penalties as FrenetMPC
        wl_, ws_ = w["limit_slack"] * h, w["slack"] * h
        Z_lim = np.array([2 * wl_, 2 * wl_, 2 * wl_ / ap**2, 2 * wl_ / ap**2])
        z_lim = np.array([10 * wl_, 10 * wl_, 10 * wl_ / ap, 10 * wl_ / ap])
        Z_trk, z_trk = np.full(4, 2 * ws_), np.full(4, 10 * ws_)
        ocp.constraints.idxsh_0 = np.arange(4)
        ocp.constraints.idxsh = np.array([0, 1, 2, 3, 7, 8, 9, 10])
        ocp.constraints.idxsh_e = np.arange(4)
        ocp.cost.Zl_0 = ocp.cost.Zu_0 = Z_lim
        ocp.cost.zl_0 = ocp.cost.zu_0 = z_lim
        ocp.cost.Zl = ocp.cost.Zu = np.concatenate([Z_lim, Z_trk])
        ocp.cost.zl = ocp.cost.zu = np.concatenate([z_lim, z_trk])
        ocp.cost.Zl_e = ocp.cost.Zu_e = Z_trk
        ocp.cost.zl_e = ocp.cost.zu_e = z_trk

        so = ocp.solver_options
        so.integrator_type = "DISCRETE"
        so.hessian_approx = "GAUSS_NEWTON"
        so.nlp_solver_type = nlp
        so.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        so.qp_solver_cond_N = qp_cond_N if qp_cond_N is not None else N
        so.qp_solver_iter_max = qp_iter_max
        so.qp_solver_warm_start = 1
        so.levenberg_marquardt = levenberg_marquardt
        so.print_level = 0
        if nlp == "SQP":
            so.nlp_solver_max_iter = sqp_max_iter
            so.tol = tol
            so.globalization = "MERIT_BACKTRACKING"
        build = Path(build_dir or os.path.expanduser("~/l9_acados_build")) / name
        build.mkdir(parents=True, exist_ok=True)
        ocp.code_export_directory = str(build / "c_generated_code")
        self.solver = AcadosOcpSolver(ocp, json_file=f"{name}.json", verbose=False)
        self.nxa = nx + 3
        for i in range(N + 1):                           # per-stage eta_k and back-offs
            if i < N:
                e = eta_k[i]
                lh = np.array([-1.0, -1.0, -ap, -ap, -BIG, -e, -e])
                uh = np.array([e**2, e**2, ap, ap, veh.P_max, e, e])
            if i == 0:
                self.solver.constraints_set(0, "lh", lh)
                self.solver.constraints_set(0, "uh", uh)
            elif i < N:
                self.solver.constraints_set(i, "lh", np.concatenate([lh, np.full(4, b_k[i])]))
                self.solver.constraints_set(i, "uh", np.concatenate([uh, np.full(4, BIG)]))
            else:
                self.solver.constraints_set(N, "lh", np.full(4, b_k[N]))
                self.solver.constraints_set(N, "uh", np.full(4, BIG))
        self.initialised = False
        self.prepared = False
        self.stats: list[dict] = []

    # ------------------------------------------------------------------ helpers
    def _set_x0(self, x0: np.ndarray, u_prev: np.ndarray) -> np.ndarray:
        xa0 = np.concatenate([x0, [u_prev[0], u_prev[1] / FS, u_prev[2] / FS]])
        self.solver.set(0, "lbx", xa0)
        self.solver.set(0, "ubx", xa0)
        return xa0

    def _initial_guess(self, xa0: np.ndarray):
        """As FrenetMPC.initial_guess: the state rolled forward at constant speed, the previous
        forces held."""
        xs = xa0.copy()
        for i in range(self.N + 1):
            self.solver.set(i, "x", xs)
            if i < self.N:
                self.solver.set(i, "u", np.array([0.0, xa0[self.nxs + 1], xa0[self.nxs + 2]]))
            xs = xs.copy()
            xs[0] += self.dt * xs[3]

    def _plan(self):
        return ([np.array(self.solver.get(i, "x")) for i in range(self.N + 1)],
                [np.array(self.solver.get(i, "u")) for i in range(self.N)])

    def _set_plan(self, X, U):
        for i in range(self.N + 1):
            self.solver.set(i, "x", X[i])
        for i in range(self.N):
            self.solver.set(i, "u", U[i])

    @staticmethod
    def _shifted(X, U, k: int = 1):
        """The plan shifted by k >= 0 steps, the last node / input repeated."""
        k = max(int(k), 0)
        return X[k:] + [X[-1]] * min(k, len(X)), U[k:] + [U[-1]] * min(k, len(U))

    def _slacks(self) -> tuple[float, float]:
        """(track-limit slack [m], physics-limit slack in FrenetMPC's units) of the current iterate."""
        ap = self.veh.alpha_peak
        trk, lim = [0.0], [0.0]
        for i in range(self.N + 1):
            sl, su = np.asarray(self.solver.get(i, "sl")), np.asarray(self.solver.get(i, "su"))
            if i < self.N:                               # ellipses (upper), slip angles (both sides)
                lim += [float(np.max(su[:2])), float(max(np.max(sl[2:4]), np.max(su[2:4]))) / ap]
            if i > 0:
                j = 4 if i < self.N else 0
                trk.append(float(np.max(sl[j:j + 4])))
        return max(trk), max(lim)

    # ------------------------------------------------------------------ interface
    def presolve(self, x0: np.ndarray, u_prev: np.ndarray, iters: int = 30) -> None:
        """L9 A1.4: a plan before the lap starts (untimed): RTI iterations at the initial state
        until converged (SQP: one solve); the iterate stays aligned with the start time and
        prepared, so that the next ``solve(..., shift=0)`` continues from it."""
        xa0 = self._set_x0(np.asarray(x0, float), np.asarray(u_prev, float))
        if not self.initialised:
            self._initial_guess(xa0)
            self.initialised = True
        if self.nlp == "SQP_RTI":
            self.solver.options_set("rti_phase", 0)
            for _ in range(iters):
                self.solver.solve()
        else:
            self.solver.solve()
        X, U = self._plan()
        self._last_plan = (X, U)
        if self.nlp == "SQP_RTI":
            self.solver.options_set("rti_phase", 1)
            self.solver.solve()

    def solve(self, x0: np.ndarray, u_prev: np.ndarray, shift: int = 1) -> tuple[np.ndarray, dict]:
        """FrenetMPC's interface. RTI: feedback with the new initial state, then shift and
        preparation for the next call (both timed); SQP: solve to convergence from the shifted
        previous solution. ``shift`` (L9 A1.3): the warm start is the previous solution shifted by
        this many 50 ms steps (the steps elapsed since the previous solve started); the default
        call shifts by one, which the preparation after each call has already done – any other
        value re-aligns and re-prepares first, inside the timed feedback."""
        xa0 = self._set_x0(np.asarray(x0, float), np.asarray(u_prev, float))
        if not self.initialised:
            self._initial_guess(xa0)
            self.initialised = True
            if self.nlp == "SQP_RTI":                    # the first call prepares too (not timed as feedback)
                self.solver.options_set("rti_phase", 1)
                self.solver.solve()
        t0 = time.perf_counter()
        last = getattr(self, "_last_plan", None)
        if shift != 1 and last is not None:
            self._set_plan(*self._shifted(*last, k=shift))
            if self.nlp == "SQP_RTI":
                self.solver.options_set("rti_phase", 1)
                self.solver.solve()
        if self.nlp == "SQP_RTI":
            self.solver.options_set("rti_phase", 2)
        status = self.solver.solve()
        u0 = self.solver.get(0, "u")
        t_fb = time.perf_counter() - t0
        fallback = (status != 0) and last is not None
        if fallback:                                     # L6 handling: the shifted previous plan
            X, U = self._shifted(*last, k=shift)
            u0 = U[0]
            slack, lim_slack = float("nan"), float("nan")
        else:
            slack, lim_slack = self._slacks()
            X, U = self._plan()
        self._last_plan = (X, U)
        self._set_plan(*self._shifted(X, U))            # warm start of the next call (one step)
        t_prep = 0.0
        if self.nlp == "SQP_RTI":
            t1 = time.perf_counter()
            self.solver.options_set("rti_phase", 1)
            self.solver.solve()
            t_prep = time.perf_counter() - t1
        u = np.asarray(u0, float) * np.array([1.0, FS, FS])
        info = {"status": int(status), "time": t_fb, "time_feedback": t_fb, "time_preparation": t_prep,
                "slack": slack, "limit_slack": lim_slack, "fallback": bool(fallback),
                "iter": int(self.solver.get_stats("sqp_iter"))}
        self.stats.append(info)
        return u, info

    def prediction(self) -> tuple[np.ndarray, np.ndarray]:
        X, U = self._last_plan
        return np.array(X)[:, :self.nxs], np.array(U) * np.array([1.0, FS, FS])


__all__ = ["AcadosFrenetMPC"]
