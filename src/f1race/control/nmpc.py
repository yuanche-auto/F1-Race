"""Nonlinear MPC for the dynamic single-track model in curvilinear coordinates (L2).

State   x = [s, n, xi, vx, vy, r, delta]    (s: centre-line arclength of the CG, unwrapped)
Input   u = [delta_rate, F_drive, F_brake]  (forces scaled by 1e4 inside the NLP)

    s'  = (vx cos xi - vy sin xi) / (1 - n kappa(s))
    n'  = vx sin xi + vy cos xi
    xi' = r - kappa(s) s'
    (vx, vy, r)' from dynamics.body_accel,  delta' = delta_rate

Two objectives on the same constraints:

* ``mode="track"``: follow a reference (n_ref, xi_ref, v_ref as functions of s),
* ``mode="mpcc"``: maximise the progress s_N - s_0 (no reference line); the
  terminal speed is bounded by a reference speed profile so that the short
  horizon cannot run into a corner it can no longer brake for.
* ``mode="filter"`` (predictive safety filter, prepared for L7): stay as close as
  possible to a proposed first input ``u_des`` (another controller's command, a
  parameter of each solve) subject to the same constraints, with the terminal speed
  bounded by the reference speed as in MPCC; the later inputs only carry the small
  smoothness cost.

Constraints: friction ellipse of both axles (factor ``eta``), slip angles within
the tyre peak, power, steering angle and rate, and track limits for the front
and rear axle points. Track limits, ellipses and slip angles are softened with
heavily (exactly) penalised slacks so that the NLP stays feasible after a
disturbance: the solution is unchanged while the constraints can be met, and
the slip angles at stage 0, fixed by the measured state, can no longer make the
problem infeasible (``soft_limits=False`` restores the hard version, kept as an
ablation). Multiple shooting with one RK4 step per MPC step, IPOPT with warm
start; if a solve fails, the shifted previous plan is applied instead. Options added
before L7 (``retry``, ``fallback``): a failed solve is retried twice with a second
IPOPT instance (no warm-started multipliers, default barrier start) – from the shifted
plan and from a rollout of its inputs – and the fallback can brake within the friction
left instead of following the shifted plan's longitudinal forces.

``steer_lag=tau`` (L6) gives the model a first-order steering actuator: the state
gets the commanded angle as an eighth entry, the input moves it, and the angle
in the tyre model follows with delta' = (delta_cmd - delta) / tau. The lag is
integrated exactly within each MPC step (the rate is constant there), the rest
of the state by RK4 with the exact steering angle at the RK4 nodes.

``residual`` (L6, round 2): a CasADi function z -> 3 added to (v_x', v_y', r') of
the model, z = (v_x, v_y, r, delta, delta_rate, F_drive, F_brake) – learned residual
dynamics. ``param_tightening``: the per-stage friction factors and track back-offs
become parameters of the NLP, set at every solve (``solve(..., eta=, backoff=)``).
``param_model`` (control expansion phase D): the seven factors of ``identify.PARAMS``
(friction, mass, downforce and drag areas, yaw inertia, tyre stiffness multiplicative,
the front share of the downforce additive) become parameters of the NLP
(``solve(..., theta=)``); the slip-angle limit tan(pi/2C)/B then moves from the bounds
into the constraint function. A list of references (a plan library) instead of one
gives the reference a plan coordinate j, a parameter of the NLP (``solve(..., plan=)``):
the reference at s is the linear interpolation between plans floor(j) and ceil(j) on the
s grid of plan ``ref_grid``. The formulation without these options is unchanged.
"""
from __future__ import annotations

import time
from dataclasses import replace

import casadi as ca
import numpy as np

from ..dynamics import DynVehicle, body_accel, tyre_forces
from ..track import Track
from .dyn_tracking import traction_limit

FS = 1e4
BRAKE_REQUEST = 1e5        # [N] fallback "brake": more than any axle can take; the
                           # controller's traction control limits it to the friction left
THETA_NOMINAL = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0])   # identify.X0: the nominal car's factors


def param_vehicle(veh: DynVehicle, th) -> DynVehicle:
    """``identify.vehicle_from`` for symbolic (or numeric) factors th = (mu, m, ClA, CdA, Iz, B, aero_front)."""
    return replace(veh, mu_x=veh.mu_x * th[0], mu_y=veh.mu_y * th[0], m=veh.m * th[1], ClA=veh.ClA * th[2],
                   CdA=veh.CdA * th[3], Iz=veh.Iz * th[4], B=veh.B * th[5], aero_front=veh.aero_front + th[6])


def _periodic_interpolant(name: str, s: np.ndarray, f: np.ndarray, length: float, laps: int = 3):
    """Linear interpolant of a periodic field over [-length, laps*length]."""
    grid = np.concatenate([s + k * length for k in range(-1, laps)])
    vals = np.tile(f, laps + 1)
    return ca.interpolant(name, "linear", [grid], vals)


def _periodic_library_interpolant(name: str, s: np.ndarray, F: np.ndarray, length: float, laps: int = 3):
    """Linear interpolant of periodic fields F (len(s), n_plans) in (s, j) over [-length, laps*length] x [0, n - 1]."""
    grid = np.concatenate([s + k * length for k in range(-1, laps)])
    vals = np.tile(F, (laps + 1, 1))
    return ca.interpolant(name, "linear", [grid, np.arange(F.shape[1], dtype=float)], vals.ravel(order="F"))


class FrenetMPC:
    def __init__(self, track: Track, veh: DynVehicle, ref: dict, mode: str = "track",
                 N: int = 30, dt: float = 0.05, eta=0.97, margin: float = 0.3,
                 weights: dict | None = None, max_iter: int = 500, soft_limits: bool | int = True,
                 backoff=None, steer_lag: float | None = None, residual=None,
                 param_tightening: bool = False, retry: bool = False, fallback: str = "shift",
                 param_model: bool = False, ref_grid: int = 0):
        """``ref``: arrays over centre-line arclength ref['s'] with keys n, xi, vx (the
        reference trajectory, e.g. an L1 plan; in MPCC mode only vx is used, as a
        terminal speed bound), or a list of such references (a plan library with the plan
        coordinate as a parameter; all on the s grid of ``ref[ref_grid]``, the default
        coordinate). ``param_model``: the model's seven factors as parameters (module doc).
        ``eta``: friction factor, a number or one value per
        stage (N). ``soft_limits``: friction ellipses and slip angles as exactly
        penalised soft constraints at every stage (True), at the first k stages only (an
        integer k), or hard everywhere (False, which makes the NLP infeasible whenever
        the measured state is already past a limit: the slip angles at stage 0 depend on
        the measured state only). ``backoff``: optional extra distance [m] kept from the
        track edges per predicted state (N + 1 values, the first unused), e.g.
        stochastic constraint tightening (L3). ``steer_lag``: steering actuator time
        constant [s] of the model (L6); the state is then 8-dim (see module doc).
        ``retry`` and ``fallback``: what happens when a solve fails (see ``solve``; the
        defaults keep the behaviour of L2–L6)."""
        eta_k = np.broadcast_to(np.asarray(eta, float), (N,)).copy()
        b_k = np.zeros(N + 1) if backoff is None else np.asarray(backoff, float)
        self.track, self.veh, self.mode, self.N, self.dt = track, veh, mode, N, dt
        self.steer_lag = steer_lag
        self.param_tightening = bool(param_tightening)
        nx = 7 if steer_lag is None else 8
        self.nxs = nx
        self.eta, self.eta_k, self.backoff = eta, eta_k, b_k
        w = {"n": 1.0, "xi": 10.0, "v": 0.2, "drate": 1.0, "dF": 0.5, "slack": 1e4, "progress": 1.0,
             "limit_slack": 1e6,           # physics limits cost far more than track limits
             "filter_drate": 100.0, "filter_F": 100.0}   # filter: per (rad/s)^2 and (1e4 N)^2
        w.update(weights or {})
        self.w = w
        L = track.length
        kap = _periodic_interpolant("kap", track.s, track.kappa, L)
        wl = _periodic_interpolant("wl", track.s, track.w_left, L)
        wr = _periodic_interpolant("wr", track.s, track.w_right, L)
        self.param_model = bool(param_model)
        self.library = isinstance(ref, (list, tuple))
        if self.param_tightening and self.param_model:
            raise ValueError("param_model with param_tightening is not implemented")
        if self.library:
            sg = np.asarray(ref[ref_grid]["s"], float)
            lib = {key: np.column_stack([np.interp(sg, np.asarray(o["s"], float), np.asarray(o[key], float), period=L)
                                         for o in ref]) for key in ("n", "xi", "vx")}
            for key in lib:                                 # the grid plan itself, not a re-interpolation of it
                lib[key][:, ref_grid] = np.asarray(ref[ref_grid][key], float)
            self.n_plans, self.plan_default = len(ref), float(ref_grid)
            nref = _periodic_library_interpolant("nref", sg, lib["n"], L)
            xiref = _periodic_library_interpolant("xiref", sg, lib["xi"], L)
            vref = _periodic_library_interpolant("vref", sg, lib["vx"], L)
        else:
            nref = _periodic_interpolant("nref", ref["s"], ref["n"], L)
            xiref = _periodic_interpolant("xiref", ref["s"], ref["xi"], L)
            vref = _periodic_interpolant("vref", ref["s"], ref["vx"], L)

        x = ca.SX.sym("x", 7)
        u = ca.SX.sym("u", 3)
        th = ca.SX.sym("th", 7)
        fargs = [th] if self.param_model else []        # the model's factors (param_model)
        vm = param_vehicle(veh, th) if self.param_model else veh
        s, n, xi, vx, vy, r, d = (x[i] for i in range(7))
        k = kap(s)
        F = tyre_forces(vm, vx, vy, r, d, u[1] * FS, u[2] * FS, lib=ca)
        dvx, dvy, dr = body_accel(vm, vx, vy, r, d, F, lib=ca)
        if residual is not None:
            rz = residual(ca.vertcat(vx, vy, r, d, u[0], u[1] * FS, u[2] * FS))
            dvx, dvy, dr = dvx + rz[0], dvy + rz[1], dr + rz[2]
        sdot = (vx * ca.cos(xi) - vy * ca.sin(xi)) / (1 - n * k)
        f = ca.Function("f", [x, u, *fargs], [ca.vertcat(sdot, vx * ca.sin(xi) + vy * ca.cos(xi),
                                                         r - k * sdot, dvx, dvy, dr, u[0])])
        h = dt
        if steer_lag is None:
            k1 = f(x, u, *fargs); k2 = f(x + h / 2 * k1, u, *fargs)
            k3 = f(x + h / 2 * k2, u, *fargs); k4 = f(x + h * k3, u, *fargs)
            step = ca.Function("step", [x, u, *fargs], [x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)])
        else:
            xl = ca.SX.sym("xl", 8)
            tau, a0, c0, rho = float(steer_lag), xl[6], xl[7], u[0]

            def d_at(t):                                # exact lag response to a ramp command
                return c0 + rho * (t - tau) + (a0 - c0 + rho * tau) * ca.exp(-t / tau)

            def f6(z, dd):                              # first six rates at steering angle dd
                return f(ca.vertcat(z, dd), u, *fargs)[:6]

            z0 = xl[:6]
            k1 = f6(z0, a0); k2 = f6(z0 + h / 2 * k1, d_at(h / 2))
            k3 = f6(z0 + h / 2 * k2, d_at(h / 2)); k4 = f6(z0 + h * k3, d_at(h))
            step = ca.Function("step", [xl, u, *fargs], [ca.vertcat(z0 + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4),
                                                                    d_at(h), c0 + rho * h)])

        X = ca.SX.sym("X", nx, N + 1)
        U = ca.SX.sym("U", 3, N)
        S = ca.SX.sym("S", 2, N + 1)                  # track-limit slacks (front, rear points)
        n_soft = N if soft_limits is True else int(soft_limits)
        T = ca.SX.sym("T", 4, n_soft)                   # ellipse (front, rear) and slip slacks
        pm, i_th = self.param_model, nx + 3 + (2 * N + 1 if self.param_tightening else 0)
        i_j = i_th + (7 if pm else 0)
        npar = i_j + (1 if self.library else 0) + (3 if mode == "filter" else 0)
        P = ca.SX.sym("P", npar)                      # initial state, previous input (, eta_k, b_k)(, theta)(, j)
        pt = self.param_tightening
        Pargs = [P[i_th:i_th + 7]] if pm else []
        vP = param_vehicle(veh, P[i_th:i_th + 7]) if pm else veh
        if self.library:
            Pj = P[i_j]
            nref_, xiref_, vref_ = nref, xiref, vref
            nref, xiref, vref = (lambda q: nref_(ca.vertcat(q, Pj))), (lambda q: xiref_(ca.vertcat(q, Pj))),                 (lambda q: vref_(ca.vertcat(q, Pj)))
        g, lbg, ubg = [], [], []
        J = 0
        g.append(X[:, 0] - P[:nx]); lbg += [0] * nx; ubg += [0] * nx
        body = veh.half_width + margin
        for i in range(N):
            g.append(X[:, i + 1] - step(X[:, i], U[:, i], *Pargs)); lbg += [0] * nx; ubg += [0] * nx
            Fi = tyre_forces(vP, X[3, i], X[4, i], X[5, i], X[6, i], U[1, i] * FS, U[2, i] * FS, lib=ca)
            uf2 = (Fi["fxf"] / (vP.mu_x * Fi["fzf"])) ** 2 + (Fi["fyf"] / (vP.mu_y * Fi["fzf"])) ** 2
            ur2 = (Fi["fxr"] / (vP.mu_x * Fi["fzr"])) ** 2 + (Fi["fyr"] / (vP.mu_y * Fi["fzr"])) ** 2
            if i < n_soft:
                ek = eta_k[i]
                # the longitudinal force of each axle stays hard-limited (always satisfiable):
                # otherwise the optimiser buys impossible braking with ellipse slack
                if pm:                                  # slip limit with the parameter B: in the constraint function
                    ap = np.tan(np.pi / (2.0 * veh.C)) / vP.B
                    slip = [Fi["af"] - ap * (T[2, i] + 1.0), Fi["af"] + ap * (T[2, i] + 1.0),
                            Fi["ar"] - ap * (T[3, i] + 1.0), Fi["ar"] + ap * (T[3, i] + 1.0)]
                    sl_lb, sl_ub = [-ca.inf, 0.0, -ca.inf, 0.0], [0.0, ca.inf, 0.0, ca.inf]
                else:
                    ap = veh.alpha_peak                 # slip slacks in units of the peak angle
                    slip = [Fi["af"] - ap * T[2, i], Fi["af"] + ap * T[2, i],
                            Fi["ar"] - ap * T[3, i], Fi["ar"] + ap * T[3, i]]
                    sl_lb, sl_ub = [-ca.inf, -ap, -ca.inf, -ap], [ap, ca.inf, ap, ca.inf]
                g += [uf2 - T[0, i], ur2 - T[1, i], *slip, U[1, i] * FS * X[3, i],
                      Fi["fxf"] / (vP.mu_x * Fi["fzf"]), Fi["fxr"] / (vP.mu_x * Fi["fzr"])]
                if pt:                                  # the same rows with eta_k a parameter
                    ek = P[nx + 3 + i]
                    g[-9:] = [g[-9] - ek**2, g[-8] - ek**2] + g[-7:-2] + [g[-2] - ek, g[-2] + ek,
                                                                         g[-1] - ek, g[-1] + ek]
                    lbg += [-ca.inf, -ca.inf, *sl_lb, -ca.inf, -ca.inf, 0.0, -ca.inf, 0.0]
                    ubg += [0.0, 0.0, *sl_ub, veh.P_max, 0.0, ca.inf, 0.0, ca.inf]
                else:
                    lbg += [-ca.inf, -ca.inf, *sl_lb, -ca.inf, -ek, -ek]
                    ubg += [ek**2, ek**2, *sl_ub, veh.P_max, ek, ek]
                J += w["limit_slack"] * h * (ca.sumsqr(T[:, i]) + 10.0 * ca.sum1(T[:, i]))
            elif pm:                                    # hard limits, the slip angles in units of the peak angle
                ap = np.tan(np.pi / (2.0 * veh.C)) / vP.B
                g += [uf2, ur2, Fi["af"] / ap, Fi["ar"] / ap, U[1, i] * FS * X[3, i]]
                lbg += [-ca.inf, -ca.inf, -1.0, -1.0, -ca.inf]
                ubg += [eta_k[i]**2, eta_k[i]**2, 1.0, 1.0, veh.P_max]
            else:
                if pt:
                    ek = P[nx + 3 + i]
                    g += [uf2 - ek**2, ur2 - ek**2, Fi["af"], Fi["ar"], U[1, i] * FS * X[3, i]]
                    lbg += [-ca.inf, -ca.inf, -veh.alpha_peak, -veh.alpha_peak, -ca.inf]
                    ubg += [0.0, 0.0, veh.alpha_peak, veh.alpha_peak, veh.P_max]
                else:
                    g += [uf2, ur2, Fi["af"], Fi["ar"], U[1, i] * FS * X[3, i]]
                    lbg += [-ca.inf, -ca.inf, -veh.alpha_peak, -veh.alpha_peak, -ca.inf]
                    ubg += [eta_k[i]**2, eta_k[i]**2, veh.alpha_peak, veh.alpha_peak, veh.P_max]
            du = U[:, i] - (P[nx:nx + 3] if i == 0 else U[:, i - 1])
            if mode == "filter" and i == 0:             # deviation from the proposed input
                ud = P[npar - 3:npar]
                J += w["filter_drate"] * (U[0, 0] - ud[0]) ** 2 + w["filter_F"] * ((U[1, 0] - ud[1]) ** 2
                                                                                   + (U[2, 0] - ud[2]) ** 2)
            else:
                J += h * (w["drate"] * U[0, i] ** 2 + w["dF"] * (du[1] ** 2 + du[2] ** 2) / h)
        for i in range(1, N + 1):
            si, ni, xii = X[0, i], X[1, i], X[2, i]
            ki = kap(si)
            for j, arm in enumerate((veh.lf, -veh.lr)):
                chord = 0.5 * arm**2 * ca.fabs(ki)
                na = ni + arm * ca.sin(xii)
                if pt:
                    bk = P[nx + 3 + N + i]
                    g += [na + (wr(si) - body - chord) + S[j, i] - bk, (wl(si) - body - chord) - na + S[j, i] - bk]
                    lbg += [0.0, 0.0]; ubg += [ca.inf, ca.inf]
                else:
                    g += [na + (wr(si) - body - chord) + S[j, i], (wl(si) - body - chord) - na + S[j, i]]
                    lbg += [b_k[i], b_k[i]]; ubg += [ca.inf, ca.inf]
            J += w["slack"] * h * ca.sumsqr(S[:, i]) + 10 * w["slack"] * h * ca.sum1(S[:, i])
            if mode == "track":
                J += h * (w["n"] * (ni - nref(si)) ** 2 + w["xi"] * (xii - xiref(si)) ** 2
                          + w["v"] * (X[3, i] - vref(si)) ** 2)
        if mode == "mpcc":
            J += -w["progress"] * (X[0, N] - X[0, 0])
        if mode in ("mpcc", "filter"):
            g.append(X[3, N] - vref(X[0, N])); lbg.append(-ca.inf); ubg.append(0.0)

        z = ca.vertcat(ca.vec(X), ca.vec(U), ca.vec(S), ca.vec(T))
        self.nx = nx * (N + 1)
        self.nu = 3 * N
        lbz = np.full(z.shape[0], -np.inf)
        ubz = np.full(z.shape[0], np.inf)
        Xi = np.arange(self.nx).reshape(N + 1, nx)         # vec() is column-major: X[:, i] contiguous
        Ui = self.nx + np.arange(self.nu).reshape(N, 3)
        Si = self.nx + self.nu + np.arange(2 * (N + 1)).reshape(N + 1, 2)
        lbz[Xi[:, 3]] = 2.0
        lbz[Xi[:, 6]], ubz[Xi[:, 6]] = -veh.delta_max, veh.delta_max
        if nx == 8:                                     # the commanded angle is the limited one
            lbz[Xi[:, 7]], ubz[Xi[:, 7]] = -veh.delta_max, veh.delta_max
        lbz[Ui[:, 0]], ubz[Ui[:, 0]] = -veh.delta_rate_max, veh.delta_rate_max
        lbz[Ui[:, 1]], ubz[Ui[:, 1]] = 0.0, np.inf
        lbz[Ui[:, 2]], ubz[Ui[:, 2]] = -np.inf, 0.0
        Ti = self.nx + self.nu + 2 * (N + 1) + np.arange(T.numel()).reshape(n_soft, 4)
        lbz[Si] = 0.0
        ubz[Si[0]] = 0.0                                # slack at the (fixed) initial state unused
        lbz[Ti] = 0.0
        self.Xi, self.Ui, self.Si, self.Ti = Xi, Ui, Si, Ti
        self.lbz, self.ubz = lbz, ubz
        self.lbg, self.ubg = np.array(lbg, float), np.array(ubg, float)
        nlp = {"x": z, "f": J, "g": ca.vertcat(*g), "p": P}
        self.solver = ca.nlpsol("mpc", "ipopt", nlp, {
            "print_time": False,
            "ipopt": {"print_level": 0, "max_iter": max_iter, "tol": 1e-4, "acceptable_tol": 1e-3,
                      "acceptable_iter": 5, "warm_start_init_point": "yes", "mu_init": 1e-3,
                      "linear_solver": "mumps"}})
        if fallback not in ("shift", "brake"):
            raise ValueError(fallback)
        self.retry, self.fallback_mode = bool(retry), fallback
        self._nlp, self._step, self._max_iter = nlp, step, max_iter
        self._solver2 = None
        self.z = None
        self._theta = THETA_NOMINAL.copy()
        self.lam_g = None
        self.lam_x = None
        self.stats: list[dict] = []

    def initial_guess(self, x0: np.ndarray, u0: np.ndarray) -> np.ndarray:
        """Roll the nominal model forward with a constant input for the first solve."""
        z = np.zeros(self.lbz.shape)
        xs = np.array(x0, float)
        for i in range(self.N + 1):
            z[self.Xi[i]] = xs
            xs = xs.copy()
            xs[0] += self.dt * xs[3]
        for i in range(self.N):
            z[self.Ui[i]] = [0.0, u0[1] / FS, u0[2] / FS]
        return z

    def solve(self, x0: np.ndarray, u_prev: np.ndarray, eta=None, backoff=None,
              u_des=None, theta=None, plan=None) -> tuple[np.ndarray, dict]:
        """``eta`` (N) and ``backoff`` (N + 1): per-solve tightening (``param_tightening``
        only; default: the values given at construction). ``u_des``: the proposed input
        (``mode="filter"`` only; rate in rad/s, forces in N). ``theta``: the model's seven
        factors (``param_model`` only; default: the nominal car). ``plan``: the plan
        coordinate (a library reference only; default: ``ref_grid``).

        A failed solve (no ``Solve_Succeeded`` / ``Solved_To_Acceptable_Level``) applies
        the shifted previous plan. With ``retry`` it is first retried with a second IPOPT
        instance (``solver_cold``) from the shifted plan and then from a rollout of its
        inputs (``info["status"]`` is then the status of the last attempt, ``info["attempts"]``
        lists all of them); with ``fallback="brake"`` the fallback keeps the shifted plan's steering
        rate and brakes with traction control instead of its longitudinal forces (the brake
        request ``BRAKE_REQUEST`` is only safe behind traction control: ``MPCController``
        refuses ``fallback="brake"`` with ``tc_eta=None``)."""
        p = np.concatenate([x0, [u_prev[0], u_prev[1] / FS, u_prev[2] / FS]])
        if self.param_tightening:
            e = self.eta_k if eta is None else np.asarray(eta, float)
            b = self.backoff if backoff is None else np.asarray(backoff, float)
            p = np.concatenate([p, e, b])
        elif eta is not None or backoff is not None:
            raise ValueError("per-solve tightening needs param_tightening=True")
        if self.param_model:
            self._theta = THETA_NOMINAL.copy() if theta is None else np.asarray(theta, float)
            p = np.concatenate([p, self._theta])
        elif theta is not None:
            raise ValueError("theta needs param_model=True")
        if self.library:
            p = np.concatenate([p, [self.plan_default if plan is None else float(plan)]])
        elif plan is not None:
            raise ValueError("plan needs a reference library")
        if self.mode == "filter":
            if u_des is None:
                raise ValueError("the filter needs u_des")
            p = np.concatenate([p, [u_des[0], u_des[1] / FS, u_des[2] / FS]])
        elif u_des is not None:
            raise ValueError("u_des only in mode='filter'")
        if self.z is None:
            z0 = self.initial_guess(x0, u_prev)
            args = {}
        else:                                           # shift the previous solution
            z0 = self.z.copy()
            X = self.z[self.Xi]
            U = self.z[self.Ui]
            z0[self.Xi[:-1]] = X[1:]
            z0[self.Ui[:-1]] = U[1:]
            z0[self.Xi[0]] = x0
            args = {"lam_g0": self.lam_g, "lam_x0": self.lam_x}
        t0 = time.time()
        sol = self.solver(x0=z0, p=p, lbx=self.lbz, ubx=self.ubz, lbg=self.lbg, ubg=self.ubg, **args)
        st = self.solver.stats()
        ok = st["return_status"] in ("Solve_Succeeded", "Solved_To_Acceptable_Level")
        attempts = [st["return_status"]]
        iters = st["iter_count"]
        if not ok and self.z is not None and self.retry:
            for name, guess in (("fresh barrier", lambda: z0), ("rollout", lambda: self.rollout(z0, x0))):
                sol = self.solver_cold()(x0=guess(), p=p, lbx=self.lbz, ubx=self.ubz, lbg=self.lbg, ubg=self.ubg)
                st = self.solver_cold().stats()
                attempts.append(f"{name}: {st['return_status']}")
                iters += st["iter_count"]
                ok = st["return_status"] in ("Solve_Succeeded", "Solved_To_Acceptable_Level")
                if ok:
                    break
        fallback = False
        if ok or self.z is None:
            z = np.asarray(sol["x"]).ravel()
            self.z, self.lam_g, self.lam_x = z, np.asarray(sol["lam_g"]).ravel(), np.asarray(sol["lam_x"]).ravel()
        else:                                           # keep the shifted previous plan
            z, fallback = z0, True
            self.z = z0
        u = z[self.Ui[0]] * np.array([1.0, FS, FS])
        if fallback and self.fallback_mode == "brake":  # brake within what the tyres have left
            u = np.array([u[0], 0.0, -BRAKE_REQUEST])
        info = {"status": st["return_status"], "iter": iters, "time": time.time() - t0,
                "slack": float(np.max(z[self.Si])), "fallback": fallback,
                "limit_slack": float(np.max(z[self.Ti])) if self.Ti.size else 0.0}
        if self.retry:
            info["attempts"] = attempts
        self.stats.append(info)
        return u, info

    def solver_cold(self):
        """A second IPOPT instance on the same NLP for retries: no warm-started
        multipliers, IPOPT's default initial barrier parameter, twice the iterations."""
        if self._solver2 is None:
            self._solver2 = ca.nlpsol("mpc_retry", "ipopt", self._nlp, {
                "print_time": False,
                "ipopt": {"print_level": 0, "max_iter": 2 * self._max_iter, "tol": 1e-4,
                          "acceptable_tol": 1e-3, "acceptable_iter": 5, "linear_solver": "mumps"}})
        return self._solver2

    def rollout(self, z0: np.ndarray, x0: np.ndarray) -> np.ndarray:
        """``z0`` with its states replaced by the model's response to its inputs from x0
        (the dynamics constraints then hold exactly at the initial guess)."""
        z = z0.copy()
        xs = np.asarray(x0, float)
        z[self.Xi[0]] = xs
        for i in range(self.N):
            xs = np.asarray(self._step(xs, z[self.Ui[i]], *([self._theta] if self.param_model else []))).ravel()
            z[self.Xi[i + 1]] = xs
        return z

    def prediction(self) -> tuple[np.ndarray, np.ndarray]:
        """Predicted states (N + 1, 7 or 8) and inputs (N, 3, forces in N) of the last solve."""
        return self.z[self.Xi].copy(), self.z[self.Ui] * np.array([1.0, FS, FS])


class MPCController:
    """Plant interface: converts the simulator state to curvilinear coordinates,
    re-solves the MPC every ``mpc.dt`` and holds the input in between."""

    def __init__(self, mpc: FrenetMPC, sim_dt: float, tc_eta: float | None = 0.99,
                 record: bool = False):
        """``tc_eta``: optional low-level traction control / ABS applied at every
        simulator step (the MPC input is held for mpc.dt while the state moves).
        ``record``: keep every prediction (for calibrating constraint tightening, L3)."""
        if getattr(mpc, "fallback_mode", "shift") == "brake" and tc_eta is None:
            raise ValueError('fallback="brake" needs the traction control (tc_eta)')
        self.mpc, self.sim_dt, self.tc_eta = mpc, sim_dt, tc_eta
        self.record = record
        self.predictions: list[tuple[float, np.ndarray, np.ndarray]] = []
        self.track = mpc.track
        self.u = np.zeros(3)
        self.t_next = 0.0
        self.s_unwrapped = None
        self.log: list[tuple] = []

    def frenet(self, x: np.ndarray) -> np.ndarray:
        tr = self.track
        px, py, psi, vx, vy, r, d = x
        s, n = tr.project(np.array([px]), np.array([py]))
        s, n = float(s[0]), float(n[0])
        if self.s_unwrapped is None:
            su = s
        else:
            prev = self.s_unwrapped
            su = prev + ((s - prev % tr.length + 0.5 * tr.length) % tr.length - 0.5 * tr.length)
        self.s_unwrapped = su
        xi = float(np.angle(np.exp(1j * (psi - tr.heading_at(s)))))
        return np.array([su, n, xi, vx, vy, r, d])

    def __call__(self, t: float, x: np.ndarray):
        xf = self.frenet(x)
        if t + 1e-9 >= self.t_next:
            u, info = self.mpc.solve(xf, self.u)
            self.u = u
            self.t_next = t + self.mpc.dt
            self.log.append((t, xf[0], xf[1], info["time"], info["status"], info["slack"]))
            if self.record:
                self.predictions.append((t, *self.mpc.prediction()))
        drate, fd, fb = float(self.u[0]), float(self.u[1]), float(self.u[2])
        if self.tc_eta is not None:
            veh = self.mpc.veh
            d_next = x[6] + np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max) * self.sim_dt
            fd, fb = traction_limit(veh, x, d_next, fd, fb, self.tc_eta)
        return drate, fd, fb


class LagAwareMPCController(MPCController):
    """Controller for an MPC with ``steer_lag`` (the correct structure, L6).

    ``observe="command"``: it receives the 7-dim state with its own commanded angle in
    the steering slot and reconstructs the road-wheel angle with an open-loop observer
    of the lag (exact in the noise-free case: the angle is a deterministic function of
    the commands). ``observe="angle"``: it receives the measured road-wheel angle and
    keeps its commanded angle by integrating its own rate commands with the actuator's
    rate and angle limits (as ``LaggedSim.apply_limits``)."""

    def __init__(self, mpc: FrenetMPC, sim_dt: float, tc_eta: float | None = 0.99,
                 record: bool = False, observe: str = "command"):
        if mpc.steer_lag is None:
            raise ValueError("the MPC has no steering lag")
        if observe not in ("command", "angle"):
            raise ValueError(observe)
        super().__init__(mpc, sim_dt, tc_eta, record)
        self.tau = float(mpc.steer_lag)
        self.observe_mode = observe
        self.d_hat = None
        self.c_prev = None
        self.t_prev = None
        self.c_hat = None

    def observe(self, t: float, dc: float) -> float:
        """Advance the road-wheel angle estimate to time t (commanded angle dc now);
        the command is a ramp between two calls."""
        if self.d_hat is None:
            self.d_hat = dc                            # start in steady steering
        else:
            h = t - self.t_prev
            if h > 0.0:
                rho = (dc - self.c_prev) / h
                tau = self.tau
                self.d_hat = dc - rho * tau + (self.d_hat - self.c_prev + rho * tau) * np.exp(-h / tau)
        self.c_prev, self.t_prev = dc, t
        return self.d_hat

    def __call__(self, t: float, x: np.ndarray):
        if self.observe_mode == "command":
            d_act, d_cmd = self.observe(t, float(x[6])), float(x[6])
        else:
            d_act = float(x[6])
            if self.c_hat is None:
                self.c_hat = d_act                     # start in steady steering
            d_cmd = self.c_hat
        xa = np.array(x, float)
        xa[6] = d_act
        xf = np.concatenate([self.frenet(xa), [d_cmd]])
        if t + 1e-9 >= self.t_next:
            u, info = self.mpc.solve(xf, self.u)
            self.u = u
            self.t_next = t + self.mpc.dt
            self.log.append((t, xf[0], xf[1], info["time"], info["status"], info["slack"]))
            if self.record:
                self.predictions.append((t, *self.mpc.prediction()))
        drate, fd, fb = float(self.u[0]), float(self.u[1]), float(self.u[2])
        veh = self.mpc.veh
        if self.observe_mode == "angle":               # own commanded angle, with the actuator limits
            dr = float(np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max))
            if (d_cmd >= veh.delta_max and dr > 0) or (d_cmd <= -veh.delta_max and dr < 0):
                dr = 0.0
            self.c_hat = float(np.clip(d_cmd + dr * self.sim_dt, -veh.delta_max, veh.delta_max))
        if self.tc_eta is not None:                    # traction control at the predicted angle
            d_next = d_act + (d_cmd - d_act) * (1.0 - np.exp(-self.sim_dt / self.tau))
            fd, fb = traction_limit(veh, xa, d_next, fd, fb, self.tc_eta)
        return drate, fd, fb


class CautiousMPCController(MPCController):
    """L6 round 2: tightening from the predictive spread of a learned residual model
    (``estimation/residual.py``). Before every solve, the spread sigma_k of the lateral
    acceleration (v_y') is evaluated along the previous solution shifted by one step
    (the constant initial guess for the first solve) and turned into

        eta_k = clip(eta0 - kappa m sigma_k / (mu_y F_z,k), eta_min, eta0)
        b_k   = kappa sum_{j<k} (k - j - 1/2) h^2 sigma_j      (errors fully correlated)

    with F_z,k = m g + downforce at the stage's speed."""

    def __init__(self, mpc: FrenetMPC, sim_dt: float, residual_model, eta0: float = 0.95,
                 eta_min: float = 0.80, kappa: float = 1.96, tc_eta: float | None = 0.99,
                 record: bool = False):
        if not mpc.param_tightening:
            raise ValueError("the MPC needs param_tightening=True")
        super().__init__(mpc, sim_dt, tc_eta, record)
        self.model, self.eta0, self.eta_min, self.kappa = residual_model, eta0, eta_min, kappa
        self.tightening: list[tuple[float, np.ndarray, np.ndarray]] = []

    def _tightening(self, xf: np.ndarray):
        mpc, veh, N, h = self.mpc, self.mpc.veh, self.mpc.N, self.mpc.dt
        if mpc.z is None:
            X = np.tile(xf, (N, 1))
            U = np.tile(self.u, (N, 1))
        else:
            Xp, Up = mpc.z[mpc.Xi], mpc.z[mpc.Ui] * np.array([1.0, FS, FS])
            X = np.vstack([Xp[1:N], Xp[N:N + 1]])
            U = np.vstack([Up[1:], Up[-1:]])
        Z = np.column_stack([X[:, 3:7], U])
        sig = self.model.predict(Z)[1][:, 1]
        v2 = X[:, 3] ** 2 + X[:, 4] ** 2
        fz = veh.m * 9.81 + veh.q_lift * v2
        eta = np.clip(self.eta0 - self.kappa * veh.m * sig / (veh.mu_y * fz), self.eta_min, self.eta0)
        b = np.zeros(N + 1)
        for k in range(1, N + 1):
            j = np.arange(k)
            b[k] = self.kappa * np.sum((k - j - 0.5) * h * h * sig[:k])
        return eta, b

    def __call__(self, t: float, x: np.ndarray):
        xf = self.frenet(x)
        if t + 1e-9 >= self.t_next:
            eta, b = self._tightening(xf)
            u, info = self.mpc.solve(xf, self.u, eta=eta, backoff=b)
            self.u = u
            self.t_next = t + self.mpc.dt
            self.log.append((t, xf[0], xf[1], info["time"], info["status"], info["slack"]))
            self.tightening.append((t, eta, b))
            if self.record:
                self.predictions.append((t, *self.mpc.prediction()))
        drate, fd, fb = float(self.u[0]), float(self.u[1]), float(self.u[2])
        if self.tc_eta is not None:
            veh = self.mpc.veh
            d_next = x[6] + np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max) * self.sim_dt
            fd, fb = traction_limit(veh, x, d_next, fd, fb, self.tc_eta)
        return drate, fd, fb
