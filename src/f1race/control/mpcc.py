"""Model predictive contouring control (MPCC) for the dynamic single-track model
(control expansion, phase A).

The contouring formulation of Liniger, Domahidi and Morari, *Optimization-based autonomous racing of
1:43 scale RC cars*, OCAM 36(5), 2015 (arXiv:1711.07300), section 4, after Lam, Manzie and Good
(CDC 2010), with the vehicle model, tyre limits and actuator limits of this repository's NMPC
(``control/nmpc.py``), so that the two differ only in the objective and the coordinates:

State   x = [X, Y, psi, vx, vy, r, delta, theta]   (Cartesian CG position and heading, body
        velocities, steering angle; theta: progress along the reference path, unwrapped arclength)
Input   u = [delta_rate, F_drive, F_brake, v_theta] (forces scaled by 1e4 inside the NLP;
        v_theta: the progress speed, Liniger's virtual input v)

    X' = vx cos psi - vy sin psi,  Y' = vx sin psi + vy cos psi,  psi' = r,
    (vx, vy, r)' from dynamics.body_accel (load transfer, aero, as the NMPC),
    delta' = delta_rate,  theta' = v_theta

Reference path (``reference``): the track centre line ("centre", Liniger's choice) or the racing line of
a plan ("raceline": the point at the plan's lateral offset n_ref(theta) on the centre line's normal),
X_ref(theta), Y_ref(theta) and its tangent angle Phi(theta) as cubic B-splines through 1 m samples
(Liniger: a cubic spline in arclength). theta is the centre-line arclength of the reference point in
both cases, so sum h v_theta is progress along the track. Contouring and lag error of a point p at the
reference point of theta (Liniger eq. 11):

    e_c = sin(Phi) (p_x - X_ref) - cos(Phi) (p_y - Y_ref)      (= -n: positive to the right)
    e_l = -cos(Phi) (p_x - X_ref) - sin(Phi) (p_y - Y_ref)

Cost (Liniger eq. 13a; each stage weighted by its length h):

    sum_{i=1..N} h (q_c e_c,i^2 + q_l e_l,i^2) - q_v sum_{i=0..N-1} h v_theta,i
    + input-rate penalties (steering rate, force increments, v_theta increments) + slack penalties

q_c low (the car need not stay on the centre line), q_l high (keeps theta at the car's projection,
so that sum h v_theta approximates the real progress - Liniger's argument).

Constraints at every stage: the dynamics (one RK4 step per stage, theta exact); the friction ellipse
of each axle (factor ``eta``), slip angles within the tyre peak, power, the longitudinal force per
axle, steering angle and rate (all as ``FrenetMPC``, ellipses and slip angles soft with exactly
penalised slacks); 0 <= v_theta <= ``v_theta_max``; the track as the two half-planes parallel to the
path at the reference point (Liniger eq. 12d: tangent to the borders), for the front and the rear
axle point, softened like the NMPC's track limits; the half-planes are those of the *centre line* at
theta (the borders are parallel to it), whichever reference path the cost uses:

    -(w_left(theta) - body) <= e_c^centre(axle point) <= w_right(theta) - body,  body = half width + margin

``speed_bound_soft`` (CX-A amendment A2; default off – the NLP is then unchanged): the speed bound with a
slack, vx_i - v_ref(theta_i - e_l,i) <= sigma_v,i, sigma_v >= 0, penalised exactly like the track limits
(``speed_slack``: quadratic + 10 x linear) – the solution is the same while the bound can be met, and a car
already faster than the plan (a car that brakes worse than the model) no longer makes the NLP infeasible.

**Deviations from Liniger et al., stated:** (1) the NLP is solved to convergence by IPOPT with the
previous solution shifted as warm start (Liniger: one QP per sample around the shifted solution - a
real-time iteration; that variant belongs to the real-time phase G); (2) a speed bound from a brakeable speed profile
(``speed_bound``: "terminal" vx_N <= v_ref, the default, or "stages" vx_i <= v_ref at every stage;
evaluated at the car's approximate projection theta - e_l): the 1:43
cars could stop within the 0.8 s horizon, this car at 85 m/s cannot within 1.5 s (~150 m braking
zones, docs/L2.md section 4), so without it the horizon runs into corners it can no longer take;
(3) the vehicle model is this repository's (load transfer, aero, friction ellipses), not Liniger's
simplified Pacejka model without load transfer; (4) ``reference="raceline"`` - with the centre line
and a 1.5 s horizon the car stays near the centre line on the straights and enters corners on a tighter
line than the speed profile of the terminal bound was computed for (development check on Spielberg,
docs/CX_A_protocol.md section 9).

At every solve theta_0 is the projection of the measured CG on the centre line (Liniger: theta
initialised from the measured progress), unwrapped against the previous solve.

``curvature_speed`` (optional; default off – the NLP is then unchanged): the curvature-integrated speed
cost of Li, Xie, Hu, Su, *Reduce Lap Time for Autonomous Racing with Curvature-Integrated MPCC Local
Trajectory Planning Method* (CiMPCC, arXiv:2502.03695, eqs. 5-10): the centre line's |curvature|,
smoothed by a moving average (``window`` m) and min-max normalised over the track to K in [0, 1], gives at
the car's projection theta_0 the weight beta = exp(-alpha K(theta_0)^2) (one beta per solve, a parameter of
the NLP); the cost adds  sum_{i=1..N} h w_ca [(1 - beta)(vx_i - v_low,i)^2 + beta (vx_i - v_high,i)^2]
(CiMPCC eq. 10). **Adaptation, stated:** CiMPCC uses two constant speeds (aggressive / safe, ratio 0.65)
for a 1:10 car; this car's speed ranges from ~25 to ~90 m/s, so v_high,i = v_ref(theta_i - e_l,i) (the plan's
speed, also the bound) and v_low,i = ``c_low`` v_high,i with c_low = 0.65, CiMPCC's ratio; CiMPCC's model is
kinematic, ours the dynamic model above.
"""
from __future__ import annotations

import time

import casadi as ca
import numpy as np

from ..dynamics import DynVehicle, body_accel, tyre_forces
from ..track import Track
from .dyn_tracking import traction_limit
from .nmpc import _periodic_interpolant

FS = 1e4


def _periodic_spline(name: str, s: np.ndarray, f: np.ndarray, length: float, laps: int = 3):
    """Cubic B-spline interpolant of a periodic field over [-length, laps*length]."""
    grid = np.concatenate([s + k * length for k in range(-1, laps)])
    vals = np.tile(f, laps + 1)
    return ca.interpolant(name, "bspline", [grid], vals)


def path_errors(track: Track, px, py, theta) -> tuple[np.ndarray, np.ndarray]:
    """Contouring and lag error (Liniger eq. 11) of points (px, py) at the centre-line reference
    points of ``theta`` (numpy; the NLP uses the same expressions with the splines)."""
    phi = track.heading_at(theta)
    p = track.point_at(theta)
    dx, dy = np.asarray(px, float) - p[:, 0], np.asarray(py, float) - p[:, 1]
    return np.sin(phi) * dx - np.cos(phi) * dy, -np.cos(phi) * dx - np.sin(phi) * dy


class DynamicMPCC:
    def __init__(self, track: Track, veh: DynVehicle, ref: dict, N: int = 30, dt: float = 0.05,
                 eta: float = 0.95, margin: float = 0.3, weights: dict | None = None,
                 v_theta_max: float = 120.0, terminal_speed: bool = True, max_iter: int = 500,
                 reference: str = "centre", speed_bound: str | None = None,
                 curvature_speed: dict | None = None, speed_bound_soft: bool = False):
        """``ref``: arrays over centre-line arclength ref['s'] with keys vx (a brakeable speed
        profile, e.g. an L1 plan solved with the same ``eta``, for the speed bound) and n (the
        plan's lateral offset, for ``reference="raceline"`` only). ``weights``: q_c, q_l, q_v,
        drate, dF, dv (v_theta increments), slack (track), limit_slack (ellipses / slip angles).
        ``speed_bound``: "none", "terminal" or "stages" (default: "terminal", or "none" with
        ``terminal_speed=False``)."""
        self.track, self.veh, self.N, self.dt = track, veh, N, dt
        self.eta = eta
        if speed_bound is None:                         # ``terminal_speed`` kept for the first version
            speed_bound = "terminal" if terminal_speed else "none"
        if speed_bound not in ("none", "terminal", "stages"):
            raise ValueError(speed_bound)
        self.speed_bound = speed_bound
        cs = None if curvature_speed is None else {"window": 50.0, "c_low": 0.65, **curvature_speed}
        if cs is not None:
            if not {"alpha", "w_ca"} <= set(cs):
                raise ValueError("curvature_speed needs alpha and w_ca")
            k = np.abs(track.kappa)
            n_w = max(int(round(cs["window"] / track.ds)), 1)
            km = np.convolve(np.concatenate([k[-n_w:], k, k[:n_w]]), np.ones(n_w) / n_w, mode="same")[n_w:-n_w]
            self._K = (km - km.min()) / (km.max() - km.min())       # CiMPCC eqs. 5-8 on the centre line
        self.curvature_speed = cs
        self.terminal_speed = speed_bound != "none"
        w = {"q_c": 0.05, "q_l": 50.0, "q_v": 1.0, "drate": 1.0, "dF": 0.5, "dv": 0.01,
             "slack": 1e4, "limit_slack": 1e6, "speed_slack": 1e4}
        w.update(weights or {})
        self.w = w
        if reference not in ("centre", "raceline"):
            raise ValueError(reference)
        self.reference = reference
        L = track.length
        Xc = _periodic_spline("Xc", track.s, track.x, L)
        Yc = _periodic_spline("Yc", track.s, track.y, L)
        cc = _periodic_spline("cc", track.s, np.cos(track.psi), L)
        sc = _periodic_spline("sc", track.s, np.sin(track.psi), L)
        if reference == "centre":
            Xr, Yr, cph, sph = Xc, Yc, cc, sc
        else:                                           # the plan's line on the centre-line normals
            nr = np.interp(track.s, np.asarray(ref["s"]) % L, np.asarray(ref["n"]), period=L)
            xy = np.column_stack([track.x, track.y]) + track.normal * nr[:, None]
            d = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
            phr = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
            Xr = _periodic_spline("Xr", track.s, xy[:, 0], L)
            Yr = _periodic_spline("Yr", track.s, xy[:, 1], L)
            cph = _periodic_spline("cph", track.s, np.cos(phr), L)
            sph = _periodic_spline("sph", track.s, np.sin(phr), L)
        wl = _periodic_interpolant("wl", track.s, track.w_left, L)
        wr = _periodic_interpolant("wr", track.s, track.w_right, L)
        vref = _periodic_interpolant("vref", ref["s"], ref["vx"], L)
        self._Xr, self._Yr = Xr, Yr

        nx, nu = 8, 4
        x = ca.SX.sym("x", nx)
        u = ca.SX.sym("u", nu)
        X, Y, psi, vx, vy, r, d, th = (x[i] for i in range(nx))
        F = tyre_forces(veh, vx, vy, r, d, u[1] * FS, u[2] * FS, lib=ca)
        dvx, dvy, dr = body_accel(veh, vx, vy, r, d, F, lib=ca)
        f = ca.Function("f", [x, u], [ca.vertcat(vx * ca.cos(psi) - vy * ca.sin(psi),
                                                 vx * ca.sin(psi) + vy * ca.cos(psi),
                                                 r, dvx, dvy, dr, u[0], u[3])])
        h = dt
        k1 = f(x, u); k2 = f(x + h / 2 * k1, u); k3 = f(x + h / 2 * k2, u); k4 = f(x + h * k3, u)
        step = ca.Function("step", [x, u], [x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)])
        self._f, self._step = f, step

        def errors_of(Xf, Yf, cf, sf):
            def errors(px, py, t):                      # Liniger eq. 11 (cos / sin of Phi by spline)
                c, s_ = cf(t), sf(t)
                nrm = ca.sqrt(c**2 + s_**2)             # the splines of cos / sin are not exactly unit
                c, s_ = c / nrm, s_ / nrm
                dx, dy = px - Xf(t), py - Yf(t)
                return s_ * dx - c * dy, -c * dx - s_ * dy
            return errors

        errors = errors_of(Xr, Yr, cph, sph)            # cost: the reference path
        errors_c = errors_of(Xc, Yc, cc, sc)            # track half-planes: the centre line

        qx, qy, qt = ca.SX.sym("qx"), ca.SX.sym("qy"), ca.SX.sym("qt")
        self.errors_fn = ca.Function("errors", [qx, qy, qt], list(errors(qx, qy, qt)))  # for tests
        self.errors_centre_fn = ca.Function("errors_c", [qx, qy, qt], list(errors_c(qx, qy, qt)))

        Xs = ca.SX.sym("X", nx, N + 1)
        U = ca.SX.sym("U", nu, N)
        S = ca.SX.sym("S", 2, N + 1)                    # track slacks (front, rear axle point)
        soft_v = bool(speed_bound_soft) and speed_bound != "none"
        self.speed_bound_soft = soft_v
        V = ca.SX.sym("V", N + 1 if soft_v else 0)      # speed-bound slacks (CX-A amendment A2)
        T = ca.SX.sym("T", 4, N)                        # ellipse (front, rear) and slip slacks
        P = ca.SX.sym("P", nx + nu + (1 if cs else 0))  # initial state, previous input (, beta)
        g, lbg, ubg = [], [], []
        J = 0
        g.append(Xs[:, 0] - P[:nx]); lbg += [0] * nx; ubg += [0] * nx
        body = veh.half_width + margin
        ap = veh.alpha_peak
        for i in range(N):
            g.append(Xs[:, i + 1] - step(Xs[:, i], U[:, i])); lbg += [0] * nx; ubg += [0] * nx
            Fi = tyre_forces(veh, Xs[3, i], Xs[4, i], Xs[5, i], Xs[6, i], U[1, i] * FS, U[2, i] * FS, lib=ca)
            uf2 = (Fi["fxf"] / (veh.mu_x * Fi["fzf"])) ** 2 + (Fi["fyf"] / (veh.mu_y * Fi["fzf"])) ** 2
            ur2 = (Fi["fxr"] / (veh.mu_x * Fi["fzr"])) ** 2 + (Fi["fyr"] / (veh.mu_y * Fi["fzr"])) ** 2
            g += [uf2 - T[0, i], ur2 - T[1, i], Fi["af"] - ap * T[2, i], Fi["af"] + ap * T[2, i],
                  Fi["ar"] - ap * T[3, i], Fi["ar"] + ap * T[3, i], U[1, i] * FS * Xs[3, i],
                  Fi["fxf"] / (veh.mu_x * Fi["fzf"]), Fi["fxr"] / (veh.mu_x * Fi["fzr"])]
            lbg += [-ca.inf, -ca.inf, -ca.inf, -ap, -ca.inf, -ap, -ca.inf, -eta, -eta]
            ubg += [eta**2, eta**2, ap, ca.inf, ap, ca.inf, veh.P_max, eta, eta]
            J += w["limit_slack"] * h * (ca.sumsqr(T[:, i]) + 10.0 * ca.sum1(T[:, i]))
            du = U[:, i] - (P[nx:nx + nu] if i == 0 else U[:, i - 1])
            J += h * (w["drate"] * U[0, i] ** 2 + w["dF"] * (du[1] ** 2 + du[2] ** 2) / h
                      + w["dv"] * du[3] ** 2 / h)
            J += -w["q_v"] * h * U[3, i]                # progress (Liniger: - gamma v_k T_s)
        for i in range(1, N + 1):
            Xi_, Yi_, psii, thi = Xs[0, i], Xs[1, i], Xs[2, i], Xs[7, i]
            ec, el = errors(Xi_, Yi_, thi)
            J += h * (w["q_c"] * ec**2 + w["q_l"] * el**2)
            for j, arm in enumerate((veh.lf, -veh.lr)):      # half-planes for the axle points
                eca, _ = errors_c(Xi_ + arm * ca.cos(psii), Yi_ + arm * ca.sin(psii), thi)
                g += [eca + (wl(thi) - body) + S[j, i], (wr(thi) - body) - eca + S[j, i]]
                lbg += [0.0, 0.0]; ubg += [ca.inf, ca.inf]
            J += w["slack"] * h * ca.sumsqr(S[:, i]) + 10 * w["slack"] * h * ca.sum1(S[:, i])
            if speed_bound == "stages" or (speed_bound == "terminal" and i == N):
                # at the car's approximate projection theta - e_l (Liniger: e_l ~ theta_A - theta_P),
                # so that a theta lagging behind the car cannot loosen the bound in a braking zone
                if soft_v:                               # exact penalty, as the track limits (amendment A2)
                    g.append(Xs[3, i] - vref(thi - el) - V[i]); lbg.append(-ca.inf); ubg.append(0.0)
                    J += w["speed_slack"] * h * (V[i] ** 2 + 10.0 * V[i])
                else:
                    g.append(Xs[3, i] - vref(thi - el)); lbg.append(-ca.inf); ubg.append(0.0)
            if cs:                                       # CiMPCC eq. 10, adapted (module docstring)
                beta, vh = P[nx + nu], vref(thi - el)
                J += h * cs["w_ca"] * ((1 - beta) * (Xs[3, i] - cs["c_low"] * vh) ** 2 + beta * (Xs[3, i] - vh) ** 2)

        z = ca.vertcat(ca.vec(Xs), ca.vec(U), ca.vec(S), ca.vec(T), V)
        self.nx, self.nu = nx, nu
        lbz = np.full(z.shape[0], -np.inf)
        ubz = np.full(z.shape[0], np.inf)
        Xi = np.arange(nx * (N + 1)).reshape(N + 1, nx)
        Ui = nx * (N + 1) + np.arange(nu * N).reshape(N, nu)
        Si = nx * (N + 1) + nu * N + np.arange(2 * (N + 1)).reshape(N + 1, 2)
        Ti = nx * (N + 1) + nu * N + 2 * (N + 1) + np.arange(4 * N).reshape(N, 4)
        lbz[Xi[:, 3]] = 2.0
        lbz[Xi[:, 6]], ubz[Xi[:, 6]] = -veh.delta_max, veh.delta_max
        lbz[Ui[:, 0]], ubz[Ui[:, 0]] = -veh.delta_rate_max, veh.delta_rate_max
        lbz[Ui[:, 1]], ubz[Ui[:, 1]] = 0.0, np.inf
        lbz[Ui[:, 2]], ubz[Ui[:, 2]] = -np.inf, 0.0
        lbz[Ui[:, 3]], ubz[Ui[:, 3]] = 0.0, v_theta_max
        lbz[Si] = 0.0
        ubz[Si[0]] = 0.0                                 # slack at the (fixed) initial state unused
        lbz[Ti] = 0.0
        Vi = nx * (N + 1) + nu * N + 2 * (N + 1) + 4 * N + np.arange(V.numel())
        lbz[Vi] = 0.0
        if soft_v:
            ubz[Vi[0]] = 0.0                             # no bound at the (measured) stage 0
        self.Vi = Vi
        self.Xi, self.Ui, self.Si, self.Ti = Xi, Ui, Si, Ti
        self.lbz, self.ubz = lbz, ubz
        self.lbg, self.ubg = np.array(lbg, float), np.array(ubg, float)
        self.solver = ca.nlpsol("mpcc", "ipopt", {"x": z, "f": J, "g": ca.vertcat(*g), "p": P}, {
            "print_time": False,
            "ipopt": {"print_level": 0, "max_iter": max_iter, "tol": 1e-4, "acceptable_tol": 1e-3,
                      "acceptable_iter": 5, "warm_start_init_point": "yes", "mu_init": 1e-3,
                      "linear_solver": "mumps"}})
        self.z = self.lam_g = self.lam_x = None
        self.stats: list[dict] = []

    # ------------------------------------------------------------------ helpers
    def initial_guess(self, x0: np.ndarray, u0: np.ndarray) -> np.ndarray:
        """First solve: the centre line ahead of theta_0 at the current speed, heading along it."""
        z = np.zeros(self.lbz.shape)
        tr = self.track
        v = max(float(x0[3]), 5.0)
        for i in range(self.N + 1):
            th = x0[7] + i * self.dt * v
            p = tr.point_at(th)[0]
            xs = np.array(x0, float)
            if i > 0:
                xs[:3] = [p[0], p[1], x0[2] + float(np.angle(np.exp(1j * (tr.heading_at(th) - x0[2]))))]
                xs[5] = xs[6] = 0.0
            xs[7] = th
            z[self.Xi[i]] = xs
        for i in range(self.N):
            z[self.Ui[i]] = [0.0, u0[1] / FS, u0[2] / FS, v]
        return z

    def solve(self, x0: np.ndarray, u_prev: np.ndarray) -> tuple[np.ndarray, dict]:
        """``x0``: [X, Y, psi, vx, vy, r, delta, theta_0]; ``u_prev``: [delta_rate, F_drive, F_brake,
        v_theta] (forces in N). Returns the first input (forces in N) and an info dict. A failed
        solve (no Solve_Succeeded / Solved_To_Acceptable_Level) applies the shifted previous plan
        (as ``FrenetMPC``'s default)."""
        p = np.concatenate([x0, [u_prev[0], u_prev[1] / FS, u_prev[2] / FS, u_prev[3]]])
        beta = None
        if self.curvature_speed:
            beta = self.beta(x0[7])
            p = np.concatenate([p, [beta]])
        if self.z is None:
            z0, args = self.initial_guess(x0, u_prev), {}
        else:
            z0 = self.z.copy()
            Xp, Up = self.z[self.Xi], self.z[self.Ui]
            z0[self.Xi[:-1]] = Xp[1:]
            z0[self.Ui[:-1]] = Up[1:]
            z0[self.Xi[0]] = x0
            args = {"lam_g0": self.lam_g, "lam_x0": self.lam_x}
        t0 = time.time()
        sol = self.solver(x0=z0, p=p, lbx=self.lbz, ubx=self.ubz, lbg=self.lbg, ubg=self.ubg, **args)
        st = self.solver.stats()
        ok = st["return_status"] in ("Solve_Succeeded", "Solved_To_Acceptable_Level")
        fallback = False
        if ok or self.z is None:
            z = np.asarray(sol["x"]).ravel()
            self.z, self.lam_g, self.lam_x = z, np.asarray(sol["lam_g"]).ravel(), np.asarray(sol["lam_x"]).ravel()
        else:
            z, fallback = z0, True
            self.z = z0
        u = z[self.Ui[0]] * np.array([1.0, FS, FS, 1.0])
        info = {"status": st["return_status"], "iter": st["iter_count"], "time": time.time() - t0,
                "slack": float(np.max(z[self.Si])), "limit_slack": float(np.max(z[self.Ti])),
                "speed_slack": float(np.max(z[self.Vi])) if self.Vi.size else 0.0,
                "fallback": fallback}
        if beta is not None:
            info["beta"] = beta
        self.stats.append(info)
        return u, info

    def beta(self, theta: float) -> float:
        """CiMPCC's weight exp(-alpha K^2) at the centre-line arclength ``theta`` (eq. 9)."""
        K = float(self.track.interp(self._K, theta))
        return float(np.exp(-self.curvature_speed["alpha"] * K ** 2))

    def prediction(self) -> tuple[np.ndarray, np.ndarray]:
        """Predicted states (N + 1, 8) and inputs (N, 4, forces in N) of the last solve."""
        return self.z[self.Xi].copy(), self.z[self.Ui] * np.array([1.0, FS, FS, 1.0])


class MPCCController:
    """Plant interface: projects the measured CG on the centre line for theta_0 (unwrapped),
    re-solves every ``mpc.dt`` and holds the input in between, with the same low-level traction
    control as ``MPCController`` (``tc_eta``, applied at every simulator step on the true state)."""

    def __init__(self, mpc: DynamicMPCC, sim_dt: float, tc_eta: float | None = 0.99):
        self.mpc, self.sim_dt, self.tc_eta = mpc, sim_dt, tc_eta
        self.track = mpc.track
        self.u = np.zeros(4)
        self.t_next = 0.0
        self.theta = None
        self.log: list[tuple] = []

    def progress(self, x: np.ndarray) -> float:
        tr = self.track
        s, _ = tr.project(np.array([x[0]]), np.array([x[1]]))
        s = float(s[0])
        if self.theta is None:
            th = s
        else:
            prev = self.theta
            th = prev + ((s - prev % tr.length + 0.5 * tr.length) % tr.length - 0.5 * tr.length)
        self.theta = th
        return th

    def __call__(self, t: float, x: np.ndarray):
        th = self.progress(x)
        if t + 1e-9 >= self.t_next:
            if self.mpc.z is None and self.u[3] == 0.0:
                self.u = np.array([0.0, 0.0, 0.0, max(float(x[3]), 5.0)])
            u, info = self.mpc.solve(np.concatenate([np.asarray(x, float), [th]]), self.u)
            self.u = u
            self.t_next = t + self.mpc.dt
            ec, _ = path_errors(self.track, [x[0]], [x[1]], np.array([th]))
            self.log.append((t, th, float(ec[0]), info["time"], info["status"], info["slack"],
                             info["fallback"]))
        drate, fd, fb = float(self.u[0]), float(self.u[1]), float(self.u[2])
        if self.tc_eta is not None:
            veh = self.mpc.veh
            d_next = x[6] + np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max) * self.sim_dt
            fd, fb = traction_limit(veh, x, d_next, fd, fb, self.tc_eta)
        return drate, fd, fb
