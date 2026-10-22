"""Model predictive path integral control (MPPI) on the NMPC's optimal control problem (control
expansion, phase B; docs/CX_B_protocol.md, revision 0 with its clarifications).

The algorithm of Williams, Drews, Goldfain, Rehg, Theodorou, *Information Theoretic Model Predictive
Control: Theory and Applications to Autonomous Driving*, IEEE T-RO 34(6), 2018 (arXiv:1707.02342),
Algorithms 1-2, with the update in the form of Vlahov et al., *MPPI-Generic* (arXiv:2409.07563), applied to
the optimal control problem of ``FrenetMPC(mode="track")`` (``control/nmpc.py``) - the same model, horizon,
objective and limits - so that it differs from the NMPC only in the optimiser.

Model (the NMPC's, nominal parameters, batched over the samples in numpy):

State   x = [s, n, xi, vx, vy, r, delta]   (Frenet, as ``FrenetMPC``)
Input   u = [delta_rate, F_x]              sampled (``inputs="net"``, revision 0); F_drive = max(F_x, 0),
                                           F_brake = min(F_x, 0)
        u = [delta_rate, F_drive, F_brake] sampled (``inputs="three"``, the NMPC's inputs; proposed amendment B1)

    s'  = (vx cos xi - vy sin xi) / (1 - n kappa(s)),   n' = vx sin xi + vy cos xi,   xi' = r - kappa(s) s'
    (vx, vy, r)' from dynamics.body_accel (no force saturation, as the NMPC),   delta' = delta_rate

one RK4 step per stage of h = 50 ms with the input held; kappa, the track widths and the reference
(n_ref, xi_ref, v_ref of a plan) as periodic linear interpolants (the NMPC's).

Cost of an input sequence from the measured state x_0 (``FrenetMPC``'s objective with its slack variables
eliminated - for given inputs the optimal slacks are these sigma and tau, so the value equals the NLP's
objective at that point; forces in units of 1e4 N in the rate term):

    J = sum_{i=1..N} h [w_n (n_i - n_ref)^2 + w_xi (xi_i - xi_ref)^2 + w_v (vx_i - v_ref)^2]
      + sum_{i=0..N-1} [h w_drate delta_rate_i^2 + w_dF (dF_drive,i^2 + dF_brake,i^2)]   (stage 0 against u_prev)
      + sum_{i=1..N} sum_{front, rear axle point} w_slack h (sigma^2 + 10 sigma)
            sigma = max(0, -(n_a + w_right - body - chord), n_a - (w_left - body - chord))
      + sum_{i=0..N-1} w_limit_slack h sum_{4} (tau^2 + 10 tau)
            tau = max(0, u_axle^2 - eta^2) for both friction ellipses, max(0, |alpha_axle| / alpha_peak - 1)
            for both slip angles

with n_a = n + arm sin xi, chord = arm^2 |kappa| / 2, body = half width + margin (all as ``FrenetMPC``). The
NMPC's hard limits are clamps of the sampled inputs inside the rollout (T-RO's g(v)): the steering rate to
+-delta_rate_max and so that delta stays within +-delta_max over the stage (exact: delta_rate is constant in a
stage); F_drive <= P_max / vx; each axle's longitudinal force |F_x,axle| <= eta mu_x F_z,axle, solved in closed
form (the normal loads depend on the force linearly through the load transfer, the rolling resistance on
F_zf + F_zr only); with three inputs (B1) F_drive >= 0, F_brake <= 0 and the power are clipped and the force
pair is scaled towards (0, 0) by the largest factor in [0, 1] that meets both axle limits (linear in the pair). A sample with vx < 2 m/s at a predicted stage (the NMPC's state bound) or a cost that is
not finite is not valid (weight 0).

One MPPI iteration (protocol section 3.2):

1. eps_k,i ~ N(0, diag(sigma_drate^2, sigma_F^2)), white over k and i (``lowpass``: filtered, below);
2. v_k = U + eps_k for the first K - K0 samples, v_k = eps_k for the last K0 = max(1, floor(alpha K));
3. roll out the clamped inputs v'_k; S_k = J(v'_k);
4. rho = min S_k over the valid samples, w_k = exp(-(S_k - rho) / lambda) / sum_j exp(-(S_j - rho) / lambda);
5. U <- sum_k w_k v'_k; apply u_0; shift U, the last entry (0, 0).

If no sample is valid, step 5's average is skipped (U stays, its u_0 is applied, it is shifted; clarification
m1a). One iteration per call; ``first_iters`` (20) at the first call, from U = 0.

``lowpass=(f_c, order)`` (``mppi_lp``, after Kicki, *LP-MPPI*, arXiv:2503.11717): each standard-normal noise
sequence is filtered along the stages by a causal digital Butterworth low-pass filter (``scipy.signal.butter``
at the sampling rate 1 / h, ``lfilter``, zero initial state) and multiplied by 1 / ||g||_2, g the filter's
impulse response over N stages, so that its stationary standard deviation is 1 before the scaling by sigma.

**Deviations from T-RO, stated (protocol section 3.2):** no importance-sampling control cost (gamma = 0: it is
a penalty on the input magnitude that the NMPC's objective does not contain); no Savitzky-Golay filter on the
update; the NMPC's analytic model instead of a learned one; the state-dependent limits clamped in the rollout.

``solve(x0, u_prev)`` has ``FrenetMPC.solve``'s interface, so ``nmpc.MPCController`` drives it unchanged
(Frenet transform, a call every 50 ms, input held, traction control). ``info["time"]`` is the time of the
iteration(s) and of computing the applied input; the diagnostic rollouts of the nominal sequence before and
after the update (``diagnostics``) are outside it.
"""
from __future__ import annotations

import time

import numpy as np

from ..dynamics import DynVehicle, body_accel, tyre_forces
from ..track import Track
from ..vehicle import G
from .nmpc import FS

DEFAULT_WEIGHTS = {"n": 1.0, "xi": 10.0, "v": 0.2, "drate": 1.0, "dF": 0.5, "slack": 1e4,
                   "limit_slack": 1e6}                   # FrenetMPC's defaults (L2)
VX_MIN = 2.0                                              # FrenetMPC's state bound on vx [m/s]


def _periodic(s, f, length: float, laps: int = 3):
    """Linear interpolant of a periodic field over [-length, laps * length] on the grid of
    ``nmpc._periodic_interpolant`` (the same breakpoints and values; ``np.interp`` with ``period`` would
    re-sort the grid at every call)."""
    s, f = np.asarray(s, float), np.asarray(f, float)
    grid = np.concatenate([s + k * length for k in range(-1, laps)])
    vals = np.tile(f, laps + 1)
    return lambda q: np.interp(q, grid, vals)


class MPPI:
    def __init__(self, track: Track, veh: DynVehicle, ref: dict, N: int = 30, dt: float = 0.05,
                 eta: float = 0.95, margin: float = 0.3, weights: dict | None = None, K: int = 1024,
                 lam: float = 1.0, sigma=(0.03, 2000.0), alpha: float = 0.01, seed=0, first_iters: int = 20,
                 lowpass: tuple[float, int] | None = None, diagnostics: bool = True, inputs: str = "net",
                 iters_per_step: int = 1):
        """``ref``: arrays over centre-line arclength ref['s'] with keys n, xi, vx (``FrenetMPC``'s);
        ``weights``: ``FrenetMPC``'s keys (n, xi, v, drate, dF, slack, limit_slack); ``sigma``: noise standard
        deviations of the steering rate [rad/s] and of each force [N]; ``seed``: anything
        ``numpy.random.default_rng`` takes (the runner passes (set code, car, replicate)); ``inputs``: "net" –
        the sampled input is [delta_rate, F_x] (protocol revision 0), "three" – [delta_rate, F_drive, F_brake],
        the NMPC's own inputs (amendment B1); ``iters_per_step``: MPPI iterations per call after the first (1 in
        the protocol; more only in the development diagnostic of the nominal car)."""
        if inputs not in ("net", "three"):
            raise ValueError(inputs)
        self.inputs, self.nu = inputs, 2 if inputs == "net" else 3
        self.track, self.veh, self.N, self.dt = track, veh, int(N), float(dt)
        self.eta, self.margin = float(eta), float(margin)
        w = dict(DEFAULT_WEIGHTS)
        w.update(weights or {})
        self.w = w
        L = track.length
        self.kap = _periodic(track.s, track.kappa, L)
        self.wl = _periodic(track.s, track.w_left, L)
        self.wr = _periodic(track.s, track.w_right, L)
        self.nref = _periodic(ref["s"], ref["n"], L)
        self.xiref = _periodic(ref["s"], ref["xi"], L)
        self.vref = _periodic(ref["s"], ref["vx"], L)
        self.body = veh.half_width + margin
        self.K, self.lam = int(K), float(lam)
        sig = np.asarray(sigma, float)
        self.sigma = np.array([sig[0], sig[1], sig[-1]]) if self.nu == 3 and sig.size == 2 else sig
        if self.sigma.size != self.nu:
            raise ValueError("sigma has the wrong length")
        self.alpha = float(alpha)
        self.K0 = max(1, int(np.floor(self.alpha * self.K)))
        self.rng = np.random.default_rng(seed)
        self.first_iters = int(first_iters)
        self.iters_per_step = int(iters_per_step)
        self.diagnostics = bool(diagnostics)
        self.lowpass = lowpass
        self._lp = None
        if lowpass is not None:
            from scipy.signal import butter, lfilter

            fc, order = lowpass
            b, a = butter(int(order), float(fc), fs=1.0 / self.dt)
            imp = np.zeros(self.N)
            imp[0] = 1.0
            g = lfilter(b, a, imp)
            self._lp = (b, a, 1.0 / float(np.sqrt(np.sum(g * g))))
        self.U = np.zeros((self.N, self.nu))               # nominal sequence ([delta_rate, F_x] or 3 inputs)
        self.U_last = None
        self.started = False
        self.stats: list[dict] = []
        self._pred = None
        self.fallback_mode = "shift"                       # MPCController's check; MPPI has no failed solve

    # ------------------------------------------------------------------ model
    def clamp(self, X: np.ndarray, V: np.ndarray) -> np.ndarray:
        """Clamped inputs (M, 3) [delta_rate, F_drive, F_brake] in N for states X (M, 7) and samples V (M, 2:
        [delta_rate, F_x]) or (M, 3: the three inputs; the force pair scaled towards 0 onto the axle limits)."""
        veh, h, eta = self.veh, self.dt, self.eta
        d, vx, vy = X[:, 6], X[:, 3], X[:, 4]
        dr = np.clip(V[:, 0], -veh.delta_rate_max, veh.delta_rate_max)
        dr = np.minimum(np.maximum(dr, (-veh.delta_max - d) / h), (veh.delta_max - d) / h)
        v2 = vx * vx + vy * vy
        res = veh.q_drag * vx * vx + veh.c_rr * (veh.m * G + veh.q_lift * v2)
        hl = veh.h_cg / veh.L
        bfz = veh.m * G * veh.lr / veh.L + veh.aero_front * veh.q_lift * v2       # F_zf = bfz - (F - res) hl
        brz = veh.m * G * veh.lf / veh.L + (1.0 - veh.aero_front) * veh.q_lift * v2  # F_zr = brz + (F - res) hl
        em, bf = eta * veh.mu_x, veh.brake_front
        if V.shape[1] == 3:                               # (amendment B1) the NMPC's three inputs
            fd = np.minimum(np.maximum(V[:, 1], 0.0), veh.P_max / np.maximum(vx, 1e-3))
            fb = np.minimum(V[:, 2], 0.0)
            scale = np.ones_like(fd)
            # the axle limits as rows a_d F_d + a_b F_b <= b (front; rear, driving; rear, braking)
            for a_d, a_b, b in ((em * hl, em * hl - bf, em * (bfz + res * hl)),
                                (1.0 - em * hl, 1.0 - bf - em * hl, em * (brz - res * hl)),
                                (-1.0 - em * hl, -(1.0 - bf) - em * hl, em * (brz - res * hl))):
                lhs = a_d * fd + a_b * fb
                with np.errstate(divide="ignore", invalid="ignore"):
                    scale = np.where(lhs > b, np.minimum(scale, np.clip(b / lhs, 0.0, 1.0)), scale)
            return np.stack([dr, fd * scale, fb * scale], axis=1)
        with np.errstate(divide="ignore", invalid="ignore"):
            fd_max = np.minimum(em * (brz - res * hl) / (1.0 - em * hl), veh.P_max / np.maximum(vx, 1e-3))
            fb_min = np.maximum(em * (bfz + res * hl) / (em * hl - bf),           # front axle
                                -em * (brz - res * hl) / ((1.0 - bf) + em * hl))    # rear axle
        fx = np.minimum(np.maximum(V[:, 1], np.minimum(fb_min, 0.0)), np.maximum(fd_max, 0.0))
        return np.stack([dr, np.maximum(fx, 0.0), np.minimum(fx, 0.0)], axis=1)

    def f(self, X: np.ndarray, V3: np.ndarray):
        """State derivative (M, 7) and the tyre forces of ``tyre_forces``."""
        s, n, xi, vx, vy, r, d = X.T
        k = self.kap(s)
        F = tyre_forces(self.veh, vx, vy, r, d, V3[:, 1], V3[:, 2])
        dvx, dvy, dr = body_accel(self.veh, vx, vy, r, d, F)
        c, sn = np.cos(xi), np.sin(xi)
        sdot = (vx * c - vy * sn) / (1.0 - n * k)
        return np.stack([sdot, vx * sn + vy * c, r - k * sdot, dvx, dvy, dr, V3[:, 0]], axis=1), F

    def step(self, X: np.ndarray, V3: np.ndarray):
        """One RK4 step of h with the input held (``FrenetMPC``'s ``step``); also the forces at X."""
        h = self.dt
        k1, F = self.f(X, V3)
        k2, _ = self.f(X + 0.5 * h * k1, V3)
        k3, _ = self.f(X + 0.5 * h * k2, V3)
        k4, _ = self.f(X + h * k3, V3)
        return X + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4), F

    def rollout(self, x0: np.ndarray, V: np.ndarray, u_prev: np.ndarray, detail: bool = False) -> dict:
        """Roll out samples V (M, N, 2) from x0 (7); u_prev: the previous control step's output (3, N).
        Returns the cost (inf where not valid), its parts, the clamped inputs and the states; ``detail``:
        also the eliminated slacks per stage, ``sigma`` (M, N + 1, 2; stage 0 unused, 0) and ``tau`` (M, N, 4:
        ellipse front, rear, slip front, rear) - ``FrenetMPC``'s S and T."""
        veh, w, h, N = self.veh, self.w, self.dt, self.N
        M = V.shape[0]
        X = np.empty((M, N + 1, 7))
        X[:, 0] = x0
        V3 = np.empty((M, N, 3))
        track_pen, limit_pen, obj = np.zeros(M), np.zeros(M), np.zeros(M)
        sig_max, tau_max = np.zeros(M), np.zeros(M)
        valid = np.ones(M, bool)
        sig_nodes = np.zeros((M, N + 1, 2)) if detail else None
        tau_nodes = np.zeros((M, N, 4)) if detail else None
        eta2, ap = self.eta ** 2, veh.alpha_peak
        prev_fd = np.full(M, u_prev[1] / FS)
        prev_fb = np.full(M, u_prev[2] / FS)
        with np.errstate(all="ignore"):
            for i in range(N):
                xi_ = X[:, i]
                v3 = self.clamp(xi_, V[:, i])
                V3[:, i] = v3
                fd, fb = v3[:, 1] / FS, v3[:, 2] / FS
                obj += h * w["drate"] * v3[:, 0] ** 2 + w["dF"] * ((fd - prev_fd) ** 2 + (fb - prev_fb) ** 2)
                prev_fd, prev_fb = fd, fb
                xn, F = self.step(xi_, v3)
                X[:, i + 1] = xn
                uf2 = (F["fxf"] / (veh.mu_x * F["fzf"])) ** 2 + (F["fyf"] / (veh.mu_y * F["fzf"])) ** 2
                ur2 = (F["fxr"] / (veh.mu_x * F["fzr"])) ** 2 + (F["fyr"] / (veh.mu_y * F["fzr"])) ** 2
                tau = np.stack([np.maximum(uf2 - eta2, 0.0), np.maximum(ur2 - eta2, 0.0),
                                np.maximum(np.abs(F["af"]) / ap - 1.0, 0.0),
                                np.maximum(np.abs(F["ar"]) / ap - 1.0, 0.0)])
                limit_pen += w["limit_slack"] * h * np.sum(tau * tau + 10.0 * tau, axis=0)
                tau_max = np.maximum(tau_max, tau.max(axis=0))
                if detail:
                    tau_nodes[:, i] = tau.T
                s, n, xi, vx = xn[:, 0], xn[:, 1], xn[:, 2], xn[:, 3]
                obj += h * (w["n"] * (n - self.nref(s)) ** 2 + w["xi"] * (xi - self.xiref(s)) ** 2
                            + w["v"] * (vx - self.vref(s)) ** 2)
                ka, wl, wr = np.abs(self.kap(s)), self.wl(s), self.wr(s)
                for j, arm in enumerate((veh.lf, -veh.lr)):
                    chord = 0.5 * arm * arm * ka
                    na = n + arm * np.sin(xi)
                    sig = np.maximum(np.maximum(-(na + wr - self.body - chord), na - (wl - self.body - chord)), 0.0)
                    track_pen += w["slack"] * h * (sig * sig + 10.0 * sig)
                    sig_max = np.maximum(sig_max, sig)
                    if detail:
                        sig_nodes[:, i + 1, j] = sig
                valid &= vx >= VX_MIN
            cost = obj + track_pen + limit_pen
        valid &= np.isfinite(cost)
        out = {"cost": np.where(valid, cost, np.inf), "valid": valid, "objective": obj, "track_pen": track_pen,
               "limit_pen": limit_pen, "sigma_max": sig_max, "tau_max": tau_max, "V3": V3, "X": X}
        if detail:
            out.update(sigma=sig_nodes, tau=tau_nodes)
        return out

    # -------------------------------------------------------------- iteration
    def noise(self) -> np.ndarray:
        E = self.rng.standard_normal((self.K, self.N, self.nu))
        if self._lp is not None:
            from scipy.signal import lfilter

            b, a, scale = self._lp
            E = lfilter(b, a, E, axis=1) * scale
        return E * self.sigma

    def iterate(self, x0: np.ndarray, u_prev: np.ndarray) -> dict:
        """One MPPI iteration on ``self.U`` (steps 1-5 without applying / shifting)."""
        K = self.K
        E = self.noise()
        V = self.U[None] + E
        V[K - self.K0:] = E[K - self.K0:]                  # zero-mean samples (T-RO's alpha)
        r = self.rollout(x0, V, u_prev)
        valid = r["valid"]
        pen = (r["track_pen"] + r["limit_pen"]) > 0.0
        out = {"valid": int(valid.sum()), "penalised_share": float(np.mean(pen[valid])) if valid.any() else float("nan")}
        if not valid.any():                                # clarification m1a: U stays as it is
            out.update(no_valid=True, ess=0.0, no_free=True, best_penalised=True)
            return out
        S = r["cost"]
        rho = float(S[valid].min())
        wts = np.zeros(K)
        wts[valid] = np.exp(-(S[valid] - rho) / self.lam)
        wts /= wts.sum()
        Vc = r["V3"] if self.nu == 3 else np.stack([r["V3"][..., 0], r["V3"][..., 1] + r["V3"][..., 2]], axis=-1)
        self.U = np.einsum("k,knj->nj", wts, Vc)
        out.update(no_valid=False, ess=float(1.0 / np.sum(wts * wts)), no_free=bool(np.all(pen[valid])),
                   best_penalised=bool(pen[int(np.argmin(S))]))
        return out

    def solve(self, x0: np.ndarray, u_prev: np.ndarray) -> tuple[np.ndarray, dict]:
        """``FrenetMPC.solve``'s interface: x0 the Frenet state (7), u_prev the previous output (3, forces
        in N); returns the input to apply (3, forces in N) and the step's record."""
        x0 = np.asarray(x0, float)
        u_prev = np.asarray(u_prev, float)
        U_before = self.U.copy()
        t0 = time.perf_counter()
        iters = self.iters_per_step if self.started else self.first_iters
        recs = [self.iterate(x0, u_prev) for _ in range(iters)]
        self.started = True
        u = self.clamp(x0[None], self.U[:1])[0]
        elapsed = time.perf_counter() - t0
        last = recs[-1]
        info = {"status": "no valid sample" if last["no_valid"] else "mppi", "time": elapsed, "iters": iters,
                "fallback": False, "no_valid": bool(last["no_valid"]), "no_free": bool(last["no_free"]),
                "best_penalised": bool(last["best_penalised"]), "ess": last["ess"], "valid": last["valid"],
                "invalid": self.K - last["valid"], "penalised_share": last["penalised_share"],
                "slack": float("nan"), "limit_slack": float("nan")}
        if iters > 1:
            info["no_valid_iterations"] = int(sum(r_["no_valid"] for r_ in recs))
        if self.diagnostics:                               # outside the timed part
            d = self.rollout(x0, np.stack([U_before, self.U]), u_prev)
            info.update(cost_before=float(d["cost"][0]), cost_after=float(d["cost"][1]),
                        slack=float(d["sigma_max"][1]), limit_slack=float(d["tau_max"][1]))
            self._pred = (d["X"][1].copy(), d["V3"][1].copy())
        self.U_last = self.U.copy()                        # the sequence whose u_0 was applied
        self.U = np.vstack([self.U[1:], np.zeros((1, self.nu))])  # shift; the last entry 0
        self.stats.append(info)
        return u, info

    def prediction(self) -> tuple[np.ndarray, np.ndarray]:
        """Predicted states (N + 1, 7) and clamped inputs (N, 3, forces in N) of the applied sequence of the
        last call (``diagnostics`` only)."""
        if self._pred is None:
            raise RuntimeError("no prediction (diagnostics off or no call yet)")
        return self._pred
