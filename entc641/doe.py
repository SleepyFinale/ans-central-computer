"""Two-level factorial analysis the way ENTC 641 teaches it (lectures 13-15).

Standardized method: the calculation matrix of coded columns, the average of the
replicates per test condition, each effect = sum(sign * ybar) / (n_conditions / 2).
Significance: variance of each condition from its replicates, pooled variance
Sp^2 = sum(v_i S_i^2) / sum(v_i), S_effect^2 = 4 Sp^2 / N (N = number of y values),
t = E / S_effect, two-tailed against t(dof, alpha/2) with dof = sum(v_i).
Model: y = average + sum over significant effects of (E / 2) * column.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy import stats


@dataclass
class Condition:
    signs: dict             # column name -> -1/+1
    ys: list


@dataclass
class EffectsResult:
    columns: list
    average: float
    effects: dict                       # column -> E
    n_y: int
    pooled_var: float = None            # Sp^2
    dof: int = 0
    s_effect: float = None
    t: dict = field(default_factory=dict)
    t_crit: float = None
    significant: list = field(default_factory=list)
    alpha: float = 0.05

    @property
    def has_variance(self):
        return self.pooled_var is not None and self.dof > 0

    def coefficients(self, only_significant=True):
        cols = self.significant if only_significant else self.columns
        return {c: self.effects[c] / 2.0 for c in cols}

    def predict(self, signs, only_significant=True):
        return self.average + sum(b * signs[c] for c, b in
                                  self.coefficients(only_significant).items())

    def prediction_interval(self, signs, only_significant=True):
        """t-based interval for one new run at this setting (half width), or None."""
        if not self.has_variance:
            return None
        cols = self.significant if only_significant else self.columns
        leverage = (1.0 + sum(signs[c] ** 2 for c in cols)) / self.n_y
        return self.t_crit * np.sqrt(self.pooled_var * (1.0 + leverage))


def analyze(conditions, columns, alpha=0.05):
    ybar = np.array([np.mean(c.ys) for c in conditions])
    half = len(conditions) / 2.0
    effects = {col: float(sum(c.signs[col] * yb for c, yb in zip(conditions, ybar)) / half)
               for col in columns}
    n_y = sum(len(c.ys) for c in conditions)
    res = EffectsResult(columns, float(ybar.mean()), effects, n_y, alpha=alpha)

    dof = sum(len(c.ys) - 1 for c in conditions if len(c.ys) >= 2)
    if dof > 0:
        res.pooled_var = float(sum((len(c.ys) - 1) * np.var(c.ys, ddof=1)
                                   for c in conditions if len(c.ys) >= 2) / dof)
        res.dof = dof
        res.s_effect = float(np.sqrt(4.0 * res.pooled_var / n_y))
        res.t_crit = float(stats.t.ppf(1 - alpha / 2, dof))
        for col in columns:
            res.t[col] = effects[col] / res.s_effect if res.s_effect > 0 else float('inf')
        res.significant = [col for col in columns if abs(res.t[col]) > res.t_crit]
    return res


def group_conditions(rows, factor_cols, column_fn, response):
    """Rows (dicts) -> Conditions keyed by the factor levels; skips missing y."""
    by = {}
    for r in rows:
        if r.get(response) in ('', None):
            continue
        key = tuple(int(float(r[f])) for f in factor_cols)
        by.setdefault(key, []).append(float(r[response]))
    return [Condition(column_fn(*k), ys) for k, ys in sorted(by.items())]


def least_squares(X, y):
    """Coefficients, R^2, adjusted R^2 and coefficient t values (normal equations)."""
    X, y = np.asarray(X, float), np.asarray(y, float)
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    resid = y - X @ beta
    n, p = X.shape
    ss_res = float(resid @ resid)
    ss_tot = float(((y - y.mean()) ** 2).sum())
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float('nan')
    adj = 1 - (1 - r2) * (n - 1) / (n - p) if n > p else float('nan')
    tvals = None
    if n > p:
        s2 = ss_res / (n - p)
        cov = s2 * np.linalg.pinv(X.T @ X)
        tvals = beta / np.sqrt(np.diag(cov))
    return beta, r2, adj, tvals, n - p
