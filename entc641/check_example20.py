#!/usr/bin/env python3
"""Check doe.py against class Example 20 (2^3 design, two replicates).

    python3 entc641/check_example20.py

Hand calculation with the lecture's formulas: condition means 10 20 4 10 8 18 6 12,
average 11, E1 = 8, E2 = -6, E3 = 0, E12 = -2, E13 = 0, E23 = 2, E123 = 0;
Sp^2 = 3.25, S_effect^2 = 4 * 3.25 / 16 = 0.8125, t critical (8 dof) = 2.306,
so only E1 and E2 are significant. Exits non-zero if doe.py disagrees.
"""

from itertools import product

import doe

Y = [(9, 11), (18, 22), (4, 4), (9, 11), (7, 9), (17, 19), (5, 7), (10, 14)]
EXPECTED = {'1': 8.0, '2': -6.0, '3': 0.0, '12': -2.0, '13': 0.0, '23': 2.0, '123': 0.0}


def cols(x1, x2, x3):
    return {'1': x1, '2': x2, '3': x3, '12': x1 * x2, '13': x1 * x3,
            '23': x2 * x3, '123': x1 * x2 * x3}


def main():
    conds = []
    for (x3, x2, x1), ys in zip(product((-1, 1), repeat=3), Y):
        conds.append(doe.Condition(cols(x1, x2, x3), list(ys)))
    res = doe.analyze(conds, list(EXPECTED))
    print(f'average {res.average:g}   Sp^2 {res.pooled_var:g}   S_effect^2 {res.s_effect ** 2:g}'
          f'   dof {res.dof}   t crit {res.t_crit:.3f}')
    for c in EXPECTED:
        print(f'  E{c:4} {res.effects[c]:6.2f}   t {res.t[c]:6.2f}'
              f'{"   significant" if c in res.significant else ""}')
    ok = (abs(res.average - 11) < 1e-9 and abs(res.pooled_var - 3.25) < 1e-9
          and abs(res.s_effect ** 2 - 0.8125) < 1e-9 and res.dof == 8
          and abs(res.t_crit - 2.306) < 1e-3 and res.significant == ['1', '2']
          and all(abs(res.effects[c] - v) < 1e-9 for c, v in EXPECTED.items()))
    print('model: y = {:g} {}'.format(res.average, ' '.join(
        f'{b:+g} X{c}' for c, b in res.coefficients().items())))
    print('PASS: matches the hand calculation' if ok else 'FAIL')
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
