#!/usr/bin/env python3
"""Audit the deployed quintic MINVO literals using Bernstein subdivision.

This is a binary64 numerical audit, not directed-rounding interval arithmetic.
Read the production matrix to avoid a second, independently maintained copy.
"""
import argparse
import json
import math
from pathlib import Path
import re

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='Production PolynomialHull.cpp')
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    literals = args.source.read_text().split('value <<', 1)[1].split(';', 1)[0]
    matrix = np.array([float(v) for v in re.findall(r'[-+]?\d+\.\d+', literals)]).reshape(6, 6)
    power = np.zeros((6, 6))
    for i in range(6):
        for j in range(6):
            degree = 5 - j
            for k in range(degree + 1):
                power[i, k] += matrix[i, j] * math.comb(degree, k) * 2**k * (-1)**(degree-k)
    bernstein = np.zeros_like(power)
    for k in range(6):
        for j in range(k+1):
            bernstein[:, k] += power[:, j] * math.comb(k, j) / math.comb(5, j)
    pieces = bernstein[None, :, :]
    for _ in range(13):
        work = pieces.copy()
        left, right = np.empty_like(work), np.empty_like(work)
        left[:, :, 0], right[:, :, -1] = work[:, :, 0], work[:, :, -1]
        for level in range(1, 6):
            work = .5 * (work[:, :, :-1] + work[:, :, 1:])
            left[:, :, level], right[:, :, 5-level] = work[:, :, 0], work[:, :, -1]
        pieces = np.concatenate((left, right))
    lower = pieces.min(axis=(0, 2))
    negative = float(-np.minimum(lower, 0).sum())
    unity = float(np.max(np.abs(matrix.sum(axis=0) - np.array([0, 0, 0, 0, 0, 1]))))
    if negative > 6.5e-8 or unity > 1e-12:
        raise ValueError('Deployed matrix exceeds the production basis error budgets')
    args.output.write_text(json.dumps(dict(
        basis_lower_bounds=lower.tolist(), negative_weight_budget=negative,
        unit_sum_power_residual=unity, subdivision_depth=13,
        deployed_negative_weight_budget=6.5e-8, deployed_unit_sum_budget=1e-12), indent=2) + '\n')


if __name__ == '__main__':
    main()
