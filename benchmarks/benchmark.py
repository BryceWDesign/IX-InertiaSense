#!/usr/bin/env python3
"""Deterministic held-out synthetic payload-wrench benchmark (stdlib only).

Physics: quasi-static wrench [F=m*g, tau=(m*r)xg] in a wrist frame.
Neither contacts nor robot dynamics are simulated. Baseline is classical OLS
with the same measurements. Disturbance conditions are fixed before scoring.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
import random
import statistics
import subprocess
import tempfile
from pathlib import Path

GRAVITY = 9.80665


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def vecnorm(a):
    return math.sqrt(sum(v*v for v in a))


def physics_rows(g):
    x, y, z = g
    return [(x, 0, 0, 0), (y, 0, 0, 0), (z, 0, 0, 0),
            (0, 0, z, -y), (0, -z, 0, x), (0, y, -x, 0)]


def solve4(a, b):
    a = [row[:] for row in a]
    b = b[:]
    for i in range(4):
        p = max(range(i, 4), key=lambda k: abs(a[k][i]))
        if abs(a[p][i]) < 1e-10:
            return None
        a[p], a[i] = a[i], a[p]
        b[p], b[i] = b[i], b[p]
        div = a[i][i]
        for j in range(i, 4):
            a[i][j] /= div
        b[i] /= div
        for k in range(4):
            if k == i: continue
            d = a[k][i]
            for j in range(i, 4): a[k][j] -= d*a[i][j]
            b[k] -= d*b[i]
    return b


def ordinary_least_squares(rows):
    h = [[0.0]*4 for _ in range(4)]
    b = [0.0]*4
    for row in rows:
        g = row[1:4]
        y = row[4:10]
        for k, coeff in enumerate(physics_rows(g)):
            weight = 1 if k < 3 else 16
            for i in range(4):
                b[i] += weight*coeff[i]*y[k]
                for j in range(4): h[i][j] += weight*coeff[i]*coeff[j]
    x = solve4(h, b)
    if x is None or x[0] <= .01:
        return None
    return {'mass_kg': x[0], 'com_sensor_m': [v/x[0] for v in x[1:]]}


def generate(morphology, disturbance, seed, n=80):
    rng = random.Random(seed)
    # Different gripper tilting capability, payload distributions, sensor noise.
    max_tilt = .38 if morphology == 'compact_arm' else .62
    mass = rng.uniform(.5, 5.5)
    com = tuple(rng.uniform(-.19, .19) for _ in range(3))
    rows = []
    for i in range(n):
        phi = i*2.399963 + rng.gauss(0, .12)
        a = max_tilt*(.65+.35*rng.random())
        g0 = (math.sin(phi)*a, math.cos(phi)*a, -1.)
        g = tuple(v*GRAVITY/vecnorm(g0) for v in g0)
        f = [mass*v for v in g]
        torque = list(cross([mass*v for v in com],g))
        fn = .11 if disturbance == 'clean' else .27
        tn = .015 if disturbance == 'clean' else .04
        f = [v+rng.gauss(0, fn) for v in f]
        torque = [v+rng.gauss(0, tn) for v in torque]
        if disturbance == 'impulsive_outliers' and rng.random() < .09:
            # Untagged corrupted but superficially healthy readings.
            axis=rng.randrange(3)
            f[axis]+=rng.choice([-1, 1])*rng.uniform(9, 24)
            torque[rng.randrange(3)]+=rng.choice([-1, 1])*rng.uniform(2, 6)
        rows.append([i*.01,*g,*f,*torque,1.,1,1])
    return mass, com, rows


def run_cpp(binary, rows):
    with tempfile.TemporaryDirectory() as td:
        fn=Path(td)/'episode.csv'
        with fn.open('w', newline='') as f:
            writer=csv.writer(f)
            writer.writerow('time,gx,gy,gz,fx,fy,fz,tx,ty,tz,quality,static,healthy'.split(','))
            writer.writerows(rows)
        process=subprocess.run([str(binary), 'fit', str(fn)], capture_output=True,text=True,check=False)
        if process.returncode not in (0, 3):
            raise RuntimeError(f'CLI error {process.returncode}: {process.stderr}')
        return json.loads(process.stdout)


def score(estimate, mass, com):
    if not estimate or not estimate.get('observable', True):
        return None
    return {'mass_abs_kg': abs(estimate['mass_kg']-mass),
            'com_error_m': vecnorm([a-b for a,b in zip(estimate['com_sensor_m'],com)])}


def benchmark(binary, repeats=40):
    cases=[]
    for morph in ['compact_arm', 'extended_arm']:
        for disturbance in ['clean', 'sensor_noise', 'impulsive_outliers']:
            cpp_metrics=[];ols_metrics=[];abstentions=0
            for idx in range(repeats):
                seed=(7919*idx+104729*(1+['compact_arm','extended_arm'].index(morph))+733*(1+['clean','sensor_noise','impulsive_outliers'].index(disturbance)))
                mass,com,rows=generate(morph,disturbance,seed)
                ours=run_cpp(binary,rows)
                reference=ordinary_least_squares(rows)
                c=score(ours,mass,com)
                b=score(reference,mass,com)
                if c is None:abstentions+=1
                else:cpp_metrics.append(c)
                if b is not None:ols_metrics.append(b)
            if not cpp_metrics or not ols_metrics:
                raise RuntimeError('no valid trial results')
            def med(data,k):return statistics.median(s[k] for s in data)
            cases.append({'morphology':morph,'disturbance':disturbance,'trials':repeats,'abstentions':abstentions,
              'ix_median_com_error_m':med(cpp_metrics,'com_error_m'),
              'ols_median_com_error_m':med(ols_metrics,'com_error_m'),
              'ix_median_mass_error_kg':med(cpp_metrics,'mass_abs_kg'),
              'ols_median_mass_error_kg':med(ols_metrics,'mass_abs_kg')})
    return {'scope':'synthetic_quasistatic_wrist_wrenches_only',
            'baseline':'weighted ordinary least squares with same measurements',
            'oracle_provided': 'gravity direction, labeled static wrench and calibrated tool tare',
            'cases':cases}


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--binary',required=True,type=Path)
    ap.add_argument('--repeats',type=int,default=40)
    ap.add_argument('--output',type=Path,default=Path('results/benchmark.json'))
    a=ap.parse_args()
    if a.repeats<1:ap.error('repeats must be positive')
    result=benchmark(a.binary.resolve(),a.repeats)
    a.output.parent.mkdir(exist_ok=True,parents=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    for c in result['cases']:
        print(f"{c['morphology']:14} {c['disturbance']:20} IX COM={c['ix_median_com_error_m']:.4f}m OLS COM={c['ols_median_com_error_m']:.4f}m IX mass={c['ix_median_mass_error_kg']:.3f}kg OLS mass={c['ols_median_mass_error_kg']:.3f}kg")

if __name__=='__main__':main()
