#!/usr/bin/env python3
"""Independent deterministic synthetic mass-change scenarios.
Not full-body dynamics, sensor tare or live control.
"""
import argparse,csv,json,math,random,subprocess,tempfile
from pathlib import Path
from benchmark import GRAVITY, cross,vecnorm,ordinary_least_squares


def series(morph, seed):
    rng=random.Random(seed)
    gtilt=.38 if morph=='compact_arm' else .62
    oldmass=rng.uniform(.75,3.0)
    newmass=oldmass+rng.uniform(1.5,3.0)
    oldcom=tuple(rng.uniform(-.15,.15) for i in range(3))
    newcom=tuple(rng.uniform(-.15,.15) for i in range(3))
    rows=[]
    for i in range(180):
        theta=i*2.399963
        g0=(math.sin(theta)*gtilt,math.cos(theta)*gtilt,-1.)
        g=tuple(GRAVITY*x/vecnorm(g0) for x in g0)
        mass,com=(oldmass,oldcom) if i<90 else (newmass,newcom)
        f=[mass*x+rng.gauss(0,.17) for x in g]
        t=[x+rng.gauss(0,.02) for x in cross([mass*x for x in com],g)]
        rows.append([i*.01,*g,*f,*t,1.,1.,1.])
    return rows,newmass,newcom


def evaluate(executable,repeats=20):
    data=[]
    for morph in ('compact_arm','extended_arm'):
        detections=[]; lastmass=[];lastcom=[];misses=0
        for j in range(repeats):
            rows,m,c=series(morph, (1000 if morph=='compact_arm' else 2000)+j)
            with tempfile.TemporaryDirectory() as td:
                path=Path(td)/'change.csv'
                with path.open('w',newline='') as f:
                    w=csv.writer(f)
                    w.writerow(['time','gx','gy','gz','fx','fy','fz','tx','ty','tz','quality','static','healthy'])
                    w.writerows(rows)
                p=subprocess.run([str(executable),'track',str(path)],text=True,capture_output=True)
                if p.returncode: raise RuntimeError(f'bad CLI result: {p.stderr} {p.stdout}')
                out=json.loads(p.stdout)
                if not out['change_events'] or out['first_change_line']<92:misses+=1
                else:detections.append(out['first_change_line']-92)
                lastmass.append(abs(out['mass_kg']-m))
                lastcom.append(vecnorm([a-b for a,b in zip(out['com_sensor_m'],c)]))
        import statistics
        data.append({'morphology':morph,'trials':repeats,'missed_or_premature_events':misses,
            'median_samples_until_detection':statistics.median(detections) if detections else None,
            'median_final_mass_abs_error_kg':statistics.median(lastmass),
            'median_final_com_error_m':statistics.median(lastcom)})
    return {'scope':'synthetic_quasistatic_payload_replacement', 'change_sample_index':90,
            'adaptive_tracker_threshold_N':4.0,'required_consecutive_mismatches':5,
            'results':data}

if __name__=='__main__':
    a=argparse.ArgumentParser()
    a.add_argument('--binary',required=True,type=Path)
    a.add_argument('--output',type=Path,default=Path('results/change_benchmark.json'))
    a.add_argument('--repeats',type=int,default=20)
    p=a.parse_args()
    result=evaluate(p.binary.resolve(),p.repeats)
    p.output.parent.mkdir(parents=True,exist_ok=True)
    p.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
