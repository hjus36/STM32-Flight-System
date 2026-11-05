#!/usr/bin/env python3
"""CSV-driven GitHub charts for LKM_FC_V1 HOST SYNTHETIC SIL ONLY.

Plots measurements emitted by the compiled C host fixture. Never substitute
these figures for real MCU/flight/sensor measurements.
"""
from __future__ import annotations
import argparse
import csv
from collections import defaultdict
from pathlib import Path


def load(path: Path) -> dict[str, list[dict[str,str]]]:
    cases: dict[str,list[dict[str,str]]] = defaultdict(list)
    with path.open(newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            cases[row['scenario']].append(row)
    return dict(cases)


def render(csv_path: Path, output_dir: Path) -> list[Path]:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError('matplotlib not installed; run: python -m pip install matplotlib') from exc
    cases = load(csv_path)
    for name in ('baseline','receiver_recovery','receiver_loss','timestamp_skew'):
        if name not in cases:
            raise ValueError(f'Missing required scenario: {name}')
    output_dir.mkdir(parents=True, exist_ok=True)
    saved=[]

    def draw(name, title, ylabel, configure, legend=True, ylim=None):
        fig, ax = plt.subplots(figsize=(11.2,4.5))
        configure(ax)
        ax.set(title=title, xlabel='Synthetic simulation time [s]', ylabel=ylabel)
        ax.grid(alpha=0.25)
        if ylim is not None:ax.set_ylim(*ylim)
        if legend:ax.legend(ncol=2, fontsize=8, loc='best')
        fig.text(0.5,0.01,'Synthetic Host SIL  |  Not Hardware or Flight Validation  |  No ESC/PWM output',
                 ha='center', fontsize=8)
        fig.tight_layout(rect=(0,0.035,1,1))
        f=output_dir/name
        fig.savefig(f,dpi=155)
        plt.close(fig)
        saved.append(f)

    def xy(s,key):
        return [float(r['t_s']) for r in s], [float(r[key]) for r in s]
    base=cases['baseline']
    def attitude(ax):
        for key,label,style in (
            ('target_roll_deg','Roll target','--'),('roll_deg','Roll response','-'),
            ('target_pitch_deg','Pitch target','--'),('pitch_deg','Pitch response','-')):
            t,v=xy(base,key)
            ax.plot(t,v,style,label=label,linewidth=1.5 if style=='-' else 1.1)
    draw('01_attitude_synthetic.png',
         'Roll/Pitch: generated PID + synthetic linear plant',
         'Angle [deg]',attitude)

    def motors(ax):
        for i in range(1,5):
            t,v=xy(base,f'motor{i}_model_n')
            ax.plot(t,v,label=f'Motor {i} (MODEL N)',linewidth=1.1)
    draw('02_virtual_mixer_thrust.png',
         'Generated Quad-X mixer: virtual motor MODEL thrust',
         'Model thrust [N] (not PWM or measured thrust)',motors,ylim=(-0.05,2.12))

    rec=cases['receiver_recovery']
    def states(ax):
        t,v=xy(rec,'actuator_state')
        ax.step(t,v,where='post',label='Host fixture guard state',linewidth=2)
        ax.axvline(1.21,linestyle='--',linewidth=1.4,label='Synthetic RC link lost')
        ax.axvline(2.21,linestyle=':',linewidth=1.3,label='Manual reset + LOW switch')
        ax.set_yticks([0,1,2,3],['Locked','Disarmed','Virtual armed','Failsafe latched'])
    draw('03_failsafe_recovery_states.png',
         'Fail-safe latch and explicit recovery (RC loss injection)',
         'Guard state (host only)',states)

    loss=cases['receiver_loss']
    def failthrust(ax):
        t=[float(x['t_s']) for x in loss]
        for i in range(1,5):
            ax.plot(t,[float(x[f'motor{i}_model_n']) for x in loss],
                    label=f'Motor {i} virtual N',linewidth=1.4)
        ax.axvline(1.21,linestyle='--',label='RC signal loss')
    draw('04_failsafe_output_zero.png',
         'Virtual output is cleared when receiver loss latches Fail-safe',
         'Model thrust [N] (not physical ESC command)',failthrust,ylim=(-0.08,2.1))

    skew=cases['timestamp_skew']
    def timestamps(ax):
        t, bno=xy(skew,'bno_age_ms')
        _, gyro=xy(skew,'icm_age_ms')
        _,signed=xy(skew,'host_skew_ms')
        ax.plot(t,bno,label='BNO host receipt age',linewidth=1.4)
        ax.plot(t,gyro,label='ICM host receipt age',linewidth=1.4)
        ax.plot(t,[abs(v) for v in signed],label='Absolute host receipt skew',linewidth=1.8)
        ax.axhline(15,linestyle='--',linewidth=1.2,label='Skew guard: 15 ms')
        ax.axvline(1.21,linestyle=':',label='Skew injection')
    draw('05_host_timestamp_skew.png',
         'Host timestamp proximity rejection (not hardware synchronization)',
         'Milliseconds [ms]',timestamps)

    # Plot all fault cases as a documented result matrix (first fault time).
    names=[x for x in cases if x!='baseline']
    fig, ax=plt.subplots(figsize=(11.3,5.4))
    failure_ms=[]
    for name in names:
        rows=cases[name]
        failing=next((int(x['t_ms']) for x in rows if x['actuator_state']=='3'),None)
        if failing is None:raise AssertionError(f'Expected failsafe absent: {name}')
        failure_ms.append(failing)
    ax.barh(names,[v/1000 for v in failure_ms],height=.7)
    ax.set(xlabel='First latched Fail-safe [synthetic s]',
           title='Deterministic fault-injection matrix: first latched Fail-safe')
    ax.grid(axis='x',alpha=.2)
    for i,sec in enumerate(failure_ms):
        ax.text(sec/1000+.012,i,f'{sec/1000:.2f}s',va='center',fontsize=8)
    ax.set_xlim(0,max(failure_ms)/1000+.25)
    fig.text(.5,.01,'Synthetic Host SIL  |  Not Hardware or Flight Validation  |  Deterministic mock inputs',
             ha='center',fontsize=8)
    fig.tight_layout(rect=(0,0.035,1,1))
    path=output_dir/'06_fault_injection_matrix.png'
    fig.savefig(path,dpi=155)
    plt.close(fig)
    saved.append(path)
    return saved

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv',type=Path)
    p.add_argument('--out',type=Path,default=Path('figures'))
    args=p.parse_args()
    for f in render(args.csv,args.out):print(f'FIGURE: {f}')
