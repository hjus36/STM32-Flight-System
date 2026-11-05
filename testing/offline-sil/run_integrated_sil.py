#!/usr/bin/env python3
"""Run real generated LKM_AW Core/Mixer + BNO parser + safety stack on HOST.

This program uses synthetic input samples; it does not compile/flash STM32,
operate hardware, or authorize axes/arming/motors. No changes under cubemx/.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime

ROOT=Path(__file__).resolve().parent
DEFAULT_PROJECT=ROOT.parents[1]/'firmware'/'stm32'/'LKM_BNO086_Bringup'
FILES=[
    'lkm_bno086_attitude.c',
    'lkm_imu_time_pair.c',
    'lkm_rp_feedback_gate.c',
    'lkm_rp_frame_contract.c',
    'lkm_fc100hz_core.c',
    'lkm_aw_core_preview.c',
    'LKM_AW_Core_F405.c',
    'LKM_AW_Ctrl_F405.c',
    'LKM_AW_Mixer_F405.c',
    'const_params.c',
    'rt_nonfinite.c',
    'lkm_fc_actuator_guard.c',
    'lkm_fc_virtual_mixer.c',
]
HEADERS=[
    'lkm_bno086_attitude.h', 'lkm_bno086_timestamp.h',
    'lkm_icm20602_gyro.h', 'lkm_imu_time_pair.h',
    'lkm_rp_feedback_gate.h', 'lkm_rp_frame_contract.h',
    'lkm_fc100hz_core.h', 'lkm_aw_core_preview.h',
    'LKM_AW_Core_F405.h', 'LKM_AW_Ctrl_F405.h',
    'LKM_AW_Mixer_F405.h', 'lkm_fc_actuator_guard.h',
    'lkm_fc_virtual_mixer.h',
]
CASES=(
    'baseline','receiver_loss','receiver_recovery','battery_low','imu_stale',
    'timestamp_skew','duplicate_sample','scheduler_overrun','axis_unverified',
    'low_bno_accuracy','nan_target','nan_mixer','emergency_stop',
    'receiver_heartbeat_stale','battery_heartbeat_stale'
)
FAULTS={
    'receiver_loss':'RECEIVER','receiver_recovery':'RECEIVER','battery_low':'BATTERY',
    'imu_stale':'SENSOR_OR_AXES','timestamp_skew':'SENSOR_OR_AXES',
    'duplicate_sample':'SENSOR_OR_AXES','scheduler_overrun':'SCHEDULER',
    'axis_unverified':'SENSOR_OR_AXES','low_bno_accuracy':'SENSOR_OR_AXES',
    'nan_target':'CONTROLLER','nan_mixer':'COMMAND','emergency_stop':'ESTOP',
    'receiver_heartbeat_stale':'HEARTBEAT','battery_heartbeat_stale':'HEARTBEAT',
}

def sha(path:Path)->str:
    h=hashlib.sha256()
    with path.open('rb') as f:
        while block:=f.read(1024*128):h.update(block)
    return h.hexdigest()

def compiler_search()->Path|None:
    for name in ('gcc','clang'):
        if exe:=shutil.which(name):
            if 'arm-none-eabi' not in Path(exe).name.lower():return Path(exe)
    if os.name=='nt':
        base=Path(os.environ.get('ProgramData','C:/ProgramData'))/'MATLAB'/'SupportPackages'
        if base.is_dir():
            for ver in sorted(base.glob('R*'),reverse=True):
                p=ver/'3P.instrset'/'mingw_w64.instrset'/'bin'/'gcc.exe'
                if p.is_file():return p
        for path in ('C:/msys64/ucrt64/bin/gcc.exe', 'C:/msys64/mingw64/bin/gcc.exe',
                     'C:/Program Files/LLVM/bin/clang.exe'):
            if Path(path).is_file():return Path(path)
    return None

def invoke(args:list,timeout:int=180)->tuple[int,str]:
    try:
        p=subprocess.run([str(x) for x in args],cwd=ROOT,
            stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
            timeout=timeout,check=False)
        return p.returncode,p.stdout.decode('utf-8',errors='replace')
    except (OSError,subprocess.TimeoutExpired) as e:
        return 125,str(e)

def check_csv(path:Path)->dict:
    cases={name:[] for name in CASES}
    with path.open(encoding='utf-8',newline='') as f:
        rdr=csv.DictReader(f)
        required=['scenario','t_ms','t_s','injection','virtual_valid',
                  'esc_enabled','controller_ready','actuator_state','actuator_result',
                  'motor1_model_n','motor2_model_n','motor3_model_n','motor4_model_n']
        if not all(x in (rdr.fieldnames or []) for x in required):
            raise AssertionError('CSV schema incomplete')
        for r in rdr:
            if r['scenario'] not in cases:raise AssertionError('Unrecognized scenario')
            cases[r['scenario']].append(r)
    if any(not rows for rows in cases.values()):raise AssertionError('Scenario missing')
    summary={}
    for name,rows in cases.items():
        virtual=0
        last_ms=0
        first_fail=None
        for idx,r in enumerate(rows):
            t=int(r['t_ms'])
            if t-last_ms!=10:raise AssertionError(f'{name}: unexpected time step {t}')
            last_ms=t
            for key in ('esc_enabled','controller_ready'):
                if r[key]!='0':raise AssertionError(f'{name}: {key} accidentally enabled')
            valid=r['virtual_valid']=='1'
            if valid:
                if int(r['actuator_state'])!=2:
                    raise AssertionError(f'{name}: virtual output outside ARMED_VIRTUAL')
                for k in range(1,5):
                    thrust=float(r[f'motor{k}_model_n'])
                    if not 0<=thrust<=2.00001:raise AssertionError(f'{name}: invalid virtual thrust')
                virtual+=1
            elif any(abs(float(r[f'motor{k}_model_n']))>1e-9 for k in range(1,5)):
                raise AssertionError(f'{name}: invalid frame emitted stale thrust')
            if name!='baseline' and int(r['actuator_state'])==3 and first_fail is None:
                first_fail=r
            if name!='baseline' and 120<=idx<220 and valid:
                raise AssertionError(f'{name}: virtual thrust after first fail')
        if name=='baseline':
            if len(rows)!=1002 or virtual!=1000 or first_fail is not None:
                raise AssertionError('Baseline must produce 1000 real generated-model host steps')
        else:
            if not first_fail or first_fail['t_ms']!='1210' or first_fail['actuator_result']!=FAULTS[name]:
                raise AssertionError(f'{name}: bad fault latch or first cause: {first_fail}')
            if name!='receiver_recovery' and rows[-1]['actuator_state']!='3':
                raise AssertionError(f'{name}: automatic rearm after latch')
            if name=='receiver_recovery':
                if rows[-1]['actuator_state']!='2' or virtual<=120:
                    raise AssertionError('Receiver explicit reset and re-arm unsuccessful')
        summary[name]={
            'samples':len(rows),'virtual_frames':virtual,
            'first_failsafe_ms':int(first_fail['t_ms']) if first_fail else None,
            'first_failsafe_reason':first_fail['actuator_result'] if first_fail else None,
            'final_state':int(rows[-1]['actuator_state']),
        }
    return summary


def build_and_run(compiler:Path,project:Path,out_dir:Path,plots:bool=True)->int:
    missing=[name for name in FILES if not (project/'Core'/'Src'/name).is_file()]
    missing+= [name for name in HEADERS if not (project/'Core'/'Inc'/name).is_file()]
    if missing:
        print('PRECHECK_FAIL: missing installed project dependencies:',', '.join(missing))
        print('Place this package NEXT TO cubemx and finish previous 1-5 installs first.')
        return 2
    cp=project/'.cproject'
    if cp.is_file():
        content=cp.read_text(encoding='utf-8',errors='replace')
        for forbidden in ('LKM_FC_ACTUATOR_HOST_FIXTURE','LKM_RP_FRAME_HOST_FIXTURE','LKM_SIL_HOST_ONLY'):
            if forbidden in content:
                print(f'PRECHECK_FAIL: production .cproject contains HOST fixture macro {forbidden}')
                return 2
    # Read-only provenance check: compare against the exact reviewed modules.
    expected=json.loads((ROOT/'EXPECTED_SOURCE_SHA256.json').read_text(encoding='utf-8'))
    changed=[rel for rel,digest in expected.items() if
             sha(project/rel)!=digest]
    if changed:
        print('PRECHECK_FAIL: prerequisite sources have changed from reviewed V1:')
        for x in changed:print('  SOURCE_CHANGED:',x)
        print('No compilation, project modification or graphs attempted.')
        return 2
    included=project/'Core'/'Inc'
    sources=[project/'Core'/'Src'/n for n in FILES]
    common=['-std=c11','-O0','-Wall','-Wextra','-I',included]
    fixture=['-DLKM_SIL_HOST_ONLY=1','-DLKM_RP_FRAME_HOST_FIXTURE=1',
             '-DLKM_FC_ACTUATOR_HOST_FIXTURE=1']
    out_dir.mkdir(parents=True,exist_ok=True)
    results=out_dir/'trace_all_scenarios.csv'
    with tempfile.TemporaryDirectory(prefix='lkm_sil_host_') as tmp:
        exe=Path(tmp)/('integrated_sil.exe' if os.name=='nt' else 'integrated_sil')
        cmd=[compiler,*common,*fixture,ROOT/'tests'/'integrated_sil.c',*sources,
             '-lm','-o',exe]
        rc,log=invoke(cmd,180)
        (out_dir/'host_build.log').write_text(log,encoding='utf-8')
        print('HOST COMPILE:', 'PASS' if rc==0 else 'FAIL')
        if rc:
            print(log[-14000:]);return 1
        rc,output=invoke([exe,results],180)
        (out_dir/'host_execution.log').write_text(output,encoding='utf-8')
        print(output[-9000:])
        if rc:
            print('INTEGRATED SIL: FAIL in actual host C regression');return 1
        matched=re.search(r'INTEGRATED HOST SIL:\s*(\d+)/(\d+) CHECKS PASS;\s*(\d+) SCENARIOS',output)
        if not matched or matched.group(1)!=matched.group(2) or int(matched.group(3))!=len(CASES):
            print('INTEGRATED SIL: FAIL (incomplete test counts)');return 1
        try:summary=check_csv(results)
        except (AssertionError,KeyError,ValueError) as e:
            print('CSV ANALYSIS FAIL:',e);return 1
        print('TRACE VERIFICATION: PASS | baseline 1000 virtual steps; 14 fault scenarios')
        production=Path(tmp)/('production_lock.exe' if os.name=='nt' else 'production_lock')
        rc,log=invoke([compiler,*common,ROOT/'tests'/'production_locked.c',
                 project/'Core'/'Src'/'lkm_fc_actuator_guard.c',
                 project/'Core'/'Src'/'lkm_rp_frame_contract.c',
                 project/'Core'/'Src'/'lkm_rp_feedback_gate.c',
                 '-lm','-o',production],90)
        if rc:
            print('PRODUCTION LOCK COMPILE: FAIL',log[-6000:]);return 1
        rc,plog=invoke([production],30)
        (out_dir/'production_lock.log').write_text(plog,encoding='utf-8')
        if rc or not re.search(r'PRODUCTION HARD LOCK:\s*\d+/(\d+) PASS',plog):
            print('PRODUCTION LOCK TEST FAIL',plog);return 1
        print(plog.strip())
        for mod,filename,macro,err_text in (
            ('actuator','lkm_fc_actuator_guard.c','LKM_FC_ACTUATOR_HOST_FIXTURE','forbidden'),
            ('frame','lkm_rp_frame_contract.c','LKM_RP_FRAME_HOST_FIXTURE','forbidden')):
            bad_obj=Path(tmp)/(mod+'.o')
            rc,log=invoke([compiler,'-std=c11','-I',included,'-DSTM32F405xx=1',
                   f'-D{macro}=1','-c',project/'Core'/'Src'/filename,
                   '-o',bad_obj],30)
            if rc==0 or err_text not in log.lower():
                print('NEGATIVE COMPILE GATE FAILED:',mod,log[-2500:]);return 1
            print(f'STM32 macro guard negative compile: PASS | {mod} (host compiler with STM32F405xx define)')
        source_hashes={n:sha(project/'Core'/'Src'/n) for n in FILES}
        (out_dir/'summary.json').write_text(json.dumps({
            'scope':'HOST_SYNTHETIC_SIL_NOT_REAL_HARDWARE',
            'C_checks_passed':int(matched.group(1)),
            'scenario_count':len(CASES),
            'production_lock':'PASS',
            'macro_compile_rejection':'PASS',
            'source_hashes_sha256':source_hashes,
            'trace_sha256':sha(results),
            'scenarios':summary},ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    figure_files=[]
    if plots:
        try:
            from tools.plot_sil import render
            figure_files=render(results,out_dir/'figures')
            print('GITHUB FIGURES:',len(figure_files),'PNG charts created')
        except (RuntimeError,ImportError) as e:
            print('PLOTS NOT CREATED:',e)
            print('CSV and summary succeeded; run python -m pip install matplotlib and tools/plot_sil.py')
    report=['# LKM_FC_V1 통합 SIL·Fail-safe 검증 보고서',
            '', '> **HOST SYNTHETIC SIL ONLY. 실제 비행·MCU 실행·센서 동기화·모터 시험 결과가 아닙니다.**','',
            f'- 컴파일러: `{compiler}`',
            f'- 실제 기존 생성 코드 및 안전 모듈: `{len(FILES)}`개 C 원본 직접 컴파일',
            f'- C 내부 검증: **{matched.group(1)}/{matched.group(2)} PASS**',
            '- 100Hz 합성 정상 시나리오: 1,000개 virtual thrust 프레임',
            '- Fail-safe 합성 시나리오: 14개',
            '- 생산용 잠금 테스트 및 호스트 매크로 ARM 컴파일 차단: PASS',
            '- UART/ESC/PWM/모터 하드웨어 호출: 없음',
            '- 시간 정합: 호스트 수신 시각 비교일 뿐 물리적 측정시각 동기화가 아님',
            '- 기체 모의 플랜트 J=0.025 kg·m², D=0.11 N·m·s/rad: **임의의 합성 가정**, 실제 QAV210 파라미터 아님',
            '', '## 시나리오 결과', '',
            '| 시나리오 | 가상 출력 프레임 | 첫 Fail-safe (ms) | 원인 | 종료 상태 |',
            '|---|---:|---:|---|---:|']
    for k,v in summary.items():
        report.append(f'| `{k}` | {v["virtual_frames"]} | {v["first_failsafe_ms"] or "—"} | {v["first_failsafe_reason"] or "—"} | {v["final_state"]} |')
    report+=['','## 그래프','','그래프는 C 프로그램의 `trace_all_scenarios.csv`를 읽어 자동 생성했습니다.','']
    for p in figure_files:report.append(f'- `{p.name}`')
    report+=['','## 핵심 한계',
       '- Simulink Academic License 생성 코드의 배포 조건은 사용자가 직접 확인해야 합니다.',
       '- `virtual_valid`는 실제 모터 아밍 권한이 아닙니다. `esc_enabled` 및 `controller_ready`는 전 구간 0입니다.',
       '- 센서 바이트, 타임스탬프, RC, 배터리, 타이밍, 모의 플랜트 모두 합성 입력입니다.',
       '- MCU의 ISR 지터·SPI 소요시간·실제 장착축·실제 ESC·비행 안정성은 검증되지 않았습니다.',
       '- 실물 시험은 별도 안전 절차로 진행해야 하며 이 자료만으로 비행 승인할 수 없습니다.',
       '']
    (out_dir/'VALIDATION_REPORT.md').write_text('\n'.join(report),encoding='utf-8')
    print('=== INTEGRATED SIL OFFLINE: PASS ===')
    print('Evidence:',out_dir)
    print('GitHub charts:',out_dir/'figures')
    print('No CubeMX/IOC/main/PWM/ESC files were changed.')
    return 0


def main()->int:
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project',type=Path,default=DEFAULT_PROJECT)
    p.add_argument('--compiler',type=Path)
    p.add_argument('--out',type=Path,help='Output directory; no CubeMX writes')
    p.add_argument('--no-plots',action='store_true')
    args=p.parse_args()
    compiler=args.compiler or compiler_search()
    if not compiler:
        print('HOST_COMPILER_NOT_FOUND: install GCC/Clang or MATLAB MinGW-w64 GCC');return 2
    proj=args.project.resolve()
    out=args.out.resolve() if args.out else (ROOT.parents[1]/'.local'/'sil-runs'/
               datetime.now().strftime('sil_v1_%Y%m%d_%H%M%S'))
    if out==proj or proj in out.parents:
        print('PRECHECK_FAIL: evidence output must NOT be inside the CubeMX project')
        return 2
    if out.exists() and any(out.iterdir()):
        print('PRECHECK_FAIL: output directory is not empty; refusing to overwrite evidence:',out)
        return 2
    print('=== LKM_FC_V1 INTEGRATED SIL + GITHUB VISUALIZATION ===')
    print('CubeMX:',proj)
    print('Compiler:',compiler)
    rc,ver=invoke([compiler,'--version'],10)
    if rc:print('HOST_COMPILER_ERROR:',ver);return 2
    print('Version:',ver.splitlines()[0] if ver else '?')
    return build_and_run(compiler,proj,out,not args.no_plots)

if __name__=='__main__':
    sys.exit(main())
