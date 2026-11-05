"""Regression checks that corrupted CSVs cannot be promoted as SIL success."""
from __future__ import annotations
import csv
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest
import sys
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('lkm_sil',ROOT/'run_integrated_sil.py')
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
DATA=ROOT/'results'/'sil_v1_20261106_040855'/'trace_all_scenarios.csv'

def load():
    with DATA.open(newline='',encoding='utf-8') as f:
        r=csv.DictReader(f);return list(r.fieldnames),list(r)

def assert_bad(rows,headers):
    with tempfile.TemporaryDirectory() as t:
        dest=Path(t)/'case.csv'
        with dest.open('w',newline='',encoding='utf-8') as f:
            w=csv.DictWriter(f,fieldnames=headers);w.writeheader();w.writerows(rows)
        try: mod.check_csv(dest)
        except (AssertionError,KeyError,ValueError):return True
        return False

class Safety(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.headers,cls.rows=load()
    def test_original_csv_valid(self):
        self.assertEqual(len(mod.check_csv(DATA)),15)
    def test_motor_enable_detected(self):
        rows=[dict(r) for r in self.rows];rows[100]['esc_enabled']='1'
        self.assertTrue(assert_bad(rows,self.headers))
    def test_controller_ready_detected(self):
        rows=[dict(r) for r in self.rows];rows[100]['controller_ready']='1'
        self.assertTrue(assert_bad(rows,self.headers))
    def test_invalid_frame_thrust_detected(self):
        rows=[dict(r) for r in self.rows];rows[0]['motor1_model_n']='1.0'
        self.assertTrue(assert_bad(rows,self.headers))
    def test_wrong_fault_timestamp_detected(self):
        rows=[dict(r) for r in self.rows]
        idx=next(i for i,r in enumerate(rows) if r['scenario']=='receiver_loss' and r['t_ms']=='1210')
        rows[idx]['actuator_state']='2'
        self.assertTrue(assert_bad(rows,self.headers))
    def test_fault_rearm_detected(self):
        rows=[dict(r) for r in self.rows]
        idx=next(i for i,r in enumerate(rows) if r['scenario']=='battery_low' and r['t_ms']=='2000')
        rows[idx]['virtual_valid']='1'
        self.assertTrue(assert_bad(rows,self.headers))
    def test_missing_scenario_detected(self):
        rows=[r for r in self.rows if r['scenario']!='nan_mixer']
        self.assertTrue(assert_bad(rows,self.headers))
    def test_extra_scenario_detected(self):
        rows=[dict(r) for r in self.rows];rows[20]['scenario']='unknown'
        self.assertTrue(assert_bad(rows,self.headers))
    def test_wrong_sample_period_detected(self):
        rows=[dict(r) for r in self.rows];rows[20]['t_ms']='9999'
        self.assertTrue(assert_bad(rows,self.headers))
    def test_invalid_virtual_state_detected(self):
        rows=[dict(r) for r in self.rows]
        idx=next(i for i,r in enumerate(rows) if r['scenario']=='baseline' and r['virtual_valid']=='1')
        rows[idx]['actuator_state']='0'
        self.assertTrue(assert_bad(rows,self.headers))

if __name__=='__main__':unittest.main(verbosity=2)
