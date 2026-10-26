# SPDX-License-Identifier: Apache-2.0
import pathlib
import sys
import unittest
from types import SimpleNamespace
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from integrations.donor_adapters import from_hapticsight,assurance_telemetry_contract
class Quality:
    def __init__(self,age,health='NOMINAL'):
        self.sample_timestamp_utc_s=100-age
        self.health=health
        self.frame='wrist'
    def is_usable(self,now_utc_s=None):return self.health!='INVALID'
    def age_ms(self,now_utc_s=None):return (now_utc_s-self.sample_timestamp_utc_s)*1000
class IntegrationTests(unittest.TestCase):
    def sample(self,age=0.01,health='NOMINAL'):
        return SimpleNamespace(quality=Quality(age,health),frame='wrist',
             force=SimpleNamespace(x=0.,y=0.,z=-9.8),torque=SimpleNamespace(x=0.,y=0.,z=0.))
    def test_reject_stale(self):
        with self.assertRaisesRegex(ValueError,'stale'):from_hapticsight(self.sample(.2),(0,0,-9.8),100)
    def test_reject_invalid(self):
        with self.assertRaisesRegex(ValueError,'unhealthy'):from_hapticsight(self.sample(.01,'INVALID'),(0,0,-9.8),100)
    def test_explicit_not_static(self):
        r=from_hapticsight(self.sample(),(0,0,-9.8),100)
        self.assertEqual(len(r),13)
        self.assertEqual(r[-2],0.0)
    def test_assurance_provenance(self):
        a=assurance_telemetry_contract('SIMULATOR','wrist',1.5,'estimated',21)
        self.assertFalse(a['validated_on_hardware'])
    def test_assurance_rejects_missing_frame(self):
        with self.assertRaises(ValueError):assurance_telemetry_contract('SIMULATOR','',1,'invalid',0)
if __name__=='__main__':unittest.main()
