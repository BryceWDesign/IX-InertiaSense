# SPDX-License-Identifier: Apache-2.0
"""Optional smoke tests against *real* user-provided donor source packages.
Set IX_DONORS_DIR to a folder containing source roots. Skipped if absent.
"""
from __future__ import annotations
import os,pathlib,sys,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
DONORS=pathlib.Path(os.environ.get('IX_DONORS_DIR','/invalid'))
HS=DONORS/'IX-HapticSight-main'/'src'
SD=DONORS/'SynapDrive-AI-main'
if HS.exists():sys.path.insert(0,str(HS))
if SD.exists():sys.path.insert(0,str(SD))
from integrations.donor_adapters import from_hapticsight,optional_synapdrive_drift
class TestDonorCompatibility(unittest.TestCase):
    @unittest.skipUnless(HS.exists(),'HapticSight original source unavailable')
    def test_haptic_sight_real_types(self):
        from ohip_interfaces.signal_health import SignalQuality, SignalHealth, SignalSourceMode
        from ohip_interfaces.force_torque import make_force_torque_sample
        s=make_force_torque_sample(frame='wrist',force_xyz=[0,0,-19.6133],torque_xyz=[0,.6,0],
           quality=SignalQuality(source_mode=SignalSourceMode.SIMULATION,health=SignalHealth.NOMINAL,
                                 sample_timestamp_utc_s=100,received_timestamp_utc_s=100,
                                 source_name='donor_test',frame='wrist'))
        row=from_hapticsight(s,(0,0,-9.80665),100.01)
        self.assertEqual(row[4:7],[0,0,-19.6133]);self.assertEqual(row[-2],0.)
    @unittest.skipUnless(SD.exists(),'SynapDrive original source unavailable')
    def test_synapdrive_real_drift(self):
        values=[[float(i)/8,float(i*i)/64] for i in range(-10,11)]
        nominal=optional_synapdrive_drift(values,[0,0])
        drift=optional_synapdrive_drift(values,[30,200])
        self.assertFalse(nominal.drifted)
        self.assertTrue(drift.drifted)
if __name__=='__main__': unittest.main()
