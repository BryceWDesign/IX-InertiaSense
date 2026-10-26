# SPDX-License-Identifier: Apache-2.0
import csv,json,pathlib,subprocess,tempfile,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
BINARIES=[ROOT/'build/inertiasense',ROOT/'build/Release/inertiasense.exe']
BINARY=next((b for b in BINARIES if b.is_file()),None)
@unittest.skipUnless(BINARY is not None,'CLI must be built first')
class CliTests(unittest.TestCase):
    def run_with_rows(self,rows,mode='fit'):
        with tempfile.TemporaryDirectory() as td:
            path=pathlib.Path(td)/'samples.csv'
            with path.open('w',newline='') as f:
                w=csv.writer(f)
                w.writerow(['time','gx','gy','gz','fx','fy','fz','tx','ty','tz','quality','static','healthy'])
                w.writerows(rows)
            p=subprocess.run([str(BINARY),mode,str(path)],capture_output=True,text=True)
            return p.returncode,json.loads(p.stdout)
    def test_refuse_unobservable(self):
        code,out=self.run_with_rows([[i*.01,0,0,-9.80665,0,0,-19.6133,0,0,0,1,1,1] for i in range(30)])
        self.assertEqual(code,3)
        self.assertFalse(out['observable'])
        self.assertEqual(out['status'],'unobservable_gravity_directions')
    def test_refuse_unhealthy_or_dynamic(self):
        code,out=self.run_with_rows([[i*.01,0,0,-9.80665,0,0,-19.6133,0,0,0,1,0,1] for i in range(12)])
        self.assertEqual(code,3)
        self.assertEqual(out['rejected'],12)
    def test_unknown_file_fails(self):
        p=subprocess.run([str(BINARY),'fit','no_such_file.csv'],capture_output=True,text=True)
        self.assertEqual(p.returncode,4)
if __name__=='__main__':unittest.main()
