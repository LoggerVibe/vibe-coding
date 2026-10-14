"""Check the offline JavaScript engine against the original Python model."""
import json
from pathlib import Path
import shutil
import subprocess
import unittest
from app import dispatch


@unittest.skipUnless(shutil.which('node'), 'Node.js is needed for browser engine parity')
class BrowserEngineTests(unittest.TestCase):
    def test_physics_and_optimizer_match_python(self):
        cases = []
        for vehicle in [{}, {'wind': 15, 'mass': 3000}, {'wind': -10, 'regen': 0}, {'auxiliary': 0}]:
            for deadline in [80, 220, 480]:
                data = {'vehicle': vehicle, 'deadline': deadline, 'route': 'ridge', 'style': 'wave'}
                user = dispatch('/api/preset', data)
                result = dispatch('/api/compare', {**data, 'speeds': user['speeds']})
                cases.append({'input': data, 'result': result})
        script = """
const fs=require('fs'), engine=require('./static/engine.js');
for(const c of JSON.parse(fs.readFileSync(0,'utf8'))){
 const user=engine.dispatch('preset',c.input);
 const result=engine.dispatch('compare',{...c.input,speeds:user.speeds});
 for(const [a,b] of [[c.result.user,result.user],...c.result.optimal.map((a,i)=>[a,result.optimal[i]])]){
  if(!!a!==!!b)throw Error('Feasibility mismatch');
  if(a && (Math.abs(a.energy-b.energy)>1e-7 || Math.abs(a.time-b.time)>1e-7 || a.valid!==b.valid))throw Error('Numerical mismatch');
 }
}
"""
        subprocess.run(['node', '-e', script], input=json.dumps(cases), text=True,
                       cwd=Path(__file__).resolve().parents[1], check=True, capture_output=True)
