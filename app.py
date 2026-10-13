"""Dependency-free local Python server for EV Calculus Lab."""
import argparse
from functools import lru_cache
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
import json
import math
from urllib.parse import urlparse
from physics import Vehicle, steady_optimum
from model import routes, get_route, optimize, evaluate, manual_step, preset, compare

ROOT=Path(__file__).resolve().parent


@lru_cache(maxsize=24)
def optimal_cached(car, custom_json, deadline):
    custom=json.loads(custom_json)
    return [optimize(r,car,deadline) for r in routes(custom)]


def dispatch(path, data):
    car=Vehicle.parse(data.get('vehicle'))
    deadline=float(data.get('deadline',220))
    if not math.isfinite(deadline) or not 80 <= deadline <= 480:
        raise ValueError('도착 제한시간은 80~480초입니다.')
    custom=data.get('custom')
    all_routes=routes(custom)
    if path=='/api/config':
        return {'routes':all_routes,'vehicle':car.json(),'steady_speed':steady_optimum(car)*3.6}
    r=get_route(data.get('route','ridge'),custom)
    if path=='/api/step':
        index=int(data['index'])
        v=float(data['v'])
        target=float(data['target'])
        if not all(math.isfinite(x) for x in (v,target)) or not 0<=target<=25:
            raise ValueError('목표 속도는 0~25m/s 범위입니다.')
        return manual_step(r,car,index,v,target)
    if path=='/api/preset':
        return preset(r,car,data.get('style','wave'),deadline)
    if path=='/api/compare':
        speeds=[float(v) for v in data.get('speeds',[])]
        user=evaluate(r,speeds,car,deadline)
        results=optimal_cached(car,json.dumps(custom,sort_keys=True),deadline)
        return {'user':user,'optimal':results,'comparison':compare(user,results)}
    if path=='/api/optimize':
        return {'optimal':optimal_cached(car,json.dumps(custom,sort_keys=True),deadline)}
    raise ValueError('지원하지 않는 API입니다.')


class Handler(SimpleHTTPRequestHandler):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,directory=str(ROOT/'static'),**kwargs)

    def end_headers(self):
        self.send_header('X-Content-Type-Options','nosniff')
        self.send_header('Cache-Control','no-store')
        super().end_headers()

    def do_POST(self):
        try:
            # Local-only API: reject cross-origin browser calls and oversized input.
            origin=self.headers.get('Origin')
            if origin and urlparse(origin).netloc!=self.headers.get('Host'):
                self.send_error(403)
                return
            size=int(self.headers.get('Content-Length',0))
            if not 0<size<=100000:
                raise ValueError('요청 크기가 올바르지 않습니다.')
            data=json.loads(self.rfile.read(size))
            if not isinstance(data,dict):
                raise ValueError('JSON 객체가 필요합니다.')
            result=dispatch(urlparse(self.path).path,data)
            status=200
        except (ValueError,TypeError,KeyError,OverflowError) as exc:
            result={'error':str(exc)}
            status=400
        content=json.dumps(result,ensure_ascii=False,allow_nan=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type','application/json; charset=utf-8')
        self.send_header('Content-Length',str(len(content)))
        self.end_headers()
        self.wfile.write(content)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description='EV Calculus Lab')
    parser.add_argument('--port',type=int,default=8000)
    args=parser.parse_args()
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    print(f'EV Calculus Lab: http://127.0.0.1:{args.port}',flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
