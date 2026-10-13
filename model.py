"""Routes, fair evaluation, and a discretized resource-constrained optimizer."""
import math
from physics import SPEEDS, Vehicle, transition, steady_optimum

STOP_SECONDS = 4.
TIME_QUANTUM = .5


def route(name, label, length, peak1, peak2, limit):
    n = round(length / 25)
    if n % 4:
        raise ValueError('경로 길이는 100m의 배수여야 합니다.')
    knots = [0., peak1, 8., peak2, 0.]
    heights = []
    for i in range(n + 1):
        u = i / n * 4
        j = min(3, int(u))
        heights.append(knots[j] + (knots[j+1]-knots[j])*(u-j))
    return {'id': name, 'label': label, 'length': length,
            's': [i*25 for i in range(n+1)], 'h': heights,
            'limits': [limit/3.6]*n, 'stops': [n//2], 'stop_seconds': STOP_SECONDS,
            'description': {'ridge': '짧지만 가파른 언덕', 'river': '완만한 강변 우회로',
                            'city': '낮은 제한속도의 도심길', 'custom': '직접 설계한 경사'}[name]}


def routes(custom=None):
    if custom is not None and not isinstance(custom, dict):
        raise ValueError('사용자 경로 설정은 JSON 객체여야 합니다.')
    result = [route('ridge', '언덕 직행', 1000, 38, 32, 60),
              route('river', '강변 우회', 1400, 12, 10, 60),
              route('city', '도심 연결', 1200, 19, 16, 40)]
    if custom:
        length, p1, p2, limit = [float(custom.get(k, d)) for k,d in
                                [('length',1200),('peak1',25),('peak2',20),('limit',50)]]
        if not all(math.isfinite(v) for v in (length,p1,p2,limit)):
            raise ValueError('경로에는 유한한 수를 입력하세요.')
        if length % 100 or not 800 <= length <= 2000 or not -20 <= p1 <= 50 or not -20 <= p2 <= 50 or not 30 <= limit <= 70:
            raise ValueError('사용자 경로: 길이 800~2000m(100m 단위), 높이 −20~50m, 제한속도 30~70km/h')
        result.append(route('custom','나의 설계',int(length),p1,p2,limit))
    return result


def get_route(key, custom=None):
    for r in routes(custom):
        if r['id'] == key:
            return r
    raise ValueError('존재하지 않는 경로입니다.')


def edge(r, i, v0, v1, car):
    return transition(r['s'][i+1]-r['s'][i], r['h'][i+1]-r['h'][i],
                      v0, v1, car, r['limits'][i])


def speed_options(r):
    n = len(r['s'])-1
    options = []
    for i in range(n+1):
        if i in [0,n] + r['stops']:
            options.append([0.])
        else:
            cap = min(r['limits'][i-1],r['limits'][i])
            options.append([v for v in SPEEDS if 0 < v <= cap])
    return options


def graph(r, car):
    """Backwards reachable edges ensure the manual driver can always finish."""
    opts = speed_options(r)
    n = len(opts)-1
    edges = [{} for _ in range(n)]
    reachable = {0.}
    for i in range(n-1,-1,-1):
        previous = set()
        for v0 in opts[i]:
            outgoing = []
            for v1 in opts[i+1]:
                if v1 in reachable:
                    e = edge(r,i,v0,v1,car)
                    if e is not None:
                        outgoing.append((v1,e))
            if outgoing:
                edges[i][v0] = outgoing
                previous.add(v0)
        reachable = previous
    return edges


def evaluate(r, speeds, car, deadline):
    n = len(r['s'])-1
    if len(speeds) != n+1 or any(not math.isfinite(v) or v < 0 for v in speeds):
        raise ValueError('전체 경로의 유효한 속도 기록이 필요합니다.')
    reasons = []
    if speeds[0] != 0 or speeds[-1] != 0:
        reasons.append('출발·도착 정지 조건 위반')
    if any(speeds[i] != 0 for i in r['stops']):
        reasons.append('필수 정차 누락')
    totals = {key:0. for key in ['draw','recovered','auxiliary','drag','rolling','grade','inertia','friction']}
    t = energy = 0.
    samples = [{'t':0.,'s':0.,'h':r['h'][0],'v':speeds[0],'a':0.,'power':0.,'energy':0.,'angle':0.}]
    for i in range(n):
        e = edge(r,i,speeds[i],speeds[i+1],car)
        if e is None:
            raise ValueError(f'{i+1}번 구간에서 제한속도·가감속·출력 조건을 위반했습니다.')
        t += e['dt']
        energy += e['energy']
        for key in totals:
            totals[key] += e[key]
        samples.append({'t':t,'s':r['s'][i+1],'h':r['h'][i+1],'v':speeds[i+1],
                        'a':e['a'],'power':e['power'],'energy':energy,'angle':e['grade_angle']})
        if i+1 in r['stops'] and speeds[i+1] == 0:
            t += STOP_SECONDS
            idle = car.auxiliary*STOP_SECONDS/3600
            energy += idle
            totals['auxiliary'] += idle
            samples.append({**samples[-1], 't':t,'a':0.,'power':car.auxiliary,'energy':energy})
    if t > deadline + 1e-8:
        reasons.append('도착 제한시간 초과')
    return {'route_id':r['id'],'label':r['label'],'speeds':speeds,'samples':samples,
            'time':t,'energy':energy,'wh_km':energy/(r['length']/1000), 'totals':totals,
            'valid':not reasons,'reasons':reasons,'distance':r['length']}


def optimize(r, car, deadline):
    """DP over position, speed and conservatively rounded elapsed time.

    At each (speed, ceil-time) state retain least energy. This is optimal for this
    finite graph and rounded clock, NOT a proof of continuous global optimality.
    """
    edges = graph(r,car)
    budget = math.floor(deadline/TIME_QUANTUM+1e-9)
    # Value: (energy Wh, previous (v,tick)). Store layers for reconstruction.
    layers = [{(0.,0):(0.,None)}]
    for i, connections in enumerate(edges):
        current = {}
        dwell = STOP_SECONDS if i+1 in r['stops'] else 0.
        idle = car.auxiliary*dwell/3600
        for (v0,tick),(energy,_) in layers[-1].items():
            for v1,e in connections.get(v0,[]):
                nt = tick + math.ceil((e['dt']+dwell)/TIME_QUANTUM-1e-9)
                if nt > budget:
                    continue
                key = (v1,nt)
                ne = energy+e['energy']+idle
                if key not in current or ne < current[key][0]:
                    current[key] = (ne,(v0,tick))
        if not current:
            return None
        layers.append(current)
    key = min(layers[-1],key=lambda key:layers[-1][key][0])
    conservative_time = key[1]*TIME_QUANTUM
    path = []
    for layer in reversed(layers):
        path.append(key[0])
        key = layer[key][1]
    result = evaluate(r,list(reversed(path)),car,deadline)
    result['conservative_time'] = conservative_time
    result['method'] = '거리 25m · 속도 2m/s · 시간 0.5s 보수적 격자 DP'
    return result


def manual_step(r, car, index, v0, target, connections=None):
    if index < 0 or index >= len(r['s'])-1 or v0 not in SPEEDS:
        raise ValueError('주행 단계 또는 속도가 올바르지 않습니다.')
    edges = connections if connections is not None else graph(r,car)
    choices = edges[index].get(v0,[])
    if not choices:
        raise ValueError('현재 조건에서 진행 가능한 운행이 없습니다.')
    v1,e = min(choices,key=lambda p:(abs(p[0]-target),p[1]['energy']))
    return {'v':v1,**e,'s':r['s'][index+1], 'h':r['h'][index+1],
            'dwell':STOP_SECONDS if index+1 in r['stops'] else 0.,
            'assisted':abs(v1-target)>1.01}


def preset(r,car,style,deadline):
    edges=graph(r,car)
    speeds=[0.]
    for i in range(len(edges)):
        if style=='smooth':
            target=10.
        elif style=='fast':
            target=20.
        elif style=='wave':
            target=16. if (i//4)%2==0 else 4.
        else:
            raise ValueError('알 수 없는 주행 예시입니다.')
        speeds.append(manual_step(r,car,i,speeds[-1],target,edges)['v'])
    return evaluate(r,speeds,car,deadline)


def compare(user, results):
    feasible=[r for r in results if r and r['valid']]
    if not feasible:
        return {'best':None,'saving':None,'feedback':['현재 시간 제약에서 격자 해를 찾지 못했습니다. 도착시간을 늘려 보세요.']}
    best=min(feasible,key=lambda r:r['energy'])
    if not user['valid']:
        return {'best':best['route_id'],'saving':None,'feedback':['비교 조건 미충족: '+', '.join(user['reasons'])+'. 시간 조건을 만족한 후 절감률을 비교하세요.']}
    same=next((r for r in feasible if r['route_id']==user['route_id']),None)
    feedback=[]
    if best['route_id']!=user['route_id']:
        feedback.append(f"전체 후보 중 {best['label']}의 계산 에너지가 가장 낮습니다. 짧은 거리만으로 효율을 판단할 수는 없습니다.")
    if same:
        diff=user['energy']-same['energy']
        feedback.append(f'같은 경로의 계산 운행과 비교한 에너지 차이는 {diff:+.1f} Wh입니다. 경로 선택 효과와 속도 조절 효과를 나누어 관찰하세요.')
    if user['totals']['recovered']>0:
        feedback.append(f"회생제동으로 {user['totals']['recovered']:.1f} Wh를 회수했습니다. 변환 손실 때문에 가속한 에너지를 전부 회수할 수는 없습니다.")
    feedback.append('그래프의 속도 기울기는 가속도, 전력 곡선 아래의 부호 있는 넓이는 배터리 에너지입니다.')
    if user['energy']<best['energy']:
        feedback.append('내 운행이 격자 기준보다 낮습니다. 시간 반올림과 속도 격자 때문에 계산 기준은 연속 최적해를 보장하지 않습니다.')
    saving=(user['energy']-best['energy'])/user['energy']*100 if user['energy']>1e-6 else None
    return {'best':best['route_id'],'saving':saving,'same_route_energy':same['energy'] if same else None,'feedback':feedback}
