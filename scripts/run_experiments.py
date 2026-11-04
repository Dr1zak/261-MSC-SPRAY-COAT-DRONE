#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path
from construction_autonomy.core import run_test_backend

parser=argparse.ArgumentParser(description='Algorithm-only deterministic test experiments')
parser.add_argument('--scenarios',nargs='+',default=['baseline'])
parser.add_argument('--seeds',nargs='+',type=int,default=[1])
parser.add_argument('--methods',nargs='+',choices=['original','nearest','2opt'],default=['original','nearest','2opt'])
parser.add_argument('--output-root',default='reports')
args=parser.parse_args()
summary=[]
for scenario in args.scenarios:
    for seed in args.seeds:
        for method in args.methods:
            try:
                out,result=run_test_backend(f'config/scenarios/{scenario}.yaml',method,seed,args.output_root)
                summary.append({'scenario':scenario,'seed':seed,'method':method,'status':'passed','report':str(out/'report.json'),'completed':result['completed'],'failed':result['failed'],'blocked':len(result['blocked']),'route_m':result['planned_route_m']})
            except Exception as exc:
                summary.append({'scenario':scenario,'seed':seed,'method':method,'status':'failed','reason':str(exc)})
out=Path(args.output_root); out.mkdir(exist_ok=True)
(out/'experiment_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
with (out/'experiment_summary.csv').open('w',newline='') as handle:
    fields=['scenario','seed','method','status','report','completed','failed','blocked','route_m','reason']
    writer=csv.DictWriter(handle,fieldnames=fields); writer.writeheader(); writer.writerows(summary)
print(json.dumps({'runs':len(summary),'failed':sum(x['status']=='failed' for x in summary)}))
