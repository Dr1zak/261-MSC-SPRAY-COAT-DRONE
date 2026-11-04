import argparse
import json
from .core import run_test_backend


def main():
    parser=argparse.ArgumentParser(description='Deterministic test backend; no UAV physics')
    parser.add_argument('--scenario',default='baseline')
    parser.add_argument('--method',choices=['original','nearest','2opt'],default='2opt')
    parser.add_argument('--seed',type=int)
    parser.add_argument('--output-root',default='reports')
    args=parser.parse_args()
    path=f'config/scenarios/{args.scenario}.yaml'
    out,result=run_test_backend(path,args.method,args.seed,args.output_root)
    print(json.dumps({'output':str(out),'completed':result['completed'],'failed':result['failed'],'blocked':len(result['blocked']),'terminal_reason':result['terminal_reason']}))


if __name__ == '__main__':
    main()
