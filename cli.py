"""Command-line reproducible experiment runner."""
import argparse
import csv
import json
from pathlib import Path
from .controllers import PIController,PreviewController
from .simulation import Scenario,run_simulation
from .metrics import summarize


def main(argv=None):
    p=argparse.ArgumentParser(description='Energy-aware highway cruise-control benchmark')
    p.add_argument('--controller',choices=['pi','preview','both'],default='both')
    p.add_argument('--duration',type=float,default=150.0)
    p.add_argument('--dt',type=float,default=0.05)
    p.add_argument('--lookahead',type=float,default=100.0)
    p.add_argument('--target-kmh',type=float,default=100.0)
    p.add_argument('--initial-kmh',type=float,default=100.0)
    p.add_argument('--preview-bias',type=float,default=0.0)
    p.add_argument('--mass-scale',type=float,default=1.0)
    p.add_argument('--headwind',type=float,default=0.0)
    p.add_argument('--output',type=Path,default=Path('results'))
    args=p.parse_args(argv)
    scenario=Scenario(duration_s=args.duration,dt_s=args.dt,lookahead_m=args.lookahead,preview_bias=args.preview_bias,mass_scale=args.mass_scale,headwind_m_s=args.headwind,target_speed_m_s=args.target_kmh/3.6,initial_speed_m_s=args.initial_kmh/3.6)
    args.output.mkdir(parents=True,exist_ok=True)
    choices=['pi','preview'] if args.controller=='both' else [args.controller]
    for name in choices:
        ctrl=PIController() if name=='pi' else PreviewController()
        rows=run_simulation(ctrl,scenario)
        metrics=summarize(rows,args.dt)
        with (args.output/f'{name}.csv').open('w',newline='',encoding='utf8') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0].__dataclass_fields__))
            writer.writeheader();writer.writerows([vars(row) for row in rows])
        (args.output/f'{name}_metrics.json').write_text(json.dumps(metrics,indent=2),encoding='utf8')
        print(name,json.dumps(metrics,indent=2))

if __name__=='__main__':
    main()
