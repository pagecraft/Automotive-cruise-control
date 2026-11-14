"""Optional plotting utility: run the CLI first, then run this script."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt

folder=Path('results')
fig,axs=plt.subplots(3,1,figsize=(10,9),sharex=True)
for name in ('pi','preview'):
    path=folder/f'{name}.csv'
    if not path.exists():
        raise SystemExit(f'Missing {path}. Run python -m energy_cruise.cli first.')
    with path.open() as f: rows=list(csv.DictReader(f))
    t=[float(r['time_s']) for r in rows]
    axs[0].plot(t,[float(r['speed_m_s'])*3.6 for r in rows],label=name)
    axs[1].plot(t,[float(r['applied_force_n']) for r in rows],label=name)
    axs[2].plot(t,[float(r['battery_kwh']) for r in rows],label=name)
axs[0].axhline(100,color='gray',linestyle='--',label='target')
for ax,ylabel in zip(axs,('Speed [km/h]','Wheel force [N]','Battery [kWh]')):
    ax.set_ylabel(ylabel);ax.grid(True);ax.legend()
axs[-1].set_xlabel('Time [s]')
fig.tight_layout();folder.mkdir(exist_ok=True);fig.savefig(folder/'comparison.png',dpi=160)
print('Saved results/comparison.png')
