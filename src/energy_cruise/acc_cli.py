import csv
from pathlib import Path
from .acc import simulate

def main():
    rows=simulate()
    path=Path("results/acc_baseline.csv")
    path.parent.mkdir(exist_ok=True,parents=True)
    with path.open("w",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print("ACC samples:",len(rows),"minimum gap:",round(min(x["gap_m"] for x in rows),2),"m")
    print("Saved:",path)

if __name__=="__main__":
    main()
