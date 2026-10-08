from pathlib import Path

def count_rs_lines(dir: Path):
    n_lines = 0
    for ob in dir.rglob('*.rs'):
        if ob.is_file():
            with ob.open('r', encoding='utf-8') as f:
                n_lines += sum(1 for _ in f)
    return n_lines


here=Path(__file__).parent.resolve()/Path("src")
print(count_rs_lines(here))



