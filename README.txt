icbs: (bug)
python run_experiments.py --instance instances\exp.txt --solver CBS

cg:
python run_experiments.py --instance instances\exp.txt --solver CBS --h 1

cg: (disjoint)
python run_experiments.py --instance instances\exp.txt --solver CBS --h 1 --disjoint

dg:
python run_experiments.py --instance instances\exp.txt --solver CBS --h 2