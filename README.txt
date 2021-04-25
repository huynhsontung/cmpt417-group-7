icbs: (bug)
python run_experiments.py --instance instances\exp.txt

cg:
python run_experiments.py --instance instances\exp.txt --h 1

cg: (disjoint)
python run_experiments.py --instance instances\exp.txt --h 1 --disjoint

dg:
python run_experiments.py --instance instances\exp.txt --h 2

wdg:
python run_experiments.py --instance instances\exp.txt --h 3