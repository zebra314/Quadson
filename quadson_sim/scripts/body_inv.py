import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.sim_setup import *
from src.quadson import *

def main():
  setup_bullet()
  setup_env()
  quadson = Quadson()
  run_simulation()

if __name__ == "__main__":
  main()
