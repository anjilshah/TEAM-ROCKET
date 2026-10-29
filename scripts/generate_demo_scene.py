from peg_sorting.simulation.scene_manager import build_scene
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    print(build_scene(args.seed))
