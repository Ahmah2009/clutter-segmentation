#!/usr/bin/env python

import poserandomization as pr
import os

def main():
    p = os.getenv("CLUTSEG_PATH")
    orig_dir = os.path.join(p, "tod_kinect_train_15")
    noisy_dir = os.path.join(p, "tod_kinect_train_15_noisy")
    test_dir = os.path.join(p, "tod_kinect_test_100")
    testdesc_file = os.path.join(p, "tod_kinect_test_100", "testdesc.txt")
    mode = "1"
    init_cfg = pr.InitConfig(
        orig_dir,
        noisy_dir,
        test_dir,
        testdesc_file,
        mode)
    noise_t_cfg = pr.ParamConfig(os.path.join(p, "clutter-segmentation", "misc", "noise_t.txt"))
    noise_r_cfg = pr.ParamConfig(os.path.join(p, "clutter-segmentation", "misc", "noise_r.txt"))
    noise_rt_cfg = pr.ParamConfig(os.path.join(p, "clutter-segmentation", "misc", "noise_rt.txt"))
    pr.experiment(init_cfg, noise_t_cfg)
    pr.experiment(init_cfg, noise_r_cfg)
    pr.experiment(init_cfg, noise_rt_cfg)

if __name__ == "__main__":
    main()

