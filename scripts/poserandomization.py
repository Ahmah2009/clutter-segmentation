#!/usr/bin/env python
#

"""This experiment shall investigate the relation between the accuracy of pose
estimations and the recognition results. The assumption is that noise in the
pose estimations will worsen the recognition results. How much negative impact
noiaw in the pose estimations has on the recognition results shall be verified
by experiment.

Two classifiers are trained, one on the original training set and one on the
training set where noise has been artificially added to the pose estimates.
The result of the experiment is then given as a confusion matrix, or in other
terms as a point in ROC space. Basically, we count true and false positives of
the respective classifiers on a common testing set. The experiment therefore
collects the following data:

stddev_t, stddev_r, orig_tp, orig_fp, noisy_tp, noisy_fp

The terminology follows the Wikipedia article about receiver operating
characteristics, which in its turn is based on "An introduction to ROC
analysis" (Fawcett, 2006)

As no attempt has been made to find out the distribution of noise in the
original pose estimations, a Gaussian noise model with two parameters stddev_t
and stddev_r is assumed. Let t be the original translation vector, and n_t ~
N(0, stddev_t), n_r ~ N(0, stddev_r) two normal distributed random variables.
Then the noisy translation vector is t' = t + n_t, using additive noise.  Let r
be the original rotation vector in axis-angle-form, then the noisy orientation
is r' = r + n_r. This approach seems reasonable, given the goals of the
experiment.

Since we have two input parameters, the question is which parts of the
parameter space shall be explored. Interesting will be small values for
the standard deviation, combinations in which either rotation or translation
remains unchanged and combinations with high standard deviation that show
that pose estimations are fundamental for tod_detecting to work.

The experiment has the following setup:
- an original training base (remains unchanged)
- a noisy training base (working copy)
Before running the experiment using this script, it is expected that both
training bases have been properly prepared. Both training bases have to
be completely built by tod_training train_all.sh.

This experiment runner tries to minimize execution time and avoids rebuilding
the training bases after pose randomization.  Except for masking, none of the
training stages actually uses pose estimations, it is just stored in the tar.gz
feature files. Therefore, for pose randomization it is sufficient to extract
recreate those archive files.

Every run on tod kinect test dataset takes about 20 seconds to complete, when
using blackbox_recognizer, that tests multiple images in one process.
Preparing the training base for the next run should be a matter of seconds.
Constraining the experiment to complete within 20 minutes, we can try about 
50 different parameter combinations.
"""

import os
import tempfile
import tarfile
import shutil
import subprocess
import ConfigParser

class InitConfig:

    def __init__(self, orig_dir, noisy_dir, test_dir, testdesc_file):
        self.orig_dir = orig_dir
        self.noisy_dir = noisy_dir
        self.test_dir = test_dir
        self.testdesc_file = testdesc_file
        # read in list of objects in training base
        config_file = open(os.path.join(orig_dir, "config.txt"))
        self.subjects = set() 
        for line in config_file: 
            line = line.strip()
            if line != "":
                subjects.add(line)


class ParamConfig:

    def __init__(self):
        self.param_set = ((0, 0), (1, 1), (2, 2)) 


class Result:

    def __init__(self, stddev_t, stddev_r, tp, fp, tn, fn):
        self.stddev_t = stddev_t
        self.stddev_r = stddev_r
        self.tp = tp
        self.fp = fp
        self.tn = tn
        self.fn = fn


class PoseRT:

    def __init__(self):
        rvec = [0, 0, 0]
        tvec = [0, 0, 0]

    def read(self, yaml_filename):
        """This method just greps the rvec and tvec data out of a YAML file. It
           does not really understand YAML format, it just looks for keywords."""
        yaml = open(yaml_filename).read()
        tvec_offs = yaml.find("tvec:")
        rvec_offs = yaml.find("rvec:")
        tvec_data_offs = yaml.find("data:")
        rvec_data_offs = yaml.rfind("data:")
        if rvec_offs < tvec_offs:
            tmp = tvec_data_offs 
            tvec_data_offs = rvec_data_offs
            rvec_data_offs = tmp
        self.tvec = self._extract_data(yaml, tvec_data_offs)
        self.rvec = self._extract_data(yaml, rvec_data_offs)

    def _extract_data(self, yaml, data_offs):
        s = yaml.find("[", data_offs)
        e = yaml.find("]", data_offs)
        sv = yaml[s+1:e-1].split(",")
        nv = [float(s) for s in sv]
        return nv

def reset(init_cfg):
    """Copy over all original pose files"""
    for s in init_cfg.subjects:
        sdir = os.path.join(init_cfg.orig_dir, s)
        for f in os.listdir(sdir):
            if f.endswith(".pose.yaml"):
                shutil.copyfile(
                    os.path.join(init_cfg.orig_dir, s, f),   
                    os.path.join(init_cfg.noisy_dir, s, f))

def randomize(init_cfg, stddev_t, stddev_r):
    for s in init_cfg.subjects:
        # Delegate to poserandomizer.cpp
        p = subprocess.Popen(
            ("rosrun",
             "semantic3d_training",
             "poserandomizer",
             os.path.join(init_cfg.noisy_dir, s),
             str(stddev_t),
             str(stddev_r),
             1, 0))  
        p.communicate()
        sdir = os.path.join(init_cfg.noisy_dir, s)
        for f in os.listdir(sdir):
            if f.endswith(".pose.yaml"):
               n = f[:-10]
            # Now read randomized pose and make sure it also appears in the 3d tar
            # gz files. Man, I hate duplication ... I know that features 2d files
            # are not read, so just delete them to make sure there will not be any
            # contradiction in pose data.
            os.remove(os.path.join(sdir, n + ".features.yaml.gz"))
            # extract f3d archives to temporary folder
            dtemp = tempfile.mkdtemp()
            a_oldfn = os.path.join(sdir, n + ".f3d.yaml.gz")
            a = tarfile.open(a_oldfn, n + ".f3d.yaml.gz", "r"))
            a.extractall(dtemp)
            a.close()
            # read yaml file
            pose = PoseRT()
            pose.read(os.path.join(sdir, f))
            # modify yaml files in temporary folders 
            yaml_oldfn = os.path.join(dtemp, n + ".f3d.yaml")
            yaml = open(yaml_oldfn, "r").read()
            yaml_newfn = os.path.join(dtemp, n + ".f3d.yaml.new")
            yaml = _replace_data(yaml, "tvec", pose.tvec)
            yaml = _replace_data(yaml, "rvec", pose.rvec)
            f = open(yaml_newfn, "w")
            f.write(yaml)
            f.close() 
            shutil.copyfile(yaml_newfn, yaml_oldfn)
            os.remove(yaml_newfn)
            # re-compress archive 
            a_newfn = os.path.join(dtemp, n + ".f3d.yaml.gz.new")
            a_new = tarfile.open(a_newfn, "w")
            a_new.add(os.path.join(dtemp. n + ".f3d.yaml"))
            a_new.close()
            shutil.copyfile(a_newfn, a_oldfn)
            shutil.rmtree(tempd)

def replace_data(yaml, vec, new):
    offs = yaml.find(vec + ":")
    s = yaml.find("[", offs)
    e = yaml.find("]", s)
    return yaml[:s+1] + str(new[0]) + ", " + str(new[1]) + ", " + str(new[2]) + yaml[e:]
   
def blackbox_recognizer(init_cfg, base_dir, stats_file):
    subprocess.Popen(
        ("rosrun",
         "cseg_util",
         "blackbox_recognizer",
         "--base=%s" % base_dir,
         "--tod_config=%s" % os.path.join(base_dir, "config.yaml"),
        "--image=%s" % init_cfg.test_dir,
        "--testdesc=%s" % init_cfg.tesdesc_file,
        "--log=%s" % tmpdevnull,
        "--stats=%s" % stats_file)

def evaluate(init_cfg, stddev_t, stddev_r):
    # run blackbox_recognizer
    orig_stats_file = tempfile.mkstemp()
    noisy_stats_file = tempfile.mkstemp()
    blackbox_recognizer(init_cfg, init_cfg.orig_dir, orig_stats_file)
    blackbox_recognizer(init_cfg, init_cfg.noisy_dir, noisy_stats_file)
    # read in result files
    orig_res = ConfigParser.ConfigParser() 
    noisy_res = ConfigParser.ConfigParser() 
    orig_res.read(orig_stats_file)
    noisy_res.read(orig_stats_file)
    # write to some experiment-scope result scope file
    # TO STANDARD OUT!!
    print "%10.6d %10.6d %10.3d %10.3d %10.3d %10.3d" % (
        stddev_t, stddev_r,
        orig_res.get("statistics", "tp"),
        orig_res.get("statistics", "fp"),
        noisy_res.get("statistics", "tp"),
        noisy_res.get("statistics", "fp"))
    os.remove(orig_stats_file)
    os.remove(noisy_stats_file)
    
def run(init_cfg, stddev_t, stddev_r):
    reset(init_cfg)
    randomize(init_cfg, stddev_t, stddev_r)
    evaluate(init_cfg, stddev_t, stddev_r)
    
def main():
    init_cfg = InitConfig(
        "/home/julius/Studium/BA/poserandomization/undistorted",
        "/home/julius/Studium/BA/poserandomization/noisy",
        "/home/julius/Studium/BA/tod_kinect_test",
        "/home/julius/Studium/BA/tod_kinect_test/test-truth.txt")
    param_cfg = param_config() 
    for stddev_t, stddev_r in param_cfg.param_set:
        # run(init_cfg, stddev_t, stddev_r) 
        pass
         
if __name__ == "__main__":
    main()

