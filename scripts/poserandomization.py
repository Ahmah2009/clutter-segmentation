#!/usr/bin/env python
#

"""This experiment shall investigate the relation between the accuracy of pose
estimations and the recognition results. The assumption is that noise in the
pose estimations will worsen the recognition results. How much negative impact
noiaw in the pose estimations has on the recognition results shall be verified
by experiment.

Two classifiers are trained, one on the original training set and one on the
training set where noise has been artificially added to the pose estimates.
The result of the experiment is then given as a two confusion matrices, or in
other terms as two points in ROC space, each point representing one classifier.
We expect the original classifier to perform more or less equally for each run,
and different runs might as well give an idea how much randomness governs the
recognition process (it uses RANSAC!).  Basically, we count true and false
positives of the respective classifiers on a common testing set. The experiment
therefore collects the two input parameters and the members of the two
confusion matrices:

stddev_t, stddev_r, orig_tp, orig_fp, orig_fn, orig_tn, noisy_tp, noisy_fp, noisy_fn, noisy_tn

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

Every run on the two tod kinect test dataset takes about 2 minutes to complete,
with a training base of 9 objects (100 runs were completed in 200 minutes on an
intel i5 quad-core processor). Constraining the experiment to complete within 2
hours, we can try about 60 different parameter combinations. Notably,
evaluating the classifiers on the testing sets does not take much time, and can
be neglected. Therefore, to improve results of an experiment it is best to
increase the number of test images. We might even just duplicate the test
images in order to average over the different results of the RANSAC matching
process.
"""

import os
import tempfile
import gzip 
import shutil
import subprocess
import ConfigParser
import optparse

log = open(os.path.join(os.getenv("HOME"), ".poserandomization.log"), "w")

class InitConfig:

    def __init__(self, orig_dir, noisy_dir, test_dir, testdesc_file, mode):
        self.orig_dir = orig_dir
        self.noisy_dir = noisy_dir
        self.test_dir = test_dir
        self.testdesc_file = testdesc_file
        self.mode = mode
        # read in list of objects in training base
        config_file = open(os.path.join(orig_dir, "config.txt"))
        self.subjects = set() 
        for line in config_file: 
            line = line.strip()
            if line != "":
                self.subjects.add(line)


class ParamConfig:

    def __init__(self, param_file):
        self.param_set = []
        f = open(param_file)
        for line in f:
            sd = line.strip().split(" ")
            self.param_set.append((float(sd[0]), float(sd[1])))
        f.close()


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
        yamlf = open(yaml_filename)
        yaml = yamlf.read()
        yamlf.close()
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
    """Copy over all original pose files and f3d archives"""
    for s in init_cfg.subjects:
        print >>log, "Resetting %s" % s
        sdir = os.path.join(init_cfg.orig_dir, s)
        for f in os.listdir(sdir):
            if f.endswith(".pose.yaml"):
                shutil.copyfile(
                    os.path.join(init_cfg.orig_dir, s, f),   
                    os.path.join(init_cfg.noisy_dir, s, f))
            elif f.endswith(".f3d.yaml.gz"):
                shutil.copyfile(
                    os.path.join(init_cfg.orig_dir, s, f),   
                    os.path.join(init_cfg.noisy_dir, s, f))

def randomize(init_cfg, stddev_t, stddev_r):
    for s in init_cfg.subjects:
        # Delegate to poserandomizer.cpp
        print >>log, "Randomizing %s" % s

        p = subprocess.Popen(
            ("rosrun",
             "clutseg_util",
             "poserandomizer",
             os.path.join(init_cfg.noisy_dir, s),
             str(stddev_t),
             str(stddev_r),
             "1", "0"), stdout=subprocess.PIPE, stderr=subprocess.PIPE)  
        p.communicate()
        sdir = os.path.join(init_cfg.noisy_dir, s)
        n = ""
        for f in os.listdir(sdir):
            if f.endswith(".pose.yaml"):
                n = f[:-10]
                # Now read randomized pose and make sure it also appears in the 3d tar
                # gz files. Man, I hate duplication ... I know that features 2d files
                # are not read, so just delete them to make sure there will not be any
                # contradiction in pose data.
                f2d_fn = os.path.join(sdir, n + ".features.yaml.gz")
                if os.path.exists(f2d_fn):
                    os.remove(f2d_fn)
                # extract f3d archives to temporary folder
                a_oldfn = os.path.join(sdir, n + ".f3d.yaml.gz")
                print >>log, "Generating updated %-100s" % a_oldfn
                a = gzip.open(a_oldfn, "rb")
                yaml = a.read()
                a.close()
                # read yaml file
                pose = PoseRT()
                pose.read(os.path.join(sdir, f))
                # inject randomize pose
                yaml = replace_data(yaml, "tvec", pose.tvec)
                yaml = replace_data(yaml, "rvec", pose.rvec)
                # re-compress archive 
                (fd, a_newfn) = tempfile.mkstemp(suffix=".f3d.yaml.gz")
                # a_newfn = os.path.join(dtemp, n + ".f3d.yaml.gz")
                g = open(a_newfn, "w")
                a_new = gzip.GzipFile(n + ".f3d.yaml", fileobj=g, mode="wb")
                a_new.write(yaml)
                a_new.close()
                g.close()
                shutil.copyfile(a_newfn, a_oldfn)
                # prevent leaks of file descriptors, will soon run out in
                # about second or third run!
                os.close(fd)
                os.remove(a_newfn)
                # shutil.rmtree(dtemp)

def replace_data(yaml, vec, new):
    offs = yaml.find(vec + ":")
    s = yaml.find("[", offs)
    e = yaml.find("]", s)
    return yaml[:s+1] + str(new[0]) + ", " + str(new[1]) + ", " + str(new[2]) + yaml[e:]
   
def blackbox_recognizer(init_cfg, base_dir, stats_file):
    (h, tmpdevnull) = tempfile.mkstemp()
    p = subprocess.Popen(
        ("rosrun",
         "clutseg_util",
         "blackbox_recognizer",
         "--base=%s" % base_dir,
         "--tod_config=%s" % os.path.join(base_dir, "config.yaml"),
        "--image=%s" % init_cfg.test_dir,
        "--testdesc=%s" % init_cfg.testdesc_file,
        "--log=%s" % tmpdevnull,
        "--mode=%s" % init_cfg.mode,
        "--verbose=%d" % 0,
        "--stats=%s" % stats_file), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    p.communicate()
    os.remove(tmpdevnull)

def evaluate(init_cfg, stddev_t, stddev_r):
    # run blackbox_recognizer
    h, orig_stats_file = tempfile.mkstemp()
    g, noisy_stats_file = tempfile.mkstemp()
    print >>log, "Evaluating original classifier"
    blackbox_recognizer(init_cfg, init_cfg.orig_dir, orig_stats_file)
    print >>log, "Evaluating noisy classifier"
    blackbox_recognizer(init_cfg, init_cfg.noisy_dir, noisy_stats_file)
    # read in result files
    orig_res = ConfigParser.ConfigParser() 
    noisy_res = ConfigParser.ConfigParser() 
    orig_res.read(orig_stats_file)
    noisy_res.read(noisy_stats_file)
    os.close(h)
    os.close(g)
    if (orig_res.has_option("statistics", "tp") and
        orig_res.has_option("statistics", "fp") and
        orig_res.has_option("statistics", "fn") and
        orig_res.has_option("statistics", "tn") and
        noisy_res.has_option("statistics", "tp") and
        noisy_res.has_option("statistics", "fp") and
        noisy_res.has_option("statistics", "fn") and
        noisy_res.has_option("statistics", "tn")):
        # write row
        print "%10.6f %10.6f %10d %10d %10d %10d %10d %10d %10d %10d" % (
            stddev_t, stddev_r,
            float(orig_res.get("statistics", "tp")),
            float(orig_res.get("statistics", "fp")),
            float(orig_res.get("statistics", "fn")),
            float(orig_res.get("statistics", "tn")),
            float(noisy_res.get("statistics", "tp")),
            float(noisy_res.get("statistics", "fp")),
            float(noisy_res.get("statistics", "fn")),
            float(noisy_res.get("statistics", "tn")))
    else:
        # fail-soft
        print >>log, "ERROR: Could not read statistics"
        print "%10.6f %10.6f %10s %10s %10s %10s %10s %10s %10s %10s" % (
            stddev_t, stddev_r, "-", "-", "-", "-", "-", "-", "-", "-")
    os.remove(orig_stats_file)
    os.remove(noisy_stats_file)
    
def run(init_cfg, stddev_t, stddev_r):
    reset(init_cfg)
    randomize(init_cfg, stddev_t, stddev_r)
    evaluate(init_cfg, stddev_t, stddev_r)
   
def experiment(init_cfg, param_cfg):
    print "%10s %10s %10s %10s %10s %10s %10s %10s %10s %10s" % (
        "stddev_t", "stddev_r",
        "orig_tp", "orig_fp",
        "orig_fn", "orig_tn",
        "noisy_tp", "noisy_fp",
        "noisy_fn", "noisy_tn")
    for stddev_t, stddev_r in param_cfg.param_set:
        print >>log, "New experiment run with stddev_t=%10.6f and stddev_r=%10.6f" % (stddev_t, stddev_r)
        run(init_cfg, stddev_t, stddev_r) 
    log.flush() 
 
def main():
    option_parser = optparse.OptionParser("poserandomization [OPTIONS]")
    option_parser.add_option("-o", "--orig", dest="orig_dir")
    option_parser.add_option("-n", "--noisy", dest="noisy_dir")
    option_parser.add_option("-t", "--test", dest="test_dir")
    option_parser.add_option("-d", "--testdesc", dest="testdesc_file")
    option_parser.add_option("-m", "--mode", dest="recognizer mode")
    option_parser.add_option("-p", "--params", dest="param_file")
    options, args = option_parser.parse_args()

    init_cfg = InitConfig(
        options.orig_dir,
        options.noisy_dir,
        options.test_dir,
        options.testdesc_file,
        options.mode)
    param_cfg = ParamConfig(options.param_file) 
    experiment(init_cfg, param_cfg)
    log.close()
         
if __name__ == "__main__":
    main()

