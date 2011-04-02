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

import optparse

def test_configuration_20110331():
    return ((0, 0), (1, 1), (2, 2)) 

def randomize(stddev_t, stddev_r):
    pass

def train():
    pass

def evaluate():
    pass

def run(stddev_t, stddev_r):
    randomize(stddev_t, stddev_r)
    train()
    evaluate()
    
def main():
    configurations = test_configurations_20110331()
    for stddev_t, stddev_r in configurations:
        run(stddev_t, stddev_r) 
         
if __name__ == "__main__":
    main()

