/**
 * Author: Julius Adorf
 */

#include "clutseg/clutseg.h"
#include "clutseg/experiment.h"
#include "clutseg/storage.h"

#include <sqlite3.h>

namespace clutseg {

    class ExperimentRunner {

        public:

            ExperimentRunner();
            ExperimentRunner(sqlite3* db, const TrainFeaturesCache & cache,
                                const ResultStorage & storage);

            /** Runs experiments, using configurations provided by a database.  After
             * an experiment has been run, the response is saved into field 'response', the
             * time will be recorded and after running the experiment 'has_run' is set to
             * false and the experiment will be serialized to the database. Running an
             * experiment might take a lot of time, depending on the given test set and the
             * choice of algorithms and parameters. This function takes interruptions into
             * account. */
            void run();

            bool terminate;

        private:

            void runExperiment(Clutsegmenter & segmenter, Experiment & exp);

            sqlite3* db_;
            TrainFeaturesCache cache_;
            ResultStorage storage_;

    };

    boost::filesystem::path cloudPath(const boost::filesystem::path & img_path);

}

