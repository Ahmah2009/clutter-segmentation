/**
 * Author: Julius Adorf
 */

#include "clutseg/experiment.h"

#include <sqlite3.h>

namespace clutseg {

    class ExperimentRunner {

        public:

            ExperimentRunner(sqlite3* db, const TrainFeaturesCache & cache);

            /** Runs experiments, using configurations provided by a database.  After
             * an experiment has been run, the response is saved into field 'response', the
             * time will be recorded and after running the experiment 'has_run' is set to
             * false and the experiment will be serialized to the database. Running an
             * experiment might take a lot of time, depending on the given test set and the
             * choice of algorithms and parameters. This function takes interruptions into
             * account. */
            void run();

        private:

            sqlite3* db_;
            TrainFeaturesCache cache_;

    };

}

