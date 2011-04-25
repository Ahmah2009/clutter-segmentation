#include "options.h"

#include <boost/program_options.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace boost;
using namespace cv;

namespace clutseg {

    int options(int ac, char **av, Options & opts)
    {

        // Declare the supported options.
        program_options::options_description desc("Allowed options");
        desc.add_options()("help", "Produce help message.");
        desc.add_options()("image,I", program_options::value < string > (&opts.imageDirectory),
                           "Test image base directory.");
        desc.add_options()("testdesc",
                           program_options::value < string > (&opts.testdescFilename),
                           "Test description file");
        desc.add_options()("base,B",
                           program_options::value < string >
                           (&opts.baseDirectory)->default_value("./"),
                           "The directory that the training base is in.");
        desc.add_options()("tod_config,f", program_options::value < string > (&opts.config),
                           "The name of the configuration file");
        desc.add_options()("result,r",
                           program_options::value < string > (&opts.resultFilename),
                           "Result file using INI-style/Python config file syntax. This one is optional.");
        desc.add_options()("stats,s",
                           program_options::value < string > (&opts.statsFilename),
                           "Statistics are stored in this file. INI-style/Python config file syntax.");
        desc.add_options()("roc",
                           program_options::value < string > (&opts.rocFilename),
                           "Generate points on a ROC graph and save it to a file.");
        desc.add_options()("table",
                           program_options::value < string > (&opts.tableFilename),
                           "Generate a CSV table containing information about guesses.");
        desc.add_options()("store",
                           program_options::value < string > (&opts.storeDirectory),
                           "Write all results to this folder in a pre-defined "
                           "manner. If you specify this option then arguments of "
                           "--result, --stats, --roc, --table will be ignored.");
        desc.add_options()("verbose,V",
                           program_options::value < int >(&opts.verbose)->default_value(1),
                           "Verbosity level");
        desc.add_options()("mode,m",
                           program_options::value < int >(&opts.mode)->default_value(1),
                           "Mode");

        program_options::variables_map vm;
        program_options::store(program_options::parse_command_line(ac, av, desc), vm);
        program_options::notify(vm);

        if (vm.count("help")) {
            cout << desc << "\n";
            return 1;
        }

        FileStorage fs;
        if (opts.config.empty()
            || !(fs = FileStorage(opts.config, FileStorage::READ)).isOpened()) {
            cout << "Must supply configuration." << "\n";
            cout << desc << endl;
            return 1;
        } else
            opts.params.read(fs[tod::TODParameters::YAML_NODE_NAME]);

        if (!vm.count("image")) {
            cout << "Must supply an image directory." << "\n";
            cout << desc << endl;
            return 1;
        }

        if (!vm.count("testdesc")) {
            cout << "Must supply a test description file." << "\n";
            cout << desc << endl;
            return 1;
        }

        if (!vm.count("base")) {
            cout << "Must supply training base directory." << "\n";
            cout << desc << endl;
            return 1;
        }

        return 0;

    }

}
