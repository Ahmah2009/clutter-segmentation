/*
 * Author: Julius Adorf
 */

#ifndef _GROUND_H_
#define _GROUND_H_

#include "clutseg/pose.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/filesystem.hpp>
    #include <map>
    #include <opencv_candidate/PoseRT.h>
    #include <string>
#include "clutseg/gcc_diagnostic_enable.h"

namespace clutseg {

    /**
     * \brief Ground truth for a set of images.
     *
     * The keys are identifiers for the images, such as paths.  The ground
     * truth for a single image is described by an instance of LabelSet.
     */
    typedef std::map<std::string, LabelSet > GroundTruth;

    /** \brief Load ground truth from a specification ground-truth file.
     *
     * The specification file uses *.ini-style (or Python configuration file)
     * syntax. Example:
     *
     * <pre>
     * [images]
     * it_hm_jc/image_00031.png = icedtea haltbare_milch jacobs_coffee
     * it_hm_jc/image_00022.png = icedtea haltbare_milch jacobs_coffee
     * at_hm_jc/image_00031.png = assam_tea haltbare_milch jacobs_coffee
     * at_hm_jc/image_00022.png = assam_tea haltbare_milch jacobs_coffee
     * it_jc_at/image_00057.png = icedtea jacobs_coffee assam_tea
     * it_jc_at/image_00031.png = icedtea jacobs_coffee assam_tea
     * </pre>
     *
     * The paths are specified relative to the location of the ground-truth file.
     * The ground truth for each image is loaded from a YAML file, named after the
     * image. For image it_hm_jc/image_00031.png, this function expects a YAML file
     * it_hm_jc/image_00031.png.ground.yaml.
     *
     * See also: LabelSet
     */
    GroundTruth loadGroundTruth(const boost::filesystem::path & filename);

    /** \brief Loads ground truth, but only the names of the images.
     *
     * The ground-truth file lists the names of the objects.
     *
     * See loadGroundTruth.
     */
    GroundTruth loadGroundTruthWithoutPoses(const boost::filesystem::path & filename);

}

#endif
