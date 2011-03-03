#include <ftd/fast_template_detector_vs.h>


int
main(
  int argc,
  char ** argv )
{  
  const int regionSize = 7;
  const int templateHorizontalSamples = 154/regionSize;
  const int templateVerticalSamples = 154/regionSize;
  const int numOfCharsPerElement = 1;
  
  ::ftd::FastTemplateDetectorVS templateDetector1(templateHorizontalSamples, templateVerticalSamples, regionSize, numOfCharsPerElement, 10);
  ::ftd::FastTemplateDetectorVS templateDetector2(templateHorizontalSamples, templateVerticalSamples, regionSize, numOfCharsPerElement, 10);
  
  templateDetector1.load(argv[1]);
  templateDetector2.load(argv[2]);
  
  templateDetector1.clearClusters ();
  templateDetector1.clusterHeuristically (4);
  
  templateDetector2.clearClusters ();
  templateDetector2.clusterHeuristically (4);
  
  templateDetector1.combine(templateDetector2);
  
  templateDetector1.clearClusters ();
  templateDetector1.clusterHeuristically (4);
  
  templateDetector1.save(argv[3]);
}


