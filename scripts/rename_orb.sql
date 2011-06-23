update pms_fe set detector_type="ORB-TOD", descriptor_type="ORB-TOD", extractor_type="ORB-TOD" where detector_type="ORB" and descriptor_type="ORB" and extractor_type="ORB";
update pms_fe set detector_type="ORB", descriptor_type="ORB", extractor_type="ORB" where detector_type="ORB-OpenCV" and descriptor_type="ORB-OpenCV" and extractor_type="ORB-OpenCV";
