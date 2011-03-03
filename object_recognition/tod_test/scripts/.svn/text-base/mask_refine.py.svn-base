# -*- coding: utf-8 -*-
import os, sys, cv, glob
from optparse import OptionParser

winname = 'mask'

def on_mouse (event, x, y, flags, param):
    need_draw = False
    color = cv.Scalar(255, 255, 255)
    if event == cv.CV_EVENT_LBUTTONDOWN or (event == cv.CV_EVENT_MOUSEMOVE and flags&cv.CV_EVENT_FLAG_LBUTTON):
      need_draw = True
    if event == cv.CV_EVENT_RBUTTONDOWN or (event == cv.CV_EVENT_MOUSEMOVE and flags&cv.CV_EVENT_FLAG_RBUTTON):
      need_draw = True
      color = cv.Scalar(0, 0, 0)
    
    if need_draw:
      cv.Circle(param[0], (x/scale, y/scale), 10, color, 20) 
      result = resizeAnd(mask_image, image, scale)
      cv.ShowImage(winname, result)

def resizeAnd(mask_image, image, scale):
  size = (mask_image.width*scale, mask_image.height*scale)
  resized_mask = cv.CreateImage(size, mask_image.depth, mask_image.nChannels)
  cv.Resize(mask_image, resized_mask)
      
  resized_image = cv.CreateImage(size, image.depth, image.nChannels)
  cv.Resize(image, resized_image)
  result = cv.CreateImage(size, image.depth, image.nChannels)
  cv.And(resized_image, resized_mask, result)
  
  return result

def doIteration(mask_image, image, scale):
  result = resizeAnd(mask_image, image, scale)
  cv.ShowImage(winname, result)
  params = [mask_image, image, scale]
  cv.SetMouseCallback (winname, on_mouse, params)

if __name__ == '__main__':
    parser = OptionParser(description='Manual tool for refinement masks in test base')
    parser.add_option('-T', '--test_dir', dest='test_dir',
                  help='bag directory in the test directory', metavar='PATH_TO_BASE')
    parser.add_option('-o', '--object_name', dest='object_name',
                  help='object name in test base', metavar='OBJECT_NAME')
    parser.add_option('-s', '--scale', dest='scale', default=1.0,
                  help='image resinig scale', metavar='SCALE')
    
    (options, args) = parser.parse_args()
    if not options.test_dir or not options.object_name:
        parser.print_help()
        sys.exit(1)

    print 'test_dir is %s' % options.test_dir
    print 'object_name is %s' % options.object_name

    object_dir = os.path.join(options.test_dir, options.object_name)
    print object_dir
    masks = glob.glob('%s/*.png' %  object_dir)
    
    scale = float(options.scale)

    cv.NamedWindow(winname)
    count = 0
    for mask in masks:
      count = count + 1
      percent = (float(count)/len(masks))*100
      print 'done %f%%' % percent
      mask_image = cv.LoadImage(mask)
      mask_name = os.path.basename(mask)
      image_path = os.path.join(options.test_dir, 'images', mask_name[0:mask_name.rfind('.mask')])
      image = cv.LoadImage(image_path)      

      doIteration(mask_image, image, scale)

      while True:
        key = ( cv.WaitKey(1) ) % 0x100
        if key == 27:
           exit()
        elif key == ord('c'):
           mask_image = cv.LoadImage(mask)
           doIteration(mask_image, image, scale)
        elif key == ord('d'):
		    os.remove(mask)
		    break
        elif key == 32:
           cv.SaveImage(mask, mask_image)
           break 
      

