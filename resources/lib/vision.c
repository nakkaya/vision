#include <stdio.h>
#include <stdlib.h>
#include "cv.h"
#include "highgui.h"

void* capture_from_cam(int i){
  CvCapture* ptr = cvCaptureFromCAM(i);
   
  /* always check */
  if (!ptr) {
    fprintf( stderr, "Cannot open initialize webcam!\n" );
    return NULL;
  }
  
  return (void*) ptr;
}

void* query_frame(void* capture){
  return (void*)cvQueryFrame((CvCapture*)capture);
}  

void release_capture(void* cap){
  CvCapture* capture = (CvCapture*)cap;
  cvReleaseCapture( &capture);
}

void* load_image(char* file, int color){
  if(color > 0)
    color = CV_LOAD_IMAGE_COLOR;
  else if(color == 0)
    color = CV_LOAD_IMAGE_GRAYSCALE;
  else if(color < 0)
    color = CV_LOAD_IMAGE_UNCHANGED;

  (void*)cvLoadImage(file, color);
}

void save_image(void* image, char* file){
  cvSaveImage( file, (IplImage*)image, NULL);
}

void release_image(void* p){
  IplImage* image = (IplImage*)p;
  cvReleaseImage(&image);
}

void release_memory(void* p){
  free(p);
}

void* isolate_hsv_range(void* image, 
                        int h1, int s1, int v1, 
                        int h2, int s2, int v2){
  // Convert the image into an HSV image
  IplImage* imgHSV = cvCreateImage(cvGetSize((IplImage*)image), 8, 3);

  cvCvtColor(image, imgHSV, CV_BGR2HSV);

  IplImage* imgThreshed = cvCreateImage(cvGetSize((IplImage*)image), 8, 1);

  cvInRangeS(imgHSV, cvScalar(h1, s1, v1, 0), cvScalar(h2, s2, v2, 0), imgThreshed);

  cvSmooth( imgThreshed, imgThreshed, CV_GAUSSIAN, 9, 9 , 0 , 0);

  cvReleaseImage(&imgHSV);

  cvSaveImage( "temp3.png", imgThreshed, NULL);
  return (void*)imgThreshed;
}

float* circles(void* image, 
               int h1, int s1, int v1, 
               int h2, int s2, int v2,
               int min_r, int max_r, int min_d){
  IplImage* threshed = isolate_hsv_range((IplImage*)image, h1, s1, v1, h2, s2, v2);

  CvMemStorage* storage = cvCreateMemStorage(0);
  cvClearMemStorage(storage);

  CvSeq* circles = cvHoughCircles(threshed, storage, CV_HOUGH_GRADIENT, 2, 
                                  min_d, 100, 40, min_r, max_r);
  cvReleaseImage(&threshed);

  if(circles->total == 0)
    return NULL;

  float* coords = malloc((1 + 3 * circles->total) * sizeof(float));
  coords[0] = (float)circles->total;

  int i,k;
  for(i=0, k=1; i<circles->total; i++, k+=3){
    float* p = (float*)cvGetSeqElem(circles, i);
    
    coords[k] = p[0];
    coords[k+1] = p[1];
    coords[k+2] = p[2];

  }

  cvReleaseMemStorage(&storage);
  return coords;
}

int* bounding_boxes(void* image, 
                    int h1, int s1, int v1, 
                    int h2, int s2, int v2){

  IplImage* threshed = isolate_hsv_range((IplImage*)image, h1, s1, v1, h2, s2, v2);

  CvSeq* boxes;
  CvMemStorage* storage = cvCreateMemStorage(0);
  cvClearMemStorage(storage);
  
  int total = cvFindContours(threshed, storage, &boxes, sizeof(CvContour), 
                             CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0,0));

  cvReleaseImage(&threshed);

  if(total == 0)
    return NULL;
  
  int* coords = malloc((1 + 4 * total) * sizeof(int));
  coords[0] = total;
  
  int k = 1;
  for(; boxes; boxes= boxes->h_next, k+=4){
    CvRect b = cvBoundingRect(boxes, 1);
      
    coords[k] = b.x;
    coords[k+1] = b.y;
    coords[k+2] = b.width;
    coords[k+3] = b.height;
  }

  cvReleaseMemStorage(&storage);
  return coords;
}

int* image_size(void* m){
  IplImage* img = (IplImage*)m;    
  int* vals = malloc(2 * sizeof(int));
  vals[0] = img->width;
  vals[1] = img->height;
  return vals;
}

int* pixels(void* m){
  IplImage* img = (IplImage*)m;
    
  int* vals = malloc(img->height * img->width * sizeof(int));

  int i,j;
  int index = 0;
  for (i = 0; i < img->height; i++){
    for (j = 0; j < img->width; j++){

      unsigned char red = CV_IMAGE_ELEM(img, uchar, i-1, (j-1)*3+2);
      unsigned char green = CV_IMAGE_ELEM(img, uchar, i-1, (j-1)*3+1);
      unsigned char blue = CV_IMAGE_ELEM(img, uchar, i-1, (j-1)*3);

      vals[index++] = 
        ((255 & 0xFF) << 24) | //alpha
        (((int)red & 0xFF) << 16) | 
        (((int)green & 0xFF) << 8) |
        (((int)blue & 0xFF) << 0);
    }
  }

  return vals;
}

int* template_match(void* i, void* t, int mode){
  IplImage* image = (IplImage*)i;
  IplImage* template = (IplImage*)t;

  IplImage* result = cvCreateImage(cvSize(image->width - template->width+1, 
                                          image->height - template->height+1), 
                                   IPL_DEPTH_32F, 1);
  cvZero(result);

  int calc = -1;
  switch(mode) {
  case 1:
    calc = CV_TM_SQDIFF; break;
  case 2:
    calc = CV_TM_SQDIFF_NORMED; break;
  case 3:
    calc = CV_TM_CCORR; break;
  case 4:
    calc = CV_TM_CCORR_NORMED; break;
  case 5:
    calc = CV_TM_CCOEFF; break;
  case 6:
    calc = CV_TM_CCOEFF_NORMED; break;
  }

  cvMatchTemplate(image, template, result, calc);

  double min_val=0, max_val=0;
  CvPoint min_loc, max_loc;
  cvMinMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc, NULL);

  cvReleaseImage(&result);

  int* vals = malloc(6 * sizeof(int));
  vals[0] = min_val;
  vals[1] = max_val;
  vals[2] = min_loc.x;
  vals[3] = min_loc.y;
  vals[4] = max_loc.x;
  vals[5] = max_loc.y;
  return vals;
}

double match_shape(void* i1, void* i2, int mode){
  IplImage* img1 = (IplImage*)i1;
  IplImage* img2 = (IplImage*)i2;

  int calc = -1;
  switch(mode) {
  case 1:
    calc = CV_CONTOURS_MATCH_I1; break;
  case 2:
    calc = CV_CONTOURS_MATCH_I2; break;
  case 3:
    calc = CV_CONTOURS_MATCH_I3; break;
  }

  return cvMatchShapes (img1, img2, calc, 0);
}

void* convert_color(void* i, int mode){
  IplImage* image = (IplImage*)i;
  IplImage* converted = cvCreateImage(cvGetSize((IplImage*)image), image->depth, image->nChannels);
  
  if(mode == 1)
    mode = CV_RGB2HSV;
  else if (mode == 2)
    mode = CV_HSV2RGB;
  else if (mode == 3)
    mode = CV_BGR2HSV;
  else if (mode == 4)
    mode = CV_HSV2BGR;

  cvCvtColor(image, converted, mode);  

  return (void*)converted;
}

void* in_range_s(void* i, int s11, int s12, int s13, int s14, int s21, int s22, int s23, int s24){
  IplImage* image = (IplImage*)i;
  IplImage* processed = cvCreateImage(cvGetSize((IplImage*)image), 8, 1);
  cvInRangeS(image, cvScalar(s11, s12, s13, s14), cvScalar(s21, s22, s23, s24), processed);
  return (void*)processed;
}

void* smooth(void* i, int mode, int param1, int param2, int param3, int param4){
  IplImage* image = (IplImage*)i;
  IplImage* ret = cvCreateImage(cvGetSize((IplImage*)image), image->depth, image->nChannels);

  if(mode == 1)
    mode = CV_BLUR_NO_SCALE;
  else if (mode == 2)
    mode = CV_BLUR;
  else if (mode == 3)
    mode = CV_GAUSSIAN;
  else if (mode == 4)
    mode = CV_MEDIAN;
  else if (mode == 5)
    mode = CV_BILATERAL;

  cvSmooth(image, ret, mode, param1, param2 , param3 , param4);

  return (void*)ret;
}
