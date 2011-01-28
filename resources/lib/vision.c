#include <stdio.h>
#include <stdlib.h>
#include "cv.h"
#include "highgui.h"

int* image_size(void* m){
  IplImage* img = (IplImage*)m;    
  int* vals = malloc(2 * sizeof(int));
  vals[0] = img->width;
  vals[1] = img->height;
  return vals;
}

// 1 - BGR
// 2 - BINARY
// 3 - HSV
// 4 - RGB
// 5 - GRAYSCALE
int* pixels(void* m, int type){
  IplImage* img = (IplImage*)m;

  int* vals = malloc(img->height * img->width * sizeof(int));
  int i,j;
  int index = 0;

  int release = 0;
  if(type == 3){
    IplImage* tmp = cvCreateImage(cvGetSize((IplImage*)img), img->depth, img->nChannels);
    cvCvtColor(img, tmp, CV_HSV2BGR);
    img = tmp;
    release = 1;
    type = 1;
  }else if(type == 4){
    IplImage* tmp = cvCreateImage(cvGetSize((IplImage*)img), img->depth, img->nChannels);
    cvCvtColor(img, tmp, CV_RGB2BGR);
    img = tmp;
    release = 1;
    type = 1;
  }else if(type == 5){
    IplImage* tmp = cvCreateImage(cvGetSize((IplImage*)img), img->depth, 3);
    cvCvtColor(img, tmp, CV_GRAY2BGR);
    img = tmp;
    release = 1;
    type = 1;
  }

  if(type == 1){
    for (i = 0; i < img->height; i++){
      for (j = 0; j < img->width; j++){

        unsigned char red = CV_IMAGE_ELEM(img, uchar, i, (j)*3+2);
        unsigned char green = CV_IMAGE_ELEM(img, uchar, i, (j)*3+1);
        unsigned char blue = CV_IMAGE_ELEM(img, uchar, i, (j)*3);

        vals[index++] = 
          ((255 & 0xFF) << 24) | //alpha
          (((int)red & 0xFF) << 16) | 
          (((int)green & 0xFF) << 8) |
          (((int)blue & 0xFF) << 0);
      }
    }
  }else if(type == 2){
    for (i = 0; i < img->height; i++){
      for (j = 0; j < img->width; j++){

        uchar pixel = CV_IMAGE_ELEM(img, uchar, i, j);

        if(pixel == 0)
          vals[index++] = 0xFF000000;
        else
          vals[index++] = 0xFFFFFFFF;
      }
    }
  }

  if(release == 1)
    cvReleaseImage(&img);

  return vals;
}

void* capture_from_cam(int i){
  CvCapture* ptr = cvCaptureFromCAM(i);
   
  /* always check */
  if (!ptr) {
    fprintf( stderr, "Cannot open initialize webcam!\n" );
    return NULL;
  }
  
  return (void*) ptr;
}

void* capture_from_file(char* f){
  CvCapture* ptr = cvCaptureFromFile(f);  
  return (void*) ptr;
}

double get_capture_property(void* c, int prop){
  CvCapture* capture = (CvCapture*)c;

  switch(prop) {
  case 1:
    prop = CV_CAP_PROP_POS_MSEC; break;
  case 2:
    prop = CV_CAP_PROP_POS_FRAMES; break;
  case 3:
    prop = CV_CAP_PROP_POS_AVI_RATIO; break;
  case 4:
    prop = CV_CAP_PROP_FRAME_WIDTH; break;
  case 5:
    prop = CV_CAP_PROP_FRAME_HEIGHT; break;
  case 6:
    prop = CV_CAP_PROP_FPS; break;
  case 7:
    prop = CV_CAP_PROP_FOURCC; break;
  case 8:
    prop = CV_CAP_PROP_FRAME_COUNT; break;
  case 9:
    prop = CV_CAP_PROP_BRIGHTNESS; break;
  case 10:
    prop = CV_CAP_PROP_CONTRAST; break;
  case 11:
    prop = CV_CAP_PROP_SATURATION; break;
  case 12:
    prop = CV_CAP_PROP_HUE; break;
  }

  return cvGetCaptureProperty(capture, prop);
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

float* hough_circles(void* img, int dp, int min_d, int p1, int p2, int min_r, int max_r){
  IplImage* image = (IplImage*)img;

  CvMemStorage* storage = cvCreateMemStorage(0);
  cvClearMemStorage(storage);

  CvSeq* circles = 
    cvHoughCircles(image, storage, CV_HOUGH_GRADIENT, dp, min_d, p1, p2, min_r, max_r);

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

int* match_template(void* i, void* t, int mode){
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

double match_shapes(void* i1, void* i2, int mode){
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
  IplImage* converted;
  
  if(mode == 1)
    mode = CV_RGB2HSV;
  else if (mode == 2)
    mode = CV_HSV2RGB;
  else if (mode == 3)
    mode = CV_BGR2HSV;
  else if (mode == 4)
    mode = CV_HSV2BGR;
  else if (mode == 5)
    mode = CV_BGR2GRAY;
  else if (mode == 6)
    mode = CV_GRAY2BGR;

  if(mode == CV_BGR2GRAY)
    converted = cvCreateImage(cvGetSize((IplImage*)image), IPL_DEPTH_8U, 1);
  else if(mode == CV_GRAY2BGR)
    converted = cvCreateImage(cvGetSize((IplImage*)image), image->depth, 3);
  else
    converted = cvCreateImage(cvGetSize((IplImage*)image), image->depth, image->nChannels);

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

void* abs_diff(void* i1, void* i2){
  IplImage* src1 = (IplImage*)i1;
  IplImage* src2 = (IplImage*)i2;
  IplImage* diff = cvCreateImage(cvGetSize((IplImage*)src1), src1->depth, src1->nChannels);

  cvAbsDiff(src1, src2, diff);

  return (void*)diff;
}

void* clone_image(void* i){
  IplImage* src = (IplImage*)i;
  return (void*)cvCloneImage(src);
}

void* threshold(void* i, double th, double maxVal, int type){
  IplImage* src = (IplImage*)i;

  if(type == 1)
    type = CV_THRESH_BINARY;
  else if (type == 2)
    type = CV_THRESH_BINARY_INV;
  else if (type == 3)
    type = CV_THRESH_TRUNC;
  else if (type == 4)
    type = CV_THRESH_TOZERO;
  else if (type == 5)
    type = CV_THRESH_TOZERO_INV;

  if(th == -1)
    type = type | CV_THRESH_OTSU;

  IplImage* binary = cvCreateImage(cvGetSize(src),IPL_DEPTH_8U,1);
  cvThreshold(src, binary, th, maxVal, type);

  return (void*)binary;
}

void* load_cascade(char* f){
  return (void*)cvLoad(f, 0, 0, 0 );
}

int* haar_detect_objects(void* i, void* c, 
                         double scale_factor, int min_neighbors, int flags, 
                         int min_w, int min_h, int max_w, int max_h){

  IplImage* image = (IplImage*)i;
  CvHaarClassifierCascade* cascade = (CvHaarClassifierCascade*)c;

  CvMemStorage* storage = cvCreateMemStorage(0);
  cvClearMemStorage(storage);

  if(flags == 1)
    flags = CV_HAAR_DO_CANNY_PRUNING;

  CvSeq* objs = cvHaarDetectObjects(image, cascade, 
                                    storage, scale_factor, min_neighbors, flags, 
                                    cvSize(min_w, min_h), cvSize(max_w, max_h));

  if(objs->total == 0){
    cvReleaseMemStorage(&storage);
    return NULL;
  }

  int* coords = malloc((1 + 4 * objs->total) * sizeof(int));
  coords[0] = objs->total;
  
  int k = 1;
  int m;
  for(m=0; m < objs->total; m++, k+=4){
    CvRect* r = (CvRect*)cvGetSeqElem(objs, m);
      
    coords[k] = r->x;
    coords[k+1] = r->y;
    coords[k+2] = r->width;
    coords[k+3] = r->height;
  }

  cvReleaseMemStorage(&storage);  
  return coords;
}

void* find_contours(void* m, int mode, int method, int x, int y){
  IplImage* image = (IplImage*)m;
  CvSeq* contours;
  CvMemStorage* storage = cvCreateMemStorage(0);
  cvClearMemStorage(storage);

  switch(mode) {
  case 1:
    mode = CV_RETR_EXTERNAL; break;
  case 2:
    mode = CV_RETR_LIST; break;
  case 3:
    mode = CV_RETR_CCOMP; break;
  case 4:
    mode = CV_RETR_TREE; break;
  }

  switch(method) {
  case 1:
    method = CV_CHAIN_CODE; break;
  case 2:
    method = CV_CHAIN_APPROX_NONE; break;
  case 3:
    method = CV_CHAIN_APPROX_SIMPLE; break;
  case 4:
    method = CV_CHAIN_APPROX_TC89_L1; break;
  case 5:
    method = CV_CHAIN_APPROX_TC89_KCOS; break;
  case 6:
    method = CV_LINK_RUNS; break;
  }

  int size;
  if(method == CV_CHAIN_CODE)
    size = sizeof(CvChain);
  else
    size = sizeof(CvContour);

  cvFindContours(image, storage, &contours, size, mode, method, cvPoint(x,y));

  if(contours == NULL)
    cvReleaseMemStorage(&storage);
  
  return contours;
}

void release_contours(void* s){
  CvSeq* seq = (CvSeq*)s;
  if(seq != NULL)
    cvReleaseMemStorage(&seq->storage);
}

int* bounding_rects(void* c){
  CvSeq* contours = (CvSeq*)c;

  if(contours == NULL)
    return NULL;

  int total = 0;
  CvSeq* t = contours;
  for(; t; t = t->h_next)
    total++;

  int* coords = malloc((1 + 4 * total) * sizeof(int));
  coords[0] = total;
  
  int k = 1;
  for(; contours; contours= contours->h_next, k+=4){
    CvRect b = cvBoundingRect(contours, 1);
      
    coords[k] = b.x;
    coords[k+1] = b.y;
    coords[k+2] = b.width;
    coords[k+3] = b.height;
  }

  return coords;
}

void* erode(void* i, int iterations){
  IplImage* out = cvCloneImage((IplImage*)i);
  cvErode(out, out, 0, iterations);
  return (void*)out;
}

void* dilate(void* i, int iterations){
  IplImage* out = cvCloneImage((IplImage*)i);
  cvDilate(out, out, 0, iterations);
  return (void*)out;
}

void* canny(void* i, int t1, int t2, int as){
  IplImage* image = (IplImage*)i;
  IplImage* edges = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);

  cvCanny(image, edges, t1, t2, as);

  return (void*)edges;
}

float* hough_lines(void* i, int method, double rho, double theta, int threshold, double param1, double param2){
  IplImage* image = (IplImage*)i;
  CvMemStorage* storage = cvCreateMemStorage(0);
  cvClearMemStorage(storage);

  switch(method) {
  case 1:
    method = CV_HOUGH_STANDARD; break;
  case 2:
    method = CV_HOUGH_PROBABILISTIC; break;
  case 3:
    method = CV_HOUGH_MULTI_SCALE; break;
  }

  CvSeq* lines = cvHoughLines2(image, storage, method, rho, theta, threshold, param1, param2);

  if(lines->total == 0)
    return NULL;

  float* vals;

  if(method == CV_HOUGH_STANDARD || method == CV_HOUGH_MULTI_SCALE){
    vals = malloc((1 + 2 * lines->total) * sizeof(float));
    vals[0] = (float)lines->total;

    int i,k;
    for(i=0, k=1; i<lines->total; i++, k+=2){
      float* line = (float*)cvGetSeqElem(lines,i);
    
      vals[k] = line[0]; // rho
      vals[k+1] = line[1]; //theta

    }
  }else{
    vals = malloc((1 + 4 * lines->total) * sizeof(float));
    vals[0] = (float)lines->total;

    int i,k;
    for(i=0, k=1; i<lines->total; i++, k+=4){
      CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
    
      vals[k] = line[0].x; 
      vals[k+1] = line[0].y;
      vals[k+2] = line[1].x; 
      vals[k+3] = line[1].y;
    }
  }

  cvReleaseMemStorage(&storage);
  return vals;
}

void* copy_region(void* i, int x, int y, int w, int h){
  IplImage* image = (IplImage*)i;
  IplImage* clone = cvCloneImage(image);

  cvSetImageROI(clone, cvRect(x, y, w, h));
  IplImage *copy = cvCreateImage(cvGetSize(clone), clone->depth, clone->nChannels);
  cvCopy(clone, copy, NULL);

  cvReleaseImage(&clone);
  return (void*)copy;
}

void* rotate_image(void *s, float angle_degrees){
  IplImage* src = (IplImage*)s;

  float m[6];
  CvMat M = cvMat(2, 3, CV_32F, m);
  int w = src->width;
  int h = src->height;
  float angle_radians = angle_degrees * ((float)CV_PI / 180.0f);
  m[0] = (float)( cos(angle_radians) );
  m[1] = (float)( sin(angle_radians) );
  m[3] = -m[1];
  m[4] = m[0];
  m[2] = w*0.5f;  
  m[5] = h*0.5f;  

  CvSize size_rotated;
  size_rotated.width = cvRound(w);
  size_rotated.height = cvRound(h);

  IplImage *image_rotated = cvCreateImage(size_rotated, src->depth, src->nChannels);

  cvGetQuadrangleSubPix(src, image_rotated, &M);

  return image_rotated;
}

void* scale_image(void *s, double factor){
  IplImage* src = (IplImage*)s;

  int w = src->width * factor;
  int h = src->height * factor;

  CvSize size_scaled;
  size_scaled.width = cvRound(w);
  size_scaled.height = cvRound(h);

  IplImage *image_scaled = cvCreateImage(size_scaled, src->depth, src->nChannels);

  cvResize(src, image_scaled, CV_INTER_LINEAR);

  return image_scaled;
}

void* video_writer(char* f, char* cc, int fps, int w, int h, int is_color){
  return (void*)cvCreateVideoWriter(f,CV_FOURCC(cc[0],cc[1],cc[2],cc[3]), fps, cvSize(w,h), is_color);;
}

void release_video_writer(void* w){
  CvVideoWriter* writer = (CvVideoWriter*)w;
  cvReleaseVideoWriter(&writer);
}

int write_frame(void* w, void* i){
  CvVideoWriter* writer = (CvVideoWriter*)w;
  IplImage* img = (IplImage*)i;
  return cvWriteFrame(writer,img);
}


/* 
   Drawing Functions 
*/

void line(void* i, int x1, int y1, int x2, int y2, int r, int g, int b, int thickness){
  IplImage* img = (IplImage*)i;
  cvLine(img, cvPoint(x1,y1), cvPoint(x2,y2), cvScalar(b,g,r,0), thickness, 8, 0);
}

void circle(void* i, int x, int y, int rds, int r, int g, int b, int thickness){
  IplImage* img = (IplImage*)i;
  cvCircle(img, cvPoint(x,y), rds, cvScalar(b,g,r,0), thickness, 8, 0);
}

void rectangle(void* i, int x1, int y1, int x2, int y2, int r, int g, int b, int thickness){
  IplImage* img = (IplImage*)i;
  cvRectangle(img, cvPoint(x1,y1), cvPoint(x2,y2), cvScalar(b,g,r,0), thickness, 8, 0);
}
