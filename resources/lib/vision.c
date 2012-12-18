#include <stdio.h>
#include <stdlib.h>
#include "cv.h"
#include "highgui.h"

#ifdef FREENECT
#include "libfreenect.h"
#include "libfreenect_sync.h"
#endif

int* image_size(void* m){
  IplImage* img = (IplImage*)m;    
  int* vals = malloc(2 * sizeof(int));
  vals[0] = img->width;
  vals[1] = img->height;
  return vals;
}

int encoded_image_size(void* m){
  CvMat* mat = (CvMat*)m;
  return mat->cols;
}

char* encoded_image_rerieve(void* m){
  CvMat* buf = (CvMat*)m;
  char* ret = malloc(buf->cols * sizeof(char));

  int col;
  int k;

  for(col = 0, k= 0; col < buf->cols; col++ , k++) {
    char* ptr = (char*)(buf->data.ptr + col);
    ret[k] = ptr[0];
  }

  cvReleaseMat(&buf);
  return ret;
}

void* encode_image(void* m, int ext, int comp){
  IplImage* img = (IplImage*)m;
  int params[3];
  char* type;

  params[1] = comp;
  params[2] = 0;

  switch(ext) {
  case 1:
    params[0] = CV_IMWRITE_PNG_COMPRESSION;
    type = ".png";
    break;
  case 2:
    params[0] = CV_IMWRITE_JPEG_QUALITY;
    type = ".jpeg";
    break;
  }

  return (void*)cvEncodeImage(type, img, params);
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

int set_capture_property(void* c, int prop, double value){
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
  }

  return cvSetCaptureProperty(capture, prop, value);
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

int grab_frame(void* capture){
  return cvGrabFrame((CvCapture*)capture);
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

#ifdef CV2_2
  CvSeq* objs = cvHaarDetectObjects(image, cascade, 
                                    storage, scale_factor, min_neighbors, flags, 
                                    cvSize(min_w, min_h), cvSize(max_w, max_h));
#else
  CvSeq* objs = cvHaarDetectObjects(image, cascade, 
                                    storage, scale_factor, min_neighbors, flags, 
                                    cvSize(min_w, min_h));
#endif

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

typedef struct {
  IplImage* mapx;
  IplImage* mapy;
} undistort_map;

void* undistort_map_from_file(char* i, char* d, int w, int h){
  CvMat *intrinsic = (CvMat*)cvLoad(i, NULL, NULL, NULL);
  CvMat *distortion = (CvMat*)cvLoad(d, NULL, NULL, NULL);

  CvSize size = cvSize(w, h);

  IplImage* mapx = cvCreateImage(size, IPL_DEPTH_32F, 1);
  IplImage* mapy = cvCreateImage(size, IPL_DEPTH_32F, 1);
  cvInitUndistortMap(intrinsic,distortion,mapx,mapy);

  undistort_map* map = malloc(sizeof(undistort_map));
  map->mapx = mapx;
  map->mapy = mapy;
  
  return (void*)map;
}

void* remap(void* i, void* m){
  IplImage* img = (IplImage*)i;
  undistort_map* map = (undistort_map*)m;
  IplImage *clone = cvCloneImage(img);

  cvRemap( img, clone, map->mapx, map->mapy, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
  return (void*)clone;
}

typedef struct{
  CvSeq * keypoints;
  CvSeq * descriptors;
  int width;
  int height;
}surf_struct;

void *extract_surf(void *i, CvArr* mask, int exteneded, double threshold, int nOctaves, int nOctaveLayers){
  IplImage* image = (IplImage*)i;
  CvSeq *keypoints = 0, *descriptors = 0;

  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSURFParams params = cvSURFParams(threshold, exteneded);
  params.nOctaves = nOctaves;
  params.nOctaveLayers = nOctaveLayers;


  cvExtractSURF(image, mask, &keypoints, &descriptors, storage, params ,0);

  surf_struct* s = malloc(sizeof(surf_struct));
  s->keypoints = keypoints;
  s->descriptors = descriptors;
  s->width = image->width;
  s->height = image->height;
  return (void*)s;
}

int *surf_points(void *s){
  surf_struct* image = (surf_struct*)s;

  int* vals = malloc(image->keypoints->total * 3 * sizeof(int) + 1);
  vals[0] = image->keypoints->total;
  int i,k;

  for( i=0, k=1; i < image->keypoints->total; i++, k+=3){
    CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(image->keypoints, i);

    vals[k] = cvRound(r->pt.x);
    vals[k+1] = cvRound(r->pt.y);
    vals[k+2] = cvRound(r->size*1.2/9.*2);
  }

  return vals;
}

double compareSURFDescriptors(const float* d1, const float* d2, double best, int length){
    double total_cost = 0;
    assert( length % 4 == 0 );
    int i;
    for( i = 0; i < length; i += 4 ){
        double t0 = d1[i] - d2[i];
        double t1 = d1[i+1] - d2[i+1];
        double t2 = d1[i+2] - d2[i+2];
        double t3 = d1[i+3] - d2[i+3];
        total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
        if( total_cost > best )
            break;
    }
    return total_cost;
}

int naiveNearestNeighbor(const float* vec, int laplacian,
                         const CvSeq* model_keypoints,
                         const CvSeq* model_descriptors){
    int length = (int)(model_descriptors->elem_size/sizeof(float));
    int i, neighbor = -1;
    double d, dist1 = 1e6, dist2 = 1e6;
    CvSeqReader reader, kreader;
    cvStartReadSeq( model_keypoints, &kreader, 0 );
    cvStartReadSeq( model_descriptors, &reader, 0 );

    for( i = 0; i < model_descriptors->total; i++ ){
        const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
        const float* mvec = (const float*)reader.ptr;
    	CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
        CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
        if( laplacian != kp->laplacian )
            continue;
        d = compareSURFDescriptors( vec, mvec, dist2, length );
        if( d < dist1 ){
            dist2 = dist1;
            dist1 = d;
            neighbor = i;
        }
        else if ( d < dist2 )
            dist2 = d;
    }
    if ( dist1 < 0.6*dist2 )
        return neighbor;
    return -1;
}

CvSeq* findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
                  const CvSeq* imageKeypoints, const CvSeq* imageDescriptors){
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* seq = cvCreateSeq(0, sizeof(CvSeq), sizeof(int), storage);

  int i;
  CvSeqReader reader, kreader;
  cvStartReadSeq(objectKeypoints, &kreader, 0);
  cvStartReadSeq(objectDescriptors, &reader, 0);
  
  for( i = 0; i < objectDescriptors->total; i++ ){
    const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
    const float* descriptor = (const float*)reader.ptr;
    CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
    CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
    int nearest_neighbor = naiveNearestNeighbor( descriptor, kp->laplacian, imageKeypoints, imageDescriptors );
    if( nearest_neighbor >= 0 ){
      cvSeqPush(seq, &i);
      cvSeqPush(seq, &nearest_neighbor);
    }
  }
  
  return seq;
}

int* locatePlanarObject(void *o, void *s){
  surf_struct* object = (surf_struct*)o;
  surf_struct* image = (surf_struct*)s;
  CvSeq* objectKeypoints = object->keypoints;
  CvSeq* objectDescriptors = object->descriptors;
  CvPoint src_corners[4] = {{0,0}, {object->width,0}, {object->width, object->height}, {0, object->height}};
  CvSeq* imageKeypoints = image->keypoints;
  CvSeq* imageDescriptors = image->descriptors;
  int i, n, k;

  CvSeq* seq = findPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors);
  
  n = seq->total/2;
  if( n < 4 )
    return 0;

  CvPoint2D32f *pt1 = (CvPoint2D32f *)malloc(sizeof(CvPoint2D32f) * n);
  CvPoint2D32f *pt2 = (CvPoint2D32f *)malloc(sizeof(CvPoint2D32f) * n);

  for( i = 0; i < n; i++ ){
    int* p1 = (int*)cvGetSeqElem(seq, i*2);
    int* p2 = (int*)cvGetSeqElem(seq, i*2+1);

    pt1[i] = ((CvSURFPoint*)cvGetSeqElem(objectKeypoints,*p1))->pt;
    pt2[i] = ((CvSURFPoint*)cvGetSeqElem(imageKeypoints,*p2))->pt;
  }

  double h[9];
  CvMat _h = cvMat(3, 3, CV_64F, h);
  CvMat _pt1, _pt2;
  _pt1 = cvMat(1, n, CV_32FC2, &pt1[0] );
  _pt2 = cvMat(1, n, CV_32FC2, &pt2[0] );
  if( !cvFindHomography(&_pt1, &_pt2, &_h, CV_RANSAC, 5, NULL))
    return 0;

  int* vals = malloc(8 * sizeof(int));

  for(i=0, k=0; i < 4; i++, k+=2 ){
    double x = src_corners[i].x, y = src_corners[i].y;
    double Z = 1./(h[6]*x + h[7]*y + h[8]);
    double X = (h[0]*x + h[1]*y + h[2])*Z;
    double Y = (h[3]*x + h[4]*y + h[5])*Z;
    vals[k] = cvRound(X);
    vals[k+1] = cvRound(Y);    
  }

  cvClearMemStorage(seq->storage);
  return vals;
}

void release_surf(void* p){
  surf_struct *surf = (surf_struct*)p;
  cvClearMemStorage(surf->keypoints->storage);
  cvClearMemStorage(surf->descriptors->storage);
  free(surf);
}

void* pyr_down(void* i){
  IplImage* img = (IplImage*)i;
  IplImage* out = cvCreateImage(cvSize(img->width/2,img->height/2), img->depth, img->nChannels);

  cvPyrDown(img, out, CV_GAUSSIAN_5x5);

  return (void*)out;
}

typedef struct {
  int vmin;          //limits for calculating hue
  int vmax;
  int smin;
  IplImage* hsv;     //input image converted to HSV
  IplImage* hue;     //hue channel of HSV image
  IplImage* mask;    //image for masking pixels
  IplImage* prob;    //probability estimates for each pixel

  CvHistogram* hist; //histogram of hue in original image

  CvRect prev_rect;  //location in previous frame
  CvBox2D curr_box;  //current location estimate
} camshift_struct;

void update_hue_image (const IplImage* image, camshift_struct* obj) {
  
  //convert to HSV color model
  cvCvtColor(image, obj->hsv, CV_BGR2HSV);
  
  //mask out-of-range values
  cvInRangeS(obj->hsv,                               //source
             cvScalar(0, obj->smin, MIN(obj->vmin, obj->vmax), 0),  //lower bound
             cvScalar(180, 256, MAX(obj->vmin, obj->vmax) ,0), //upper bound
             obj->mask);                             //destination
  
  //extract the hue channel, split: src, dest channels
  cvSplit(obj->hsv, obj->hue, 0, 0, 0 );
}

camshift_struct* camshift_init (void* i, int x, int y, int w, int h, int vmin, int vmax, int smin) {
  IplImage* image = (IplImage*)i;
  CvRect region = cvRect(x, y, w, h);
  camshift_struct* obj;

  if((obj = malloc(sizeof *obj)) != NULL) {
    obj->vmin = vmin;
    obj->vmax = vmax;
    obj->smin = smin;
    //create-image: size(w,h), bit depth, channels
    obj->hsv  = cvCreateImage(cvGetSize(image), 8, 3);
    obj->mask = cvCreateImage(cvGetSize(image), 8, 1);
    obj->hue  = cvCreateImage(cvGetSize(image), 8, 1);
    obj->prob = cvCreateImage(cvGetSize(image), 8, 1);

    int hist_bins = 30;           //number of histogram bins
    float hist_range[] = {0,180}; //histogram range
    float* range = hist_range;
    obj->hist = cvCreateHist(1,             //number of hist dimensions
                             &hist_bins,    //array of dimension sizes
                             CV_HIST_ARRAY, //representation format
                             &range,        //array of ranges for bins
                             1);            //uniformity flag
  }
  
  //create a new hue image
  update_hue_image(image, obj);

  float max_val = 0.f;
  
  //create a histogram
  cvSetImageROI(obj->hue, region);
  cvSetImageROI(obj->mask, region);
  cvCalcHist(&obj->hue, obj->hist, 0, obj->mask);
  cvGetMinMaxHistValue(obj->hist, 0, &max_val, 0, 0 );
  cvConvertScale(obj->hist->bins, obj->hist->bins,
                 max_val ? 255.0/max_val : 0, 0);
  cvResetImageROI(obj->hue);
  cvResetImageROI(obj->mask);
  
  //store the previous location
  obj->prev_rect = region;

  return obj;
}

void* cam_shift_back_project_image (void* c){
  camshift_struct* obj = (camshift_struct*)c;
  return (void*)obj->prob;
}

float* camshift_track (void* i, void* o) {
  IplImage* image = (IplImage*)i;
  camshift_struct* obj = (camshift_struct*)o;
  CvConnectedComp components;

  update_hue_image(image, obj);

  //create a probability image
  cvCalcBackProject(&obj->hue, obj->prob, obj->hist);
  cvAnd(obj->prob, obj->mask, obj->prob, 0);

  cvCamShift(obj->prob, obj->prev_rect,
             cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1),
             &components, &obj->curr_box);

  //update location and angle
  obj->prev_rect = components.rect;
  obj->curr_box.angle = -obj->curr_box.angle;


  float* coords = malloc(5 * sizeof(float));

  coords[0] = obj->curr_box.center.x;
  coords[1] = obj->curr_box.center.y;
  coords[2] = obj->curr_box.size.width;
  coords[3] = obj->curr_box.size.height;
  coords[4] = obj->curr_box.angle;

  return coords;
}

void release_camshift(void* o){
  camshift_struct* obj = (camshift_struct*)o;
  cvReleaseImage(&obj->hsv);
  cvReleaseImage(&obj->hue);
  cvReleaseImage(&obj->mask);
  cvReleaseImage(&obj->prob);
  cvReleaseHist(&obj->hist);
  free(obj);
}

int* max_rect(int x1, int y1, int w1, int h1, int x2, int y2, int w2, int h2){
  CvRect r1 = cvRect(x1, y1, w1, h1);
  CvRect r2 = cvRect(x2, y2, w2, h2);

  CvRect r = cvMaxRect(&r1, &r2);
  int* coords = malloc(4 * sizeof(int));
  coords[0] = r.x;
  coords[1] = r.y;
  coords[2] = r.width;
  coords[3] = r.height;

  return coords;
}

float* good_features_to_track(void* j, int max_count, double quality, double min_distance, int win_size){
  IplImage* img = (IplImage*)j;

  IplImage* grey = cvCreateImage( cvGetSize(img), 8, 1 );
  cvCvtColor( img, grey, CV_BGR2GRAY );

  IplImage* eig = cvCreateImage( cvGetSize(grey), 32, 1 );
  IplImage* temp = cvCreateImage( cvGetSize(grey), 32, 1 );

  CvPoint2D32f* points = (CvPoint2D32f*)malloc(max_count * sizeof(CvPoint2D32f));

  cvGoodFeaturesToTrack(grey, eig, temp, points, &max_count, quality, min_distance, 0, 3, 0, 0.04);
  cvFindCornerSubPix(grey, points, max_count, cvSize(win_size,win_size), cvSize(-1,-1), 
                     cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
  
  float* ret = (float*)malloc((1 + 2 * max_count) * sizeof(float));
  ret[0] = max_count;
  
  int i,k;
  for(i = 0, k = 1; i < max_count; i++, k+=2){
    ret[k] = points[i].x;
    ret[k+1] = points[i].y;
  }

  free(points);
  cvReleaseImage( &eig );
  cvReleaseImage( &temp );

  return ret;
}

float* calc_optical_flow_pyr_lk(void* a, void* b, int count, float* points, int win_size, int level){
  IplImage* imgA = (IplImage*)a;
  IplImage* imgB = (IplImage*)b;

  IplImage* greyA = cvCreateImage( cvGetSize(imgA), 8, 1 );
  cvCvtColor( imgA, greyA, CV_BGR2GRAY );

  IplImage* greyB = cvCreateImage( cvGetSize(imgB), 8, 1 );
  cvCvtColor( imgB, greyB, CV_BGR2GRAY );

  char features_found[count];

  CvSize pyr_sz = cvSize(imgA->width+8, imgB->height/3);

  IplImage* pyrA = cvCreateImage(pyr_sz, IPL_DEPTH_32F, 1);
  IplImage* pyrB = cvCreateImage(pyr_sz, IPL_DEPTH_32F, 1);

  CvPoint2D32f* cornersB = (CvPoint2D32f*)malloc(count * sizeof(CvPoint2D32f));
  CvPoint2D32f* cornersA = (CvPoint2D32f*)malloc(count * sizeof(CvPoint2D32f));

  int i,k;
  for(i = 0, k = 0; i < count * 2; i+=2, k++){
    cornersA[k].x = points[i];
    cornersA[k].y = points[i+1];
  }

  cvCalcOpticalFlowPyrLK(greyA, greyB, pyrA, pyrB, cornersA, cornersB, count,
                         cvSize( win_size, win_size ), level, features_found, 0,
                         cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), 0 );

  float* ret = (float*)malloc((3 * count) * sizeof(float));

  for(i = 0, k = 0; i < count; i++, k+=3){
    ret[k] = cornersB[i].x;
    ret[k+1] = cornersB[i].y;
    ret[k+2] = features_found[i];
  }

  free(cornersA);
  free(cornersB);
  cvReleaseImage(&pyrA);
  cvReleaseImage(&pyrB);
  cvReleaseImage(&greyA);
  cvReleaseImage(&greyB);
  return ret;
}

/* 
   Drawing Functions 
*/

void ellipse_box(void* i, float x, float y, float w, float h, float a, int r, int g, int b, int thickness){
  IplImage* img = (IplImage*)i;
  CvBox2D box;
  box.center.x = x;
  box.center.y = y;
  box.size.width = w;
  box.size.height = h;
  box.angle = a;
  cvEllipseBox(img, box, cvScalar(b,g,r,0), thickness, 8, 0);
}

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

void contours(void *i, void *c, int r1, int g1, int b1, int r2, int g2, int b2, int level, int thickness){
  IplImage* img = (IplImage*)i;
  CvSeq* contours = (CvSeq*)c;
  cvDrawContours(img, contours, cvScalar(b1,g1,r1,0), cvScalar(b2,g2,r2,0), level, thickness, 8, cvPoint(0,0));
}

void* flip_image(void* i, int mode){
  IplImage* image = (IplImage*)i;
  IplImage* flipped = cvCreateImage(cvGetSize((IplImage*)image), image->depth, image->nChannels);;
  
  cvFlip(image, flipped, mode);  

  return (void*)flipped;
}

/* From https://github.com/OpenKinect/libfreenect/blob/master/wrappers/opencv/libfreenect_cv.c */

#ifdef FREENECT
IplImage *freenect_sync_get_depth_cv(int index)
{
	static IplImage *image = 0;
	static char *data = 0;
	if (!image) image = cvCreateImageHeader(cvSize(640,480), 16, 1);
	unsigned int timestamp;
	if (freenect_sync_get_depth((void**)&data, &timestamp, index, FREENECT_DEPTH_11BIT))
	    return NULL;
	cvSetData(image, data, 640*2);
	return image;
}

IplImage *freenect_sync_get_rgb_cv(int index)
{
	static IplImage *image = 0;
	static char *data = 0;
	if (!image) image = cvCreateImageHeader(cvSize(640,480), 8, 3);
	unsigned int timestamp;
	if (freenect_sync_get_video((void**)&data, &timestamp, index, FREENECT_VIDEO_RGB))
	    return NULL;
	cvSetData(image, data, 640*3);
	return image;
}

IplImage *GlViewColor(IplImage *depth)
{
	static IplImage *image = 0;
	if (!image) image = cvCreateImage(cvSize(640,480), 8, 3);
	unsigned char *depth_mid = (unsigned char*)(image->imageData);
	int i;
	for (i = 0; i < 640*480; i++) {
		int lb = ((short *)depth->imageData)[i] % 256;
		int ub = ((short *)depth->imageData)[i] / 256;
		switch (ub) {
			case 0:
				depth_mid[3*i+2] = 255;
				depth_mid[3*i+1] = 255-lb;
				depth_mid[3*i+0] = 255-lb;
				break;
			case 1:
				depth_mid[3*i+2] = 255;
				depth_mid[3*i+1] = lb;
				depth_mid[3*i+0] = 0;
				break;
			case 2:
				depth_mid[3*i+2] = 255-lb;
				depth_mid[3*i+1] = 255;
				depth_mid[3*i+0] = 0;
				break;
			case 3:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 255;
				depth_mid[3*i+0] = lb;
				break;
			case 4:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 255-lb;
				depth_mid[3*i+0] = 255;
				break;
			case 5:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 0;
				depth_mid[3*i+0] = 255-lb;
				break;
			default:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 0;
				depth_mid[3*i+0] = 0;
				break;
		}
	}
	return image;
}

void* query_kinect_frame(){
  IplImage *image = freenect_sync_get_rgb_cv(0);
  if (!image)
    return NULL;

  cvCvtColor(image, image, CV_RGB2BGR);

  return (void*)image;
}

void* query_kinect_depth(){
  IplImage *depth = freenect_sync_get_depth_cv(0);
  if (!depth)
    return NULL;

  IplImage *ret = GlViewColor(depth);
  return (void*)ret;
}
#endif
