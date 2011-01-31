/* Code from Learning OpenCV */
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>

int n_boards = 0;		//Number of snapshots of the chessboard
int cam_index;			//Frames to be skipped
int board_w;			//Enclosed corners horizontally on the chessboard
int board_h;			//Enclosed corners vertically on the chessboard

int main(){
  setbuf(stdout,NULL);
  CvCapture* capture;

  printf("Camera index = ");
  scanf("%d",&cam_index);

  printf("Numbers of spanspots = ");
  scanf("%d",&n_boards);

  board_w  = 9;
  board_h  = 6;
  
  int board_total  = board_w * board_h;		   //Total enclosed corners on the board
  CvSize board_sz = cvSize( board_w, board_h );

  capture = cvCreateCameraCapture(cam_index);
  if(!capture){
    printf("\nCouldn't open the camera\n"); 
    return -1;
  }

  //Allocate storage for the parameters according to total number of corners and number of snapshots
  CvMat* image_points      = cvCreateMat(n_boards*board_total,2,CV_32FC1);
  CvMat* object_points     = cvCreateMat(n_boards*board_total,3,CV_32FC1);
  CvMat* point_counts      = cvCreateMat(n_boards,1,CV_32SC1);
  CvMat* intrinsic_matrix  = cvCreateMat(3,3,CV_32FC1);
  CvMat* distortion_coeffs = cvCreateMat(4,1,CV_32FC1);

  //Note:
  //Intrinsic Matrix - 3x3    Lens Distorstion Matrix - 4x1
  //	[fx 0 cx]	      [k1 k2 p1 p2   k3(optional)]
  //	[0 fy cy]
  //	[0  0  1]

  CvPoint2D32f* corners = malloc( sizeof(CvPoint2D32f) * board_total );
  int corner_count;
  int successes = 0;
  int step, frame = 0;

  cvNamedWindow("Snapshot", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Raw Video", CV_WINDOW_AUTOSIZE);

  IplImage *image = cvQueryFrame(capture);
  IplImage *gray_image = cvCreateImage(cvGetSize(image),8,1);

  printf("Press Enter to Capture.\n",successes,n_boards);
  while(cvWaitKey(10) == -1){
    cvShowImage( "Raw Video", image);
    image = cvQueryFrame(capture);
  }

  while(1){
    //Find chessboard corners:
    int found = cvFindChessboardCorners
      (image, board_sz, corners, &corner_count,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

    cvCvtColor(image, gray_image, CV_BGR2GRAY);
    //Get Subpixel accuracy on those corners
    cvFindCornerSubPix(gray_image, corners, corner_count, cvSize(11,11),cvSize(-1,-1), 
                       cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

    cvDrawChessboardCorners(image, board_sz, corners, corner_count, found);

    // If we got a good board, add it to our data
    if( corner_count == board_total ){
      cvShowImage( "Snapshot", image );
      //show in color if we did collect the image
      step = successes*board_total;
      int i;
      int j;
      for( i=step, j=0; j<board_total; ++i,++j ) {
        CV_MAT_ELEM(*image_points, float,i,0) = corners[j].x;
        CV_MAT_ELEM(*image_points, float,i,1) = corners[j].y;
        CV_MAT_ELEM(*object_points,float,i,0) = (float) j/board_w;
        CV_MAT_ELEM(*object_points,float,i,1) = (float) (j%board_w);
        CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
      }
      CV_MAT_ELEM(*point_counts, int,successes,0) = board_total;    
      successes++;
      printf("\r%d successful Snapshots out of %d collected.",successes,n_boards);
    }else{
      //Show Gray if we didn't collect the image
      printf("\rUnsuccessful snapshot.",successes,n_boards);
      cvShowImage( "Snapshot", gray_image );
    }

    if(successes >= n_boards)
      break;

    printf(" Press Any Key to Capture.\n",successes,n_boards);
    
    while(cvWaitKey(10) == -1){
      cvShowImage( "Raw Video", image);
      image = cvQueryFrame( capture );
    }
  }

  cvDestroyWindow("Snapshot");

  printf("\n\n *** Calbrating the camera now...\n");
	
  //Allocate matrices according to successful number of captures
  CvMat* object_points2  = cvCreateMat(successes*board_total,3,CV_32FC1);
  CvMat* image_points2   = cvCreateMat(successes*board_total,2,CV_32FC1);
  CvMat* point_counts2   = cvCreateMat(successes,1,CV_32SC1);

  //Tranfer the points to matrices
  int i;
  for(i = 0; i<successes*board_total; ++i){
    CV_MAT_ELEM( *image_points2, float, i, 0)  =  CV_MAT_ELEM( *image_points, float, i, 0);
    CV_MAT_ELEM( *image_points2, float,i,1)    =  CV_MAT_ELEM( *image_points, float, i, 1);
    CV_MAT_ELEM( *object_points2, float, i, 0) =  CV_MAT_ELEM( *object_points, float, i, 0);
    CV_MAT_ELEM( *object_points2, float, i, 1) =  CV_MAT_ELEM( *object_points, float, i, 1);
    CV_MAT_ELEM( *object_points2, float, i, 2) =  CV_MAT_ELEM( *object_points, float, i, 2);
  } 
	
  for(i=0; i<successes; ++i){ 
    //These are all the same number
    CV_MAT_ELEM( *point_counts2, int, i, 0) =  CV_MAT_ELEM( *point_counts, int, i, 0);
  }

  cvReleaseMat(&object_points);
  cvReleaseMat(&image_points);
  cvReleaseMat(&point_counts);

  //Initialize the intrinsic matrix with both the two focal lengths in a ratio of 1.0
  CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
  CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;

  //Calibrate the camera
  cvCalibrateCamera2(object_points2, image_points2, point_counts2,  cvGetSize( image ), 
                     intrinsic_matrix, distortion_coeffs, NULL, NULL,0 );
  //CV_CALIB_FIX_ASPECT_RATIO

  //Save values to file
  printf(" *** Calibration Done!\n\n");
  printf("Storing Intrinsics.xml and Distortions.xml files...\n");
  cvSave("Intrinsics.xml",intrinsic_matrix,NULL,NULL,cvAttrList(NULL,NULL));
  cvSave("Distortion.xml",distortion_coeffs,NULL,NULL,cvAttrList(NULL,NULL));
  printf("Files saved.\n\n");

  printf("Starting corrected display....\n");

  //Sample: load the matrices from the file
  CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml", NULL, NULL, NULL);
  CvMat *distortion = (CvMat*)cvLoad("Distortion.xml", NULL, NULL, NULL);

  // Build the undistort map used for all subsequent frames.
  IplImage* mapx = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
  IplImage* mapy = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
  cvInitUndistortMap(intrinsic,distortion,mapx,mapy);

  // Run the camera to the screen, showing the raw and the undistorted image.
  cvNamedWindow( "Undistort", CV_WINDOW_AUTOSIZE);

  while(image){
    IplImage *t = cvCloneImage(image);
    cvShowImage( "Raw Video", image );			// Show raw image
    cvRemap( t, image, mapx, mapy, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0));  // Undistort image
    cvReleaseImage(&t);
    cvShowImage("Undistort", image);			// Show corrected image

    //Handle pause/unpause and ESC
    int c = cvWaitKey(15);
    if(c == 'p'){ 
      c = 0;
      while(c != 'p' && c != 27)
        c = cvWaitKey(250);  
    }

    if(c == 27)
      break;

    image = cvQueryFrame( capture );
  }
  return 0;
}
