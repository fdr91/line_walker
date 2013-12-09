/**
 * @file display_stage.c
 * @author nicolas.brulez@parrot.com
 * @date 2012/09/25
 *
 * This stage is a naive example of how to display video using GTK2 + Cairo
 * In a complete application, all GTK handling (gtk main thread + widgets/window creation)
 *  should NOT be handled by the video pipeline (see the Navigation linux example)
 *
 * The window will be resized according to the picture size, and should not be resized bu the user
 *  as we do not handle any gtk event except the expose-event
 *
 * This example is not intended to be a GTK/Cairo tutorial, it is only an example of how to display
 *  the AR.Drone live video feed. The GTK Thread is started here to improve the example readability
 *  (we have all the gtk-related code in one file)
 */

// Self header fil/
#include "display_stage.h"

// OpenCV headers
#include <opencv/cv.h>
#include <opencv/highgui.h>

// Function for convert image to OpenCV format

IplImage *ipl_image_from_data(uint8_t* data, int reduced_image, int width, int height)
{
  IplImage *currframe;
  IplImage *dst;

  currframe = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);
  dst = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);

  currframe->imageData = data;
  cvCvtColor(currframe, dst, CV_BGR2RGB);
  cvReleaseImage(&currframe);

  return dst;
}

// Funcs pointer definition
const vp_api_stage_funcs_t display_stage_funcs = {
    NULL,
    (vp_api_stage_open_t) display_stage_open,
    (vp_api_stage_transform_t) display_stage_transform,
    (vp_api_stage_close_t) display_stage_close
};

// Extern so we can make the ardrone_tool_exit() function (ardrone_testing_tool.c)
// return TRUE when we close the video window
extern int exit_program;


// Picture size getter from input buffer size
// This function only works for RGB565 buffers (i.e. 2 bytes per pixel)
static void getPicSizeFromBufferSize (uint32_t bufSize, uint32_t *width, uint32_t *height)
{
    if (NULL == width || NULL == height)
    {
        return;
    }

    switch (bufSize)
    {
    case 50688: //QCIF > 176*144 *2bpp
        *width = 176;
        *height = 144;
        break;
    case 153600: //QVGA > 320*240 *2bpp
        *width = 320;
        *height = 240;
        break;
    case 691200: //360p > 640*360 *3bpp
        *width = 640;
        *height = 360;
        break;
    case 2764800: //720p > 1280*720 *3bpp
        *width = 1280;
        *height = 720;
        break;
    default:
        *width = 0;
        *height = 0;
        break;
    }
}

// Get actual frame size (without padding)
void getActualFrameSize (display_stage_cfg_t *cfg, uint32_t *width, uint32_t *height)
{
    if (NULL == cfg || NULL == width || NULL == height)
    {
        return;
    }

    *width = cfg->decoder_info->width;
    *height = cfg->decoder_info->height;
}


C_RESULT display_stage_open (display_stage_cfg_t *cfg)
{
    // Check that we use RGB565
    if (2 != cfg->bpp)
    {
        // If that's not the case, then don't display anything
        cfg->paramsOK = FALSE;
    }
    else
    {
        // Else, start GTK thread and window
        cfg->paramsOK = TRUE;
        cfg->frameBuffer = NULL;
        cfg->fbSize = 0;
        START_THREAD (gtk, cfg);
    }
    return C_OK;
}


IplImage* prepare_image(const IplImage* src)
{
    IplImage* dst=cvCreateImage(cvSize(src->width,src->height),IPL_DEPTH_8U,1);
    cvCvtColor(src,dst,CV_BGR2GRAY);
    cvInRangeS(dst,cvScalar(0,0,0,0),cvScalar(17,0,0,0),dst);

    return dst;
}

CvSeq* find_black_square(IplImage* src)
{
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* contours=0;
    cvFindContours(src, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

    float max_area=0;
    CvSeq* max_cont_seq=NULL;
    CvSeq* seq0;
    for(seq0 = contours; seq0!=0;seq0 = seq0->h_next)
    {
            float tmp = fabs(cvContourArea(seq0, CV_WHOLE_SEQ, 0));
            if(tmp>max_area)
            {
                max_area = tmp;
                max_cont_seq = seq0;
            }
    }
    return max_cont_seq;
}



CvPoint find_cont_center(const CvSeq* cont)
{
    FILE * fo;
    fo = fopen("gc_find","wt");
    if(cont==NULL)
        return cvPoint(-1,-1);
    CvMoments moments;
    cvMoments(cont,&moments,0);
    CvPoint gravity_center;
    if(moments.m00!=0)
    {
        gravity_center.x=moments.m10/moments.m00;
        gravity_center.y=moments.m01/moments.m00;
    }
    else
    {
        gravity_center.x=-1;
        gravity_center.y=-1;
    }
    fprintf("gc.x=%3d, gc.y=%3d", gravity_center.x, gravity_center.y);
    fclose(fo);
    return gravity_center;

}



C_RESULT display_stage_transform (display_stage_cfg_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
    uint32_t width = 0, height = 0;

    static CvPoint prev_gravity_center={.x=320, .y=220};
    getPicSizeFromBufferSize (in->size, &width, &height);
	bottom_image = ipl_image_from_data((uint8_t*)in->buffers[0], 1, width, height);

	IplImage* binary=prepare_image(bottom_image);
	CvSeq* cont=find_black_square(binary);

	vp_os_mutex_lock(&gravity_center_lock);
    gravity_center=find_cont_center(cont);
    if(gravity_center.x<0&&gravity_center.y<0)
        gravity_center=prev_gravity_center;
    vp_os_mutex_unlock(&gravity_center_lock);
    prev_gravity_center=gravity_center;
	IplImage* tmp=cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);
	cvSetZero(tmp);
	cvCircle(tmp, gravity_center, 3, cvScalar(255, 255, 0, 0), 1, 8, 0);
	cvDrawContours(tmp, cont, CV_RGB(255,216,0), CV_RGB(0,0,250), 0, 1, 8, cvPoint(0,0)); // рисуем контур
    cvNamedWindow("square", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("bottom", CV_WINDOW_AUTOSIZE);
    cvShowImage("square", tmp);
    cvShowImage("bottom", bottom_image);
    int tmp1;
    tmp1=cvWaitKey(10);
    vp_os_mutex_lock(&key_update_lock);
    control_var=tmp1;
    vp_os_mutex_unlock(&key_update_lock);
    /*if((control_var&255)==27)
        exit_program=0;*/
    cvReleaseImage(&bottom_image);
    cvReleaseImage(&tmp);
    cvReleaseImage(&binary);
    return C_OK;
}

C_RESULT display_stage_close (display_stage_cfg_t *cfg)
{
    // Free all allocated memory
    if (NULL != cfg->frameBuffer)
    {
        vp_os_free (cfg->frameBuffer);
        cfg->frameBuffer = NULL;
    }

    return C_OK;
}
