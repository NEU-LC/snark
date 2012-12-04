/**************************************************************************
 *
 * Title:   xb3stereo
 * Copyright:   (C) 2006,2007,2008 Don Murray donm@ptgrey.com
 *
 * Description:
 *
 *    Get an image set from a Bumblebee or Bumblebee2 via DMA transfer
 *    using libdc1394 and process it with the Triclops stereo
 *    library. Based loosely on 'grab_gray_image' from libdc1394 examples.
 *
 *-------------------------------------------------------------------------
 *     License: LGPL
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *************************************************************************/

//=============================================================================
// Copyright © 2006,2007,2008 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research Inc.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//
//=============================================================================

//=============================================================================
//
// xb3stereo.cpp
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include <dc1394/conversions.h>
#include <dc1394/control.h>
#include <dc1394/utils.h>


//=============================================================================
// PGR Includes
//=============================================================================
#include <snark/imaging/stereo/ptgrey/pgr_registers.h>
#include <snark/imaging/stereo/ptgrey/pgr_stereocam.h>
#include "pnmutils.h"

//=============================================================================
// Copy an RGB buffer to a TriclopsInput
//
// NOTE: this is unfortunate and inefficient.  All libdc1394 functions
// that deal with RGB images expect them to be RGB only.  I.e. RGBRGBRGB...
// Triclops library expects RGB images to be RGBU (where U is unused).
// The advantage of RGBU is that the pixels are word aligned.
// The disadvantage is obviously they take 33% more memory.
//
// Ideally, to make things more efficient, one should convert either
// the libdc1394 color conversions to be able to make RGBU images, or
// change the Triclops library to accept RGB images.
//
// For now, we are saddled with this extra copy! :(
//
// NOTE: it is also assumed that the TriclopsInput contains valid size
// information so it knows how big the RGB buffer is.  And that the
// TriclopsInput is of type RGB_32BIT_PACKED and that its memory is
// already allocated
//
void
convertColorTriclopsInput( TriclopsInput* colorInput,
               unsigned char* pucRGB )
{
   unsigned char* pucInputData = (unsigned char*) colorInput->u.rgb32BitPacked.data;
   for ( int i = 0, j = 0; i < colorInput->nrows * colorInput->ncols*3; )
   {
      // get R, G and B
      pucInputData[j+2] = pucRGB[i++];
      pucInputData[j+1] = pucRGB[i++];
      pucInputData[j] = pucRGB[i++];
      // increment the Input counter once more to skip the "U" byte
      j += 4;
   }
   return;
}

//=============================================================================
// a simple function to write a .pgm file
int
writePgm( char*     szFilename,
     unsigned char* pucBuffer,
     int        width,
     int        height )
{
  FILE* stream;
  stream = fopen( szFilename, "wb" );
  if( stream == NULL)
  {
     perror( "Can't open image file" );
     return 1;
  }

  fprintf( stream, "P5\n%u %u 255\n", width, height );
  fwrite( pucBuffer, width, height, stream );
  fclose( stream );
  return 0;
}

//=============================================================================
// a simple function to write a .ppm file
int
writePpm( char*     szFilename,
     unsigned char* pucBuffer,
     int        width,
     int        height )
{
  FILE* stream;
  stream = fopen( szFilename, "wb" );
  if( stream == NULL)
  {
     perror( "Can't open image file" );
     return 1;
  }

  fprintf( stream, "P6\n%u %u 255\n", width, height );
  fwrite( pucBuffer, 3*width, height, stream );
  fclose( stream );
  return 0;
}


//=============================================================================
// cleanup_and_exit()
// This is called when the program exits and destroys the existing connections
// to the 1394 drivers
void
cleanup_and_exit( dc1394camera_t* camera )
{
   dc1394_capture_stop( camera );
   dc1394_video_set_transmission( camera, DC1394_OFF );
   dc1394_camera_free( camera );
   exit( 0 );
}



//=============================================================================
// MAIN
//
int main( int argc, char *argv[] )
{


   dc1394camera_t*  camera;
   dc1394error_t    err;

   char*        szShortCal  = NULL;
   char*        szWideCal   = NULL;
   char*        szShortDisp = NULL;
   char*        szWideDisp  = NULL;
   char*        szColorRect = NULL;

   switch( argc )
   {
      default:
     printf( "Usage: %s <short.cal> <wide.cal> "
         "[wide-disp-out short-disp-out col-rect-out]\n",
         argv[0] );
     return 1;
      case 6:
     szWideDisp = argv[3];
     szShortDisp    = argv[4];
     szColorRect    = argv[5];
     // fall through to case 3 is deliberate
      case 3:
     szShortCal = argv[1];
     szWideCal  = argv[2];
     break;
   }

   //===================================================================
   // read in the TriclopsContexts
   TriclopsError e;
   TriclopsContext wideTriclops;
   TriclopsContext shortTriclops;
   printf( "Getting TriclopsContexts from files... \n" );
   e = triclopsGetDefaultContextFromFile( &shortTriclops,
                      szShortCal );
   if ( e != TriclopsErrorOk )
   {
      fprintf( stderr, "Can't get short context from file\n" );
      return 1;
   }

   e = triclopsGetDefaultContextFromFile( &wideTriclops,
                      szWideCal );
   if ( e != TriclopsErrorOk )
   {
      fprintf( stderr, "Can't get wide context from file\n" );
      return 1;
   }
   printf( "...done\n" );

   // make sure we are in subpixel mode
   triclopsSetSubpixelInterpolation( wideTriclops, 1 );
   triclopsSetSubpixelInterpolation( shortTriclops, 1 );


   //===================================================================
   // Find cameras on the 1394 buses
    dc1394_t * d;
   dc1394camera_list_t * list;
   unsigned int nThisCam;


   d = dc1394_new ();

     // Enumerate cameras connected to the PC
   err = dc1394_camera_enumerate (d, &list);

   if ( err != DC1394_SUCCESS )
   {
      fprintf( stderr, "Unable to look for cameras\n\n"
           "Please check \n"
           "  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded \n"
           "  - if you have read/write access to /dev/raw1394\n\n");
      return 1;
   }

   //  get the camera nodes and describe them as we find them
   if (list->num == 0)
   {
      fprintf( stderr, "No cameras found!\n");
      return 1;
   }



   printf( "There were %d camera(s) found attached to your PC\n", list->num  );

    // Identify cameras. Use the first stereo camera that is found
    for ( nThisCam = 0; nThisCam < list->num; nThisCam++ )
    {
        camera = dc1394_camera_new(d, list->ids[nThisCam].guid);

        if(!camera)
        {
            printf("Failed to initialize camera with guid %llx", list->ids[nThisCam].guid);
            continue;
        }

        printf( "Camera %d model = '%s'\n", nThisCam, camera->model );

        if ( isStereoCamera(camera))
        {

         printf( "Using this camera\n" );
         break;
        }

        dc1394_camera_free(camera);
   }

   if ( nThisCam == list->num )
   {
      printf( "No stereo cameras were detected\n" );
      return 0;
   }


   PGRStereoCamera_t stereoCamera;

   // query information about this stereo camera
   err = queryStereoCamera( camera, &stereoCamera );
   if ( err != DC1394_SUCCESS )
   {
      fprintf( stderr, "Cannot query all information from camera\n" );
      cleanup_and_exit( camera );
   }

   if ( stereoCamera.model != BUMBLEBEEXB3 )
   {
      fprintf( stderr, "Stereo camera found was not a BB XB3\n" );
      cleanup_and_exit( camera );
   }

   // override the default nBytesPerPixel to be 3, because we really do
   // want to run at 3 camera mode
   stereoCamera.nBytesPerPixel = 3;

   // set the capture mode
   printf( "Setting stereo video capture mode\n" );
   err = setStereoVideoCapture( &stereoCamera );
   if ( err != DC1394_SUCCESS )
   {
      fprintf( stderr, "Could not set up video capture mode\n" );
      cleanup_and_exit( stereoCamera.camera );
   }

   // have the camera start sending us data
   printf( "Start transmission\n" );
   err = startTransmission( &stereoCamera );
   if ( err != DC1394_SUCCESS )
   {
      fprintf( stderr, "Unable to start camera iso transmission\n" );
      cleanup_and_exit( stereoCamera.camera );
   }

   //===================================================================
   // Allocate all the buffers.
   // Unfortunately color processing is a bit inefficient because of the
   // number of data copies.  Color data needs to be
   // - de-interleaved into separate bayer tile images
   // - color processed into RGB images
   // - de-interleaved to extract the green channel for stereo (or other
   //   mono conversion)

   // size of buffer for all images at mono8
   unsigned int   nBufferSize = stereoCamera.nRows *
                                stereoCamera.nCols *
                                stereoCamera.nBytesPerPixel;

   // allocate a buffer to hold the de-interleaved images
   unsigned char* pucDeInterlacedBuffer = new unsigned char[ nBufferSize ];



   // define all the buffers you could ever want, color or mono
   // leave allocation till later
   // color buffers
   // buffer for the 3 color images color processed and stacked
   unsigned char* pucRGBBuffer      = NULL;
   // buffer for the green channel that approximates the mono signal
   unsigned char* pucGreenBuffer    = NULL;
   unsigned char* pucRightRGB       = NULL;
   unsigned char* pucLeftRGB        = NULL;
   unsigned char* pucCenterRGB      = NULL;

   // mono buffers
   unsigned char* pucRightMono      = NULL;
   unsigned char* pucLeftMono       = NULL;
   unsigned char* pucCenterMono     = NULL;
   TriclopsInput  colorInput;

   // allocation color processing buffers
   if ( stereoCamera.bColor )
   {
      pucRGBBuffer      = new unsigned char[ 3 * nBufferSize ];
      pucGreenBuffer        = new unsigned char[ nBufferSize ];
      // allocate a color input for color image rectification
      colorInput.nrows      = stereoCamera.nRows;
      colorInput.ncols      = stereoCamera.nCols;
      colorInput.rowinc     = stereoCamera.nCols*4;
      colorInput.inputType  = TriInp_RGB_32BIT_PACKED;
      colorInput.u.rgb32BitPacked.data
     = (void*) (new unsigned char[colorInput.nrows*colorInput.rowinc]);
   }



   //===================================================================
   //
   //
   // MAIN PROCESSING LOOP
   //
   //

   //for ( int i = 0; i < 100; i++ )
   for ( int i = 0; i < 10; i++ )
   {

      printf( "Grab %d\n", i );

      TriclopsInput wideInput, shortInput;
      if ( stereoCamera.bColor )
      {
     // get the images from the capture buffer and do all required
     // processing
     extractImagesColorXB3( &stereoCamera,
                DC1394_BAYER_METHOD_NEAREST,
                pucDeInterlacedBuffer,
                pucRGBBuffer,
                pucGreenBuffer,
                &pucRightRGB,
                &pucLeftRGB,
                &pucCenterRGB,
                &shortInput,
                &wideInput );
     // save the raw images
     char filename[100];
     sprintf( filename, "right-raw-02%d.ppm", i );
     writePpm( filename, pucRightRGB, 1280, 960 );
     sprintf( filename, "center-raw-02%d.ppm", i );
     writePpm( filename, pucCenterRGB, 1280, 960 );
     sprintf( filename, "left-raw-02%d.ppm", i );
     writePpm( filename, pucLeftRGB, 1280, 960 );
      }
      else
      {
     // get the images from the capture buffer and do all required
     // processing
     extractImagesMonoXB3( &stereoCamera,
                   pucDeInterlacedBuffer,
                   &pucRightMono,
                   &pucLeftMono,
                   &pucCenterMono,
                   &shortInput,
                   &wideInput );
      }

      // do stereo processing on both image sets
      e = triclopsRectify( wideTriclops, &wideInput );
      if ( e != TriclopsErrorOk )
      {
     fprintf( stderr, "triclopsRectify failed!\n" );
     break;
      }

      e = triclopsStereo( wideTriclops );
      if ( e != TriclopsErrorOk )
      {
     fprintf( stderr, "triclopsRectify failed!\n" );
     break;
      }

      e = triclopsRectify( shortTriclops, &shortInput );
      if ( e != TriclopsErrorOk )
      {
     fprintf( stderr, "triclopsRectify failed!\n" );
     break;
      }

      e = triclopsStereo( shortTriclops );
      if ( e != TriclopsErrorOk )
      {
     fprintf( stderr, "triclopsRectify failed!\n" );
     break;
      }

      // get pointers to all the images for your own processing requirements
      TriclopsImage shortRectifiedRight, shortRectifiedLeft;
      TriclopsImage wideRectifiedRight, wideRectifiedLeft;
      TriclopsImage16 shortDisparity, wideDisparity;

      triclopsGetImage( shortTriclops, TriImg_RECTIFIED,
            TriCam_RIGHT, &shortRectifiedRight );
      triclopsGetImage( shortTriclops, TriImg_RECTIFIED,
            TriCam_LEFT, &shortRectifiedLeft );
      triclopsGetImage( wideTriclops, TriImg_RECTIFIED,
            TriCam_RIGHT, &wideRectifiedRight );
      triclopsGetImage( wideTriclops, TriImg_RECTIFIED,
            TriCam_LEFT, &wideRectifiedLeft );
      triclopsGetImage16( shortTriclops, TriImg16_DISPARITY,
              TriCam_REFERENCE, &shortDisparity );
      triclopsGetImage16( wideTriclops, TriImg16_DISPARITY,
              TriCam_REFERENCE, &wideDisparity );

      // now - what to do with all these images?  Well, for now to prove
      // that they are actually valid, lets save them

      char filename[100];
      sprintf( filename, "short-right-rectified-%02d.pgm", i );
      pgmWriteFromTriclopsImage( filename, &shortRectifiedRight );
      sprintf( filename, "short-left-rectified-%02d.pgm", i );
      pgmWriteFromTriclopsImage( filename, &shortRectifiedLeft );
      sprintf( filename, "wide-right-rectified-%02d.pgm", i );
      pgmWriteFromTriclopsImage( filename, &wideRectifiedRight );
      sprintf( filename, "wide-left-rectified-%02d.pgm", i );
      pgmWriteFromTriclopsImage( filename, &wideRectifiedLeft );

      sprintf( filename, "short-disparity-%02d.pgm", i );
      pgmWriteFromTriclopsImage16( filename, &shortDisparity );
      sprintf( filename, "wide-disparity-%02d.pgm", i );
      pgmWriteFromTriclopsImage16( filename, &wideDisparity );




      // rectify the right image with both the short and wide contexts
      if ( stereoCamera.bColor )
      {

     TriclopsColorImage shortRectifiedColor;
     TriclopsColorImage wideRectifiedColor;
     // copy the RGB buffer into an input structure
     convertColorTriclopsInput( &colorInput, pucRightRGB );
     triclopsRectifyColorImage( shortTriclops,
                    TriCam_RIGHT,
                    &colorInput,
                    &shortRectifiedColor );
     triclopsRectifyColorImage( wideTriclops,
                    TriCam_RIGHT,
                    &colorInput,
                    &wideRectifiedColor );

     sprintf( filename, "short-color-right-%02d.pgm", i );
     ppmWriteFromTriclopsColorImage( filename, &shortRectifiedColor );
     sprintf( filename, "wide-color-right-%02d.pgm", i );
     ppmWriteFromTriclopsColorImage( filename, &wideRectifiedColor );

      }
   }

   printf( "Done processing loops\n" );
   printf( "Stop transmission\n" );
   //  Stop data transmission
   if ( dc1394_video_set_transmission( stereoCamera.camera, DC1394_OFF ) != DC1394_SUCCESS )
   {
      fprintf( stderr, "Couldn't stop the camera?\n" );
   }


   delete[] pucDeInterlacedBuffer;
   // delete color processing buffers if req'd
   if ( pucRGBBuffer )
      delete[] pucRGBBuffer;
   if ( pucGreenBuffer )
      delete[] pucGreenBuffer;

   // close camera
   cleanup_and_exit( camera );

   return 0;
}

