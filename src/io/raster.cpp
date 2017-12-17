/*
 *  Copyright (c) 2015  Balint Cristian (cristian.balint@gmail.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

/* raster.cpp */
/* Raster I/O */

#include <math.h>

#include "gdal.h"
#include "gdal_priv.h"
#include "cpl_string.h"
#include "cpl_csv.h"

#include <opencv2/opencv.hpp>

#include "superpixel.hpp"

using namespace cv;



void LoadRaster( std::string InFilename, std::vector< cv::Mat >& raster )
{
  int rasters = 0;
  int channel = 0;

  std::string prev_dType = "";
  int prev_XSize = 0, prev_YSize = 0;

    GDALDataset* piDataset;

    // open the dataset
    piDataset = (GDALDataset*) GDALOpen(InFilename.c_str(), GA_ReadOnly);

    if( piDataset == NULL )
    {
      printf("\nERROR: Couldn't open dataset %s\n",InFilename.c_str());
      exit( 1 );
    }

    if( piDataset->GetGCPCount() > 0 )
    {
      printf("\nERROR: Cannot handle raster with GCP points.\n");
      exit( 1 );
    }

    rasters++;
    printf ("\nLoading %s \n", InFilename.c_str());

    // count raster bands
    int nBands = piDataset->GetRasterCount();
    printf ("  Scene have [%i] bands\n", nBands);


    // gather all bands
    for ( int iB = 0; iB < nBands; iB++ )
    {

      int iX, iY;
      int iXBlock, iYBlock;
      int nXBlockSize, nYBlockSize;

      cv::Mat Channel;
      cv::Mat pabyData;
      GDALDataType rType;

      // get parameters from input dataset
      const int nXSize = piDataset->GetRasterBand(iB+1)->GetXSize();
      const int nYSize = piDataset->GetRasterBand(iB+1)->GetYSize();
      piDataset->GetRasterBand(iB+1)->GetBlockSize( &nXBlockSize, &nYBlockSize );

      rType = piDataset->GetRasterBand(iB+1)->GetRasterDataType();

      int nXBlocks = (nXSize + nXBlockSize - 1) / nXBlockSize;
      int nYBlocks = (nYSize + nYBlockSize - 1) / nYBlockSize;

      std::string dType;

      switch( rType )
      {
        case GDT_Byte:
          dType = "Byte";
          Channel = cv::Mat( nYSize, nXSize, CV_8U);
          pabyData = cv::Mat( nYBlockSize, nXBlockSize, CV_8U);
          break;

        case GDT_UInt16:
          dType = "UInt16";
          Channel = cv::Mat( nYSize, nXSize, CV_16U);
          pabyData = cv::Mat( nYBlockSize, nXBlockSize, CV_16U);
          break;

        case GDT_Int16:
          dType = "Int16";
          Channel = cv::Mat( nYSize, nXSize, CV_16S);
          pabyData = cv::Mat( nYBlockSize, nXBlockSize, CV_16S);
          break;

        case GDT_Int32:
          dType = "Int32";
          Channel = cv::Mat( nYSize, nXSize, CV_32S);
          pabyData = cv::Mat( nYBlockSize, nXBlockSize, CV_32S);
          break;

        case GDT_Float32:
          dType = "Float32";
          Channel = cv::Mat( nYSize, nXSize, CV_32F);
          pabyData = cv::Mat( nYBlockSize, nXBlockSize, CV_32F);
          break;

        case GDT_Float64:
          dType = "Float64";
          Channel = cv::Mat( nYSize, nXSize, CV_64F);
          pabyData = cv::Mat( nYBlockSize, nXBlockSize, CV_64F);
          break;

        default:
          printf ("\nERROR: Unsupported raster data type.\n");
          exit ( 1 );
      }

      channel++;

      // consistency
      if ( channel > 1 )
      {
        if ( ( prev_XSize != nXSize )
           ||( prev_YSize != nYSize ) )
        {
          printf ("\nERROR: CH #%i has different size: (%iP x %iL) than previous (%iP x %iL).\n",
                  channel, nXSize, nYSize, prev_XSize, prev_YSize);
          exit ( 1 );
        }
        if ( prev_dType != dType )
        {
          printf ("\nERROR: CH #%i has different data type [%s] then previous [%s]\n",
                 channel, dType.c_str(), prev_dType.c_str());
          exit ( 1 );
        }
      }

      // store previous
      prev_dType = dType;
      prev_XSize = nXSize;
      prev_YSize = nYSize;

      printf ("  CH: #%03i chann: (#%i Band | [%s])\n", channel, iB+1, dType.c_str());
      printf ("           areas: (%i Pixels x %i Lines) pixels\n", nXSize, nYSize);
      printf ("           tiles: (%i Columns x %i Rows) blocks\n", nXBlocks, nYBlocks);
      printf ("           block: (%i Pixels x %i Lines) pixels / tile\n", nXBlockSize, nYBlockSize);
      printf ("           ");

      CPLErr error;

      for( iYBlock = 0; iYBlock < nYBlocks; iYBlock++ )
      {
          for( iXBlock = 0; iXBlock < nXBlocks; iXBlock++ )
          {
               int nXValid, nYValid;

               // compute the portion of the block that is valid
               // for partial edge blocks.
               if( (iXBlock+1) * nXBlockSize > nXSize )
                 nXValid = nXSize - iXBlock * nXBlockSize;
               else
                 nXValid = nXBlockSize;

               if( (iYBlock+1) * nYBlockSize > nYSize )
                 nYValid = nYSize - iYBlock * nYBlockSize;
               else
                 nYValid = nYBlockSize;

               error = piDataset->GetRasterBand(iB+1)->ReadBlock( iXBlock, iYBlock, pabyData.data );

               if ( error != CE_None )
                 printf("ERROR: GetRasterBand()\n");

               // cache some computations
               const int iXAllBlocks = iXBlock * nXBlockSize;
               const int iYAllBlocks = iYBlock * nYBlockSize;

               for (iY = 0; iY < nYValid; iY++)
               {
                    // cache some computations
                    const int iYOffset = iY * nXBlockSize;
                    const int iYShifts = iY + iYAllBlocks;

                    for (iX = 0; iX < nXValid; iX++)
                    {
                         switch( rType )
                         {
                           case GDT_Byte:
                             Channel.at<uchar>( iYShifts, iX + iXAllBlocks )
                                       = pabyData.at<uchar>( iYOffset + iX );
                             break;

                           case GDT_UInt16:
                             Channel.at<ushort>( iYShifts, iX + iXAllBlocks )
                                       = pabyData.at<ushort>( iYOffset + iX );
                             break;

                           case GDT_Int16:
                             Channel.at<short>( iYShifts, iX + iXAllBlocks )
                                       = pabyData.at<short>( iYOffset + iX );
                             break;

                           case GDT_Int32:
                             Channel.at<int>( iYShifts, iX + iXAllBlocks )
                                       = pabyData.at<int>( iYOffset + iX );
                             break;

                           case GDT_Float32:
                             Channel.at<float>( iYShifts, iX + iXAllBlocks )
                                       = pabyData.at<float>( iYOffset + iX );
                             break;

                           case GDT_Float64:
                             Channel.at<double>( iYShifts, iX + iXAllBlocks )
                                       = pabyData.at<double>( iYOffset + iX );
                             break;

                           default:
                             printf ("\nERROR: Unsupported raster data type.\n");
                             exit ( 1 );
                         }
                    }
               }
          }
          GDALTermProgress( (float)((iYBlock+1) / (float)nYBlocks), NULL, NULL);
      }
      pabyData.empty();
      raster.push_back(Channel);
      GDALTermProgress( 1.0f, NULL, NULL );
    }
    GDALTermProgress( 1.0f, NULL, NULL );
    GDALClose( (GDALDatasetH) piDataset );
  
}

void ComputeStats( const cv::Mat klabels,
                   const std::vector< cv::Mat > raster,
                   cv::Mat& labelpixels, 
                   cv::Mat& avgCH, 
                   cv::Mat& stdCH, 
                   cv::Mat& varCH )
{
  avgCH = Scalar::all(0);
  stdCH = Scalar::all(0);
  labelpixels = Scalar::all(0);

  const int m_bands = (int) raster.size();
  Mat sumCH = Mat::zeros(m_bands, klabels.cols*klabels.rows, CV_64F);

  printf ("Compute Statistics\n");

  printf ("       Computing CLASS polygons intensity\n");
  printf ("       ");
  for (int y = 0; y < klabels.rows; y++)
  {
      const int yklabels = y * klabels.cols;
      for (int x = 0; x < klabels.cols; x++)
      {
          const int i = yklabels + x;
          const int k = klabels.at<u_int32_t>(i);
          // gather how many pixels per class we have
          labelpixels.at<u_int32_t>( klabels.at<u_int32_t>(i) )++;
          // summ all pixel intensities
          #pragma omp parallel for schedule(dynamic)
          for (int b = 0; b < m_bands; b++)
          {
            switch ( raster[b].depth() )
            {
              case CV_8U:
                sumCH.at<double>(b,k) += (double) raster[b].at<uchar>(i);
                break;
              case CV_8S:
                sumCH.at<double>(b,k) += (double) raster[b].at<char>(i);
                break;
              case CV_16U:
                sumCH.at<double>(b,k) += (double) raster[b].at<ushort>(i);
                break;
              case CV_16S:
                sumCH.at<double>(b,k) += (double) raster[b].at<short>(i);
                break;
              case CV_32S:
                sumCH.at<double>(b,k) += (double) raster[b].at<u_int32_t>(i);
                break;
              case CV_32F:
                sumCH.at<double>(b,k) += (double) raster[b].at<float>(i);
                break;
              case CV_64F:
                sumCH.at<double>(b,k) += (double) raster[b].at<double>(i);
                break;
              default:
                CV_Error( Error::StsInternal, "\nERROR: Invalid raster depth" );
                break;
            }
          }
      }
      GDALTermProgress( (float)(y+1) / (float)(klabels.rows), NULL, NULL );
  }
  GDALTermProgress( 1.0f, NULL, NULL );

  printf ("       Computing CLASS averaged intensity\n");
  printf ("       ");
  for (int k = 0; k < labelpixels.rows; k++)
  {
      #pragma omp parallel for schedule(dynamic)
      for (int b = 0; b < m_bands; b++)
      {
          avgCH.at<double>(b,k) = sumCH.at<double>(b,k) / (double) labelpixels.at<u_int32_t>(k);
      }
      GDALTermProgress( (float)(k+1) / (float)(labelpixels.rows), NULL, NULL );
  }
  GDALTermProgress( 1.0f, NULL, NULL );

  printf ("       Computing CLASS standard deviation\n");
  printf ("       ");
  for (int y = 0; y < klabels.rows; y++)
  {
      const int yklabels = y * klabels.cols;
      for (int x = 0; x < klabels.cols; x++)
      {
          const int i = yklabels + x;
          const int k = klabels.at<u_int32_t>(i);
          #pragma omp parallel for schedule(dynamic)
          for (int b = 0; b < m_bands; b++)
          {
            switch ( raster[b].depth() )
            {
              case CV_8U:
                stdCH.at<double>(b,k) += pow( (double) raster[b].at<uchar>(i)
                             - avgCH.at<double>(b,klabels.at<u_int32_t>(i)), 2 );
                break;
              case CV_8S:
                stdCH.at<double>(b,k) += pow( (double) raster[b].at<char>(i)
                             - avgCH.at<double>(b,klabels.at<u_int32_t>(i)), 2 );
                break;
              case CV_16U:
                stdCH.at<double>(b,k) += pow( (double) raster[b].at<ushort>(i)
                             - avgCH.at<double>(b,klabels.at<u_int32_t>(i)), 2 );
                break;
              case CV_16S:
                stdCH.at<double>(b,k) += pow( (double) raster[b].at<short>(i)
                             - avgCH.at<double>(b,klabels.at<u_int32_t>(i)), 2 );
                break;
              case CV_32S:
                stdCH.at<double>(b,k) += pow( (double) raster[b].at<u_int32_t>(i)
                             - avgCH.at<double>(b,klabels.at<u_int32_t>(i)), 2 );
                break;
              case CV_32F:
                stdCH.at<double>(b,k) += pow( (double) raster[b].at<float>(i)
                             - avgCH.at<double>(b,klabels.at<u_int32_t>(i)), 2 );
                break;
              case CV_64F:
                stdCH.at<double>(b,k) += pow( (double) raster[b].at<double>(i)
                             - avgCH.at<double>(b,klabels.at<u_int32_t>(i)), 2 );
                break;
              default:
                CV_Error( Error::StsInternal, "\nERROR: Invalid raster depth" );
                break;
            }
          }
      }
      GDALTermProgress( (float)(y+1) / (float)(klabels.rows), NULL, NULL );
  }
  GDALTermProgress( 1.0f, NULL, NULL );

  printf ("       Normalize CLASS standard deviation\n");
  printf ("       ");

  for (int k = 0; k < labelpixels.rows; k++)
  {
      #pragma omp parallel for schedule(dynamic)
      for (int b = 0; b < m_bands; b++)
      {
          stdCH.at<double>(b,k) = sqrt(stdCH.at<double>(b,k) / labelpixels.at<u_int32_t>(k));
      }
      GDALTermProgress( (float)(k+1) / (float)(labelpixels.rows), NULL, NULL );
  }
  GDALTermProgress( 1.0f, NULL, NULL );

  printf ("       Computing CLASS variance\n");
  printf ("       ");
  for (int k = 0; k < labelpixels.rows; k++)
  {
      #pragma omp parallel for schedule(dynamic)
      for (int b = 0; b < m_bands; b++)
      {
            switch ( raster[b].depth() )
            {
              case CV_8U:
                varCH.at<double>(b,k) += pow( (double) stdCH.at<double>(b,k), 2 );
                break;
              case CV_8S:
              varCH.at<double>(b,k) += pow( (double) stdCH.at<double>(b,k), 2 );
                break;
              case CV_16U:
              varCH.at<double>(b,k) += pow( (double) stdCH.at<double>(b,k), 2 );
                break;
              case CV_16S:
              varCH.at<double>(b,k) += pow( (double) stdCH.at<double>(b,k), 2 );
                break;
              case CV_32S:
              varCH.at<double>(b,k) += pow( (double) stdCH.at<double>(b,k), 2 );
                break;
              case CV_32F:
              varCH.at<double>(b,k) += pow( (double) stdCH.at<double>(b,k), 2 );
                break;
              case CV_64F:
              varCH.at<double>(b,k) += pow( (double) stdCH.at<double>(b,k), 2 );
                break;
              default:
                CV_Error( Error::StsInternal, "\nERROR: Invalid raster depth" );
                break;
            }          
      }
      GDALTermProgress( (float)(k+1) / (float)(labelpixels.rows), NULL, NULL );
  }
  GDALTermProgress( 1.0f, NULL, NULL );


}
