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

/*
 "SLIC Superpixels Compared to State-of-the-art Superpixel Methods"
 Radhakrishna Achanta, Appu Shaji, Kevin Smith, Aurelien Lucchi, Pascal Fua,
 and Sabine Susstrunk, IEEE TPAMI, Volume 34, Issue 11, Pages 2274-2282,
 November 2012.

 "SLIC Superpixels" Radhakrishna Achanta, Appu Shaji, Kevin Smith,
 Aurelien Lucchi, Pascal Fua, and Sabine Süsstrunk, EPFL Technical
 Report no. 149300, June 2010.

 "SEEDS: Superpixels Extracted via Energy-Driven Sampling",
 Van den Bergh M., Boix X., Roig G., de Capitani B. and Van Gool L.,
 In European Conference on Computer Vision (Vol. 7, pp. 13-26)., 2012

 "Superpixel Segmentation using Linear Spectral Clustering"
 Zhengqin Li, Jiansheng Chen,
 IEEE Conference on Computer Vision and Pattern Recognition (CVPR),
 Jun. 2015

 */

/* slic-segment.cpp */
/* SLIC superpixels segment */

#include <omp.h>
#include <set>
#include <fstream>
#include <iostream>

#include "gdal.h"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"
#include "cpl_string.h"
#include "cpl_csv.h"

#include "superpixel.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

using namespace std;
using namespace cv;
using namespace cv::ximgproc;

int main(int argc, char ** argv)
{
  const char *algo = "";
  vector< string > InFilenames;
  const char *OutFilename = NULL;
  const char *OutFormat = "ESRI Shapefile";
  
  // general defaults
  int niter = 0;
  bool blur = false;
  int regionsize = 0;

  // some counters
  int64 startTime, endTime;
  int64 startSecond, endSecond;
  double frequency = cv::getTickFrequency();

  // register
  GDALAllRegister();
  OGRRegisterAll();

  argc = GDALGeneralCmdLineProcessor( argc, &argv, 0 );

  if( argc < 1 )
    exit( -argc );

  // default help
  bool help = false;
  bool askhelp = false;

  /*
   * load arguments
   */
  for( int i = 1; i < argc; i++ )
  {
    if( argv[i][0] == '-' )
    {
      if( EQUAL( argv[i],"-help" ) ) {
        askhelp = true;
        break;
      }
      if( EQUAL( argv[i],"-algo" ) ) {
        algo = argv[i+1];
        i++; continue;
      }
      if( EQUAL( argv[i],"-region" ) ) {
        regionsize = atoi(argv[i+1]);
        i++; continue;
      }
      if( EQUAL( argv[i],"-niter" ) ) {
        niter = atoi(argv[i+1]);
        i++; continue;
      }
      if( EQUAL( argv[i],"-out" ) ) {
        OutFilename = argv[i+1];
        i++; continue;
      }
      if( EQUAL( argv[i],"-of" ) ) {
        OutFormat = argv[i+1];
        i++; continue;
      }
      if( EQUAL( argv[i],"-blur" ) ) {
        blur = true;
        continue;
      }
      printf( "Invalid %s option.\n\n", argv[i] );
      help = true;
    }
    else if( argv[i][0] != '-' )
    {
      InFilenames.push_back( argv[i] );
      continue;
    }
  }

  if ( !askhelp )
  {
    // check parameters
    if ( EQUAL( algo, "SLIC"  )
      || EQUAL( algo, "SLICO" ) )
    {
        if (!niter) niter = 10;
        if (!regionsize) regionsize = 10;
    }
    else if ( EQUAL( algo, "SEEDS" ) )
    {
        if (!niter) niter = 20;
        if (!regionsize) regionsize = 10;
    }
    else if ( EQUAL( algo, "LSC" ) )
    {
        if (!niter) niter = 20;
        if (!regionsize) regionsize = 10;
    }
    else
    {
      if ( EQUAL(algo, "" ) )
        printf( "\nERROR: No algorithm specified.\n" );
      else
        printf( "\nERROR: Invalid algorithm: %s\n", algo );
      help = true;
    }
    if ( InFilenames.size() == 0 )
    {
      printf( "\nERROR: No input file specified.\n" );
      help = true;
    }
    if (! OutFilename )
    {
      printf( "\nERROR: No output file specified.\n" );
      help = true;
    }
  }

  if ( help || askhelp ) {
    printf( "\nUsage: gdal-segment [-help] src_rgb src_entropy -out dst_vector\n"
            "    [-of <output_format> 'ESRI Shapefile' is default]\n"
            "    [-b R B (N-th band from R-th raster)]\n" 
            "    [-algo <LSC, SLICO, SLIC, SEEDS>]\n"
            "    [-niter <1..500>] [-region <pixels>]\n"
            "    [-blur (apply 3x3 gaussian blur)]\n\n"
            "Default niter: 10 iterations\n\n" );

    GDALDestroyDriverManager();
    exit( 1 );
  }

  #if GDALVER >= 2
    GDALDriverManager *poR = GetGDALDriverManager();
    GDALDriver *poDriver = poR->GetDriverByName( OutFormat );
  #else 
    OGRSFDriverRegistrar *poR = OGRSFDriverRegistrar::GetRegistrar();
    OGRSFDriver *poDriver = poR->GetDriverByName( OutFormat );
  #endif

  // check drivers
  if( poDriver == NULL )
  {
    printf( "Unable to find driver `%s'.\n", OutFormat );
    printf( "The following drivers are available:\n" );
    for( int i = 0; i < poR->GetDriverCount(); i++ )
    {
      #if GDALVER >= 2
        const char *prop = poR->GetDriver(i)->GetMetadataItem( GDAL_DCAP_RASTER );
        if ( prop == NULL )
        {
          const char *name = poR->GetDriver(i)->GetDescription();
          const char *desc = poR->GetDriver(i)->GetMetadataItem( GDAL_DMD_LONGNAME );
          printf( "  -> '%s' (%s)\n", name, desc );
        }
        #else
          printf( "  -> '%s'\n", poR->GetDriver(i)->GetName());
        #endif
    }
    exit( 1 );
  }

  printf( "Segments raster using: %s\n", algo );
  printf( "Process use parameter: region=%i niter=%i\n", regionsize, niter );

  /*
   * load rgb, entropy, HSV, and Lab images
   */
  startTime = cv::getTickCount();
  std::vector< cv::Mat > raster;
  std::vector< cv::Mat > entropy;
  std::vector< cv::Mat > hsv;
  std::vector< cv::Mat > lab;

  printf("rgb: %s\n", InFilenames[0].c_str());
  printf("local entropy: %s\n", InFilenames[1].c_str());
  printf("hsv: %s\n", InFilenames[2].c_str());
  printf("lab: %s\n", InFilenames[3].c_str());

  LoadRaster( InFilenames[0], raster );
  LoadRaster( InFilenames[1], entropy );
  LoadRaster( InFilenames[2], hsv );
  LoadRaster( InFilenames[3], lab );
  endTime = cv::getTickCount();
  printf( "Time: %.6f sec\n\n", ( endTime - startTime ) / frequency );

  if ( blur )
  {
    printf( "Apply Gaussian Blur (3x3 kernel)\n" );
    startTime = cv::getTickCount();
    for( size_t b = 0; b < raster.size(); b++ )
    {
      GaussianBlur( raster[b], raster[b], Size( 3, 3 ), 0.0f, 0.0f, BORDER_DEFAULT );
      GDALTermProgress( (float)b / (float)raster.size(), NULL, NULL );
    }
    GDALTermProgress( 1.0f, NULL, NULL );
    endTime = cv::getTickCount();
    printf( "Time: %.6f sec\n\n", ( endTime - startTime ) / frequency );
  }

  /*
   * init segments
   */
  printf( "Init Superpixels\n" );
  Ptr<SuperpixelSLIC> slic;
  Ptr<SuperpixelSEEDS> seed;
  Ptr<SuperpixelLSC> lsc;
  cv::Mat klabels;
  size_t m_labels = 0;

  size_t m_bands = raster.size();
  size_t entropy_bands = entropy.size();
  size_t hsv_bands = hsv.size();
  size_t lab_bands = lab.size();

  startTime = cv::getTickCount();
  if ( EQUAL( algo, "SLIC" ) || EQUAL( algo, "SLICO" ) )
  {
    slic = createSuperpixelSLIC( raster, SLIC, regionsize, 10.0f );
    m_labels = slic->getNumberOfSuperpixels();
  }
  else if ( EQUAL( algo, "SLICO" ) )
  {
    slic = createSuperpixelSLIC( raster, SLICO, regionsize, 10.0f );
    m_labels = slic->getNumberOfSuperpixels();
  }
  else if ( EQUAL( algo, "LSC" ) ) 
  {
    lsc = createSuperpixelLSC( raster, regionsize, 0.075f );
    m_labels = lsc->getNumberOfSuperpixels();    
  }
  else if ( EQUAL( algo, "SEEDS" ) )
  {
    // only few datatype is supported
    if ( ( raster[0].depth() != CV_8U ) &&( raster.size() != 3 ) )
    {
      printf( "\nERROR: Input datatype is not supported by SEED. Use RGB or Gray with Byte types.\n" );
      exit( 0 );
    }

    int clusters = int(((float)raster[0].cols / (float)regionsize) * ((float)raster[0].rows / (float)regionsize));
    seed = createSuperpixelSEEDS( raster[0].cols, raster[0].rows, raster.size(), clusters, 1, 2, 5, true );
    m_labels = seed->getNumberOfSuperpixels();
  }
  else
  {
    printf( "\nERROR: No such algorithm: [%s].\n", algo );
    exit( 1 );
  }
  endTime = cv::getTickCount();
  printf( "Time: %.6f sec\n\n", ( endTime - startTime ) / frequency );
  printf( "Grow Superpixels: #%i iterations\n", niter );
  printf( "           inits: %lu superpixels\n", m_labels );
   
  /*
   * start compute segments
   */
  startSecond = cv::getTickCount();
  if ( EQUAL( algo, "SLIC" ) || EQUAL( algo, "SLICO" ) )
  {
    slic->iterate( niter );    
    slic->enforceLabelConnectivity();
    m_labels = slic->getNumberOfSuperpixels();
    slic->getLabels( klabels );
    slic.release();
  }
  else if ( EQUAL( algo, "LSC" ) ) 
  {
    lsc->iterate( niter );    
    lsc->enforceLabelConnectivity();
    m_labels = lsc->getNumberOfSuperpixels();
    lsc->getLabels( klabels );
    lsc.release();
  } 
  else if ( EQUAL( algo, "SEEDS" ) ) 
  {
    cv::Mat whole;
    cv::merge(raster,whole);
    seed->iterate( whole, niter );    
    m_labels = seed->getNumberOfSuperpixels();
    seed->getLabels( klabels );
    seed.release();
  }
  endSecond = cv::getTickCount();
  printf( "          update: %lu superpixels (merged in %.6f sec)\n", m_labels, ( endSecond - startSecond ) / frequency );

  /*
   * get segments contour
   */
  startTime = cv::getTickCount();
  std::vector< std::vector< LINE > > linelists( m_labels );
  LabelContours( klabels, linelists );
  endTime = cv::getTickCount();
  printf( "Time: %.6f sec\n\n", ( endTime - startTime ) / frequency );

  /*
   * declaration
   */  
   Mat labelpixels(m_labels, 1, CV_32S);
   Mat avgRGB(m_bands, m_labels, CV_64F);
   Mat stdRGB(m_bands, m_labels, CV_64F);
   Mat varRGB(m_bands, m_labels, CV_64F);
   Mat avgEntropy(entropy_bands, m_labels, CV_64F);
   Mat stdEntropy(entropy_bands, m_labels, CV_64F);
   Mat varEntropy(entropy_bands, m_labels, CV_64F);
   Mat avgHSV(hsv_bands, m_labels, CV_64F);
   Mat stdHSV(hsv_bands, m_labels, CV_64F);
   Mat varHSV(hsv_bands, m_labels, CV_64F);
   Mat avgLab(lab_bands, m_labels, CV_64F);
   Mat stdLab(lab_bands, m_labels, CV_64F);
   Mat varLab(lab_bands, m_labels, CV_64F);
 
  std::vector<double> area( m_labels ); 
  std::vector<double> perimeter( m_labels );   
  std::vector<double> width( m_labels );
  std::vector<double> height( m_labels ); 
  std::vector<double> elongation( m_labels );
  std::vector<double> circularity( m_labels );
  std::vector<double> rectangularity( m_labels );
  std::vector<double> compactness( m_labels );

  /*
  * staticsts
  */
  startTime = cv::getTickCount();
  ComputeStats( klabels, raster, labelpixels, avgRGB, stdRGB, varRGB);  
  ComputeStats( klabels, entropy, labelpixels, avgEntropy, stdEntropy, varEntropy);
  ComputeStats( klabels, hsv, labelpixels, avgHSV, stdHSV, varHSV);
  ComputeStats( klabels, lab, labelpixels, avgLab, stdLab, varLab);
  endTime = cv::getTickCount();
  printf( "Computed statistics: %.6f sec\n\n", ( endTime - startTime ) / frequency );

  /* 
  * geometries
  */
  startTime = cv::getTickCount();
  ComputeGeometries(m_labels, 
                  labelpixels, 
                  area, perimeter, width, height, 
                  elongation, circularity, rectangularity, compactness, 
                  linelists );
  endTime = cv::getTickCount();
  printf( "Computed geometries: %.6f sec\n\n", ( endTime - startTime ) / frequency );

  /*
  * dump vector
  */
  startTime = cv::getTickCount();
  SavePolygons( InFilenames, 
                OutFilename, 
                OutFormat,                 
                m_bands, 
                entropy_bands,
                hsv_bands, 
                lab_bands,
                labelpixels, 
                avgRGB, stdRGB, varRGB, 
                avgEntropy, stdEntropy, varEntropy,
                avgHSV, stdHSV, varHSV,
                avgLab, stdLab, varLab,
                area, perimeter, width, height, 
                elongation, circularity, rectangularity, compactness, 
                linelists );
  endTime = cv::getTickCount();
  printf( "Saved polygon: %.6f sec\n\n", ( endTime - startTime ) / frequency );

  /*
  * write csv
  */
  startTime = cv::getTickCount();
  writeCSV( OutFilename,      
            m_labels,          
            m_bands, 
            entropy_bands,                
            hsv_bands, 
            lab_bands, 
            avgRGB, stdRGB, varRGB, 
            avgEntropy, stdEntropy, varEntropy,
            avgHSV, stdHSV, varHSV, 
            avgLab, stdLab, varLab,
            area, perimeter, width, height, 
            elongation, circularity, rectangularity, compactness );
  endTime = cv::getTickCount();
  printf( "Wrote CSF file: %.6f sec\n\n", ( endTime - startTime ) / frequency );
  
  /*
  * write arff
  */
  startTime = cv::getTickCount();
  writeARFF( OutFilename,      
            m_labels,          
            m_bands, 
            entropy_bands,                
            hsv_bands, 
            lab_bands, 
            avgRGB, stdRGB, varRGB, 
            avgEntropy, stdEntropy, varEntropy,
            avgHSV, stdHSV, varHSV, 
            avgLab, stdLab, varLab,
            area, perimeter, width, height, 
            elongation, circularity, rectangularity, compactness );
  endTime = cv::getTickCount();
  printf( "Wrote ARFF file: %.6f sec\n\n", ( endTime - startTime ) / frequency );

  printf( "Finish.\n" );

  return 0;

}
