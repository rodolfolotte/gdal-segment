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

/* superpixel.hpp */
/* main include */

#ifndef SLICSEG_H
#define SLICSEG_H

#include <opencv2/opencv.hpp>
#include <math.h>

#ifdef _WIN
typedef __int32 u_int32_t;
#endif

typedef struct LINE {
  unsigned int sX;
  unsigned int sY;
  unsigned int eX;
  unsigned int eY;
} LINE;

std::string remove_extension(const std::string& filename);

// raster operation
void LoadRaster( std::string InFilename, std::vector< cv::Mat >& raster );

// raster statistics
void ComputeStats( const cv::Mat klabels,
                   const std::vector< cv::Mat > raster,
                   cv::Mat& labelpixels,                   
                   cv::Mat& avgCH,
                   cv::Mat& stdCH,
                   cv::Mat& varCH );

// polygon geometries
void ComputeGeometries ( size_t m_labels,
                  cv::Mat& labelpixels,
                  std::vector<double>& area, 
                  std::vector<double>& perimeter, 
                  std::vector<double>& width, 
                  std::vector<double>& height, 
                  std::vector<double>& elongation, 
                  std::vector<double>& circularity, 
                  std::vector<double>& rectangularity, 
                  std::vector<double>& compactness,                   
                  std::vector< std::vector< LINE > >& linelists );

// vector contours
void LabelContours( const cv::Mat klabels, std::vector< std::vector< LINE > >& linelists);

// vector dump
void SavePolygons( const std::vector< std::string > InFilenames,
                   const char *OutFilename, 
                   const char *OutFormat,                   
                   const u_int32_t m_bands,
                   const u_int32_t entropy_bands,
                   const u_int32_t hsv_bands,
                   const u_int32_t lab_bands,
                   const cv::Mat labelpixels,                   
                   const cv::Mat avgRGB,
                   const cv::Mat stdRGB,
                   const cv::Mat varRGB,                   
                   const cv::Mat avgEntropy,
                   const cv::Mat stdEntropy,
                   const cv::Mat varEntropy,
                   const cv::Mat avgHSV, 
                   const cv::Mat stdHSV, 
                   const cv::Mat varHSV,
                   const cv::Mat avgLab, 
                   const cv::Mat stdLab, 
                   const cv::Mat varLab,
                   const std::vector<double> area,
                   const std::vector<double> perimeter,                   
                   const std::vector<double> width, 
                   const std::vector<double> height, 
                   const std::vector<double> elongation, 
                   const std::vector<double> circularity, 
                   const std::vector<double> rectangularity, 
                   const std::vector<double> compactness,
                   std::vector< std::vector< LINE > >& linelists );

// write csv
void writeCSV( const char *OutFilename,                                         
               size_t m_labels,      
               const u_int32_t m_bands,
               const u_int32_t entropy_bands,              
               const u_int32_t hsv_bands,
               const u_int32_t lab_bands,    
               const cv::Mat avgRGB,
               const cv::Mat stdRGB,
               const cv::Mat varRGB,                    
               const cv::Mat avgEntropy,
               const cv::Mat stdEntropy,
               const cv::Mat varEntropy,
               const cv::Mat avgHSV,
               const cv::Mat stdHSV,
               const cv::Mat varHSV,
               const cv::Mat avgLab,
               const cv::Mat stdLab,
               const cv::Mat varLab,
               const std::vector<double> area,
               const std::vector<double> perimeter,                   
               const std::vector<double> width, 
               const std::vector<double> height, 
               const std::vector<double> elongation, 
               const std::vector<double> circularity, 
               const std::vector<double> rectangularity, 
               const std::vector<double> compactness );

// write arff
void writeARFF( const char *OutFilename,                                         
                size_t m_labels,      
                const u_int32_t m_bands,
                const u_int32_t entropy_bands,              
                const u_int32_t hsv_bands,
                const u_int32_t lab_bands,    
                const cv::Mat avgRGB,
                const cv::Mat stdRGB,
                const cv::Mat varRGB,                    
                const cv::Mat avgEntropy,
                const cv::Mat stdEntropy,
                const cv::Mat varEntropy,
                const cv::Mat avgHSV,
                const cv::Mat stdHSV,
                const cv::Mat varHSV,
                const cv::Mat avgLab,
                const cv::Mat stdLab,
                const cv::Mat varLab,
                const std::vector<double> area,
                const std::vector<double> perimeter,                   
                const std::vector<double> width, 
                const std::vector<double> height, 
                const std::vector<double> elongation, 
                const std::vector<double> circularity, 
                const std::vector<double> rectangularity, 
                const std::vector<double> compactness );

#endif
