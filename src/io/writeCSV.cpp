/*
 *  Copyright (c) 2017  Rodolfo Lotte (lotte@dsr.inpe.br - rodolfo.lotte@gmail.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERRGBANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "superpixel.hpp"

using namespace std;

void writeCSV(const char *OutFilename,        
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
              const std::vector<double> compactness )
{
  
  ofstream csvSegments;
  string filename_no_extension = remove_extension(OutFilename);
  string filename = filename_no_extension + ".csv";
  csvSegments.open(filename.c_str(),  fstream::in | fstream::out | fstream::trunc);    
  
  std::string headerAvg, headerStd, headerVar;
  stringstream value;
  csvSegments << "id, "; 

  // rgb
  for (size_t b = 0; b < m_bands; b++)
  {    
    value << b + 1;
    headerAvg = value.str() + "_avg, ";
    csvSegments << headerAvg;

    headerStd = value.str() + "_std, ";
    csvSegments << headerStd;

    headerVar = value.str() + "_var, ";
    csvSegments << headerVar;    
    
    value.str("");
  }

  // local entropy
  for (size_t b = 0; b < entropy_bands; b++)
  {      
    value << b + 1;
    headerAvg = value.str() + "_e_avg, ";
    csvSegments << headerAvg;

    headerStd = value.str() + "_e_std, ";
    csvSegments << headerStd;

    headerVar = value.str() + "_e_var, ";
    csvSegments << headerVar;    

    value.str("");
  }

  // hsv
  for (size_t b = 0; b < hsv_bands; b++)
  {      
    value << b + 1;
    headerAvg = value.str() + "_hsv_avg, ";
    csvSegments << headerAvg;

    headerStd = value.str() + "_hsv_std, ";
    csvSegments << headerStd;

    headerVar = value.str() + "_hsv_var, ";
    csvSegments << headerVar;    

    value.str("");
  }

  // lab
  for (size_t b = 0; b < lab_bands; b++)
  {      
    value << b + 1;
    headerAvg = value.str() + "_lab_avg, ";
    csvSegments << headerAvg;

    headerStd = value.str() + "_lab_std, ";
    csvSegments << headerStd;

    headerVar = value.str() + "_lab_var, ";
    csvSegments << headerVar;    

    value.str("");
  }

  // geometries
  csvSegments << "area, perimeter, width, height, elongation, circularity, rectang, compactness, class\n";  
  for (size_t k = 0; k < m_labels; k++)
  {
    csvSegments << k << ", ";
    for(u_int32_t avg=0; avg < m_bands; avg++)
    {
      csvSegments << avgRGB.at<double>(avg,k) << ", ";
    }
    for(u_int32_t std=0; std < m_bands; std++)
    {
      csvSegments << stdRGB.at<double>(std,k) << ", ";
    }
    for(u_int32_t var=0;var < m_bands; var++)
    {
      csvSegments << varRGB.at<double>(var,k) << ", ";
    }
    
    for(u_int32_t avg=0; avg < entropy_bands; avg++)
    {
      csvSegments << avgEntropy.at<double>(avg,k) << ", ";
    }
    for(u_int32_t std=0; std < entropy_bands; std++)
    {
      csvSegments << stdEntropy.at<double>(std,k) << ", ";
    }
    for(u_int32_t var=0;var < entropy_bands; var++)
    {
      csvSegments << varEntropy.at<double>(var,k) << ", ";
    }

    for(u_int32_t avg=0; avg < hsv_bands; avg++)
    {
      csvSegments << avgHSV.at<double>(avg,k) << ", ";
    }
    for(u_int32_t std=0; std < hsv_bands; std++)
    {
      csvSegments << stdHSV.at<double>(std,k) << ", ";
    }
    for(u_int32_t var=0;var < hsv_bands; var++)
    {
      csvSegments << varHSV.at<double>(var,k) << ", ";
    }

    for(u_int32_t avg=0; avg < lab_bands; avg++)
    {
      csvSegments << avgLab.at<double>(avg,k) << ", ";
    }
    for(u_int32_t std=0; std < lab_bands; std++)
    {
      csvSegments << stdLab.at<double>(std,k) << ", ";
    }
    for(u_int32_t var=0;var < lab_bands; var++)
    {
      csvSegments << varLab.at<double>(var,k) << ", ";
    }
    csvSegments << area[k] << ", " << perimeter[k] << ", " << width[k] << ", " << height[k] << ", " << elongation[k] << ", " << circularity[k] << ", " << rectangularity[k] << ", " << compactness[k] << ", " << "?\n";
  }

  csvSegments.close();  
}

 

